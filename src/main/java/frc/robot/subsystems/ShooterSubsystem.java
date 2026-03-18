package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.Telemetry;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ShooterSubsystem controls a turret (absolute encoder + position control) and a two-motor
 * flywheel (master + follower). This is a lightweight, conservative implementation that uses
 * percent open-loop control for the flywheel and position control for the turret. 
 */
@Logged
public class ShooterSubsystem extends SubsystemBase {
  private final Telemetry telemetry;

  // Shooter owns the turret and all Fuel-moving mechanisms used during a shot.
  public final TurretSubsystem turret;
  public final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  public final FeederSubsystem feeder = new FeederSubsystem();
  public final CollectorSubsystem collector = new CollectorSubsystem();

  private final CommandSwerveDrivetrain drivetrain;

  private static final AngularVelocity MANUAL_PROFILE_LOW_VELOCITY = RPM.of(3250);
  private static final AngularVelocity MANUAL_PROFILE_HIGH_VELOCITY = RPM.of(4000);
  private static final AngularVelocity FLYWHEEL_APPLY_TOLERANCE = RPM.of(25);

  // Offsets for turret position relative to robot center (in robot frame)
  private static final Translation2d TURRET_OFFSET = new Translation2d(Inches.of(7.0).in(Meters), 0); // x: forward, y: left
  // Offsets for hub position (applied in field frame)
  private static final Translation2d HUB_OFFSET = new Translation2d(0, 0);
  // Offsets for bump position (applied in field frame)
  private static final Translation2d BUMP_OFFSET = new Translation2d(0, 0);

  // Distance-to-velocity mapping for hub shots (linear interpolation).
  // At VELOCITY_DISTANCE_MIN, shoot at VELOCITY_AT_MIN.
  // Velocity increases by VELOCITY_SLOPE per meter beyond VELOCITY_DISTANCE_MIN.
  private static final Distance VELOCITY_DISTANCE_MIN = Meters.of(3.0);
  private static final AngularVelocity VELOCITY_AT_MIN = RPM.of(3000);
  private static final AngularVelocity VELOCITY_SLOPE = RPM.of(400.0 / 1.65); // RPM per meter
  private static final double SHOOTER_READY_WAIT_SECONDS = 0.6;

  private AngularVelocity flywheelVelocity = MANUAL_PROFILE_LOW_VELOCITY;
  private AngularVelocity manualFlywheelVelocity = MANUAL_PROFILE_LOW_VELOCITY;

  @NotLogged
  private AngularVelocity lastAppliedFlywheelVelocity = null;

  private boolean m_isSpinup = false;
  private String manualProfileName = "Fixed3250";
  @Logged(importance = Logged.Importance.CRITICAL)
  private String activeShotProfile = manualProfileName;

  @Logged(importance = Logged.Importance.CRITICAL)
  private Translation2d turretFieldPosition = new Translation2d();
  @Logged(importance = Logged.Importance.CRITICAL)
  private Translation2d vectorToTarget = new Translation2d();
  @Logged(importance = Logged.Importance.CRITICAL)
  private Rotation2d fieldAngleToTarget = new Rotation2d();
  @Logged(importance = Logged.Importance.CRITICAL)
  private Rotation2d turretRelativeAngle = new Rotation2d();

  @Logged(importance = Logged.Importance.CRITICAL)
  private Angle turretAngleToTarget = Degrees.of(0);
  @Logged(importance = Logged.Importance.CRITICAL)
  private Distance distanceToTarget = Meters.of(0);

  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain, Telemetry telemetry) {
    this.drivetrain = drivetrain;
    this.telemetry = telemetry;
    this.turret = new TurretSubsystem(telemetry);
  }
  
  @Override
  public void periodic() {
    // Targeting commands update the desired RPM; periodic only pushes changes when needed.
    applyDesiredFlywheelVelocityIfNeeded();
    updateTelemetry();
  }

  // Shared aiming path for tracked shots: compute geometry, point turret, and update desired RPM.
  private void updateTargeting(Pose2d robotPose, Translation2d targetPosition) {
    turretFieldPosition = getTurretFieldPosition(robotPose);
    vectorToTarget = targetPosition.minus(turretFieldPosition);
    // Robot-relative turret angle: subtract robot heading from field angle
    fieldAngleToTarget = new Rotation2d(vectorToTarget.getAngle().getMeasure());
    turretRelativeAngle = fieldAngleToTarget.minus(robotPose.getRotation());

    turretAngleToTarget = turretRelativeAngle.unaryMinus().getMeasure();
    distanceToTarget = Meters.of(turretFieldPosition.getDistance(targetPosition));

    turret.setAngleDirect(turretAngleToTarget);
    this.setFlywheelVelocityOnDistance(distanceToTarget);

    telemetry.putNumber("Shooter/TargetFieldX", targetPosition.getX());
    telemetry.putNumber("Shooter/TargetFieldY", targetPosition.getY());
  }

  private void updateTelemetry() {
    telemetry.putNumber("Shooter/TurretFieldX", turretFieldPosition.getX());
    telemetry.putNumber("Shooter/TurretFieldY", turretFieldPosition.getY());
    telemetry.putNumber("Shooter/TurretAngleToTargetDeg", turretAngleToTarget.in(Degrees));
    telemetry.putNumber("Shooter/DistanceToTargetM", distanceToTarget.in(Meters));
    telemetry.putNumber("Shooter/FlywheelVelocityRPM", getFlywheelVelocity().in(RPM));
    telemetry.putString("Shooter/ActiveShotProfile", activeShotProfile);
  }

  @Logged(importance = Logged.Importance.CRITICAL)
  public AngularVelocity getFlywheelVelocity() {
    return this.flywheelVelocity;
  }

  private void setDesiredFlywheelVelocity(AngularVelocity velocity) {
    this.flywheelVelocity = velocity;
  }

  // Avoid resending the same setpoint every loop while still letting tracked shots adjust on the fly.
  private void applyDesiredFlywheelVelocityIfNeeded() {
    if (!isSpinup()) {
      lastAppliedFlywheelVelocity = null;
      return;
    }

    if (lastAppliedFlywheelVelocity == null
        || !lastAppliedFlywheelVelocity.isNear(flywheelVelocity, FLYWHEEL_APPLY_TOLERANCE)) {
      flywheel.setVelocityDirect(flywheelVelocity);
      lastAppliedFlywheelVelocity = flywheelVelocity;
    }
  }

  /**
   * Set flywheel velocity from measured shot distance using a simple linear fit.
   * 
   * <p>Uses a piecewise linear model: at distances <= VELOCITY_DISTANCE_MIN, the flywheel
   * spins at VELOCITY_AT_MIN (e.g., 60 rot/s). For greater distances, velocity increases
   * by VELOCITY_SLOPE rot/s per meter. This compensates for energy loss over distance.
   * 
   * @param distance Distance from turret to the active target.
   *                 Should be >= 0 meters.
   */
  public void setFlywheelVelocityOnDistance(Distance distance) {
    double distanceMeters = distance.in(Meters);
    double minDistanceMeters = VELOCITY_DISTANCE_MIN.in(Meters);
    
    // Linear interpolation: start at VELOCITY_AT_MIN, increase by VELOCITY_SLOPE per meter
    AngularVelocity targetVelocity;
    if (distanceMeters <= minDistanceMeters) {
      targetVelocity = VELOCITY_AT_MIN;
    } else {
      double excessDistance = distanceMeters - minDistanceMeters;
      targetVelocity = VELOCITY_AT_MIN.plus(VELOCITY_SLOPE.times(excessDistance));
    }
    this.setDesiredFlywheelVelocity(targetVelocity);
  }

  public Translation2d getTurretFieldPosition(Pose2d robotPose) {
    // Rotate the turret offset by the robot's heading, then add to robot position
    Translation2d rotatedOffset = TURRET_OFFSET.rotateBy(robotPose.getRotation());
    return robotPose.getTranslation().plus(rotatedOffset);
  }

  /**
   * Get the hub position for our alliance, including tunable offsets.
   * @return Hub center position as Translation2d (with offsets applied)
   */
  public Translation2d getHubPosition() {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    Translation2d hubBase = Constants.Field.getHubPose(isBlue).getTranslation();
    return hubBase.plus(HUB_OFFSET);
  }

  /**
   * Get straight-line distance from TURRET to the target.
   * Uses actual turret position, not robot center.
   *
   * @return Distance in meters
   */
  public Distance getDistanceToTarget(Pose2d robotPose, Translation2d targetPosition) {
    Translation2d turretPosition = getTurretFieldPosition(robotPose);
    return Meters.of(turretPosition.getDistance(targetPosition));
  }

  /**
   * Get the ROBOT-RELATIVE turret angle needed to point at the target.
   * 
   * <p>Converts the turret field position to a field-relative target angle to a turret-relative
   * command by subtracting the robot's heading. This angle can be directly fed to the
   * turret motor controller.
   * 
   * <p>This returns the angle the turret should hold relative to the robot chassis.
   * 
   * @param robotPose Current robot pose (position and heading) from odometry.
   * @return Robot-relative turret angle in radians (-π to +π). Positive = counterclockwise.
   */
  public Angle getTurretAngleToTarget(Pose2d robotPose, Translation2d targetPosition) {
    Translation2d vectorToTarget = targetPosition.minus(getTurretFieldPosition(robotPose));
    Rotation2d fieldAngleToTarget = new Rotation2d(vectorToTarget.getAngle().getMeasure());
    Rotation2d turretRelativeAngle = fieldAngleToTarget.minus(robotPose.getRotation());

    // Turret positive direction is opposite the field-angle sign convention used above.
    return turretRelativeAngle.unaryMinus().getMeasure();
  }

  // Aim at the nearer bump-side lane by picking the offset on our half of the field.
  private Translation2d getBumpTarget(Pose2d robotPose) {
    if (robotPose.getY() < Constants.Field.FIELD_HALF_WIDTH.in(Meters)) {
      return getHubPosition().minus(Constants.Field.BUMP_CENTER_OFFSET).plus(BUMP_OFFSET);
    }
    return getHubPosition().plus(Constants.Field.BUMP_CENTER_OFFSET).plus(BUMP_OFFSET);    
  }

  private void targetHub() {
    activeShotProfile = "HubTrack";
    updateTargeting(drivetrain.getState().Pose, getHubPosition());
  }

  private void targetBump() {
    activeShotProfile = "BumpTrack";
    Pose2d robotPose = drivetrain.getState().Pose;
    updateTargeting(robotPose, getBumpTarget(robotPose));
  }

  private void setManualShotProfile(AngularVelocity velocity, String profileName) {
    manualFlywheelVelocity = velocity;
    manualProfileName = profileName;
    restoreManualShotProfile();
  }

  // When tracked aiming ends, fall back to the last manual preset instead of leaving a stale dynamic RPM.
  private void restoreManualShotProfile() {
    activeShotProfile = manualProfileName;
    setDesiredFlywheelVelocity(manualFlywheelVelocity);
  }

  private void setSpinup(boolean isSpinup) {
    this.m_isSpinup = isSpinup;
    if (!isSpinup) {
      lastAppliedFlywheelVelocity = null;
    }
  }

  private boolean isSpinup() {
    return this.m_isSpinup;
  }

  private boolean isFlywheelReady() {
    return isSpinup() && flywheel.isAtSpeed(getFlywheelVelocity());
  }

  private Command fuelFeedCommand() {
    return feeder.feederStartCommand()
        .alongWith(collector.collectorStartCommand())
        .withName("ShooterFuelFeedCommand");
  }

  private Command fuelUnstuckCommand() {
    return feeder.feederUnstuckCommand()
        .alongWith(collector.collectorUnstuckCommand())
        .andThen(feeder.feederStartCommand().withTimeout(0.2))
        .withName("ShooterFuelUnstuckCommand");
  }

  // -- Commands -----------------------------------------------------------
  // Set the requested flywheel speed; periodic applies it only when spinup is armed.
  public Command setFlywheelVelocityCommand(AngularVelocity velocity) {
    return Commands.runOnce(() -> setDesiredFlywheelVelocity(velocity))
        .withName("setFlywheelVelocityCommand");
  }

  public Command setFlywheelVelocityCommand(Supplier<AngularVelocity> velocitySupplier) {
    return Commands.runOnce(() -> setDesiredFlywheelVelocity(velocitySupplier.get()))
        .withName("setFlywheelVelocityCommand");
  }

  public Command shooterSpinupCommand(Supplier<AngularVelocity> velocitySupplier) {
    return Commands.runOnce(() -> setSpinup(true))
      .andThen(setFlywheelVelocityCommand(velocitySupplier))
      // The flywheel command keeps running; periodic updates the setpoint if targeting changes it.
      .andThen(flywheel.flywheelStartCommand(this::getFlywheelVelocity))
      .withTimeout(0.2)
      .withName("shooterSpinupCommand");
  }

  public Command shooterSpinupCommand(AngularVelocity velocity) {
    return shooterSpinupCommand(() -> velocity);
  }

  public Command shooterSpinupCommand() {
    return shooterSpinupCommand(this::getFlywheelVelocity);
  }

  public Command shooterSpinupIfNeededCommand() {
    return Commands.either(
            Commands.none(),
            shooterSpinupCommand(),
            this::isSpinup)
        .withName("shooterSpinupIfNeededCommand");
  }

  public Command shooterStopCommand() {
    return runOnce(() -> setSpinup(false))
      .andThen(flywheel.flywheelStopCommand()
          .alongWith(feeder.feederStopCommand())
          .alongWith(collector.collectorStopCommand()))
      .withTimeout(0.2)
      .withName("shooterStopCommand");
  }
  
  public Command shooterStartCommand() {
    return Commands.waitUntil(this::isFlywheelReady)
      .withTimeout(SHOOTER_READY_WAIT_SECONDS)
      // After the short wait, feed anyway if we are still armed; operators sometimes want the fallback shot.
      .andThen(fuelFeedCommand())
      .onlyIf(this::isSpinup)
      .withName("shooterStartCommand");
  }

  public Command shooterUnstuckCommand() {
    return fuelUnstuckCommand()
      .withName("shooterUnstuckCommand");
  }

  public Command setManualLowShotProfileCommand() {
    return Commands.runOnce(() -> setManualShotProfile(MANUAL_PROFILE_LOW_VELOCITY, "Fixed3250"))
      .withName("SetManualLowShotProfileCommand");
  }

  public Command setManualHighShotProfileCommand() {
    return Commands.runOnce(() -> setManualShotProfile(MANUAL_PROFILE_HIGH_VELOCITY, "Fixed4000"))
      .withName("SetManualHighShotProfileCommand");
  }

  /**
   * One-shot aim command for autonomous routines.
   * Applies cached turret angle and flywheel velocity once, then finishes.
   * Use this in PathPlanner autos before shooting.
   */
  public Command targetHubOnceCommand() {
    return Commands.runOnce(this::targetHub, turret)
      .withName("targetHubOnceCommand");
  }

  /**
   * Continuous aim command for teleop button hold.
   * Repeatedly updates turret angle and flywheel velocity while running.
   * Use this when binding to a button.
   */
  public Command targetHubCommand() {
    // Hold to keep recomputing turret angle and RPM from the current estimated pose.
    return Commands.runEnd(this::targetHub, this::restoreManualShotProfile, turret)
      .withName("targetHubCommand");
  }

  public Command targetBumpCommand() {
    // Same as hub tracking, but the target point moves to the nearer bump-side lane.
    return Commands.runEnd(this::targetBump, this::restoreManualShotProfile, turret)
      .withName("targetBumpCommand");
  }
}
