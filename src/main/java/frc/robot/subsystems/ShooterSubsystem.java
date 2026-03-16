package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
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
import frc.robot.Constants;
import frc.robot.Telemetry;

/**
 * ShooterSubsystem controls a turret (absolute encoder + position control) and a two-motor
 * flywheel (master + follower). This is a lightweight, conservative implementation that uses
 * percent open-loop control for the flywheel and position control for the turret. 
 */
@Logged
public class ShooterSubsystem extends SubsystemBase {

    // Holds and manages turret, hood and flywheel

  public final TurretSubsystem turret;
  public final FlywheelSubsystem flywheel;
  public final FeederSubsystem feeder;
  public final CollectorSubsystem collector;

  private final Telemetry telemetry;
  private final CommandSwerveDrivetrain drivetrain;

  private Translation2d hubPosition = new Translation2d();
  private Translation2d currentTurretFieldPosition = new Translation2d();
  private Angle currentAngleToHub = Degrees.of(0);
  private Angle currentTurretAngleToHub = Degrees.of(0);
  private Distance currentDistanceToHub = Meters.of(0);

  // Offsets for turret position relative to robot center (in robot frame)
  private static final Translation2d TURRET_OFFSET = new Translation2d(0, 0); // x: forward, y: left
  // Offsets for hub position (applied in field frame)
  private static final Translation2d HUB_OFFSET = new Translation2d(0, 0);

  // Distance-to-velocity mapping for hub shots (linear interpolation).
  // At VELOCITY_DISTANCE_MIN, shoot at VELOCITY_AT_MIN.
  // Velocity increases by VELOCITY_SLOPE per meter beyond VELOCITY_DISTANCE_MIN.
  private static final Distance VELOCITY_DISTANCE_MIN = Meters.of(3.0);
  private static final AngularVelocity VELOCITY_AT_MIN = RPM.of(3000);
  private static final AngularVelocity VELOCITY_SLOPE = RPM.of(400.0 / 1.65); // RPM per meter

  private AngularVelocity flywheelVelocity = RPM.of(8000);
  private boolean m_isSpinup = false;

  public ShooterSubsystem(Telemetry tele, CommandSwerveDrivetrain drivetrain) {
    // inject telemetry into nested subsystems so they can publish centrally
    this.telemetry = tele;
    this.turret = new TurretSubsystem(telemetry);
    this.flywheel = new FlywheelSubsystem(telemetry);
    this.feeder = new FeederSubsystem(telemetry);
    this.collector = new CollectorSubsystem(telemetry);

    this.drivetrain = drivetrain;
  }
  
  @Override
  public void periodic() {
    updateTargeting();
    updateTelemetry();
  }

  private void updateTargeting() {
    Pose2d robotPose = drivetrain.getState().Pose;

    hubPosition = getHubPosition();
    currentTurretFieldPosition = getTurretFieldPosition(robotPose);
    
    // Field-relative angle: use Translation2d.minus() and getAngle()
    Translation2d vectorToHub = hubPosition.minus(currentTurretFieldPosition);
    currentAngleToHub = Radians.of(vectorToHub.getAngle().getRadians());

    // Robot-relative turret angle: subtract robot heading from field angle
    Rotation2d fieldAngleRot = new Rotation2d(currentAngleToHub.in(Radians));
    Rotation2d turretRelativeAngle = fieldAngleRot.minus(robotPose.getRotation());
    currentTurretAngleToHub = Radians.of(-turretRelativeAngle.getRadians());
    currentDistanceToHub = Meters.of(currentTurretFieldPosition.getDistance(hubPosition));
  }

  private void updateTelemetry() {
    telemetry.putNumber("Shooter/FlywheelVelocityRPM", getFlywheelVelocity().in(RPM));

    telemetry.putNumber("Shooter/HubFieldX", hubPosition.getX());
    telemetry.putNumber("Shooter/HubFieldY", hubPosition.getY());
    telemetry.putNumber("Shooter/TurretFieldX", currentTurretFieldPosition.getX());
    telemetry.putNumber("Shooter/TurretFieldY", currentTurretFieldPosition.getY());
    telemetry.putNumber("Shooter/AngleToHubDeg", currentAngleToHub.in(Degrees));
    telemetry.putNumber("Shooter/TurretAngleToHubDeg", currentTurretAngleToHub.in(Degrees));
    telemetry.putNumber("Shooter/DistanceToHubM", currentDistanceToHub.in(Meters));
  }

  @Logged
  public AngularVelocity getFlywheelVelocity() {
    return this.flywheelVelocity;
  }

  private void setFlywheelVelocityDirect(AngularVelocity velocity) {
    this.flywheelVelocity = velocity;
    if (isSpinup()) {
      this.flywheel.setVelocityDirect(velocity);
    }
  }

  /**
   * Set flywheel velocity based on distance to hub using linear interpolation.
   * 
   * <p>Uses a piecewise linear model: at distances <= VELOCITY_DISTANCE_MIN, the flywheel
   * spins at VELOCITY_AT_MIN (e.g., 60 rot/s). For greater distances, velocity increases
   * by VELOCITY_SLOPE rot/s per meter. This compensates for energy loss over distance.
   * 
   * @param distance Distance from turret to hub (typically from {@link #getDistanceToHub}).
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
    this.setFlywheelVelocityDirect(targetVelocity);
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
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    boolean isBlue = alliance == DriverStation.Alliance.Blue;
    Translation2d hubBase = Constants.Field.getHubPose(isBlue).getTranslation();
    return hubBase.plus(HUB_OFFSET);
  }

  /**
   * Get straight-line distance from TURRET to the hub center.
   * Uses actual turret position, not robot center.
   * @return Distance in meters
   */
  public Distance getDistanceToHub(Pose2d robotPose) {
      Translation2d hubPosition = getHubPosition();
      Translation2d turretPosition = getTurretFieldPosition(robotPose);
      return Meters.of(turretPosition.getDistance(hubPosition));
  }

  /**
   * Get the FIELD-RELATIVE angle from turret to hub.
   * 
   * <p>Coordinate frame: 0 radians points along the positive X axis of the field.
   * Returns the absolute angle from the turret's field position to the hub center,
   * independent of the robot's heading. Used by {@link #getTurretAngleToHub} to
   * compute robot-relative turret commands.
   * 
   * @param robotPose Current robot pose (position and heading) from odometry.
   * @return Field-relative angle to hub in radians (-π to +π).
   * @see #getTurretFieldPosition(Pose2d) for turret offset calculation
   * @see #getHubPosition() for hub center with tuning offsets
   */
  public Angle getAngleToHub(Pose2d robotPose) {
    Translation2d vectorToHub = getHubPosition().minus(getTurretFieldPosition(robotPose));
    return Radians.of(vectorToHub.getAngle().getRadians());
  }

  /**
   * Get the ROBOT-RELATIVE turret angle needed to point at the hub.
   * 
   * <p>Converts the field-relative hub angle ({@link #getAngleToHub}) to a turret-relative
   * command by subtracting the robot's heading. This angle can be directly fed to the
   * turret motor controller.
   * 
   * <p>Coordinate frame: The turret is assumed to be centered on the robot. This method
   * returns the angle the turret needs to rotate to from its current position to align
   * with the hub, accounting for the robot's field heading.
   * 
   * @param robotPose Current robot pose (position and heading) from odometry.
   * @return Robot-relative turret angle in radians (-π to +π). Positive = counterclockwise.
   * @see #getAngleToHub(Pose2d) for the field-relative angle calculation
   */
  public Angle getTurretAngleToHub(Pose2d robotPose) {
    Rotation2d fieldAngleToHub = new Rotation2d(getAngleToHub(robotPose).in(Radians));
    Rotation2d turretRelativeAngle = fieldAngleToHub.minus(robotPose.getRotation());

    // Negate because turret convention may differ; adjust if needed
    return Radians.of(-turretRelativeAngle.getRadians());
  }

  private void targetHub() {
    turret.setAngleDirect(currentTurretAngleToHub);
    this.setFlywheelVelocityOnDistance(currentDistanceToHub);
  }

  private void setSpinup(boolean isSpinup) {
    this.m_isSpinup = isSpinup;
  }

  private boolean isSpinup() {
    return this.m_isSpinup;
  }

  // -- Commands -----------------------------------------------------------
  // spin flywheel up to the given velocity
  public Command setFlywheelVelocityCommand(AngularVelocity velocity) {
    return Commands.runOnce(() -> setFlywheelVelocityDirect(velocity))
        .withName("setFlywheelVelocityCommand");
  }

  public Command setFlywheelVelocityCommand(Supplier<AngularVelocity> velocitySupplier) {
    return Commands.runOnce(() -> setFlywheelVelocityDirect(velocitySupplier.get()))
        .withName("setFlywheelVelocityCommand");
  }

  public Command shooterSpinupCommand(Supplier<AngularVelocity> velocitySupplier) {
    return Commands.runOnce(() -> setSpinup(true))
      .andThen(setFlywheelVelocityCommand(velocitySupplier))
      .andThen(flywheel.flywheelStartCommand(velocitySupplier).alongWith(feeder.feederStartCommand()))
      .withTimeout(0.2)
      .withName("shooterSpinupCommand");
  }

  public Command shooterSpinupCommand(AngularVelocity velocity) {
    return shooterSpinupCommand(() -> velocity);
  }

  public Command shooterSpinupCommand() {
    return shooterSpinupCommand(this::getFlywheelVelocity);
  }

  public Command shooterStopCommand() {
  return runOnce(() -> setSpinup(false))
    .andThen(flywheel.flywheelStopCommand().alongWith(feeder.feederStopCommand().alongWith(collector.collectorStopCommand())))
    .withTimeout(0.2)
    .withName("shooterStopCommand");
  }
  
  public Command shooterStartCommand() {
    // start collector wheel to feed balls
    return feeder.feederStartCommand().alongWith(collector.collectorStartCommand().onlyIf(this::isSpinup))
      .withTimeout(0.2)
      .withName("shooterStartCommand");
  }

  public Command shooterUnstuckCommand() {
    // reverse collector briefly to clear jams
      return collector.collectorUnstuckCommand().alongWith(feeder.feederUnstuckCommand())
        .withName("shooterUnstuckCommand");
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
    return Commands.run(this::targetHub, turret)
      .withName("targetHubCommand");
  }
}
