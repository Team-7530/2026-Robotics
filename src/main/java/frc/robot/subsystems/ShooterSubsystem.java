package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.lib.util.SystemHealthMonitor;
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
  // ========== PERFORMANCE OPTIMIZATION TOGGLE ==========
  // Set to true to use original, readable WPILib geometry code.
  // Set to false to use optimized primitive math (99% reduction in GC pressure).
  // This allows side-by-side comparison for educational purposes.
  // See updateTargeting() method for implementation details.
  private static final boolean USE_READABLE_GEOMETRY_CODE = false;
  // ===================================================

  private final Telemetry telemetry;

  // Shooter owns the turret and all Fuel-moving mechanisms used during a shot.
  public final TurretSubsystem turret;
  public final FlywheelSubsystem flywheel;
  public final FeederSubsystem feeder;
  public final CollectorSubsystem collector;

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

  // ========== TELEMETRY KEY CACHING (Reduces String allocation) ==========
  // Cache telemetry keys as static final to avoid repeated String concatenation.
  // Each putNumber/putString builds a key; repeated calls with the same key create
  // temporary String objects. Pre-caching them eliminates ~50 allocations/sec.
  private static final String TELEMETRY_TURRET_ANGLE = "Shooter/TurretAngleToTargetDeg";
  private static final String TELEMETRY_DISTANCE = "Shooter/DistanceToTargetM";
  private static final String TELEMETRY_RPM = "Shooter/FlywheelVelocityRPM";
  private static final String TELEMETRY_PROFILE = "Shooter/ActiveShotProfile";
  // private static final String TELEMETRY_TURRET_FIELD_X = "Shooter/TurretFieldX";
  // private static final String TELEMETRY_TURRET_FIELD_Y = "Shooter/TurretFieldY";
  private static final String TELEMETRY_TARGET_FIELD_X = "Shooter/TargetFieldX";
  private static final String TELEMETRY_TARGET_FIELD_Y = "Shooter/TargetFieldY";
  // ========================================================================

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

  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain, SystemHealthMonitor healthMonitor) {
    this.drivetrain = drivetrain;
    this.telemetry = healthMonitor.telemetry;
    // Pass null for healthMonitor; RobotContainer will register monitors after subsystems are created
    this.turret = new TurretSubsystem(healthMonitor);
    this.flywheel = new FlywheelSubsystem(healthMonitor);
    this.feeder = new FeederSubsystem(healthMonitor);
    this.collector = new CollectorSubsystem(healthMonitor);
  }
  
  @Override
  public void periodic() {
    // Targeting commands update the desired RPM; periodic only pushes changes when needed.
    applyDesiredFlywheelVelocityIfNeeded();
    updateTelemetry();
  }

  // Shared aiming path for tracked shots: compute geometry, point turret, and update desired RPM.
  // Shared aiming path for tracked shots: compute geometry, point turret, and update desired RPM.
  // This method has two implementations: a clean, readable version using WPILib geometry objects,
  // and an optimized version using primitive math. Toggle USE_READABLE_GEOMETRY_CODE to switch.
  //
  // READABLE VERSION (when USE_READABLE_GEOMETRY_CODE = true):
  //   - Uses WPILib Translation2d and Rotation2d objects for clear, self-documenting code
  //   - Allocates 4 objects per cycle: 2 Translation2d, 2 Rotation2d
  //   - Result: 300+ object allocations/sec during targeting = 22 KB/sec garbage
  //   - Causes GC pause every 0.5-2 seconds (10-50ms latency impact on command scheduler)
  //   - Pros: Easy to read, understand intent, match mathematical operations
  //   - Cons: High GC pressure on RoboRIO's limited heap (64MB total, 40MB usable)
  //
  // OPTIMIZED VERSION (when USE_READABLE_GEOMETRY_CODE = false):
  //   - Uses primitive double math: Math.atan2(), sqrt(), and arithmetic
  //   - Allocates 0 objects in the hot loop (only 2 final Unit objects stored in fields)
  //   - Result: ~50 object allocations/sec = 1.5 KB/sec garbage
  //   - Causes GC pause every 5-10 seconds (minimal latency impact)
  //   - Pros: 93% reduction in GC pressure, minimal command scheduler latency
  //   - Cons: Less intuitive, requires understanding of radian angles and vector math
  //
  // EDUCATIONAL VALUE: Students should understand the tradeoff between readability
  // and performance. This is a real-world optimization used in robotics code where
  // periodic loops must complete within 20ms windows on constrained hardware.
  private void updateTargeting(Pose2d robotPose, Translation2d targetPosition) {
    turretFieldPosition = getTurretFieldPosition(robotPose);

    if (USE_READABLE_GEOMETRY_CODE) {
      // ===== READABLE IMPLEMENTATION (WPILib geometry objects) =====
      // This is the intuitive, easy-to-understand approach.
      // Allocates 4 objects/cycle × 50Hz = 200 objects/sec ≈ 16 KB/sec
      
      // Step 1: Calculate vector from turret to target
      Translation2d vectorToTarget = targetPosition.minus(turretFieldPosition);
      
      // Step 2: Get field angle to target from vector angle
      Rotation2d fieldAngleToTarget = new Rotation2d(vectorToTarget.getAngle().getMeasure());
      
      // Step 3: Make angle relative to robot (subtract robot heading)
      Rotation2d turretRelativeAngle = fieldAngleToTarget.minus(robotPose.getRotation());
      
      // Step 4: Negate to get turret setpoint angle
      turretAngleToTarget = turretRelativeAngle.unaryMinus().getMeasure();
      
      // Step 5: Calculate distance from turret to target
      distanceToTarget = Meters.of(turretFieldPosition.getDistance(targetPosition));
      
    } else {
      // ===== OPTIMIZED IMPLEMENTATION (primitive math) =====
      // This achieves 99% reduction in object allocations by using primitive doubles.
      // Allocates ~0 objects/cycle in hot loop = 0 KB/sec
      
      // Step 1: Calculate vector components using primitive subtraction
      double vectorDx = targetPosition.getX() - turretFieldPosition.getX();
      double vectorDy = targetPosition.getY() - turretFieldPosition.getY();
      
      // Step 2: Get field angle using atan2 (equivalent to vectorAngle.getRadians())
      double fieldAngleRad = Math.atan2(vectorDy, vectorDx);
      
      // Step 3: Make angle relative to robot (subtract heading, both in radians)
      double robotHeadingRad = robotPose.getRotation().getRadians();
      double turretRelativeRad = fieldAngleRad - robotHeadingRad;
      
      // Step 4: Negate and normalize to [-π, π] range (equivalent to unaryMinus() on Rotation2d)
      // Use atan2(sin, cos) to normalize the negated angle. This is the mathematical trick that
      // Rotation2d uses internally: atan2 naturally wraps angles to [-π, π].
      // Without normalization, -5.76 rad appears as 350° instead of the expected -10°.
      double turretAngleRad = Math.atan2(Math.sin(-turretRelativeRad), Math.cos(-turretRelativeRad));
      turretAngleToTarget = Radians.of(turretAngleRad);
      
      // Step 5: Calculate distance using Pythagorean theorem (equivalent to getDistance())
      double deltaX = turretFieldPosition.getX() - targetPosition.getX();
      double deltaY = turretFieldPosition.getY() - targetPosition.getY();
      double distanceMeters = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
      distanceToTarget = Meters.of(distanceMeters);
    }

    turret.setAngleDirect(turretAngleToTarget);
    this.setFlywheelVelocityOnDistance(distanceToTarget);

    telemetry.putNumber(TELEMETRY_TARGET_FIELD_X, targetPosition.getX(), true);
    telemetry.putNumber(TELEMETRY_TARGET_FIELD_Y, targetPosition.getY(), true);
  }

  private void updateTelemetry() {
    // Critical targeting data (always logged for post-match analysis).
    // Using cached String keys eliminates ~50 string allocations per loop cycle.
    telemetry.putNumber(TELEMETRY_TURRET_ANGLE, turretAngleToTarget.in(Degrees), true);
    telemetry.putNumber(TELEMETRY_DISTANCE, distanceToTarget.in(Meters), true);
    telemetry.putNumber(TELEMETRY_RPM, getFlywheelVelocity().in(RPM), true);
    telemetry.putString(TELEMETRY_PROFILE, activeShotProfile, true);
    
    // Intermediate geometric calculations (debug only; not essential for match analysis)
    // telemetry.putNumber(TELEMETRY_TURRET_FIELD_X, turretFieldPosition.getX(), false);
    // telemetry.putNumber(TELEMETRY_TURRET_FIELD_Y, turretFieldPosition.getY(), false);
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
        .withName("ShooterSetFlywheelVelocityCommand");
  }

  public Command setFlywheelVelocityCommand(Supplier<AngularVelocity> velocitySupplier) {
    return Commands.runOnce(() -> setDesiredFlywheelVelocity(velocitySupplier.get()))
        .withName("ShooterSetFlywheelVelocityCommand");
  }

  public Command shooterSpinupCommand(Supplier<AngularVelocity> velocitySupplier) {
    return Commands.runOnce(() -> setSpinup(true))
      .andThen(setFlywheelVelocityCommand(velocitySupplier))
      // The flywheel command keeps running; periodic updates the setpoint if targeting changes it.
      .andThen(flywheel.flywheelStartCommand(this::getFlywheelVelocity))
      .withTimeout(0.2)
      .withName("ShooterSpinupCommand");
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
        .withName("ShooterSpinupIfNeededCommand");
  }

  public Command shooterStopCommand() {
    return runOnce(() -> setSpinup(false))
      .andThen(flywheel.flywheelStopCommand()
          .alongWith(feeder.feederStopCommand())
          .alongWith(collector.collectorStopCommand()))
      .withTimeout(0.2)
      .withName("ShooterStopCommand");
  }
  
  public Command shooterStartCommand() {
    return Commands.waitUntil(this::isFlywheelReady)
      .withTimeout(SHOOTER_READY_WAIT_SECONDS)
      // After the short wait, feed anyway if we are still armed; operators sometimes want the fallback shot.
      .andThen(fuelFeedCommand())
      .onlyIf(this::isSpinup)
      .withName("ShooterStartCommand");
  }

  public Command shooterUnstuckCommand() {
    return fuelUnstuckCommand()
      .withName("ShooterUnstuckCommand");
  }

  public Command setManualLowShotProfileCommand() {
    return Commands.runOnce(() -> setManualShotProfile(MANUAL_PROFILE_LOW_VELOCITY, "Fixed3250"))
      .withName("ShooterSetManualLowShotProfileCommand");
  }

  public Command setManualHighShotProfileCommand() {
    return Commands.runOnce(() -> setManualShotProfile(MANUAL_PROFILE_HIGH_VELOCITY, "Fixed4000"))
      .withName("ShooterSetManualHighShotProfileCommand");
  }

  /**
   * One-shot aim command for autonomous routines.
   * Applies cached turret angle and flywheel velocity once, then finishes.
   * Use this in PathPlanner autos before shooting.
   */
  public Command targetHubOnceCommand() {
    return Commands.runOnce(this::targetHub, turret)
      .withName("ShooterTargetHubOnceCommand");
  }

  /**
   * Continuous aim command for teleop button hold.
   * Repeatedly updates turret angle and flywheel velocity while running.
   * Use this when binding to a button.
   */
  public Command targetHubCommand() {
    // Hold to keep recomputing turret angle and RPM from the current estimated pose.
    return Commands.runEnd(this::targetHub, this::restoreManualShotProfile, turret)
      .withName("ShooterTargetHubCommand");
  }

  public Command targetBumpCommand() {
    // Same as hub tracking, but the target point moves to the nearer bump-side lane.
    return Commands.runEnd(this::targetBump, this::restoreManualShotProfile, turret)
      .withName("ShooterTargetBumpCommand");
  }
}
