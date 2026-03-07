package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
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

  private static final double TURRET_X_OFFSET = 0;// Inches.of(8).in(Meters);
  private static final double TURRET_Y_OFFSET = 0;
  private static final double HUB_OFFSET_X = 0;
  private static final double HUB_OFFSET_Y = 0;

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
    currentAngleToHub = Radians.of(Math.atan2(hubPosition.getY() - currentTurretFieldPosition.getY(), 
                                              hubPosition.getX() - currentTurretFieldPosition.getX()));

    double turretAngle = currentAngleToHub.in(Degrees) - robotPose.getRotation().getDegrees();
    currentTurretAngleToHub = Degrees.of(-(((turretAngle + 180) % 360 + 360) % 360 - 180));
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
  }

  public void setFlywheelVelocityOnDistance(Distance distance) {
    double distanceMeters = distance.in(Meters);
    // Example linear mapping: 2m -> 4000 RPM, 5m -> 8000 RPM
    double rpm = 3000.0;
    if (distanceMeters > 3.0)
      rpm += (distanceMeters - 3.0) * (400 / 1.65);
    this.setFlywheelVelocityDirect(RPM.of(rpm));
  }

  public Translation2d getTurretFieldPosition(Pose2d robotPose) {
    double robotHeadingRad = robotPose.getRotation().getRadians();
    double cos = Math.cos(robotHeadingRad);
    double sin = Math.sin(robotHeadingRad);

    double fieldOffsetX = TURRET_X_OFFSET * cos - TURRET_Y_OFFSET * sin;
    double fieldOffsetY = TURRET_X_OFFSET * sin + TURRET_Y_OFFSET * cos;

    return new Translation2d(
        robotPose.getX() + fieldOffsetX,
        robotPose.getY() + fieldOffsetY
    );
  }

  /**
   * Get the hub position for our alliance, including tunable offsets.
   * @return Hub center position as Translation2d (with offsets applied)
   */
  public Translation2d getHubPosition() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    boolean isBlue = alliance == DriverStation.Alliance.Blue;
    Translation2d toHub = Constants.Field.getHubPose(isBlue).getTranslation();

    double offsetX = HUB_OFFSET_X;
    double offsetY = HUB_OFFSET_Y;

    return new Translation2d(toHub.getX() + offsetX, toHub.getY() + offsetY);
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
   * Get the FIELD-RELATIVE angle from TURRET to hub.
   * 0 degrees = toward positive X axis of field.
   * @return Angle in degrees
   */
  public Angle getAngleToHub(Pose2d robotPose) {
    Translation2d hubPosition = getHubPosition();
    Translation2d turretPosition = getTurretFieldPosition(robotPose);

    double dx = hubPosition.getX() - turretPosition.getX();
    double dy = hubPosition.getY() - turretPosition.getY();

    return Radians.of(Math.atan2(dy, dx));
  }

  /**
   * Get the TURRET angle needed to point at the hub.
   * This is ROBOT-RELATIVE (accounts for robot heading).
   * @return Turret angle in degrees (-180 to +180)
   */
  public Angle getTurretAngleToHub(Pose2d robotPose) {
      double fieldAngleToHub = getAngleToHub(robotPose).in(Degrees);
      double robotHeading = robotPose.getRotation().getDegrees();

      double turretAngle = fieldAngleToHub - robotHeading;
      turretAngle = ((turretAngle + 180) % 360 + 360) % 360 - 180;

      return Degrees.of(-turretAngle);
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
    return Commands.runOnce(() -> setSpinup(true))
      .andThen(flywheel.flywheelStartCommand(this::getFlywheelVelocity).alongWith(feeder.feederStartCommand()))
      .withTimeout(0.2)
      .withName("shooterSpinupCommand");
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
        .withName("shooterUnstuckCommand")
        .withTimeout(1.0);
  }

  public Command aimTurretAtHubCommand(CommandSwerveDrivetrain drivetrain) {
    return Commands.run(() -> {
      Pose2d robotPose = drivetrain.getState().Pose;
      Angle turretAngle = getTurretAngleToHub(robotPose);
      turret.setAngleDirect(turretAngle);
    }, turret)
      .withName("AimTurretAtHubCommand");
  }

  public Command setFlywheelAtHubCommand(CommandSwerveDrivetrain drivetrain) {
    return Commands.run(() -> {
      Pose2d robotPose = drivetrain.getState().Pose;
      Distance distance = getDistanceToHub(robotPose);
      this.setFlywheelVelocityOnDistance(distance);
    }, flywheel)
      .withName("SetFlywheelAtHubCommand");
  }

  public Command targetHubCommand(CommandSwerveDrivetrain drivetrain) {
    return Commands.run(() -> {
      Pose2d robotPose = drivetrain.getState().Pose;
      Angle turretAngle = getTurretAngleToHub(robotPose);
      Distance distance = getDistanceToHub(robotPose);
      turret.setAngleDirect(turretAngle);
      this.setFlywheelVelocityOnDistance(distance);
    }, turret)
      .withName("targetHubCommand");
  }

  public Command targetHub3Command() {
    return Commands.runOnce(() -> {
      turret.setAngleDirect(currentTurretAngleToHub);
      this.setFlywheelVelocityOnDistance(currentDistanceToHub);
    }, turret)
      .withName("targetHub3Command");
  }

  public Command targetHub2Command() {
    return Commands.run(() -> {
      turret.setAngleDirect(currentTurretAngleToHub);
      this.setFlywheelVelocityOnDistance(currentDistanceToHub);
    }, turret)
      .withName("targetHubCommand");
  }
}
