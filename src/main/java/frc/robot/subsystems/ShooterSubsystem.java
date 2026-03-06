package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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

  private AngularVelocity flywheelVelocity = RPM.of(8000);
  private boolean m_isSpinup = false;
  private Telemetry telemetry;

  private static final double TURRET_X_OFFSET = 0;
  private static final double TURRET_Y_OFFSET = 0;
  private static final double HUB_OFFSET_X = -0.4;
  private static final double HUB_OFFSET_Y = -0.4;
      
  public ShooterSubsystem(frc.robot.Telemetry tele) {

    // inject telemetry into nested subsystems so they can publish centrally
    this.telemetry = tele;
    this.turret = new TurretSubsystem(telemetry);
    this.flywheel = new FlywheelSubsystem(telemetry);
    this.feeder = new FeederSubsystem(telemetry);
    this.collector = new CollectorSubsystem(telemetry);
  }
  
  @Override
  public void periodic() {
    updateTelemetry();
  }

  private void updateTelemetry() {
  }

  public double getVelocity() {
    return this.flywheelVelocity.magnitude();
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


  // -- Commands -----------------------------------------------------------
  // spin flywheel up to the given velocity
  public Command setFlywheelVelocityCommand(AngularVelocity velocity) {
    flywheelVelocity = velocity;
    if (m_isSpinup) {
      return flywheel.flywheelStartCommand(() -> flywheelVelocity)
        .withName("setFlywheelVelocityCommand");
    }
    return Commands.none();
  }

  public Command shooterSpinupCommand(AngularVelocity velocity) {
    m_isSpinup = true;
    flywheelVelocity = velocity;
    return flywheel.flywheelStartCommand(() -> velocity)
      .alongWith(feeder.feederStartCommand())
      .withName("shooterSpinupCommand");
  }

  public Command shooterSpinupCommand() {
    return flywheel.flywheelStartCommand(() -> flywheelVelocity)
      .alongWith(feeder.feederStartCommand())
      .withName("shooterSpinupCommand");
  }

  public Command shooterStopCommand() {
    m_isSpinup = false;
  return flywheel.flywheelStopCommand()
    .alongWith(feeder.feederStopCommand())
    .alongWith(collector.collectorStopCommand())
    .withName("shooterStopCommand");
  }
  
  public Command shooterStartCommand() {
    // start collector wheel to feed balls
    if (!m_isSpinup) {
      return collector.collectorStartCommand()
        .withName("shooterStartCommand");
    }
    return Commands.none();
  }

  public Command shooterUnstuckCommand() {
    // reverse collector briefly to clear jams
    if (m_isSpinup) {
      // if we're trying to shoot, run the collector unstuck command in parallel with the feeder to minimize downtime
      return collector.collectorUnstuckCommand()
        .alongWith(feeder.feederUnstuckCommand())
        .withName("shooterUnstuckCommand")
        .withTimeout(1.0)
        .andThen(feeder.feederStartCommand())
        .alongWith(collector.collectorStartCommand());
    }
    return collector.collectorUnstuckCommand()
        .alongWith(feeder.feederUnstuckCommand())
        .withName("shooterUnstuckCommand")
        .withTimeout(1.0)
        .andThen(this.shooterStopCommand());
  }

  public Command aimTurretAtHubCommand(CommandSwerveDrivetrain drivetrain) {
    return Commands.run(() -> {
      Pose2d robotPose = drivetrain.getState().Pose;
      Angle turretAngle = getTurretAngleToHub(robotPose);
      turret.setAngleDirect(turretAngle);
    }, turret)
      .withName("AimTurretAtHubCommand")
      .withTimeout(2.0);
  }
}
