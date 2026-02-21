package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ShooterSubsystem controls a turret (absolute encoder + position control) and a two-motor
 * flywheel (master + follower). This is a lightweight, conservative implementation that uses
 * percent open-loop control for the flywheel and position control for the turret. 
 */
public class ShooterSubsystem extends SubsystemBase {

    // Holds and manages turret, hood and flywheel

  public final TurretSubsystem turret;
  public final FlywheelSubsystem flywheel;
  public final FeederSubsystem feeder;

  private boolean m_isTeleop = false;
      
  public ShooterSubsystem(frc.robot.Telemetry telemetry) {
    // inject telemetry into nested subsystems so they can publish centrally
    this.turret = new TurretSubsystem(telemetry);
    this.flywheel = new FlywheelSubsystem(telemetry);
    this.feeder = new FeederSubsystem(telemetry);
  }
  
  @Override
  public void periodic() {
    updateTelemetry();
  }

  /**
   * Teleop controls
   *
   * @param tspeed a double that sets the turret speed during teleop
   * @param fspeed a double that sets the flywheel speed during teleop
   */
  public void teleop(double tspeed, double fspeed) {
    tspeed = MathUtil.applyDeadband(tspeed, STICK_DEADBAND);
    fspeed = MathUtil.applyDeadband(fspeed, STICK_DEADBAND);
  
    if ((tspeed != 0.0) || (fspeed != 0.0)) {
      m_isTeleop = true;
      turret.teleop(tspeed);
      flywheel.teleop(fspeed);
      // feeder.teleop(feedSpeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      turret.teleop(0.0);
      flywheel.teleop(0.0);
      // feeder.teleop(0.0);
    }
  }

  private void updateTelemetry() {
  }

  // -- Commands -----------------------------------------------------------
  public Command turretToAngleCommand(double degrees) {
    // move turret to specified field-relative angle
    return turret.setTurretAngleDegrees(degrees)
        .withName("TurretToAngleCommand");
  }

  public Command flywheelToVelocityCommand(double velocity) {
    // spin flywheel up to the given RPM
  return flywheel.flywheelStartCommand(velocity)
    .withName("FlywheelToVelocityCommand");
  }

  public Command flywheelToPercentCommand(double pct) {
    // set flywheel power directly (open-loop)
  return flywheel.setDutyCycle(pct)
    .withName("FlywheelToPercentCommand");
  }

  public Command flywheelStopCommand() {
    // set flywheel power directly (open-loop)
  return flywheel.flywheelStopCommand()
    .withName("FlywheelStopCommand");
  }
  
  public Command feederStartCommand() {
    // start feeder wheel to feed balls
    return feeder.feederStartCommand();
  }

  public Command feederStopCommand() {
    // stop feeder wheel
    return feeder.feederStopCommand();
  }

  public Command feederUnstuckCommand() {
    // reverse feeder briefly to clear jams
    return feeder.feederUnstuckCommand();
  }

  /** Return a command that runs the flywheel backward briefly to unjam it. */
  public Command flywheelUnstuckCommand() {
    return flywheel.flywheelUnstuckCommand();
  }

}
