package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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

  private AngularVelocity flywheelVelocity = RPM.of(8000);
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
  // spin flywheel up to the given velocity
  public Command setFlywheelVelocityCommand(AngularVelocity velocity) {
    return runOnce(() -> flywheelVelocity = velocity);
  }

  public Command turretToAngleCommand(Angle angle) {
    // move turret to specified field-relative angle
    return turret.setAngle(angle, Degrees.of(0.1))
        .withName("TurretToAngleCommand");
  }

  public Command flywheelToPercentCommand(double pct) {
    // set flywheel power directly (open-loop)
  return flywheel.setDutyCycle(pct)
    .withName("FlywheelToPercentCommand");
  }

  public Command flywheelStartCommand(AngularVelocity velocity) {
    // set flywheel power directly (open-loop)
  return flywheel.flywheelStartCommand(velocity)
    .withName("FlywheelStartCommandWithVelocity");
  }

  public Command flywheelStartCommand() {
    // set flywheel power directly (open-loop)
  return flywheel.flywheelStartCommand(flywheelVelocity)
    .withName("FlywheelStartCommand");
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
