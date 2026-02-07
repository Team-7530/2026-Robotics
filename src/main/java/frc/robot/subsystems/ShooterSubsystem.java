package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
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

  public final TurretSubsystem turret = new TurretSubsystem();
  public final FlywheelSubsystem flywheel = new FlywheelSubsystem();
  public final FeederSubsystem feeder = new FeederSubsystem();

  private boolean m_isTeleop = false;
      
  public ShooterSubsystem() {
  
  }
  
  @Override
  public void periodic() {
    updateSmartDashboard();
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
    } else if (m_isTeleop) {
      m_isTeleop = false;
      turret.teleop(0.0);
      flywheel.teleop(0.0);
    }
  }
  // -- SmartDashboard ----------------------------------------------------
  private void updateSmartDashboard() {
  }

  // -- Commands -----------------------------------------------------------
  public Command turretToAngleCommand(double degrees) {
    return run(() -> turret.setTurretAngleDegrees(degrees))
        .withName("TurretToAngleCommand")
        .until(() -> MathUtil.isNear(degrees, turret.getTurretAngleDegrees(), 2.0))
        .withTimeout(5.0);
  }

  public Command shooterToVelocityCommand(double velocity) {
  return run(() -> flywheel.setVelocity(RPM.of(velocity)))
        .withName("ShooterToVelocityCommand")
        .withTimeout(5.0);
  }

  public Command shooterToPercentCommand(double pct) {
    return run(() -> flywheel.setDutyCycle(pct))
        .withName("ShooterToPercentCommand")
        .withTimeout(5.0);
  }

  public Command shootCommand() {
    return feeder.feederStartCommand();
  }

  public Command stopShootCommand() {
    return feeder.feederStopCommand();
  }

  public Command feederUnstuckCommand() {
    return feeder.feederUnstuckCommand();
  }
}
