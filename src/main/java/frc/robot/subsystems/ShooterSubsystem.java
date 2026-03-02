package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  public final CollectorSubsystem collector;

  private AngularVelocity flywheelVelocity = RPM.of(8000);
  private boolean m_isSpinup = false;
      
  public ShooterSubsystem(frc.robot.Telemetry telemetry) {
    // inject telemetry into nested subsystems so they can publish centrally
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
  @Logged
  public double getVelocity() {
    return this.flywheelVelocity.magnitude();
  }

  // -- Commands -----------------------------------------------------------
  // spin flywheel up to the given velocity
  public Command setFlywheelVelocityCommand(AngularVelocity velocity) {
    flywheelVelocity = velocity;
    if (m_isSpinup) {
      return flywheel.flywheelStartCommand(flywheelVelocity)
        .withName("setFlywheelVelocityCommand");
    }
    return Commands.none();
  }

  public Command shooterSpinupCommand(AngularVelocity velocity) {
    m_isSpinup = true;
    flywheelVelocity = velocity;
    return this.shooterSpinupCommand();
  }

  public Command shooterSpinupCommand() {
    return flywheel.flywheelStartCommand(flywheelVelocity)
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
}
