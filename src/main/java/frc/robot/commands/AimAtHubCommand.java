package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Continuously point the turret at the center of the alliance's scoring hub.
 * <p>
 * If the vision system has a current global pose estimate (i.e. at least one
 * apriltag was visible during the last update) it will be used; otherwise the
 * drivetrain's odometry pose is the fallback.  Calling code can therefore
 * schedule this in autonomous or teleop and it will adapt as soon as a
 * vision fix becomes available.
 */
public class AimAtHubCommand extends Command {
  private final ShooterSubsystem shooter;
  private final CommandSwerveDrivetrain drivetrain;

  public AimAtHubCommand(ShooterSubsystem shooter,
                         CommandSwerveDrivetrain drivetrain) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // ensure the limelight is running the correct pipeline for our alliance's
    // hub tags; this makes the command self-contained from a setup standpoint.
    // vision.setHubPipelineForAlliance(
    // DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
    //                         == DriverStation.Alliance.Blue);
  }

  @Override
  public void execute() {
    Angle turretAngle = shooter.getTurretAngleToHub(drivetrain.getState().Pose);
    shooter.turret.setAngleDirect(turretAngle);
  }

  @Override
  public boolean isFinished() {
    // keep running until explicitly cancelled by caller
    return false;
  }
}
