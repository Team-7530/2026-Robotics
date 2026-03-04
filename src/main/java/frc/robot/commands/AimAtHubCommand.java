package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

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
  private final VisionSubsystem vision;
  private final CommandSwerveDrivetrain drivetrain;

  public AimAtHubCommand(ShooterSubsystem shooter,
                         VisionSubsystem vision,
                         CommandSwerveDrivetrain drivetrain) {
    this.shooter = shooter;
    this.vision = vision;
    this.drivetrain = drivetrain;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // ensure the limelight is running the correct pipeline for our alliance's
    // hub tags; this makes the command self-contained from a setup standpoint.
    vision.setHubPipelineForAlliance(
    DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                            == DriverStation.Alliance.Blue);
  }

  @Override
  public void execute() {
    // prefer a vision-derived pose if available, otherwise use odometry
    Optional<Pose2d> maybeVisionPose =
      vision.getVisionMeasurement_MT1().map(est -> est.pose);
    Pose2d currentPose = maybeVisionPose.orElse(drivetrain.getState().Pose);

    // pick the appropriate hub position for the current alliance; the constant
    // already includes any centering offset from the raw tag coordinates.
    boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                        == DriverStation.Alliance.Blue;
    Translation2d toHub = Constants.Field.getHubPose(isBlue)
                            .getTranslation()
                            .minus(currentPose.getTranslation());
    double turretAngle = toHub.getAngle().getDegrees();
    shooter.turret.setAngleDirect(Degrees.of(turretAngle));
  }

  @Override
  public boolean isFinished() {
    // keep running until explicitly cancelled by caller
    return false;
  }
}
