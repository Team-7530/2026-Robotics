package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathOnTheFlyCommand extends Command {

  private PathConstraints pathConstraints;
  private Command pathCommand;

  /** Creates a new driveToPose. */

  /**
   * Makes a new path towards a set position
   *
   * @param drivetrain
   * @param targetPose Pose2d
   */
  public PathOnTheFlyCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    this.pathConstraints =
        new PathConstraints(1.0, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    this.pathCommand = AutoBuilder.pathfindToPose(targetPose, pathConstraints, 0.0);

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    pathCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
