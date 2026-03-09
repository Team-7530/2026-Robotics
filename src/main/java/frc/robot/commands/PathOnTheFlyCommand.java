package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathOnTheFlyCommand extends SequentialCommandGroup {

  /** Creates a new driveToPose. */

  /**
   * Makes a new path towards a set position
   *
   * @param drivetrain
   * @param targetPose Pose2d
   */
  public PathOnTheFlyCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
    PathConstraints pathConstraints =
        new PathConstraints(1.0, 1.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    addCommands(AutoBuilder.pathfindToPose(targetPose, pathConstraints, 0.0));
    setName("PathOnTheFlyCommand");
  }
}
