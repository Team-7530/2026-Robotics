package frc.robot.operator_interface;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Test controller bindings for drivetrain characterization and tuning.
 *
 * <p>This class contains commented-out button bindings for:
 * - SysId routines (forward/reverse quasistatic and dynamic)
 * - Manual drive test routines (forward/backward at fixed velocity)
 * - PathPlanner pose-based navigation tests
 *
 * <p>To enable these bindings during tuning:
 * 1. Call {@link #configure(OperatorInterface, CommandSwerveDrivetrain)} from RobotContainer
 * 2. Uncomment the desired button mapping blocks
 * 3. Re-build and deploy to the robot
 *
 * <p><b>Warning:</b> These bindings are for test/tuning only and should never be active during
 * competition.
 */
public class TestControllerBindings {
  /**
   * Configure test controller bindings for drivetrain tuning.
   *
   * @param oi The OperatorInterface instance
   * @param drivetrain The CommandSwerveDrivetrain instance
   */
  public static void configure(OperatorInterface oi, CommandSwerveDrivetrain drivetrain) {
    // Example sysId routines (uncomment to enable):
    // oi.getStartButton()
    //     .and(oi.getYButton())
    //     .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // oi.getStartButton()
    //     .and(oi.getXButton())
    //     .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // oi.getBackButton().and(oi.getYButton()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // oi.getBackButton().and(oi.getXButton()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

    // Example drive test routines (uncomment to enable):
    // oi.getStartButton()
    //     .and(oi.getAButton())
    //     .whileTrue(drivetrain.applyRequest(() ->
    //         new RobotCentric().withVelocityX(0.5).withVelocityY(0)));
    // oi.getStartButton()
    //     .and(oi.getBButton())
    //     .whileTrue(drivetrain.applyRequest(() ->
    //         new RobotCentric().withVelocityX(-0.5).withVelocityY(0)));

    // Example PathPlanner pose-based navigation tests (uncomment to enable):
    // oi.getBackButton()
    //     .and(oi.getAButton())
    //     .whileTrue(
    //         new PathOnTheFlyCommand(
    //             drivetrain, new Pose2d(16.24, 0.8, Rotation2d.fromDegrees(-60))));
    // oi.getBackButton()
    //     .and(oi.getBButton())
    //     .whileTrue(
    //         new PathOnTheFlyCommand(
    //             drivetrain, new Pose2d(10.0, 0.8, Rotation2d.fromDegrees(60))));
  }

  /**
   * Schedule the PathPlanner warmup command. This prepares PathPlanner's internal pathfinding
   * structures on startup to improve latency during autonomous and teleop.
   *
   * @return A command that performs the warmup
   */
  public static Command getPathPlannerWarmupCommand() {
    return PathfindingCommand.warmupCommand();
  }
}
