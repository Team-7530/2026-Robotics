package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Simple command that delegates to {@link VisionSubsystem#updateGlobalPose}
 * on every scheduler run.  The subsystem already guards against doing the
 * update unless the drivetrain is essentially stopped, but having this as a
 * named command makes it easy to schedule manually or wire it into a chooser
 * for testing.
 */
public class UpdateGlobalPoseWhenStoppedCommand extends Command {
  private final VisionSubsystem vision;
  private final CommandSwerveDrivetrain drivetrain;

  public UpdateGlobalPoseWhenStoppedCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
    this.vision = vision;
    this.drivetrain = drivetrain;
  addRequirements(vision);
  }

  @Override
  public void initialize() {
    // no initialization required
  }

  @Override
  public void execute() {
    // vision subsystem will only actually push a measurement when the robot is
    // nearly motionless; keeping this command running continuously is harmless.
    vision.updateGlobalPose(drivetrain);
  }

  @Override
  public boolean isFinished() {
    // never finish on its own; typically used as a default/long-running command.
    return false;
  }
}
