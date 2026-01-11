package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveTeleopCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final OperatorInterface oi;

  public SwerveTeleopCommand(CommandSwerveDrivetrain drivetrain, OperatorInterface oi) {
    this.drivetrain = drivetrain;
    this.oi = oi;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivetrain.setDriveControl(
        oi.getTranslateX(),
        oi.getTranslateY(),
        oi.getRotate(),
        oi.getRobotRelative().getAsBoolean());
  }
}
