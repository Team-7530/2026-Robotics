package frc.robot;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.sim.Mechanisms;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static RobotContainer instance;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.RobotCentric forwardStraight =
  //     new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(DriveTrainConstants.maxSpeed.vxMetersPerSecond);

  /* Operator Interface */
  public OperatorInterface oi = new OperatorInterface() {};

  /* Subsystems */
  public final PowerDistribution power = new PowerDistribution();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final VisionSubsystem vision = new VisionSubsystem();
  public final ArmSubsystem arm = new ArmSubsystem();
  public final WristSubsystem wrist = new WristSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ClimberSubsystem climber = new ClimberSubsystem();

  /* Path follower */
  private SendableChooser<Command> autoChooser;
  private Command autonomousCommand;

  private Mechanisms mechanism = new Mechanisms();

  public static RobotContainer GetInstance() {
    return instance;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    instance = this;

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    drivetrain.setMaxSpeeds(DriveTrainConstants.cruiseSpeed);
    configureAutoPaths();
    configureAutoCommands();
    configureTelemetry();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      SmartDashboard.putNumber("DriveTrain/Drive Scaling", oi.driveScalingValue());
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureButtonBindings() {
    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // x-stance
    oi.getXStanceButton().whileTrue(drivetrain.applyRequest(() -> brake));

    oi.driveScalingUp()
        .onTrue(Commands.runOnce(() -> drivetrain.setMaxSpeeds(DriveTrainConstants.maxSpeed)));
    oi.driveScalingDown()
        .onTrue(Commands.runOnce(() -> drivetrain.setMaxSpeeds(DriveTrainConstants.cruiseSpeed)));
    oi.driveScalingSlow()
        .onTrue(Commands.runOnce(() -> drivetrain.setMaxSpeeds(DriveTrainConstants.slowSpeed)))
        .onFalse(Commands.runOnce(() -> drivetrain.setMaxSpeeds(DriveTrainConstants.cruiseSpeed)));

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    // oi.getStartButton()
    //     .and(oi.getYButton())
    //     .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // oi.getStartButton()
    //     .and(oi.getXButton())
    //     .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // oi.getBackButton().and(oi.getYButton()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // oi.getBackButton().and(oi.getXButton()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

    // // Run test routines (forward/back at .5 m/s) when holding start and A/B.
    // oi.getStartButton()
    //     .and(oi.getAButton())
    //     .whileTrue(
    //         drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    // oi.getStartButton()
    //     .and(oi.getBButton())
    //     .whileTrue(
    //         drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // // Run test pose routines when holding back and A/B.
    // oi.getBackButton()
    //     .and(oi.getAButton())
    //     .whileTrue(
    //         new PathOnTheFlyCommand(
    //             drivetrain, new Pose2d(16.24, 0.8, Rotation2d.fromDegrees(-60))));
    // oi.getBackButton()
    //     .and(oi.getBButton())
    //     .whileTrue(
    //         new PathOnTheFlyCommand(
    //             drivetrain, new Pose2d(13.85, 2.67, Rotation2d.fromDegrees(124))));

    oi.getAButton().onTrue(intake.intakeCommand());
    oi.getXButton().onTrue(intake.outtakeL2Command());
    oi.getBButton().onTrue(intake.outtakeL1Command());
    oi.getYButton().onTrue(this.cruisePositionCommand());

    oi.getPOVUp().onTrue(this.getCoralPositionCommand());
    oi.getPOVDown().onTrue(this.climbPositionCommand());
    oi.getPOVLeft().onTrue(this.l1ScoringPositionCommand());
    oi.getPOVRight().onTrue(this.l2ScoringPositionCommand());

    oi.getLeftBumper().onTrue(climber.clampCommand(false));
    oi.getRightBumper().onTrue(climber.clampCommand(true));

    oi.getLeftTrigger().onTrue(this.cruisePositionCommand());
    oi.getRightTrigger().onTrue(new SequentialCommandGroup(this.getCoralPositionCommand(), intake.intakeCommand()));

    //oi.getLeftTrigger().onTrue(climber.climbToStartPositionCommand());
    //oi.getRightTrigger().onTrue(climber.climbToFullPositionCommand());

    //    oi.getStartButton().onTrue(Commands.runOnce(() -> climber.resetMotorPostion()));
    //    oi.getBackButton().onTrue(new DoAllResetCommand(arm, wrist, climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    // Add commands to Autonomous Sendable Chooser
    autoChooser = AutoBuilder.buildAutoChooser("Forward");

    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new SwerveTeleopCommand(drivetrain, oi));

    arm.setDefaultCommand(Commands.run(() -> arm.teleop(-oi.getLeftThumbstickY()), arm));
    wrist.setDefaultCommand(Commands.run(() -> wrist.teleop(oi.getLeftThumbstickX()), wrist));
    climber.setDefaultCommand(
        Commands.run(() -> climber.teleopClimb(-oi.getRightThumbstickY()), climber));
    vision.setDefaultCommand(vision.updateGlobalPoseCommand(drivetrain));
  }

  private void configureAutoPaths() {
    NamedCommands.registerCommand("Intake", intake.intakeCommand());
    NamedCommands.registerCommand("Outtake", intake.outtakeL2Command());
    NamedCommands.registerCommand("OuttakeSpin", intake.outtakeL1Command());
    NamedCommands.registerCommand("SetClimbPos", this.climbPositionCommand());
    NamedCommands.registerCommand("SetCruisePos", this.cruisePositionCommand());
    NamedCommands.registerCommand("GetCoral", this.getCoralPositionCommand());
    NamedCommands.registerCommand("SetL1Score", this.l1ScoringPositionCommand());
    NamedCommands.registerCommand("SetL2Score", this.l2ScoringPositionCommand());
    NamedCommands.registerCommand("UpdatePose", vision.updateGlobalPoseCommand(drivetrain));
  }

  private void configureTelemetry() {
    drivetrain.registerTelemetry(logger::telemeterize);

    SmartDashboard.putData("AutoChooser", autoChooser);
    SmartDashboard.putData("Intake", intake.intakeCommand());
    SmartDashboard.putData("Outtake", intake.outtakeL2Command());
    SmartDashboard.putData("OuttakeSpin", intake.outtakeL1Command());
    SmartDashboard.putData("GetCoral", this.getCoralPositionCommand());
    SmartDashboard.putData("SetClimbPos", this.climbPositionCommand());
    SmartDashboard.putData("SetCruisePos", this.cruisePositionCommand());
    SmartDashboard.putData("SetL1Score", this.l1ScoringPositionCommand());
    SmartDashboard.putData("SetL2Score", this.l2ScoringPositionCommand());
    SmartDashboard.putData("ClimbToFull", climber.climbToFullPositionCommand());
    SmartDashboard.putData("UpdatePose", vision.updateGlobalPoseCommand(drivetrain));
    SmartDashboard.putData(
        "L2Backup", drivetrain.driveDistanceCommand(ScoringConstants.L2BackupAmount, 0.5));
  }

  public void robotPeriodic() {
    mechanism.update(
        arm.getRotorPosition(),
        arm.getPosition(),
        wrist.getPosition(),
        climber.getRotorPosition(),
        climber.getPosition());
  }

  public void simulationInit() {}

  public void simulationPeriodic() {

    // Update camera simulation
    vision.simulationPeriodic(drivetrain.getState().Pose);

    var debugField = vision.getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(drivetrain.getState().Pose);
    debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.getModulePoses());

    // // Calculate battery voltage sag due to current draw
    // var batteryVoltage =
    //         BatterySim.calculateDefaultBatteryLoadedVoltage(drivetrain.getCurrentDraw());
    // Using max(0.1, voltage) here isn't a *physically correct* solution,
    // but it avoids problems with battery voltage measuring 0.
    RoboRioSim.setVInVoltage(Math.max(0.1, RobotController.getBatteryVoltage()));

    PhysicsSim.getInstance().run();
  }

  public void autonomousInit() {
    autonomousCommand = this.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  public void autonomousPeriodic() {}

  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  public void teleopPeriodic() {}

  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  public void testPeriodic() {}

  public void testExit() {}

  public void disabledInit() {}

  public void disabledPeriodic() {
    this.updateOI();
  }

  public Command armWristToPositionCommand(double armPos, double wristPos) {
    return new ParallelCommandGroup(
            arm.armToPositionCommand(armPos), wrist.wristToPositionCommand(wristPos))
        .withName("ArmWristToPositionCommand")
        .withTimeout(5.0);
  }

  public Command cruisePositionCommand() {
    return armWristToPositionCommand(
            ScoringConstants.CruiseArmPosition, ScoringConstants.CruiseWristPosition)
        .withName("cruisePositionCommand");
  }

  public Command getCoralPositionCommand() {
    return armWristToPositionCommand(
            ScoringConstants.LoadArmPosition, ScoringConstants.LoadWristPosition)
        .withName("getCoralPositionCommand");
  }

  public Command l1ScoringPositionCommand() {
    return armWristToPositionCommand(
            ScoringConstants.L1ArmPosition, ScoringConstants.L1WristPosition)
        .withName("l1ScoringPositionCommand");
  }

  public Command l2ScoringPositionCommand() {
    return armWristToPositionCommand(
            ScoringConstants.L2ArmPosition, ScoringConstants.L2WristPosition)
        .withName("l2ScoringPositionCommand");
  }

  public Command climbPositionCommand() {
    return new SequentialCommandGroup(
            arm.armToPositionCommand(ScoringConstants.ClimbArmPosition), 
            wrist.wristToPositionCommand(ScoringConstants.ClimbWristPosition))
        .withName("climbPositionCommand")
        .withTimeout(5)
        .finallyDo(
            () -> {
              arm.stop();
              wrist.stop();
            });
  }
}
