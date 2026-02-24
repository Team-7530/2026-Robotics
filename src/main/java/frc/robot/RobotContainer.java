package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
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
  // private final SwerveRequest.RobotCentric forwardStraight =
  //     new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(DriveTrainConstants.maxSpeed.vxMetersPerSecond);

  /* Operator Interface */
  public OperatorInterface oi = new OperatorInterface() {};

  /* Subsystems */
  public final PowerDistribution power = new PowerDistribution();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final VisionSubsystem vision = new VisionSubsystem(logger);
  public final ShooterSubsystem shooter = new ShooterSubsystem(logger);
  public final RakeArmSubsystem rakeArm = new RakeArmSubsystem(logger);
  public final RakeIntakeSubsystem rakeIntake = new RakeIntakeSubsystem(logger);
  public final CollectorSubsystem collector = new CollectorSubsystem(logger);

  /* Path follower */
  private SendableChooser<Command> autoChooser;
  private Command autonomousCommand;

  public static RobotContainer GetInstance() {
    return instance;
  }

  /** Return the shared Telemetry instance used by the robot. */
  public Telemetry getTelemetry() {
    return logger;
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
      // logger.putNumber("DriveTrain/Drive Scaling", oi.driveScalingValue());
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureButtonBindings() {
    oi.getResetGyroButton().onTrue(drivetrain.resetGyroCommand());

    oi.getXStanceButton().whileTrue(drivetrain.setXStanceCommand());

    oi.driveScalingUp()
        .onTrue(drivetrain.setMaxSpeedsCommand(DriveTrainConstants.maxSpeed));
    oi.driveScalingDown()
        .onTrue(drivetrain.setMaxSpeedsCommand(DriveTrainConstants.cruiseSpeed));
    oi.driveScalingSlow()
        .onTrue(drivetrain.setMaxSpeedsCommand(DriveTrainConstants.slowSpeed))
        .onFalse(drivetrain.setMaxSpeedsCommand(DriveTrainConstants.cruiseSpeed));

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

    //Operator Controls
    oi.getAButton().onTrue(shooter.flywheelStopCommand());
    oi.getBButton().onTrue(collector.collectorStopCommand());
    oi.getXButton().onTrue(collector.collectorUnstuckCommand());
    oi.getYButton().onTrue(shooter.feederUnstuckCommand());

    oi.getLeftBumper().onTrue(collector.collectorStartCommand());
    oi.getRightBumper().onTrue(rakeIntake.rakeIntakeStartCommand()).onFalse(rakeIntake.rakeIntakeStopCommand());

    oi.getLeftTrigger().onTrue(shooter.flywheelToVelocityCommand(6000)).onFalse(shooter.flywheelStopCommand());
    oi.getRightTrigger().onTrue(shooter.feederStartCommand()).onFalse(shooter.feederStopCommand());
    
    oi.getPOVUp().onTrue(rakeArm.rakeArmRetractCommand());
    oi.getPOVDown().onTrue(rakeArm.rakeArmDeployCommand());
    oi.getPOVLeft().onTrue(shooter.turretToAngleCommand(20));
    oi.getPOVRight().onTrue(shooter.turretToAngleCommand(0));

    oi.getStartButton().onTrue(Commands.runOnce(() -> shooter.turret.seedTurretPositionCommand()));

    // back button toggles continuous hub-aiming for testing; cancels immediately
    // when released by virtue of being a run-while-true command.
    oi.getBackButton().whileTrue(new AimAtHubCommand(shooter, vision, drivetrain));

    //Testing Controls
    oi.getTestOI().getAButton().onTrue(shooter.flywheelStopCommand());
    oi.getTestOI().getBButton().onTrue(collector.collectorStopCommand());
    oi.getTestOI().getXButton().onTrue(collector.collectorUnstuckCommand());
    oi.getTestOI().getYButton().onTrue(shooter.feederUnstuckCommand());
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

    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new SwerveTeleopCommand(drivetrain, oi));

    shooter.setDefaultCommand(Commands.run(() -> shooter.teleop(oi.getRightThumbstickX(), -oi.getRightThumbstickY()), shooter));
    rakeArm.setDefaultCommand(Commands.run(() -> rakeArm.teleop(oi.getLeftThumbstickY()), rakeArm));
    rakeIntake.setDefaultCommand(Commands.run(() -> rakeIntake.teleop(-oi.getLeftThumbstickX()), rakeIntake));

    // climber.setDefaultCommand(
    //     Commands.run(() -> climber.teleopClimb(-oi.getRightThumbstickY()), climber));
    vision.setDefaultCommand(vision.updateGlobalPoseCommand(drivetrain));
  }

  private void configureAutoPaths() {
    NamedCommands.registerCommand("aimRange", shooter.turretToAngleCommand(0));
    NamedCommands.registerCommand("shoot", shooter.feederStartCommand());
    NamedCommands.registerCommand("climb", Commands.runOnce(() -> System.out.println("Climb command executed")));
    NamedCommands.registerCommand("UpdatePose", vision.updateGlobalPoseCommand(drivetrain));
    NamedCommands.registerCommand("collectorCommand", collector.collectorStartCommand());
    NamedCommands.registerCommand("rakeDeploy", rakeArm.rakeArmDeployCommand());
    NamedCommands.registerCommand("rakeRetract", rakeArm.rakeArmRetractCommand());
  // simple command class that repeatedly calls vision.updateGlobalPose; useful
  // if you want to schedule the behaviour without making it the default.
    NamedCommands.registerCommand("UpdateStoppedPose",
      new UpdateGlobalPoseWhenStoppedCommand(vision, drivetrain));

  // aim at the alliance hub using the latest vision/odometry pose
    NamedCommands.registerCommand("AimAtHub",
      new AimAtHubCommand(shooter, vision, drivetrain));
  }

  private void configureTelemetry() {
    drivetrain.registerTelemetry(logger::telemeterize);

    logger.putData("AutoChooser", autoChooser);
    logger.putData("UpdatePose", vision.updateGlobalPoseCommand(drivetrain));
    logger.putData("UpdateStoppedPose", new UpdateGlobalPoseWhenStoppedCommand(vision, drivetrain));
    logger.putData("AimAtHub", new AimAtHubCommand(shooter, vision, drivetrain));
  }

  public void robotPeriodic() {}

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
  }

  public void autonomousInit() {
    autonomousCommand = this.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
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
}
