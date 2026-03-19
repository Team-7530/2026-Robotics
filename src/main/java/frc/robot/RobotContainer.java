package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.util.SystemHealthMonitor;

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
@Logged
public class RobotContainer {
  // Shared dashboard/logger helper. Keep this local to the container instead of
  // having subsystems reach back through a global singleton.
  private final Telemetry logger = new Telemetry(
      DriveTrainConstants.maxSpeed.vxMetersPerSecond,
      Constants.DEBUG_LOGGING || edu.wpi.first.wpilibj.RobotBase.isSimulation());

  /* Operator Interface */
  public OperatorInterface oi = new OperatorInterface() {};

  /* Subsystems */
  public final CommandSwerveDrivetrain drivetrain;
  public final VisionSubsystem vision;
  public final ShooterSubsystem shooter;
  public final RakeArmSubsystem rakeArm;
  public final RakeIntakeSubsystem rakeIntake;
  public final SystemHealthMonitor healthMonitor;

  /* Path follower */
  private SendableChooser<Command> autoChooser;
  @NotLogged
  private Command autonomousCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.healthMonitor = new SystemHealthMonitor(logger);

    this.drivetrain = TunerConstants.createDrivetrain();
    this.vision = new VisionSubsystem(logger);
    this.shooter = new ShooterSubsystem(drivetrain, healthMonitor);
    this.rakeArm = new RakeArmSubsystem(healthMonitor);
    this.rakeIntake = new RakeIntakeSubsystem(healthMonitor);

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    drivetrain.setMaxSpeeds(DriveTrainConstants.cruiseSpeed);
    logger.setMaxSpeed(DriveTrainConstants.cruiseSpeed.vxMetersPerSecond);
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
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureButtonBindings() {

    configureDriverControls();
    configureOperatorControls();
    configureTestingControls();
    configureDriveTestingControls();
  }

  private void configureDriverControls() {
    oi.getResetGyroButton().onTrue(drivetrain.resetGyroCommand());

    oi.getXStanceButton().whileTrue(drivetrain.setXStanceCommand());

    oi.driveScalingUp()
        .onTrue(setDriveMaxSpeedsCommand(DriveTrainConstants.maxSpeed));
    oi.driveScalingDown()
        .onTrue(setDriveMaxSpeedsCommand(DriveTrainConstants.cruiseSpeed));
    oi.driveScalingSlow()
        .onTrue(setDriveMaxSpeedsCommand(DriveTrainConstants.slowSpeed))
        .onFalse(setDriveMaxSpeedsCommand(DriveTrainConstants.cruiseSpeed));

    drivetrain.setDefaultCommand(new SwerveTeleopCommand(drivetrain, oi));
  }

  private void configureOperatorControls() {
    // A/B select a tracked shot mode; X/Y restore fixed-RPM manual profiles.
    oi.getAButton()
      .onTrue(vision.setHubPipelineCommand())
      .whileTrue(shooter.targetHubCommand().alongWith(vision.updateGlobalPoseCommand(drivetrain)));
    oi.getBButton()
      .onTrue(vision.setBumpPipelineCommand())
      .whileTrue(shooter.targetBumpCommand().alongWith(vision.updateGlobalPoseCommand(drivetrain)));
    oi.getXButton().onTrue(shooter.setManualLowShotProfileCommand());
    oi.getYButton().onTrue(shooter.setManualHighShotProfileCommand());

    oi.getLeftBumper()
      .onTrue(shooter.shooterSpinupCommand());
    oi.getRightBumper()
      .onTrue(shooter.shooterStopCommand());

    oi.getLeftTrigger()
      // Trigger fires whichever profile is currently active.
      .onTrue(shooter.shooterSpinupIfNeededCommand().andThen(shooter.shooterStartCommand()))
      .onFalse(shooter.shooterUnstuckCommand());

    oi.getRightTrigger()
      .onTrue(rakeArm.rakeArmDeployCommand().alongWith(rakeIntake.rakeIntakeStartCommand()))
      .onFalse(rakeIntake.rakeIntakeStopCommand());
    
    oi.getPOVUp().onTrue(rakeArm.rakeArmRetractCommand());
    oi.getPOVDown().onTrue(rakeArm.rakeArmDeployCommand());
    oi.getPOVLeft().onTrue(shooter.turret.setAngleCommand(Degrees.of(0)));
    oi.getPOVRight().onTrue(rakeArm.rakeArmUpCommand());

    oi.getStartButton().onTrue(shooter.turret.seedTurretPositionCommand());
    oi.getBackButton().whileTrue(vision.updateGlobalPoseCommand(drivetrain));

    shooter.turret.setDefaultCommand(Commands.run(() -> shooter.turret.teleop(oi.getLeftThumbstickX()), shooter.turret));    
    rakeArm.setDefaultCommand(Commands.run(() -> rakeArm.teleop(-oi.getRightThumbstickY()), rakeArm));
  }

  private void configureTestingControls() {
    OperatorInterface testOI = oi.getTestOI();

    // Testing controls: only activate these when neither modifier button is held
    // (Start/Back are used as sysid modifiers below).
    testOI.getAButton()
      .and(testOI.getStartButton().negate())
      .and(testOI.getBackButton().negate())
      .onTrue(shooter.flywheel.flywheelStopCommand());
    testOI.getBButton()
      .and(testOI.getStartButton().negate())
      .and(testOI.getBackButton().negate())
      .onTrue(shooter.collector.collectorStopCommand());
    testOI.getXButton()
      .and(testOI.getStartButton().negate())
      .and(testOI.getBackButton().negate())
      .onTrue(shooter.collector.collectorUnstuckCommand());
    testOI.getYButton()
      .and(testOI.getStartButton().negate())
      .and(testOI.getBackButton().negate())
      .onTrue(shooter.feeder.feederUnstuckCommand());

    // sysID helpers bound to the test controller.  hold Start or Back together
    // with a face button to exercise each mechanism's built-in sysid routine.
    // these commands are short-lived (they run until the button is released)
    // and log data to SignalLogger for later analysis.
    testOI.getStartButton()
      .and(testOI.getAButton())
      .whileTrue(shooter.turret.sysIdCommand());
    testOI.getStartButton()
      .and(testOI.getBButton())
      .whileTrue(rakeArm.sysIdCommand());
    testOI.getStartButton()
      .and(testOI.getXButton())
      .whileTrue(shooter.flywheel.sysIdCommand());
    testOI.getStartButton()
      .and(testOI.getYButton())
      .whileTrue(shooter.feeder.sysIdCommand());

    testOI.getBackButton()
      .and(testOI.getAButton())
      .whileTrue(shooter.collector.sysIdCommand());
    testOI.getBackButton()
      .and(testOI.getBButton())
      .whileTrue(rakeIntake.sysIdCommand());

    rakeIntake.setDefaultCommand(Commands.run(() -> rakeIntake.teleop(-testOI.getLeftThumbstickX()), rakeIntake));
    shooter.flywheel.setDefaultCommand(Commands.run(() -> shooter.flywheel.teleop(-testOI.getRightThumbstickY()), shooter.flywheel));
  }

  private void configureDriveTestingControls() {
    // Example PathOnTheFly bindings are intentionally disabled. If needed for tuning,
    // re-enable them temporarily in this method.
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
    logger.putData("AutoChooser", autoChooser);

    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
  }

  private void configureDefaultCommands() {

    // vision.setDefaultCommand(vision.updateGlobalPoseCommand(drivetrain));
  }

  private void configureAutoPaths() {
    NamedCommands.registerCommand("aimRange", shooter.targetHubOnceCommand());
    NamedCommands.registerCommand("spinup", shooter.shooterSpinupCommand());
    NamedCommands.registerCommand("shoot", shooter.shooterStartCommand());
    NamedCommands.registerCommand("stopShoot", shooter.shooterStopCommand());
    NamedCommands.registerCommand("rakeDeploy", rakeArm.rakeArmDeployCommand());
    NamedCommands.registerCommand("rakeUp", rakeArm.rakeArmUpCommand());
    NamedCommands.registerCommand("rakeRetract", rakeArm.rakeArmRetractCommand());
    NamedCommands.registerCommand("rakeIntake", rakeIntake.rakeIntakeStartCommand());
    NamedCommands.registerCommand("rakeIntakeStop", rakeIntake.rakeIntakeStopCommand());
    NamedCommands.registerCommand("updatePose", vision.updateGlobalPoseOnceCommand(drivetrain));
    NamedCommands.registerCommand("resetPose", vision.resetGlobalPoseCommand(drivetrain));
  }

  private void configureTelemetry() {
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private Command setDriveMaxSpeedsCommand(ChassisSpeeds speeds) {
    return Commands.runOnce(() -> {
      drivetrain.setMaxSpeeds(speeds);
      logger.setMaxSpeed(speeds.vxMetersPerSecond);
    }).withName("SetDriveMaxSpeeds");
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

  public void autonomousPeriodic() {
  }

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
    drivetrain.setOperatorPerspectiveForward(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero );
  }

  public void testPeriodic() {
    vision.updateGlobalPose(drivetrain);
  }

  public void testExit() {}

  public void disabledInit() {}

  public void disabledPeriodic() {
    this.updateOI();
  }
}
