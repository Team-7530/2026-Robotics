package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

import frc.robot.Telemetry;
import frc.lib.util.SystemHealthMonitor;
import frc.lib.util.SystemHealthMonitor.MotorHealthMonitor;

@Logged
public class RakeArmSubsystem extends SubsystemBase {
  private static final CANBus kCANBus = CANBUS_FD;

  private static final int RAKEARMMOTOR_ID = 30;
  private static final int RAKEARMENCODER_ID = 31;

  private static final double kRakeArmChainRatio = 60.0 / 8.0; // 8:60 ratio
  private static final double kRakeArmGearboxRatio = 45.0; // 45:1
  private static final double kRakeArmEncoderOffset = 0.11; // offset needed so encoder 0 is horizontal
  
  private static final double RAKEARM_KS = 0.0;
  private static final double RAKEARM_KP = 110.0; // 70
  private static final double RAKEARM_KI = 0.0;
  private static final double RAKEARM_KD = 0.0;
  private static final AngularVelocity RAKEARM_kMaxV = RPM.of(5000);
  private static final AngularAcceleration RAKEARM_kMaxA = RotationsPerSecondPerSecond.of(2500);
  private static final Angle kRakeArmPositionDeploy = Degrees.of(0);
  private static final Angle kRakeArmPositionRetract = Degrees.of(131.0);
  private static final Angle kRakeArmPositionUp = Degrees.of(90.0);
  
  private static final double kRakeArmTeleopSpeed = 0.2;
  
  // Motor health monitoring threshold
  private static final double RAKEARM_STALL_THRESHOLD = 80.0;  // Kraken X60 warning at 80A
    
  private final TalonFX m_rakeArmMotor = new TalonFX(RAKEARMMOTOR_ID, kCANBus);
  private final CANcoder m_rakeArmEncoder = new CANcoder(RAKEARMENCODER_ID, kCANBus);

  // YAMS controller and mechanism (initialized at declaration to match FlywheelSubsystem style)
  private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // PID Constants
      .withClosedLoopController(RAKEARM_KP, RAKEARM_KI, RAKEARM_KD, RAKEARM_kMaxV, RAKEARM_kMaxA)
      .withSimClosedLoopController(RAKEARM_KP, RAKEARM_KI, RAKEARM_KD, RAKEARM_kMaxV, RAKEARM_kMaxA)
      // Feedforward Constants
      .withFeedforward(new ArmFeedforward(RAKEARM_KS, 0, 0))
      .withSimFeedforward(new ArmFeedforward(RAKEARM_KS, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("RakeArmMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // For example gearbox(3,4) is the same as gearbox("3:1","4:1")
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(kRakeArmChainRatio, kRakeArmGearboxRatio)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      // External Encoder
      .withExternalEncoder(m_rakeArmEncoder)
      .withExternalEncoderGearing(1.0)
      .withExternalEncoderInverted(false)
      .withExternalEncoderZeroOffset(Rotations.of(kRakeArmEncoderOffset))
      .withUseExternalFeedbackEncoder(true);

  private final SmartMotorController m_rakeArmSMC = new TalonFXWrapper(m_rakeArmMotor, DCMotor.getKrakenX60Foc(1), smc_config);

  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Inches.of(23.0))
      .withMaxRobotLength(Inches.of(34.0))
      .withRelativePosition(new Translation3d(Inches.of(-10), Inches.of(-2), Inches.of(1)));

  private final ArmConfig m_rakeArmConfig = new ArmConfig(m_rakeArmSMC)
      // Length of the arm.
      .withLength(Meters.of(0.135))
      // Angle limits
      .withHardLimit(Degrees.of(0), Degrees.of(132))
      .withStartingPosition(Degrees.of(0))
      // Mass of the flywheel.
      .withMass(Pounds.of(1))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("RakeArm", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(robotToMechanism);

  private final Arm m_rakeArm = new Arm(m_rakeArmConfig);
  
  @Logged(importance = Logged.Importance.DEBUG)
  private boolean m_isTeleop = false;

  // Health monitoring (owned by this subsystem, not central monitoring)
  private final MotorHealthMonitor motorHealth;

  private final Telemetry telemetry;

  /**
   * Creates a new RakeArmSubsystem.
   * 
   * @param telemetry the telemetry instance for health monitoring
   * @param healthMonitor the system health monitor to register with (pass null to skip registration)
   */
  public RakeArmSubsystem(Telemetry telemetry, SystemHealthMonitor healthMonitor) {
    this.telemetry = telemetry;
    this.motorHealth = new MotorHealthMonitor(
        m_rakeArmMotor,
        "RakeArm",
        telemetry,
        RAKEARM_STALL_THRESHOLD
    );
    if (healthMonitor != null) {
      healthMonitor.registerMonitor(motorHealth);
    }
  }
    
  @Override
  public void periodic() {
    motorHealth.update();
    this.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_rakeArm.simIterate();
  }

  /**
   * Sets the rake position
   *
   * @param angle angle in degrees
   */
  public Command setRakeArmAngle(Angle angle) {
    return m_rakeArm.setAngle(angle);
  }

  /** Returns the rake angle */
  @Logged(importance = Logged.Importance.CRITICAL)
  public Angle getRakeArmPosition() {
    return m_rakeArm.getAngle();
  }

  /**
   * Run a simple SysId routine on the rake arm motor.  This is useful when
   * characterizing the mechanism during practice-field tuning. The command
   * will apply a 10 V step for 5 s while logging the response; modify the
   * parameters if you need a different profile.
   */
  public Command sysIdCommand() {
    return m_rakeArm.sysId(Volts.of(10), Volts.of(1).per(Seconds), Seconds.of(5));
  }

  /** Move the rake to the deployed floor-intake angle. */
  public Command rakeArmDeployCommand() {
    return m_rakeArm.setAngle(kRakeArmPositionDeploy)
      .withName("RakeArmDeployCommand")
      .withTimeout(0.2);
  }

  public Command rakeArmRetractCommand() {
    return m_rakeArm.setAngle(kRakeArmPositionRetract)
      .withName("RakeArmRetractCommand")
      .withTimeout(0.2);
  }

  public Command rakeArmUpCommand() {
    return m_rakeArm.setAngle(kRakeArmPositionUp)
      .withName("RakeArmUpCommand")
      .withTimeout(0.2);
  }

  /** Stop the rake arm motor immediately. */
  public Command rakeArmStopCommand() {
    return runOnce(this::rakeArmStop)
      .withName("RakeArmStopCommand");
  }

  /** Stops the rake arm motor immediately (open-loop stop). */
  public void rakeArmStop() {
    m_rakeArmSMC.stopClosedLoopController();
    m_rakeArmSMC.setDutyCycle(0.0);
  }

  /**
   * Teleop controls
   *
   * @param rspeed rake target speed during teleop
   */
  public void teleop(double rspeed) {
    rspeed = MathUtil.applyDeadband(rspeed, STICK_DEADBAND);

    if (rspeed != 0.0) {
      m_isTeleop = true;
      m_rakeArmSMC.setDutyCycle(rspeed * kRakeArmTeleopSpeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      this.rakeArmStop();
    }
  }

  private void updateTelemetry() {
    m_rakeArm.updateTelemetry();
  }

  /** Get the rake arm motor for health monitoring. */
  public TalonFX getRakeArmMotor() {
    return m_rakeArmMotor;
  }

  /** Get the health monitor for this subsystem. */
  public MotorHealthMonitor getHealthMonitor() {
    return motorHealth;
  }
}
