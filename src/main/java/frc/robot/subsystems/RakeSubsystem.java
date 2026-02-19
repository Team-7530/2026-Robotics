package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
// SmartDashboard access replaced by centralized Telemetry
import frc.robot.Telemetry;
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

public class RakeSubsystem extends SubsystemBase {

  public static final CANBus kCANBus = CANBUS_FD;

  public static final int RAKEARMMOTOR_ID = 30;
  public static final int RAKEARMENCODER_ID = 31;

  // public static final double kRakeArmEncoderOffset = 0.0; // add 0.25 offset, sub it later

  public static final double kRakeArmChainRatio = 60.0 / 8.0; // 8:60 ratio
  public static final double kRakeArmGearboxRatio = 45.0; // 45:1
  public static final double kRakeArmGearRatio = kRakeArmChainRatio * kRakeArmGearboxRatio;

  public static final double RAKEARM_KG = 0.0;
  public static final double RAKEARM_KS = 0.0;
  public static final double RAKEARM_KV = 0.0;
  public static final double RAKEARM_KA = 0.0;
  public static final double RAKEARM_KP = 35.0; // 70
  public static final double RAKEARM_KI = 0.0;
  public static final double RAKEARM_KD = 0.0;
  public static final AngularVelocity RAKEARM_kMaxV = RPM.of(5000);
  public static final AngularAcceleration RAKEARM_kMaxA = RotationsPerSecondPerSecond.of(2500);

  public static final Angle kRakeArmPositionMax = Degrees.of(-90.0);
  public static final Angle kRakeArmPositionMin = Degrees.of(0.0);

  public static final double kRakeArmTeleopSpeed = 0.1;
  public static final double kRakeArmTeleopFactor = 0.05;

  public static final int RAKEINTAKEMOTOR_ID = 31;
  public static final InvertedValue kRakeIntakeInverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue kRakeIntakeNeutralMode = NeutralModeValue.Coast;
  public static final double rakeIntake_peakForwardVoltage = 10.0;
  public static final double rakeIntake_peakReverseVoltage = -10.0;
  public static final double peakForwardTorqueCurrent = 40.0;
  public static final double peakReverseTorqueCurrent = -40.0;

  public static final double kRakeIntakeChainRatio = 1.0;
  public static final double kRakeIntakeGearboxRatio = 3.0;
  public static final double kRakeIntakeGearRatio = kRakeIntakeChainRatio * kRakeIntakeGearboxRatio;

  public static final double rakeIntakeMotorTorqueKS = 0.0;
  public static final double rakeIntakeMotorTorqueKP = 8.0;
  public static final double rakeIntakeMotorTorqueKI = 0.2;
  public static final double rakeIntakeMotorTorqueKD = 0.001;

  public static final double rakeIntakeVelocity = -3.0;
  public static final double rakeIntakeUnstuckVelocity = 3.0;

  
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
      .withTelemetry("RakeMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // For example gearbox(3,4) is the same as gearbox("3:1","4:1")
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(kRakeArmChainRatio, kRakeArmGearboxRatio)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      // External Encoder
      .withExternalEncoder(m_rakeArmEncoder)
      .withExternalEncoderGearing(1.0)
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
      .withHardLimit(Degrees.of(-100), Degrees.of(200))
      .withStartingPosition(Degrees.of(0))
//    .withHorizontalZero(Degrees.of(0))
      // Mass of the flywheel.
      .withMass(Pounds.of(1))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("RakeArm", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(robotToMechanism);

  private final Arm m_rakeArm = new Arm(m_rakeArmConfig);

  private boolean m_isTeleop = false;
  private final Telemetry telemetry;

  public RakeSubsystem(Telemetry telemetry) {
    this.telemetry = telemetry;
  }
    
  @Override
  public void periodic() {
    m_rakeArm.updateTelemetry();
    try {
      telemetry.putNumber("Rake Postion", this.getRakePosition().in(Degrees));
    } catch (Exception e) {
      // ignore telemetry failures
    }
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
  public Command setRakeAngle(Angle angle) {
    return m_rakeArm.setAngle(angle);
  }

  /** Returns the rake angle */
  @Logged
  public Angle getRakePosition() {
    return m_rakeArm.getAngle();
  }

  /**
   * Sets the rake motor speed
   *
   * @param wspeed double, target speed
   */
  public Command setRakeSpeed(double wspeed) {
    return m_rakeArm.set(wspeed);
  }

  /** Stops motor and activates brakes */
  public Command stopRake() {
    return m_rakeArm.set(0.0);
  }

  /**
   * Teleop controls
   *
   * @param rspeed rake target speed during teleop
   * @param cspeed collector target speed during teleop
   */
  public void teleop(double rspeed, double cspeed) {
  rspeed = MathUtil.applyDeadband(rspeed, STICK_DEADBAND);
  cspeed = MathUtil.applyDeadband(cspeed, STICK_DEADBAND);

    if ((rspeed != 0.0) || (cspeed != 0.0)) {
      m_isTeleop = true;
      this.setRakeSpeed(rspeed * kRakeArmTeleopSpeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      this.stopRake();
    }
  }

  /** Upddates the Smart Dashboard */
  // private void updateSmartDashboard() {
  //   SmartDashboard.putNumber("Rake Postion", this.getRakePosition().in(Degrees));
  // }

}
