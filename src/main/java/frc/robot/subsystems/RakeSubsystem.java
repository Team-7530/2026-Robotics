package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public static final int RAKEMOTOR_ID = 30;
  public static final int RAKEENCODER_ID = 31;

  public static final double kRakeEncoderOffset = -0.171; // add 0.25 offset, sub it later

  public static final double kRakeChainRatio = 1.0; // 1:1
  public static final double kRakeGearboxRatio = 45.0; // 1:45
  public static final double kRakeGearRatio = kRakeChainRatio * kRakeGearboxRatio;

  public static final double RAKE_KG = 0.0;
  public static final double RAKE_KS = 0.0;
  public static final double RAKE_KV = 0.0;
  public static final double RAKE_KA = 0.0;
  public static final double RAKE_KP = 35.0; // 70
  public static final double RAKE_KI = 0.0;
  public static final double RAKE_KD = 0.0;
  public static final AngularVelocity RAKE_kMaxV = RPM.of(5000);
  public static final AngularAcceleration RAKE_kMaxA = RotationsPerSecondPerSecond.of(2500);

  public static final Angle kRakePositionMax = Degrees.of(45.0);
  public static final Angle kRakePositionMin = Degrees.of(0.0);

  public static final double kRakeTeleopSpeed = 0.1;
  public static final double kRakeTeleopFactor = 0.05;

  public static final int COLLECTORMOTOR_ID = 65;
  public static final InvertedValue kCollectorInverted = InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue kCollectorNeutralMode = NeutralModeValue.Coast;
  public static final double collector_peakForwardVoltage = 10.0;
  public static final double collector_peakReverseVoltage = -10.0;
  public static final double peakForwardTorqueCurrent = 40.0;
  public static final double peakReverseTorqueCurrent = -40.0;

  public static final double kCollectorChainRatio = 24.0 / 10.0;
  public static final double kCollectorGearboxRatio = 1.0;
  public static final double kCollectorGearRatio = kCollectorChainRatio * kCollectorGearboxRatio;

  public static final double collectorMotorTorqueKS = 0.0;
  public static final double collectorMotorTorqueKP = 8.0;
  public static final double collectorMotorTorqueKI = 0.2;
  public static final double collectorMotorTorqueKD = 0.001;

  public static final double collectorVelocity = -3.0;
  public static final double collectorUnstuckVelocity = 3.0;

  
  private final TalonFX m_rakeMotor = new TalonFX(RAKEMOTOR_ID, kCANBus);
  private final CANcoder m_rakeEncoder = new CANcoder(RAKEENCODER_ID, kCANBus);

  // YAMS controller and mechanism (initialized at declaration to match FlywheelSubsystem style)
  private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // PID Constants
      .withClosedLoopController(RAKE_KP, RAKE_KI, RAKE_KD, RAKE_kMaxV, RAKE_kMaxA)
      .withSimClosedLoopController(RAKE_KP, RAKE_KI, RAKE_KD, RAKE_kMaxV, RAKE_kMaxA)
      // Feedforward Constants
      .withFeedforward(new ArmFeedforward(RAKE_KS, 0, 0))
      .withSimFeedforward(new ArmFeedforward(RAKE_KS, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("RakeMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // For example gearbox(3,4) is the same as gearbox("3:1","4:1")
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(kRakeChainRatio, kRakeGearboxRatio)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      // External Encoder
      .withExternalEncoder(m_rakeEncoder)
      .withExternalEncoderGearing(1.0)
      .withUseExternalFeedbackEncoder(true);

  private final SmartMotorController m_rakeSMC = new TalonFXWrapper(m_rakeMotor, DCMotor.getKrakenX60Foc(1), smc_config);

  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Inches.of(23.0))
      .withMaxRobotLength(Inches.of(34.0))
      .withRelativePosition(new Translation3d(Inches.of(-10), Inches.of(-2), Inches.of(1)));

  private final ArmConfig m_rakeConfig = new ArmConfig(m_rakeSMC)
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

  private final Arm m_rake = new Arm(m_rakeConfig);

  private boolean m_isTeleop = false;

  public RakeSubsystem() {

  }
    
  @Override
  public void periodic() {
    updateSmartDashboard();
    m_rake.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_rake.simIterate();
  }

  /**
   * Sets the rake position
   *
   * @param angle angle in degrees
   */
  public Command setRakeAngle(Angle angle) {
    return m_rake.setAngle(angle);
  }

  /** Returns the rake angle */
  public Angle getRakePosition() {
    return m_rake.getAngle();
  }

  /**
   * Sets the rake motor speed
   *
   * @param wspeed double, target speed
   */
  public Command setRakeSpeed(double wspeed) {
    return m_rake.set(wspeed);
  }

  /** Stops motor and activates brakes */
  public Command stopRake() {
    return m_rake.set(0.0);
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
      this.setRakeSpeed(rspeed * kRakeTeleopSpeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      this.stopRake();
    }
  }

  /** Upddates the Smart Dashboard */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Rake Postion", this.getRakePosition().in(Degrees));
  }

}
