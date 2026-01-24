package frc.robot.subsystems;

import static frc.robot.Constants.POSITION_TOLERANCE;
import static frc.robot.Constants.STICK_DEADBAND;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.sim.PhysicsSim;

public class RakeSubsystem extends SubsystemBase {

  public static final CANBus CANBUS = CANBus.roboRIO();
  public static final int RAKEMOTOR_ID = 33;
  public static final int RAKEENCODER_ID = 34;

  public static final InvertedValue kRakeInverted = InvertedValue.Clockwise_Positive;
  public static final NeutralModeValue kRakeNeutralMode = NeutralModeValue.Brake;
  public static final SensorDirectionValue kRakeEncoderDirection = SensorDirectionValue.Clockwise_Positive;
  public static final double kRakeEncoderOffset = -0.171; // add 0.25 offset, sub it later

  public static final double kRakeChainRatio = 1.0; // 1:1
  public static final double kRakeGearboxRatio = 45.0; // 1:45
  public static final double kRakeGearRatio = kRakeChainRatio * kRakeGearboxRatio;

  public static final double rakeMotorKG = 0.0;
  public static final double rakeMotorKS = 0.0;
  public static final double rakeMotorKS_slow = 0.0;
  public static final double rakeMotorKV = 0.0;
  public static final double rakeMotorKA = 0.0;
  public static final double rakeMotorKP = 35.0; // 70
  public static final double rakeMotorKP_slow = 25.0;
  public static final double rakeMotorKI = 0.0;
  public static final double rakeMotorKD = 0.0;
  public static final double MMagicCruiseVelocity = 0;
  public static final double MMagicAcceleration = 0;
  public static final double MMagicJerk = 0;
  public static final double MMagicExpo_kV = 4.8; // kV is around 0.12 V/rps
  public static final double MMagicExpo_kA = 4.8; // Use a slower kA of 0.1 V/(rps/s)
  public static final double peakForwardVoltage = 8.0; // Peak output of 8 volts
  public static final double peakReverseVoltage = -8.0; // Peak output of 8 volts

  public static final double kRakePositionMax = 0.25;
  public static final double kRakePositionMin = -0.334;

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

  
  private final TalonFX m_rakeMotor = new TalonFX(RAKEMOTOR_ID, CANBUS);
  private final CANcoder m_rakeEncoder = new CANcoder(RAKEENCODER_ID, CANBUS);
  private final MotionMagicExpoVoltage m_rakeRequest = new MotionMagicExpoVoltage(0).withSlot(0);
  private final DutyCycleOut m_rakePercentRequest = new DutyCycleOut(0);
  private final NeutralOut m_brake = new NeutralOut();

  private final TalonFX m_CollectorMotor = new TalonFX(COLLECTORMOTOR_ID, CANBUS);

  private final VelocityTorqueCurrentFOC m_collectorRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut m_collectorPercentRequest = new DutyCycleOut(0);

  private double rakeTargetPosition = 0;
  private boolean m_isTeleop = false;

  public RakeSubsystem() {
    initRakeConfigs();
    initEncoderConfigs();
    initCollectorConfigs();

    if (RobotBase.isSimulation()) initSimulation();
  }

  private void initRakeConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
  configs.MotorOutput.Inverted = kRakeInverted;
  configs.MotorOutput.NeutralMode = kRakeNeutralMode;
  configs.Voltage.PeakForwardVoltage = peakForwardVoltage;
  configs.Voltage.PeakReverseVoltage = peakReverseVoltage;

  configs.Slot0.kG = rakeMotorKG;
  configs.Slot0.kS = rakeMotorKS;
  configs.Slot0.kV = rakeMotorKV;
  configs.Slot0.kA = rakeMotorKA;
  configs.Slot0.kP = rakeMotorKP;
  configs.Slot0.kI = rakeMotorKI;
  configs.Slot0.kD = rakeMotorKD;
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

  configs.Slot1.kG = rakeMotorKG;
  configs.Slot1.kS = rakeMotorKS_slow;
  configs.Slot1.kV = rakeMotorKV;
  configs.Slot1.kA = rakeMotorKA;
  configs.Slot1.kP = rakeMotorKP_slow;
  configs.Slot1.kI = rakeMotorKI;
  configs.Slot1.kD = rakeMotorKD;
    configs.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.Feedback.FeedbackRemoteSensorID = m_rakeEncoder.getDeviceID();
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;//
    configs.Feedback.SensorToMechanismRatio = 1.0;
  configs.Feedback.RotorToSensorRatio = kRakeGearRatio;

  configs.MotionMagic.MotionMagicCruiseVelocity = MMagicCruiseVelocity;
  configs.MotionMagic.MotionMagicAcceleration = MMagicAcceleration;
  configs.MotionMagic.MotionMagicJerk = MMagicJerk;
  configs.MotionMagic.MotionMagicExpo_kV = MMagicExpo_kV;
  configs.MotionMagic.MotionMagicExpo_kA = MMagicExpo_kA;

  configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kRakePositionMax;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kRakePositionMin;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    StatusCode s = m_rakeMotor.getConfigurator().apply(configs);
    if (!s.isOK()) System.out.println("Could not apply top configs, error code: " + s);
  }

  private void initEncoderConfigs() {
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(0.5));
  configs.MagnetSensor.SensorDirection = kRakeEncoderDirection;
    // add offset of 0.25 to abs value so total range keeps sign. sub it below
  configs.MagnetSensor.withMagnetOffset(Units.Rotations.of(kRakeEncoderOffset));

    StatusCode s = m_rakeEncoder.getConfigurator().apply(configs);
    if (!s.isOK()) System.out.println("Could not apply top configs, error code: " + s);

    // set starting position to current absolute position
    s = m_rakeEncoder.setPosition(m_rakeEncoder.getAbsolutePosition().getValueAsDouble());
    if (!s.isOK()) System.out.println("Could not apply position to encoder, error code: " + s);
  }

  private void initCollectorConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

  configs.MotorOutput.Inverted = kCollectorInverted;
  configs.MotorOutput.NeutralMode = kCollectorNeutralMode;
  configs.Voltage.PeakForwardVoltage = collector_peakForwardVoltage;
  configs.Voltage.PeakReverseVoltage = collector_peakReverseVoltage;
  configs.TorqueCurrent.PeakForwardTorqueCurrent = peakForwardTorqueCurrent;
  configs.TorqueCurrent.PeakReverseTorqueCurrent = peakReverseTorqueCurrent;

  configs.Slot0.kS = collectorMotorTorqueKS;
  configs.Slot0.kP = collectorMotorTorqueKP;
  configs.Slot0.kI = collectorMotorTorqueKI;
  configs.Slot0.kD = collectorMotorTorqueKD;

    StatusCode s = m_CollectorMotor.getConfigurator().apply(configs);
    if (!s.isOK()) System.out.println("Could not apply top configs, error code: " + s);
  }
    
  private void initSimulation() {
  PhysicsSim.getInstance()
    .addTalonFX(m_rakeMotor, m_rakeEncoder, kRakeGearRatio, 0.001);
    m_rakeEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  /**
   * Sets the rake position
   *
   * @param pos position between 0 and 1
   */
  public void setRakePosition(double pos) {
    m_isTeleop = false;
  rakeTargetPosition =
    MathUtil.clamp(pos, kRakePositionMin, kRakePositionMax);
    m_rakeMotor.setControl(m_rakeRequest.withPosition(rakeTargetPosition));
  }

  /** Returns the rake position as a double */
  public double getRakePosition() {
    return m_rakeEncoder.getPosition().getValueAsDouble();
  }

  /** Returns true if motor is at target position or within tolerance range */
  public boolean isRakeAtPosition() {
  return MathUtil.isNear(rakeTargetPosition, this.getRakePosition(), POSITION_TOLERANCE);
  }

  /**
   * Sets the rake motor speed
   *
   * @param wspeed double, target speed
   */
  public void setRakeSpeed(double wspeed) {
    rakeTargetPosition = 0;
    m_rakeMotor.setControl(m_rakePercentRequest.withOutput(wspeed));
  }

  /** Stops motor and activates brakes */
  public void stopRake() {
    rakeTargetPosition = 0;
    m_rakeMotor.setControl(m_brake);
  }

  /**
   * Sets the motor Target velocity
   *
   * @param Cvelocity left motors target velocity
   */
  public void setCollectorVelocity(double Cvelocity) {
    m_CollectorMotor.setControl(
      m_collectorRequest.withVelocity(Cvelocity * kCollectorGearRatio));
  }

  /**
   * returns the motor Target velocity
   */
  public double getCollectorVelocity() {
    return m_CollectorMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Sets collector motor speed
   *
   * @param speed double
   */
  public void setCollectorSpeed(double speed) {
      m_CollectorMotor.setControl(m_collectorPercentRequest.withOutput(speed));
  }

  /** Activates motor brakes */
  public void collectorStop() {
      m_CollectorMotor.setControl(m_brake);
  }

  /** Sets motors to constants intake speed */
  public void collectorIn() {
  this.setCollectorVelocity(collectorVelocity);
  }

  public void collectorUnstuck() {
  this.setCollectorVelocity(collectorUnstuckVelocity);
  }

  /**
   * Teleop controls
   *
   * @param wspeed rake target speed during teleop
   */
  public void teleop(double wspeed) {
  wspeed = MathUtil.applyDeadband(wspeed, STICK_DEADBAND);

    if (wspeed != 0.0) {
      m_isTeleop = true;
  this.setRakeSpeed(wspeed * kRakeTeleopSpeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      this.stopRake();
    }
  }

  /** Upddates the Smart Dashboard */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Rake Postion", this.getRakePosition());
    SmartDashboard.putNumber("Rake TargetPostion", rakeTargetPosition);
    SmartDashboard.putNumber("CollectorIntake Speed", this.getCollectorVelocity());
  }

  public Command rakeToPositionCommand(double position) {
    return run(() -> this.setRakePosition(position))
        .withName("RakeToPositionCommand")
        .until(this::isRakeAtPosition)
        .withTimeout(5.0);
  }

  public Command collectorCommand() {
    return run(() -> this.collectorIn())
        .withName("CollectorCommand")
        .withTimeout(5.0)
        .finallyDo(() -> this.collectorStop());
  }

  public Command collectorUnstuckCommand() {
  return run(() -> this.collectorIn())
      .withName("CollectorUnstuckCommand")
      .withTimeout(5.0)
      .finallyDo(() -> this.collectorStop());
  }

}
