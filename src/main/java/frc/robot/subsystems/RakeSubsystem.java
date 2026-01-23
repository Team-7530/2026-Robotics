package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.RakeConstants;
import frc.robot.sim.PhysicsSim;

public class RakeSubsystem extends SubsystemBase {

  private final TalonFX m_rakeMotor =
      new TalonFX(RakeConstants.RAKEMOTOR_ID, RakeConstants.CANBUS);
  private final CANcoder m_rakeEncoder =
      new CANcoder(RakeConstants.RAKEENCODER_ID, RakeConstants.CANBUS);
  private final MotionMagicExpoVoltage m_rakeRequest = new MotionMagicExpoVoltage(0).withSlot(0);
  private final DutyCycleOut m_rakePercentRequest = new DutyCycleOut(0);
  private final NeutralOut m_brake = new NeutralOut();

  private final TalonFX m_CollectorMotor =
      new TalonFX(CollectorConstants.COLLECTORMOTOR_ID, CollectorConstants.CANBUS);

  private final VelocityTorqueCurrentFOC m_collectorRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
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
    configs.MotorOutput.Inverted = RakeConstants.kRakeInverted;
    configs.MotorOutput.NeutralMode = RakeConstants.kRakeNeutralMode;
    configs.Voltage.PeakForwardVoltage = RakeConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = RakeConstants.peakReverseVoltage;

    configs.Slot0.kG = RakeConstants.rakeMotorKG;
    configs.Slot0.kS = RakeConstants.rakeMotorKS;
    configs.Slot0.kV = RakeConstants.rakeMotorKV;
    configs.Slot0.kA = RakeConstants.rakeMotorKA;
    configs.Slot0.kP = RakeConstants.rakeMotorKP;
    configs.Slot0.kI = RakeConstants.rakeMotorKI;
    configs.Slot0.kD = RakeConstants.rakeMotorKD;
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.Slot1.kG = RakeConstants.rakeMotorKG;
    configs.Slot1.kS = RakeConstants.rakeMotorKS_slow;
    configs.Slot1.kV = RakeConstants.rakeMotorKV;
    configs.Slot1.kA = RakeConstants.rakeMotorKA;
    configs.Slot1.kP = RakeConstants.rakeMotorKP_slow;
    configs.Slot1.kI = RakeConstants.rakeMotorKI;
    configs.Slot1.kD = RakeConstants.rakeMotorKD;
    configs.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.Feedback.FeedbackRemoteSensorID = m_rakeEncoder.getDeviceID();
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;//
    configs.Feedback.SensorToMechanismRatio = 1.0;
    configs.Feedback.RotorToSensorRatio = RakeConstants.kRakeGearRatio;

    configs.MotionMagic.MotionMagicCruiseVelocity = RakeConstants.MMagicCruiseVelocity;
    configs.MotionMagic.MotionMagicAcceleration = RakeConstants.MMagicAcceleration;
    configs.MotionMagic.MotionMagicJerk = RakeConstants.MMagicJerk;
    configs.MotionMagic.MotionMagicExpo_kV = RakeConstants.MMagicExpo_kV;
    configs.MotionMagic.MotionMagicExpo_kA = RakeConstants.MMagicExpo_kA;

    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = RakeConstants.kRakePositionMax;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RakeConstants.kRakePositionMin;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    StatusCode s = m_rakeMotor.getConfigurator().apply(configs);
    if (!s.isOK()) System.out.println("Could not apply top configs, error code: " + s);
  }

  private void initEncoderConfigs() {
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(0.5));
    configs.MagnetSensor.SensorDirection = RakeConstants.kRakeEncoderDirection;
    // add offset of 0.25 to abs value so total range keeps sign. sub it below
    configs.MagnetSensor.withMagnetOffset(Units.Rotations.of(RakeConstants.kRakeEncoderOffset));

    StatusCode s = m_rakeEncoder.getConfigurator().apply(configs);
    if (!s.isOK()) System.out.println("Could not apply top configs, error code: " + s);

    // set starting position to current absolute position
    s = m_rakeEncoder.setPosition(m_rakeEncoder.getAbsolutePosition().getValueAsDouble());
    if (!s.isOK()) System.out.println("Could not apply position to encoder, error code: " + s);
  }

  private void initCollectorConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.Inverted = CollectorConstants.kCollectorInverted;
    configs.MotorOutput.NeutralMode = CollectorConstants.kCollectorNeutralMode;
    configs.Voltage.PeakForwardVoltage = CollectorConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = CollectorConstants.peakReverseVoltage;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = CollectorConstants.peakForwardTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = CollectorConstants.peakReverseTorqueCurrent;

    configs.Slot0.kS = CollectorConstants.collectorMotorTorqueKS;
    configs.Slot0.kP = CollectorConstants.collectorMotorTorqueKP;
    configs.Slot0.kI = CollectorConstants.collectorMotorTorqueKI;
    configs.Slot0.kD = CollectorConstants.collectorMotorTorqueKD;

    StatusCode s = m_CollectorMotor.getConfigurator().apply(configs);
    if (!s.isOK()) System.out.println("Could not apply top configs, error code: " + s);
  }
    
  private void initSimulation() {
    PhysicsSim.getInstance()
        .addTalonFX(m_rakeMotor, m_rakeEncoder, RakeConstants.kRakeGearRatio, 0.001);
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
        MathUtil.clamp(pos, RakeConstants.kRakePositionMin, RakeConstants.kRakePositionMax);
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
      m_collectorRequest.withVelocity(Cvelocity * CollectorConstants.kCollectorGearRatio));
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
      this.setCollectorVelocity(CollectorConstants.collectorVelocity);
  }

  public void collectorUnstuck() {
    this.setCollectorVelocity(CollectorConstants.collectorUnstuckVelocity);
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
      this.setRakeSpeed(wspeed * RakeConstants.kRakeTeleopSpeed);
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
