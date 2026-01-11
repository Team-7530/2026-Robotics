package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
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
import frc.robot.sim.PhysicsSim;

public class WristSubsystem extends SubsystemBase {

  private final TalonFX m_wristMotor =
      new TalonFX(WristConstants.WRISTMOTOR_ID, WristConstants.CANBUS);
  private final CANcoder m_wristEncoder =
      new CANcoder(WristConstants.WRISTENCODER_ID, WristConstants.CANBUS);

  private final MotionMagicExpoVoltage m_wristRequest = new MotionMagicExpoVoltage(0).withSlot(0);
  private final DutyCycleOut m_manualRequest = new DutyCycleOut(0);
  private final NeutralOut m_brake = new NeutralOut();

  private double wristTargetPosition = 0;
  private int wristSlot = 0;
  private boolean m_isTeleop = false;

  public WristSubsystem() {
    initEncoderConfigs();
    initWristConfigs();

    if (RobotBase.isSimulation()) initSimulation();
  }

  private void initWristConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.Inverted = WristConstants.kWristInverted;
    configs.MotorOutput.NeutralMode = WristConstants.kWristNeutralMode;
    configs.Voltage.PeakForwardVoltage = WristConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = WristConstants.peakReverseVoltage;

    configs.Slot0.kG = WristConstants.wristMotorKG;
    configs.Slot0.kS = WristConstants.wristMotorKS;
    configs.Slot0.kV = WristConstants.wristMotorKV;
    configs.Slot0.kA = WristConstants.wristMotorKA;
    configs.Slot0.kP = WristConstants.wristMotorKP;
    configs.Slot0.kI = WristConstants.wristMotorKI;
    configs.Slot0.kD = WristConstants.wristMotorKD;
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.Slot1.kG = WristConstants.wristMotorKG;
    configs.Slot1.kS = WristConstants.wristMotorKS_slow;
    configs.Slot1.kV = WristConstants.wristMotorKV;
    configs.Slot1.kA = WristConstants.wristMotorKA;
    configs.Slot1.kP = WristConstants.wristMotorKP_slow;
    configs.Slot1.kI = WristConstants.wristMotorKI;
    configs.Slot1.kD = WristConstants.wristMotorKD;
    configs.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.Feedback.FeedbackRemoteSensorID = m_wristEncoder.getDeviceID();
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;//
    configs.Feedback.SensorToMechanismRatio = 1.0;
    configs.Feedback.RotorToSensorRatio = WristConstants.kWristGearRatio;

    configs.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MMagicCruiseVelocity;
    configs.MotionMagic.MotionMagicAcceleration = WristConstants.MMagicAcceleration;
    configs.MotionMagic.MotionMagicJerk = WristConstants.MMagicJerk;
    configs.MotionMagic.MotionMagicExpo_kV = WristConstants.MMagicExpo_kV;
    configs.MotionMagic.MotionMagicExpo_kA = WristConstants.MMagicExpo_kA;

    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WristConstants.kWristPositionMax;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.kWristPositionMin;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    StatusCode status = m_wristMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
  }

  private void initEncoderConfigs() {
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(0.5));
    configs.MagnetSensor.SensorDirection = WristConstants.kWristEncoderDirection;
    // add offset of 0.25 to abs value so total range keeps sign. sub it below
    configs.MagnetSensor.withMagnetOffset(Units.Rotations.of(WristConstants.kWristEncoderOffset));

    StatusCode status = m_wristEncoder.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
    // set starting position to current absolute position
    status = m_wristEncoder.setPosition(m_wristEncoder.getAbsolutePosition().getValueAsDouble());
  }

  private void initSimulation() {
    PhysicsSim.getInstance()
        .addTalonFX(m_wristMotor, m_wristEncoder, WristConstants.kWristGearRatio, 0.001);
    m_wristEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  /**
   * Sets the wrist position
   *
   * @param pos position between 0 and 1
   */
  public void setPosition(double pos) {
    m_isTeleop = false;
    wristTargetPosition =
        MathUtil.clamp(pos, WristConstants.kWristPositionMin, WristConstants.kWristPositionMax);
    wristSlot = (((this.getPosition()) * (wristTargetPosition)) <= 0.0) ? 1 : 0;

    m_wristMotor.setControl(m_wristRequest.withPosition(wristTargetPosition).withSlot(wristSlot));
  }

  /** Returns the wrist position as a double */
  public double getPosition() {
    return m_wristEncoder.getPosition().getValueAsDouble();
  }

  /** Returns true if motor is at target position or within tolerance range */
  public boolean isAtPosition() {
    return MathUtil.isNear(wristTargetPosition, this.getPosition(), POSITION_TOLERANCE);
  }

  /**
   * Sets the wrist motor speed
   *
   * @param wspeed double, target speed
   */
  public void setSpeed(double wspeed) {
    wristTargetPosition = 0;
    m_wristMotor.setControl(m_manualRequest.withOutput(wspeed));
  }

  /** Stops motor and activates brakes */
  public void stop() {
    wristTargetPosition = 0;
    m_wristMotor.setControl(m_brake);
  }

  /** Tells motor to hold current position (can cause jitter) */
  public void hold() {
    this.setPosition(this.getPosition());
  }

  /**
   * Teleop controls
   *
   * @param wspeed wrist target speed during teleop
   */
  public void teleop(double wspeed) {
    wspeed = MathUtil.applyDeadband(wspeed, STICK_DEADBAND);

    if (wspeed != 0.0) {
      m_isTeleop = true;
      this.setSpeed(wspeed * WristConstants.kWristTeleopSpeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      if (USE_POSITIONCONTROL) {
        this.hold();
      } else {
        this.stop();
      }
    }
  }

  /** Upddates the Smart Dashboard */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Wrist Postion", this.getPosition());
    SmartDashboard.putNumber("Wrist TargetPostion", wristTargetPosition);
    SmartDashboard.putNumber("Wrist Slot", wristSlot);
  }

  public Command wristToPositionCommand(double position) {
    return run(() -> this.setPosition(position))
        .withName("WristToPositionCommand")
        .until(this::isAtPosition)
        .withTimeout(5.0);
  }
}
