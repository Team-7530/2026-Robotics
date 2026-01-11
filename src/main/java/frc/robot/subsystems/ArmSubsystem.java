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

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX m_armMotor = new TalonFX(ArmConstants.ARMMOTOR_ID, ArmConstants.CANBUS);
  private final CANcoder m_armEncoder =
      new CANcoder(ArmConstants.ARMENCODER_ID, ArmConstants.CANBUS);

  private final MotionMagicExpoVoltage m_armRequest = new MotionMagicExpoVoltage(0).withSlot(0);
  private final DutyCycleOut m_manualRequest = new DutyCycleOut(0);

  private final NeutralOut m_brake = new NeutralOut();

  private double armTargetPosition = 0;
  private boolean m_isTeleop = false;

  public ArmSubsystem() {
    initEncoderConfigs();
    initArmConfigs();

    if (RobotBase.isSimulation()) initSimulation();
  }

  private void initArmConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.Inverted = ArmConstants.kArmInverted;
    configs.MotorOutput.NeutralMode = ArmConstants.kArmNeutralMode;
    configs.Voltage.PeakForwardVoltage = ArmConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = ArmConstants.peakReverseVoltage;

    configs.Slot0.kG = ArmConstants.armMotorKG;
    configs.Slot0.kS = ArmConstants.armMotorKS;
    configs.Slot0.kV = ArmConstants.armMotorKV;
    configs.Slot0.kA = ArmConstants.armMotorKA;
    configs.Slot0.kP = ArmConstants.armMotorKP;
    configs.Slot0.kI = ArmConstants.armMotorKI;
    configs.Slot0.kD = ArmConstants.armMotorKD;
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.Feedback.FeedbackRemoteSensorID = m_armEncoder.getDeviceID();
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    configs.Feedback.SensorToMechanismRatio = 1.0;
    configs.Feedback.RotorToSensorRatio = ArmConstants.kArmGearRatio;

    configs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MMagicCruiseVelocity;
    configs.MotionMagic.MotionMagicAcceleration = ArmConstants.MMagicAcceleration;
    configs.MotionMagic.MotionMagicJerk = ArmConstants.MMagicJerk;
    configs.MotionMagic.MotionMagicExpo_kV = ArmConstants.MMagicExpo_kV;
    configs.MotionMagic.MotionMagicExpo_kA = ArmConstants.MMagicExpo_kA;

    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.kArmPositionMax;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.kArmPositionMin;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    StatusCode status = m_armMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
  }

  private void initEncoderConfigs() {
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(0.5));
    configs.MagnetSensor.SensorDirection = ArmConstants.kArmEncoderDirection;
    configs.MagnetSensor.withMagnetOffset(Units.Rotations.of(ArmConstants.kArmEncoderOffset));

    StatusCode status = m_armEncoder.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
    // set starting position to current absolute position
    m_armEncoder.setPosition(m_armEncoder.getAbsolutePosition().getValueAsDouble());
  }

  private void initSimulation() {
    PhysicsSim.getInstance()
        .addTalonFX(m_armMotor, m_armEncoder, ArmConstants.kArmGearRatio, 0.001);
    m_armEncoder.setPosition(0.25);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  /**
   * Moves the arm to a specific position
   *
   * @param pos position between 0 and 1
   */
  public void setPosition(double pos) {
    m_isTeleop = false;
    armTargetPosition =
        MathUtil.clamp(pos, ArmConstants.kArmPositionMin, ArmConstants.kArmPositionMax);

    m_armMotor.setControl(m_armRequest.withPosition(armTargetPosition));
  }

  /** Returns the arm encoder position as a double */
  public double getPosition() {
    return m_armEncoder.getPosition().getValueAsDouble();
  }

  /** Returns the arm rotor position as a double */
  public double getRotorPosition() {
    return m_armMotor.getRotorPosition().getValueAsDouble();
  }

  /** Returns true if arm is at the target position or is within the error tolerance range */
  public boolean isAtPosition() {
    return MathUtil.isNear(armTargetPosition, this.getPosition(), POSITION_TOLERANCE);
  }

  /**
   * Sets the arm rotation speed
   *
   * @param aspeed requires a double
   */
  public void setSpeed(double aspeed) {
    armTargetPosition = 0;
    m_armMotor.setControl(m_manualRequest.withOutput(aspeed));
  }

  /** Stops moter movement and activates the motor brake */
  public void stop() {
    armTargetPosition = 0;
    m_armMotor.setControl(m_brake);
  }

  /** Attempts to hold the motor at its current position (can cause jitter) */
  public void hold() {
    this.setPosition(this.getPosition());
  }

  /**
   * Teleop controls
   *
   * @param aspeed a double that sets the arm speed during teleop
   */
  public void teleop(double aspeed) {
    aspeed = MathUtil.applyDeadband(aspeed, STICK_DEADBAND);

    if (aspeed != 0.0) {
      m_isTeleop = true;
      this.setSpeed(aspeed * ArmConstants.kArmTeleopSpeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      if (USE_POSITIONCONTROL) {
        this.hold();
      } else {
        this.setSpeed(0.0);
      }
    }
  }

  /** Updates the Smart Dashboard */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Arm Postion", this.getPosition());
    SmartDashboard.putNumber("Arm TargetPostion", armTargetPosition);
  }

  public Command armToPositionCommand(double position) {
    return run(() -> this.setPosition(position))
        .withName("ArmToPositionCommand")
        .until(this::isAtPosition)
        .withTimeout(5.0);
  }
}
