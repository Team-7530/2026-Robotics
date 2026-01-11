package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.sim.PhysicsSim;

public class ClimberSubsystem implements Subsystem {

  private final TalonFX m_ClimbMotor =
      new TalonFX(ClimberConstants.CLIMBMOTOR_ID, ClimberConstants.CANBUS);
  private final TalonFX m_ClimbMotorFollower =
      new TalonFX(ClimberConstants.CLIMBMOTORFOLLOWER_ID, ClimberConstants.CANBUS);
  private final CANcoder m_ClimbEncoder =
      new CANcoder(ClimberConstants.CLIMBENCODER_ID, ClimberConstants.CANBUS);
  private final Servo m_ClimberClampServo = new Servo(ClimberConstants.CLAMPSERVO_ID);
  private final Servo m_ClimberClampServoFollower =
      new Servo(ClimberConstants.CLAMPSERVOFOLLOWER_ID);

  private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicTorqueCurrentFOC m_climbRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(1);
  private final DutyCycleOut m_manualRequest = new DutyCycleOut(0);
  private final NeutralOut m_brake = new NeutralOut();

  private double m_targetPosition = 0.0;
  private boolean m_isTeleop = false;
  private boolean m_isClamped = false;

  public ClimberSubsystem() {
    initEncoderConfigs();
    initClimberConfigs();

    if (RobotBase.isSimulation()) initSimulation();
  }

  private void initClimberConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.Inverted = ClimberConstants.kClimberInverted;
    configs.MotorOutput.NeutralMode = ClimberConstants.kClimberNeutralMode;
    configs.Voltage.PeakForwardVoltage = ClimberConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = ClimberConstants.peakReverseVoltage;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = ClimberConstants.peakForwardTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = ClimberConstants.peakReverseTorqueCurrent;

    StatusCode status = m_ClimbMotorFollower.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    configs.Slot0.kP = ClimberConstants.climbMotorKP;
    configs.Slot0.kI = ClimberConstants.climbMotorKI;
    configs.Slot0.kD = ClimberConstants.climbMotorKD;
    configs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    configs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.Slot1.kP = ClimberConstants.climbMotorTorqueKP;
    configs.Slot1.kI = ClimberConstants.climbMotorTorqueKI;
    configs.Slot1.kD = ClimberConstants.climbMotorTorqueKD;
    configs.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    configs.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    configs.Feedback.FeedbackRemoteSensorID = m_ClimbEncoder.getDeviceID();
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    configs.Feedback.SensorToMechanismRatio = 1.0;
    configs.Feedback.RotorToSensorRatio = ClimberConstants.kClimberGearRatio;

    configs.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.MMagicCruiseVelocity;
    configs.MotionMagic.MotionMagicAcceleration = ClimberConstants.MMagicAcceleration;
    configs.MotionMagic.MotionMagicJerk = ClimberConstants.MMagicJerk;

    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.kClimberPositionMax;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.kClimberPositionMin;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    status = m_ClimbMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    /* Follower is opposite, so we need to invert */
    m_ClimbMotorFollower.setControl(new Follower(m_ClimbMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    m_ClimberClampServo.set(ClimberConstants.kUnclampedPosition);
    m_ClimberClampServoFollower.set(ClimberConstants.kUnclampedPositionFollower);
  }

  private void initEncoderConfigs() {
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(0.5));
    configs.MagnetSensor.SensorDirection = ClimberConstants.kClimberEncoderDirection;
    configs.MagnetSensor.withMagnetOffset(
        Units.Rotations.of(ClimberConstants.kClimberEncoderOffset));

    StatusCode status = m_ClimbEncoder.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }
    // set starting position to current absolute position
    m_ClimbEncoder.setPosition(m_ClimbEncoder.getAbsolutePosition().getValueAsDouble());
  }

  private void initSimulation() {
    PhysicsSim.getInstance()
        .addTalonFX(m_ClimbMotor, m_ClimbEncoder, ClimberConstants.kClimberGearRatio, 0.001);
    m_ClimbEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  /** Returns climber to start position */
  public void start() {
    this.setClamp(false);
    this.setPosition(ClimberConstants.kTargetClimberStart);
  }

  /** Closes the clamp and moves to climb position */
  public void climb() {
    this.setClamp(true);

    m_isTeleop = false;
    m_targetPosition = ClimberConstants.kTargetClimberFull;
    m_ClimbMotor.setControl(m_climbRequest.withPosition(m_targetPosition));
  }

  /** Stows the climber */
  public void stow() {
    this.setClamp(false);
    this.setPosition(ClimberConstants.kClimberPositionMax);
  }

  /**
   * Sets the climber target position
   *
   * @param pos double between kClimberPositionMin and kClimberPositionMax
   */
  public void setPosition(double pos) {
    m_isTeleop = false;
    m_targetPosition =
        MathUtil.clamp(
            pos, ClimberConstants.kClimberPositionMin, ClimberConstants.kClimberPositionMax);

    if (!m_isClamped || (m_targetPosition > this.getPosition())) { // is climbing or no ratchet
      m_ClimbMotor.setControl(m_positionRequest.withPosition(m_targetPosition));
    }
  }

  /** Returns the current climb motor position as a double */
  public double getPosition() {
    return m_ClimbEncoder.getPosition().getValueAsDouble();
  }

  /** Returns the current climb rotor position as a double */
  public double getRotorPosition() {
    return m_ClimbMotor.getRotorPosition().getValueAsDouble();
  }

  /** Returns true if climber is at the position or within the tolerance range */
  public boolean isAtPosition(double position) {
    return MathUtil.isNear(position, this.getPosition(), 0.01);
  }

  /** Returns true if climber is at the target position or within the tolerance range */
  public boolean isAtPosition() {
    return this.isAtPosition(m_targetPosition);
  }

  /**
   * sets the speed of the climber
   *
   * @param speed target speed
   */
  public void setSpeed(double speed) {
    m_targetPosition = 0.0;

    if (!m_isClamped || (speed > 0.0)) // is climbing or no ratchet
    m_ClimbMotor.setControl(m_manualRequest.withOutput(speed));
  }

  /** Stops the motor and activates the brake */
  public void stop() {
    m_targetPosition = 0.0;
    m_ClimbMotor.setControl(m_brake);
  }

  /**
   * Handles climber controls during teleop
   *
   * @param val controller deadband
   */
  public void teleopClimb(double val) {
    val = MathUtil.applyDeadband(val, STICK_DEADBAND);

    if (val != 0.0) {
      m_isTeleop = true;
      this.setSpeed(val * ClimberConstants.kClimberSpeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      this.stop();
    }
  }

  /**
   * Opens or closes the clamp
   *
   * @param clampOn bool
   */
  public void setClamp(boolean clampOn) {
    m_isClamped = clampOn;
    this.stop();

    m_ClimberClampServo.set(
        m_isClamped ? ClimberConstants.kClampedPosition : ClimberConstants.kUnclampedPosition);
    m_ClimberClampServoFollower.set(
        m_isClamped
            ? ClimberConstants.kClampedPositionFollower
            : ClimberConstants.kUnclampedPositionFollower);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    StatusCode status = m_ClimbMotor.getConfigurator().refresh(configs);
    if (status == StatusCode.OK) {
      configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = m_isClamped ? ClimberConstants.kTargetClimberFull : ClimberConstants.kClimberPositionMax;     

      status = m_ClimbMotor.getConfigurator().apply(configs);
      if (!status.isOK()) {
        System.out.println("Could not apply configs, error code: " + status.toString());
      }
    }
  }

  /** Returns true if clamp is closed, false if open */
  public boolean getClamp() {
    return m_isClamped;
  }

  /** Updates the Smart Dashboard */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Climber Postion", this.getPosition());
    SmartDashboard.putNumber("Climber TargetPostion", m_targetPosition);
  }

  public Command climbToStowPositionCommand() {
    return run(() -> this.stow())
        .withName("ClimbToStowPositionCommand")
        .until(this::isAtPosition)
        .withTimeout(5.0)
        .finallyDo(() -> this.stop());
  }

  public Command climbToFullPositionCommand() {
    return run(() -> this.climb())
        .withName("ClimbToFullPositionCommand")
        .until(this::isAtPosition)
        .withTimeout(5.0);
  }

  public Command climbToStartPositionCommand() {
    return run(() -> this.start())
        .withName("ClimbToStartPositionCommand")
        .until(this::isAtPosition)
        .withTimeout(5.0)
        .finallyDo(() -> this.stop());
  }

  public Command clampCommand(boolean clamp) {
    return runOnce(() -> this.setClamp(clamp)).withName("ClimbClampCommand");
  }
}
