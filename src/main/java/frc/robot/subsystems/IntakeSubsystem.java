package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX m_LIntakeMotor =
      new TalonFX(IntakeConstants.LINTAKEMOTOR_ID, IntakeConstants.CANBUS);
  private final TalonFX m_RIntakeMotor =
      new TalonFX(IntakeConstants.RINTAKEMOTOR_ID, IntakeConstants.CANBUS);
  private final CANrange m_RangeSensor =
      new CANrange(IntakeConstants.RANGESENSOR_ID, IntakeConstants.CANBUS);

  private final VelocityTorqueCurrentFOC m_velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut m_manualRequest = new DutyCycleOut(0);

  private final NeutralOut m_brake = new NeutralOut();

  private boolean m_isTeleop = false;

  public IntakeSubsystem() {
    initIntakeConfigs();
    initRangeSensor();
  }

  private void initIntakeConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.Inverted = IntakeConstants.kLIntakeInverted;
    configs.MotorOutput.NeutralMode = IntakeConstants.kIntakeNeutralMode;
    configs.Voltage.PeakForwardVoltage = IntakeConstants.peakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = IntakeConstants.peakReverseVoltage;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = IntakeConstants.peakForwardTorqueCurrent;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = IntakeConstants.peakReverseTorqueCurrent;

    configs.Slot0.kS = IntakeConstants.intakeMotorTorqueKS;
    configs.Slot0.kP = IntakeConstants.intakeMotorTorqueKP;
    configs.Slot0.kI = IntakeConstants.intakeMotorTorqueKI;
    configs.Slot0.kD = IntakeConstants.intakeMotorTorqueKD;

    configs.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANrange;
    configs.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    configs.HardwareLimitSwitch.ReverseLimitRemoteSensorID = m_RangeSensor.getDeviceID();
    configs.HardwareLimitSwitch.ReverseLimitEnable = true;

    StatusCode status = m_LIntakeMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }

    configs.MotorOutput.Inverted = IntakeConstants.kRIntakeInverted;
    status = m_RIntakeMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply bottom configs, error code: " + status.toString());
    }
  }

  private void initRangeSensor() {
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.FovParams.FOVCenterX = IntakeConstants.kRangeFOVCenterX;
    config.FovParams.FOVCenterY = IntakeConstants.kRangeFOVCenterY;
    config.FovParams.FOVRangeX = IntakeConstants.kRangeFOVRangeX;
    config.FovParams.FOVRangeY = IntakeConstants.kRangeFOVRangeY;
    config.ProximityParams.ProximityThreshold = IntakeConstants.kProxThreshold;
    config.ProximityParams.ProximityHysteresis = IntakeConstants.kProxHysteresis;
    config.ProximityParams.MinSignalStrengthForValidMeasurement = IntakeConstants.kMinSigStrength;

    /* User can change the configs if they want, or leave it empty for factory-default */
    StatusCode status = m_RangeSensor.getConfigurator().apply(config);
    if (!status.isOK()) {
      System.out.println("Could not apply top configs, error code: " + status.toString());
    }

    /* Set the signal update rate */
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        m_RangeSensor.getDistance(),
        m_RangeSensor.getSignalStrength(),
        m_RangeSensor.getIsDetected());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  /**
   * Sets the motors Target velocity
   *
   * @param Lvelocity left motors target velocity
   * @param Rvelocity Right motors target velocity
   */
  public void setIntakeVelocity(double Lvelocity, double Rvelocity) {
    m_isTeleop = false;
    m_LIntakeMotor.setControl(
        m_velocityRequest.withVelocity(Lvelocity * IntakeConstants.kIntakeGearRatio));
    m_RIntakeMotor.setControl(
        m_velocityRequest.withVelocity(Rvelocity * IntakeConstants.kIntakeGearRatio));
  }

  /**
   * Sets intake motor speed
   *
   * @param speed double
   */
  public void setIntakeSpeed(double speed) {
    m_LIntakeMotor.setControl(m_manualRequest.withOutput(speed));
    m_RIntakeMotor.setControl(m_manualRequest.withOutput(speed));
  }

  /** Activates motor brakes */
  public void intakeStop() {
    m_LIntakeMotor.setControl(m_brake);
    m_RIntakeMotor.setControl(m_brake);
  }

  /** Sets motors to constants intake speed */
  public void intakeIn() {
    this.setIntakeVelocity(IntakeConstants.intakeVelocity, IntakeConstants.intakeVelocity);
  }

  /** Sets motors to constants out speed */
  public void intakeOut() {
    this.setIntakeVelocity(IntakeConstants.outtakeL2Velocity, IntakeConstants.outtakeL2Velocity);
  }

  /** Outputs coral but spins it for L1 scoring */
  public void intakeOutSpin() {
    this.setIntakeVelocity(IntakeConstants.outtakeL1VelocityL, IntakeConstants.outtakeL1VelocityR);
  }

  /** Returns true if range sensor detects coral */
  public boolean hasCoralLoaded() {
    return m_RangeSensor.getIsDetected().getValue();
  }

  /** Teleop controls for Intake */
  public void teleop(double intake) {
    intake = MathUtil.applyDeadband(intake, STICK_DEADBAND);

    if (intake != 0.0) {
      m_isTeleop = true;
      this.setIntakeSpeed(intake * 0.1);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      this.intakeStop();
    }
  }

  /** Updates the Smart Dashboard */
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("LIntake Speed", m_LIntakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("RIntake Speed", m_RIntakeMotor.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Distance", m_RangeSensor.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Strength", m_RangeSensor.getSignalStrength().getValueAsDouble());
    SmartDashboard.putBoolean("Detected", m_RangeSensor.getIsDetected().getValue());
  }

  public Command intakeCommand() {
    return run(() -> this.intakeIn())
        .withName("IntakeCommand")
        .until(this::hasCoralLoaded)
        .withTimeout(5.0)
        .finallyDo(() -> this.intakeStop());
  }

  public Command outtakeL1Command() {
    return run(() -> this.intakeOutSpin())
        .withName("OuttakeL1Command")
        .withTimeout(1.0)
        .finallyDo(() -> this.intakeStop());
  }

  public Command outtakeL2Command() {
    return run(() -> this.intakeOut())
        .withName("OuttakeL2Command")
        .withTimeout(1.0)
        .finallyDo(() -> this.intakeStop());
  }
}
