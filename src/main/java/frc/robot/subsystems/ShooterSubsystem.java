package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sim.PhysicsSim;

/**
 * ShooterSubsystem controls a turret (absolute encoder + position control) and a two-motor
 * flywheel (master + follower). This is a lightweight, conservative implementation that uses
 * percent open-loop control for the flywheel and position control for the turret. Replace IDs
 * in `Constants.ShooterConstants` with your real CAN IDs.
 */
public class ShooterSubsystem extends SubsystemBase {

  // turret
  private final TalonFX m_turretMotor = new TalonFX(ShooterConstants.TURRET_MASTER_ID, ShooterConstants.CANBUS);
  private final CANcoder m_turretEncoder = new CANcoder(ShooterConstants.TURRET_ENCODER_ID, ShooterConstants.CANBUS);
  private final MotionMagicExpoVoltage m_turretRequest = new MotionMagicExpoVoltage(0).withSlot(0);
  private final DutyCycleOut m_manualRequest = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();

  // flywheel
  private final TalonFX m_flywheelMaster = new TalonFX(ShooterConstants.FLYWHEEL_MASTER_ID, ShooterConstants.CANBUS);
  private final TalonFX m_flywheelFollower = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_ID, ShooterConstants.CANBUS);
  private final VelocityTorqueCurrentFOC m_flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut m_flywheelPercentRequest = new DutyCycleOut(0);
  private double lastFlywheelPercent = 0.0;

  private boolean m_isTeleop = false;
  private double turretTargetDeg = 0.0;

  public ShooterSubsystem() {
    initTurretConfigs();
    initEncoderConfigs();

  // Configure flywheel PID slot
  TalonFXConfiguration flyConfigs = new TalonFXConfiguration();
  flyConfigs.MotorOutput.Inverted = ShooterConstants.kFlywheelInverted;
  flyConfigs.MotorOutput.NeutralMode = ShooterConstants.kFlywheelNeutralMode;
  flyConfigs.Voltage.PeakForwardVoltage = ShooterConstants.kFlywheelPeakForwardVoltage;
  flyConfigs.Voltage.PeakReverseVoltage = ShooterConstants.kFlywheelPeakReverseVoltage;

  flyConfigs.Slot0.kS = ShooterConstants.FLYWHEEL_kS;
  flyConfigs.Slot0.kP = ShooterConstants.FLYWHEEL_kP;
  flyConfigs.Slot0.kI = ShooterConstants.FLYWHEEL_kI;
  flyConfigs.Slot0.kD = ShooterConstants.FLYWHEEL_kD;

  StatusCode s = m_flywheelMaster.getConfigurator().apply(flyConfigs);
  if (!s.isOK()) System.out.println("Could not apply flywheel configs: " + s);
  // No built-in follow API used; we'll mirror velocity commands to the follower below.

    if (RobotBase.isSimulation()) initSimulation();
  }

  private void initTurretConfigs() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // basic motor outputs
    configs.MotorOutput.Inverted = ShooterConstants.kTurretInverted; // reuse inversion enum
    configs.MotorOutput.NeutralMode = ShooterConstants.kTurretNeutralMode;
    configs.Voltage.PeakForwardVoltage = ShooterConstants.kTurretPeakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = ShooterConstants.kTurretPeakReverseVoltage;

    // simple feedforward / PID slots (re-use arm tuning as safe defaults)
    configs.Slot0.kS = ShooterConstants.turretMotorKS;
    configs.Slot0.kV = ShooterConstants.turretMotorKV;
    configs.Slot0.kA = ShooterConstants.turretMotorKA;
    configs.Slot0.kP = ShooterConstants.turretMotorKP;
    configs.Slot0.kI = ShooterConstants.turretMotorKI;
    configs.Slot0.kD = ShooterConstants.turretMotorKD;

    // feedback from CANcoder
    configs.Feedback.FeedbackRemoteSensorID = m_turretEncoder.getDeviceID();
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    configs.Feedback.SensorToMechanismRatio = 1.0;
    configs.Feedback.RotorToSensorRatio = ShooterConstants.kTurretGearRatio;

    configs.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.MMagicCruiseVelocity;
    configs.MotionMagic.MotionMagicAcceleration = ShooterConstants.MMagicAcceleration;
    configs.MotionMagic.MotionMagicJerk = ShooterConstants.MMagicJerk;

    StatusCode status = m_turretMotor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply turret configs, error code: " + status.toString());
    }
  }

  private void initEncoderConfigs() {
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(0.5));
    configs.MagnetSensor.SensorDirection = ShooterConstants.kTurretEncoderDirection;
    configs.MagnetSensor.withMagnetOffset(Units.Rotations.of(ShooterConstants.kTurretEncoderOffset));

    StatusCode status = m_turretEncoder.getConfigurator().apply(configs);
    if (!status.isOK()) {
      System.out.println("Could not apply turret encoder configs, error code: " + status.toString());
    }
    // initialize to current absolute position
    m_turretEncoder.setPosition(m_turretEncoder.getAbsolutePosition().getValueAsDouble());
  }

  private void initSimulation() {
    PhysicsSim.getInstance().addTalonFX(m_turretMotor, m_turretEncoder, ShooterConstants.kTurretGearRatio, 0.001);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  // -- Turret control -----------------------------------------------------
  /**
   * Sets the turret angle in degrees (clamped to configured limits).
   */
  public void setTurretAngleDegrees(double degrees) {
    m_isTeleop = false;
    turretTargetDeg = MathUtil.clamp(degrees, ShooterConstants.TURRET_MIN_DEG, ShooterConstants.TURRET_MAX_DEG);
    // convert degrees to rotations (1 rotation = 360 degrees) and apply gear ratio
    double rotations = (turretTargetDeg / 360.0) * ShooterConstants.kTurretGearRatio;
    m_turretMotor.setControl(m_turretRequest.withPosition(rotations));
  }

  public double getTurretAngleDegrees() {
    double rotations = m_turretEncoder.getPosition().getValueAsDouble();
    return rotations / ShooterConstants.kTurretGearRatio * 360.0;
  }

  public void stopTurret() {
    m_turretMotor.setControl(m_neutral);
  }

  // -- Flywheel control --------------------------------------------------
  /**
   * Set flywheel speed as percent output (-1.0..1.0).
   */
  public void setFlywheelPercent(double pct) {
    pct = MathUtil.clamp(pct, -1.0, 1.0);
    m_flywheelMaster.setControl(m_flywheelPercentRequest.withOutput(pct));
    // mirror to follower (no high-level follow helper used)
    m_flywheelFollower.setControl(m_flywheelPercentRequest.withOutput(pct));
    lastFlywheelPercent = pct;
  }

  /**
   * Set flywheel velocity by RPM. This implementation maps RPM to percent based on
   * `ShooterConstants.FLYWHEEL_MAX_RPM`. For closed-loop control replace this method with
   * a velocity closed-loop set using appropriate sensor configuration.
   */
  public void setFlywheelVelocityRPM(double rpm) {
    // Convert desired flywheel RPM to motor rotations per second (sensor units)
    double desiredFlywheelRps = rpm / 60.0;
    double motorRps = desiredFlywheelRps * ShooterConstants.kFlywheelGearRatio;
    m_flywheelMaster.setControl(m_flywheelVelocityRequest.withVelocity(motorRps));
    // mirror to follower
    m_flywheelFollower.setControl(m_flywheelVelocityRequest.withVelocity(motorRps));
    lastFlywheelPercent = MathUtil.clamp(rpm / ShooterConstants.FLYWHEEL_MAX_RPM, -1.0, 1.0);
  }

  public void stopFlywheel() {
    setFlywheelPercent(0.0);
  }

  public double getFlywheelPercent() {
    return lastFlywheelPercent;
  }

  /** Returns flywheel speed in RPM (based on master sensor). */
  public double getFlywheelRPM() {
    double motorRps = m_flywheelMaster.getVelocity().getValueAsDouble();
    double flywheelRps = motorRps / ShooterConstants.kFlywheelGearRatio;
    return flywheelRps * 60.0;
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
      turretTargetDeg = 0;
      m_turretMotor.setControl(m_manualRequest.withOutput(aspeed * ShooterConstants.kTurretTeleopSpeed));
    } else if (m_isTeleop) {
      m_isTeleop = false;
      m_turretMotor.setControl(m_manualRequest.withOutput(0.0));
    }
  }
  // -- SmartDashboard ----------------------------------------------------
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Shooter/TurretAngleDeg", this.getTurretAngleDegrees());
    SmartDashboard.putNumber("Shooter/TurretTargetDeg", turretTargetDeg);
    SmartDashboard.putNumber("Shooter/FlywheelPercent", this.getFlywheelPercent());
  }

  // -- Commands -----------------------------------------------------------
  public Command turretToAngleCommand(double degrees) {
    return run(() -> this.setTurretAngleDegrees(degrees))
        .withName("TurretToAngleCommand")
        .until(() -> Math.abs(this.getTurretAngleDegrees() - degrees) < 2.0)
        .withTimeout(5.0);
  }

  public Command shooterToVelocityCommand(double velocity) {
    return run(() -> this.setFlywheelVelocityRPM(velocity))
        .withName("ShooterToVelocityCommand")
        .withTimeout(5.0);
  }

  public Command shooterToPercentCommand(double pct) {
    return run(() -> this.setFlywheelPercent(pct))
        .withName("ShooterToPercentCommand")
        .withTimeout(5.0);
  }
}
