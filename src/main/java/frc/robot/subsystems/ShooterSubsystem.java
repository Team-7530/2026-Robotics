package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;

import static frc.robot.Constants.STICK_DEADBAND;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

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
 * percent open-loop control for the flywheel and position control for the turret. 
 */
public class ShooterSubsystem extends SubsystemBase {

  public static final CANBus CANBUS = new CANBus("CANFD");

  // Placeholder CAN IDs - update to match your wiring
  public static final int TURRET_MASTER_ID = 51;
  public static final int TURRET_ENCODER_ID = 52;

  public static final int FLYWHEEL_MASTER_ID = 53;
  public static final int FLYWHEEL_FOLLOWER_ID = 54;

  public static final int INTAKEMOTOR_ID = 35;

  // Turret limits in degrees (180-degree travel centered on 0)
  public static final double TURRET_MIN_DEG = -90.0;
  public static final double TURRET_MAX_DEG = 90.0;

  public static final InvertedValue kTurretInverted = InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue kTurretNeutralMode = NeutralModeValue.Brake;
  public static final SensorDirectionValue kTurretEncoderDirection =
      SensorDirectionValue.CounterClockwise_Positive;
  public static final double kTurretEncoderOffset = 0.0;

  public static final double kTurretChainRatio = 1.0 / 1.0;
  public static final double kTurretGearboxRatio = 1.0; // 1:1
  public static final double kTurretGearRatio = kTurretChainRatio * kTurretGearboxRatio;

  public static final double turretMotorKS = 0.0;
  public static final double turretMotorKV = 0.0;
  public static final double turretMotorKA = 0.0;
  public static final double turretMotorKP = 30.0; // 45
  public static final double turretMotorKI = 0.0;
  public static final double turretMotorKD = 0.0;
  public static final double MMagicCruiseVelocity = 1;
  public static final double MMagicAcceleration = 2;
  public static final double MMagicJerk = 8000;
  public static final double MMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
  public static final double MMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

  public static final double kTurretPeakForwardVoltage = 8.0; // Peak output of 8 volts
  public static final double kTurretPeakReverseVoltage = -8.0; // Peak output of 8 volts

  // Flywheel tuning
  public static final double FLYWHEEL_MAX_RPM = 6000.0; // adjust to your flywheel
  public static final double FLYWHEEL_DEFAULT_RPM = 3000.0;

  public static final InvertedValue kFlywheelInverted = InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue kFlywheelNeutralMode = NeutralModeValue.Brake;

  public static final double kFlywheelChainRatio = 1.0 / 1.0;
  public static final double kFlywheelGearboxRatio = 1.0; // 1:1
  public static final double kFlywheelGearRatio = kFlywheelChainRatio * kFlywheelGearboxRatio;

  // Flywheel control tuning (Velocity closed-loop)
  public static final double FLYWHEEL_kS = 0.0;
  public static final double FLYWHEEL_kP = 0.1;
  public static final double FLYWHEEL_kI = 0.0;
  public static final double FLYWHEEL_kD = 0.0;

  public static final double kFlywheelPeakForwardVoltage = 8.0; // Peak output of 8 volts
  public static final double kFlywheelPeakReverseVoltage = -8.0; // Peak output of 8 volts

  // Intake tuning
  public static final InvertedValue kIntakeInverted = InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue kIntakeNeutralMode = NeutralModeValue.Coast;
  public static final double kIntakePeakForwardVoltage = 10.0; // Peak output of 8 volts
  public static final double kIntakePeakReverseVoltage = -10.0; // Peak output of 8 volts
  public static final double kIntakePeakForwardTorqueCurrent = 40.0; // Peak output of 40 amps
  public static final double kIntakePeakReverseTorqueCurrent = -40.0; // Peak output of 40 amps

  public static final double kIntakeChainRatio = 24.0 / 10.0; // 24:10
  public static final double kIntakeGearboxRatio = 1.0; // 1:1
  public static final double kIntakeGearRatio = kIntakeChainRatio * kIntakeGearboxRatio;

  /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
  public static final double intakeMotorTorqueKS = 0.0; // Static feedforward gain
  public static final double intakeMotorTorqueKP = 8.0; // error of 1 rps results in 8 amps output
  public static final double intakeMotorTorqueKI = 0.2; // error of 1 rps incr by 0.2 amps per sec
  public static final double intakeMotorTorqueKD = 0.001; // 1000 rps^2 incr 1 amp output

  public static final double intakeVelocity = -3.0;

  // Flywheel tuning
  public static final double INTAKE_MAX_RPM = 6000.0; // adjust to your intake
  public static final double INTAKE_DEFAULT_RPM = 3000.0;

  public static final double kTurretTeleopSpeed = 0;

  // turret
  private final TalonFX m_turretMotor = new TalonFX(TURRET_MASTER_ID, CANBUS);
  private final CANcoder m_turretEncoder = new CANcoder(TURRET_ENCODER_ID, CANBUS);
  private final MotionMagicExpoVoltage m_turretRequest = new MotionMagicExpoVoltage(0).withSlot(0);
  private final DutyCycleOut m_turretManualRequest = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();

  // flywheel
  private final TalonFX m_flywheelMaster = new TalonFX(FLYWHEEL_MASTER_ID, CANBUS);
  private final TalonFX m_flywheelFollower = new TalonFX(FLYWHEEL_FOLLOWER_ID, CANBUS);
  private final VelocityTorqueCurrentFOC m_flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut m_flywheelPercentRequest = new DutyCycleOut(0);
  private double lastFlywheelPercent = 0.0;

  // intake
  private final TalonFX m_intakeMotor = new TalonFX(INTAKEMOTOR_ID, CANBUS);
  private final VelocityTorqueCurrentFOC m_intakeVelocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut m_intakePercentRequest = new DutyCycleOut(0);
  private double lastIntakePercent = 0.0;

  private boolean m_isTeleop = false;
  private double turretTargetDeg = 0.0;

  public ShooterSubsystem() {
    initTurretConfigs();
    initFlywheelConfigs();
    initIntakeConfigs();

  // No built-in follow API used; we'll mirror velocity commands to the follower below.

    if (RobotBase.isSimulation()) initSimulation();
  }

  private void initTurretConfigs() {
    TalonFXConfiguration turretConfigs = new TalonFXConfiguration();

    // basic motor outputs
    turretConfigs.MotorOutput.Inverted = kTurretInverted; // reuse inversion enum
    turretConfigs.MotorOutput.NeutralMode = kTurretNeutralMode;
    turretConfigs.Voltage.PeakForwardVoltage = kTurretPeakForwardVoltage;
    turretConfigs.Voltage.PeakReverseVoltage = kTurretPeakReverseVoltage;

    // simple feedforward / PID slots (re-use arm tuning as safe defaults)
    turretConfigs.Slot0.kS = turretMotorKS;
    turretConfigs.Slot0.kV = turretMotorKV;
    turretConfigs.Slot0.kA = turretMotorKA;
    turretConfigs.Slot0.kP = turretMotorKP;
    turretConfigs.Slot0.kI = turretMotorKI;
    turretConfigs.Slot0.kD = turretMotorKD;
    // feedback from CANcoder
    turretConfigs.Feedback.FeedbackRemoteSensorID = m_turretEncoder.getDeviceID();
    turretConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turretConfigs.Feedback.SensorToMechanismRatio = 1.0;
    turretConfigs.Feedback.RotorToSensorRatio = kTurretGearRatio;

    turretConfigs.MotionMagic.MotionMagicCruiseVelocity = MMagicCruiseVelocity;
    turretConfigs.MotionMagic.MotionMagicAcceleration = MMagicAcceleration;
    turretConfigs.MotionMagic.MotionMagicJerk = MMagicJerk;
    StatusCode s = m_turretMotor.getConfigurator().apply(turretConfigs);
    if (!s.isOK()) System.out.println("Could not apply turret configs, error code: " + s);

    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(0.5));
    configs.MagnetSensor.SensorDirection = kTurretEncoderDirection;
    configs.MagnetSensor.withMagnetOffset(Units.Rotations.of(kTurretEncoderOffset));

    s = m_turretEncoder.getConfigurator().apply(configs);
    if (!s.isOK()) System.out.println("Could not apply turret encoder configs, error code: " + s);
    
    // initialize to current absolute position
    m_turretEncoder.setPosition(m_turretEncoder.getAbsolutePosition().getValueAsDouble());
  }

  private void initFlywheelConfigs() {
    // Configure flywheel PID slot
    TalonFXConfiguration flyConfigs = new TalonFXConfiguration();
    flyConfigs.MotorOutput.Inverted = kFlywheelInverted;
    flyConfigs.MotorOutput.NeutralMode = kFlywheelNeutralMode;
    flyConfigs.Voltage.PeakForwardVoltage = kFlywheelPeakForwardVoltage;
    flyConfigs.Voltage.PeakReverseVoltage = kFlywheelPeakReverseVoltage;

    flyConfigs.Slot0.kS = FLYWHEEL_kS;
    flyConfigs.Slot0.kP = FLYWHEEL_kP;
    flyConfigs.Slot0.kI = FLYWHEEL_kI;
    flyConfigs.Slot0.kD = FLYWHEEL_kD;

    StatusCode s = m_flywheelMaster.getConfigurator().apply(flyConfigs);
    if (!s.isOK()) System.out.println("Could not apply flywheel configs: " + s);

    s = m_flywheelFollower.getConfigurator().apply(flyConfigs);
    if (!s.isOK()) System.out.println("Could not apply flywheel configs: " + s);
    m_flywheelFollower.setControl(new Follower(m_flywheelMaster.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  private void initIntakeConfigs() {
    TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();

    // basic motor outputs
    intakeConfigs.MotorOutput.Inverted = kIntakeInverted; // reuse inversion enum
    intakeConfigs.MotorOutput.NeutralMode = kIntakeNeutralMode;
    intakeConfigs.Voltage.PeakForwardVoltage = kIntakePeakForwardVoltage;
    intakeConfigs.Voltage.PeakReverseVoltage = kIntakePeakReverseVoltage;
    intakeConfigs.TorqueCurrent.PeakForwardTorqueCurrent = kIntakePeakForwardTorqueCurrent;
    intakeConfigs.TorqueCurrent.PeakReverseTorqueCurrent = kIntakePeakReverseTorqueCurrent;

    intakeConfigs.Slot0.kS = intakeMotorTorqueKS;
    intakeConfigs.Slot0.kP = intakeMotorTorqueKP;
    intakeConfigs.Slot0.kI = intakeMotorTorqueKI;
    intakeConfigs.Slot0.kD = intakeMotorTorqueKD;

    intakeConfigs.MotionMagic.MotionMagicCruiseVelocity = MMagicCruiseVelocity;
    intakeConfigs.MotionMagic.MotionMagicAcceleration = MMagicAcceleration;
    intakeConfigs.MotionMagic.MotionMagicJerk = MMagicJerk;

    StatusCode s = m_turretMotor.getConfigurator().apply(intakeConfigs);
    if (!s.isOK()) System.out.println("Could not apply turret configs, error code: " + s);
  }

  private void initSimulation() {
    PhysicsSim.getInstance().addTalonFX(m_turretMotor, m_turretEncoder, kTurretGearRatio, 0.001);
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
    turretTargetDeg = MathUtil.clamp(degrees, TURRET_MIN_DEG, TURRET_MAX_DEG);
    // convert degrees to rotations (1 rotation = 360 degrees) and apply gear ratio
    double rotations = (turretTargetDeg / 360.0) * kTurretGearRatio;
    m_turretMotor.setControl(m_turretRequest.withPosition(rotations));
  }

  public double getTurretAngleDegrees() {
    double rotations = m_turretEncoder.getPosition().getValueAsDouble();
    return rotations / kTurretGearRatio * 360.0;
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
   * `FLYWHEEL_MAX_RPM`. For closed-loop control replace this method with
   * a velocity closed-loop set using appropriate sensor configuration.
   */
  public void setFlywheelVelocityRPM(double rpm) {
    // Convert desired flywheel RPM to motor rotations per second (sensor units)
    double desiredFlywheelRps = rpm / 60.0;
    double motorRps = desiredFlywheelRps * kFlywheelGearRatio;
    m_flywheelMaster.setControl(m_flywheelVelocityRequest.withVelocity(motorRps));
    // mirror to follower
    m_flywheelFollower.setControl(m_flywheelVelocityRequest.withVelocity(motorRps));
    lastFlywheelPercent = MathUtil.clamp(rpm / FLYWHEEL_MAX_RPM, -1.0, 1.0);
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
    double flywheelRps = motorRps / kFlywheelGearRatio;
    return flywheelRps * 60.0;
  }

    // -- Intake control --------------------------------------------------
  /**
   * Set intake speed as percent output (-1.0..1.0).
   */
  public void setIntakePercent(double pct) {
    pct = MathUtil.clamp(pct, -1.0, 1.0);
    m_intakeMotor.setControl(m_intakePercentRequest.withOutput(pct));
    lastIntakePercent = pct;
  }

  /**
   * Set flywheel velocity by RPM. This implementation maps RPM to percent based on
   * `FLYWHEEL_MAX_RPM`. For closed-loop control replace this method with
   * a velocity closed-loop set using appropriate sensor configuration.
   */
  public void setIntakeVelocityRPM(double rpm) {
    // Convert desired intake RPM to motor rotations per second (sensor units)
    double desiredIntakeRps = rpm / 60.0;
    double motorRps = desiredIntakeRps * kIntakeGearRatio;
    m_intakeMotor.setControl(m_intakeVelocityRequest.withVelocity(motorRps));
    lastIntakePercent = MathUtil.clamp(rpm / INTAKE_MAX_RPM, -1.0, 1.0);
  }

  public void stopIntake() {
    setIntakePercent(0.0);
  }

  public double getIntakePercent() {
    return lastIntakePercent;
  }

  /** Returns intake speed in RPM (based on master sensor). */
  public double getIntakeRPM() {
    double motorRps = m_intakeMotor.getVelocity().getValueAsDouble();
    double intakeRps = motorRps / kIntakeGearRatio;
    return intakeRps * 60.0;
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
      m_turretMotor.setControl(m_turretManualRequest.withOutput(aspeed * kTurretTeleopSpeed));
    } else if (m_isTeleop) {
      m_isTeleop = false;
      m_turretMotor.setControl(m_turretManualRequest.withOutput(0.0));
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
