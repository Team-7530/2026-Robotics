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
 * percent open-loop control for the flywheel and position control for the turret. 
 */
public class TurretSubsystem extends SubsystemBase {

  public static final CANBus CANBUS = new CANBus("CANFD");

  // Placeholder CAN IDs - update to match your wiring
  public static final int TURRET_MASTER_ID = 51;
  public static final int TURRET_ENCODER_ID = 52;
  public static final int TURRET_ENCODER2_ID = 53;

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

  public static final double kTurretTeleopSpeed = 0;

  // turret
  private final TalonFX m_turretMotor = new TalonFX(TURRET_MASTER_ID, CANBUS);
  private final CANcoder m_turretEncoder = new CANcoder(TURRET_ENCODER_ID, CANBUS);
  private final MotionMagicExpoVoltage m_turretRequest = new MotionMagicExpoVoltage(0).withSlot(0);
  private final DutyCycleOut m_turretManualRequest = new DutyCycleOut(0);
  private final NeutralOut m_neutral = new NeutralOut();

  private boolean m_isTeleop = false;
  private double turretTargetDeg = 0.0;

  public TurretSubsystem() {
    initTurretConfigs();

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
  }

  // -- Commands -----------------------------------------------------------
  public Command turretToAngleCommand(double degrees) {
    return run(() -> this.setTurretAngleDegrees(degrees))
        .withName("TurretToAngleCommand")
        .until(() -> Math.abs(this.getTurretAngleDegrees() - degrees) < 2.0)
        .withTimeout(5.0);
  }
}
