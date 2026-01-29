package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.STICK_DEADBAND;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;

// YAMS pivot-style controller
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  // TalonFX hardware + YAMS controller
  private final TalonFX m_turretTalon = new TalonFX(TURRET_MASTER_ID, CANBUS);
  private final CANcoder m_turretEncoder = new CANcoder(TURRET_ENCODER_ID, CANBUS);

  private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withIdleMode(MotorMode.BRAKE)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
      .withClosedLoopController(turretMotorKP, turretMotorKI, turretMotorKD)
      .withFeedforward(new SimpleMotorFeedforward(turretMotorKV, turretMotorKA, 0))
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withExternalEncoder(m_turretEncoder);

  private final SmartMotorController m_turretMotor = new TalonFXWrapper(m_turretTalon, edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1), smc_config);

  private boolean m_isTeleop = false;
  private double turretTargetDeg = 0.0;

  public TurretSubsystem() {
    initTurretConfigs();

    // telemetry
    m_turretMotor.setupTelemetry();

    if (RobotBase.isSimulation()) initSimulation();
  }

  private void initTurretConfigs() {
    CANcoderConfiguration configs = new CANcoderConfiguration();
    configs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Units.Rotations.of(0.5));
    configs.MagnetSensor.SensorDirection = kTurretEncoderDirection;
    configs.MagnetSensor.withMagnetOffset(Units.Rotations.of(kTurretEncoderOffset));

    StatusCode s = m_turretEncoder.getConfigurator().apply(configs);
    if (!s.isOK()) System.out.println("Could not apply turret encoder configs, error code: " + s);
    
    // initialize to current absolute position
    m_turretEncoder.setPosition(m_turretEncoder.getAbsolutePosition().getValueAsDouble());
  }

  private void initSimulation() {
    PhysicsSim.getInstance().addTalonFX(m_turretTalon, m_turretEncoder, kTurretGearRatio, 0.001);
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
    m_turretMotor.setPosition(Rotations.of(rotations));
  }

  public double getTurretAngleDegrees() {
    double rotations = m_turretEncoder.getPosition().getValueAsDouble();
    return rotations / kTurretGearRatio * 360.0;
  }

  public void stopTurret() {
    m_turretMotor.stopClosedLoopController();
    m_turretMotor.setDutyCycle(0.0);
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
      m_turretMotor.setDutyCycle(aspeed * kTurretTeleopSpeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      m_turretMotor.setDutyCycle(0.0);
    }
  }
  // -- SmartDashboard ----------------------------------------------------
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Shooter/TurretAngleDeg", this.getTurretAngleDegrees());
    SmartDashboard.putNumber("Shooter/TurretTargetDeg", turretTargetDeg);
    // additional telemetry from YAMS controller
    try {
      SmartDashboard.putNumber("Shooter/TurretRotorPos", m_turretMotor.getRotorPosition().in(Rotations));
    } catch (Exception e) {
      // ignore
    }
  }

  // -- Commands -----------------------------------------------------------
  public Command turretToAngleCommand(double degrees) {
    return run(() -> this.setTurretAngleDegrees(degrees))
        .withName("TurretToAngleCommand")
        .until(() -> Math.abs(this.getTurretAngleDegrees() - degrees) < 2.0)
        .withTimeout(5.0);
  }
}
