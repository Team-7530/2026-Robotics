package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

// YAMS pivot-style controller
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

  public static final CANBus kCANBus = CANBUS_FD;

  // Placeholder CAN IDs - update to match your wiring
  public static final int TURRET_MASTER_ID = 70;
  public static final int TURRET_ANALOG_ID = 1;

  // Turret limits in degrees (180-degree travel centered on 0)
  public static final double TURRET_MIN_DEG = -90.0;
  public static final double TURRET_MAX_DEG = 90.0;

  public static final double kTurretOffset = 0.0;

  public static final double kTurretChainRatio = 1.0 / 1.0;
  public static final double kTurretGearboxRatio = 1.0; // 1:1
  public static final double kTurretGearRatio = kTurretChainRatio * kTurretGearboxRatio;

  public static final double TURRET_KS = 0.0;
  public static final double TURRET_KV = 0.0;
  public static final double TURRET_KA = 0.0;
  public static final double TURRET_KP = 30.0; // 45
  public static final double TURRET_KI = 0.0;
  public static final double TURRET_KD = 0.0;
  public static final AngularVelocity TURRET_kMaxV = DegreesPerSecond.of(180);
  public static final AngularAcceleration TURRET_kMaxA = DegreesPerSecondPerSecond.of(90);

  public static final double kTurretTeleopSpeed = 0;

  // TalonFX hardware + YAMS controller
  private final TalonFX m_turretMotor = new TalonFX(TURRET_MASTER_ID, kCANBus);
  private final AnalogPotentiometer m_turretPotentiometer = new AnalogPotentiometer(TURRET_ANALOG_ID, 360.0);

  private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // PID Constants
      .withClosedLoopController(TURRET_KP, TURRET_KI, TURRET_KD, TURRET_kMaxV, TURRET_kMaxA)
      .withSimClosedLoopController(TURRET_KP, TURRET_KI, TURRET_KD, TURRET_kMaxV, TURRET_kMaxA)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(TURRET_KV, TURRET_KA, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(TURRET_KV, TURRET_KA, 0))
      // Telemetry name and verbosity level
      .withTelemetry("TurretMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // For example gearbox(3,4) is the same as gearbox("3:1","4:1")
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(kTurretChainRatio, kTurretGearboxRatio)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  private final SmartMotorController m_turretSMC = new TalonFXWrapper(m_turretMotor, DCMotor.getKrakenX44(1), smc_config);

  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Inches.of(23.0))
      .withMaxRobotLength(Inches.of(34.0))
      .withRelativePosition(new Translation3d(Inches.of(0.0), Inches.of(8), Inches.of(8)));

  PivotConfig m_turretconfig = new PivotConfig(m_turretSMC)
      .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
      .withWrapping(Degrees.of(-180), Degrees.of(180)) // Wrapping enabled bc the pivot can spin infinitely
      .withHardLimit(Degrees.of(TURRET_MIN_DEG), Degrees.of(TURRET_MAX_DEG)) // Hard limit bc wiring prevents infinite spinning
      .withTelemetry("Turret", TelemetryVerbosity.HIGH) // Telemetry
      .withMOI(Meters.of(0.25), Pounds.of(4)) // MOI Calculation
      .withMechanismPositionConfig(robotToMechanism);

  Pivot m_turret = new Pivot(m_turretconfig);

  private boolean m_isTeleop = false;
  private double turretTargetDeg = 0.0;

  public TurretSubsystem() {

    seedTurretPosition();
  }

  private void seedTurretPosition() {
    Angle potAngle = Degrees.of(m_turretPotentiometer.get() + kTurretOffset);
    m_turretSMC.setEncoderPosition(potAngle);

    SmartDashboard.putNumber("Turret/SeededTurretDeg", potAngle.in(Degrees));
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    m_turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_turret.simIterate();
  }

  public Command setAngle(Angle angle) {
    return m_turret.setAngle(angle);
  }

  public void setAngleDirect(Angle angle) {
    m_turretSMC.setPosition(angle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return m_turret.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return m_turret.getAngle();
  }

  public Command sysId() {
    return m_turret.sysId(
                    Volts.of(4.0), // maximumVoltage
                    Volts.per(Second).of(0.5), // step
                    Seconds.of(8.0) // duration
    );
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return m_turret.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return m_turret.set(dutyCycle);
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
    m_turretSMC.setPosition(Rotations.of(rotations));
  }

  public double getTurretAngleDegrees() {
    double rotations = m_turret.getAngle().in(Rotations);
    return rotations / kTurretGearRatio * 360.0;
  }

  public void stopTurret() {
    m_turretSMC.stopClosedLoopController();
    m_turretSMC.setDutyCycle(0.0);
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
      m_turretSMC.setDutyCycle(aspeed * kTurretTeleopSpeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      m_turretSMC.setDutyCycle(0.0);
    }
  }

  // -- SmartDashboard ----------------------------------------------------
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Turret/TurretAngleDeg", this.getTurretAngleDegrees());
    SmartDashboard.putNumber("Turret/TurretTargetDeg", turretTargetDeg);
    // additional telemetry from YAMS controller
    try {
      SmartDashboard.putNumber("Turret/TurretRotorPos", m_turretSMC.getRotorPosition().in(Rotations));
    } catch (Exception e) {
      // ignore
    }
  }

  // -- Commands -----------------------------------------------------------
  public Command turretToAngleCommand(double degrees) {
    return run(() -> this.setTurretAngleDegrees(degrees))
        .withName("TurretToAngleCommand")
        .until(() -> MathUtil.isNear(degrees, this.getTurretAngleDegrees(), 2.0))
        .withTimeout(5.0);
  }

  public Command seedTurretPositionCommand() {
    return run(() -> seedTurretPosition())
        .withName("SeedTurretPositionCommand");
  }

}
