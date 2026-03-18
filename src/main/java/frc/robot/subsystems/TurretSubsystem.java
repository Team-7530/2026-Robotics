package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import frc.robot.Telemetry;
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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class TurretSubsystem extends SubsystemBase {
  private final Telemetry telemetry;

  private static final CANBus kCANBus = CANBUS_FD;

  // Update these CAN IDs to match the robot wiring.
  private static final int TURRET_MASTER_ID = 62;
  private static final int TURRET_ANALOG_ID = 0;

  // Turret travel is limited by wiring and frame geometry.
  private static final Angle TURRET_MIN_DEG = Degrees.of(-80.0);
  private static final Angle TURRET_MAX_DEG = Degrees.of(80.0);

  private static final double kTurretOffset = 19.5;

  private static final double kTurretChainRatio = 200.0 / 20.0; // 20:200 ratio (20 teeth on motor sprocket, 200 teeth on turret sprocket)
  private static final double kTurretGearboxRatio = 20.0; // 20:1

  private static final double TURRET_KP = 180.0; // 45
  private static final double TURRET_KI = 0.0;
  private static final double TURRET_KD = 0.0;
  private static final double TURRET_KV = 0.0;
  private static final double TURRET_KA = 0.0;

  private static final AngularVelocity TURRET_kMaxV = DegreesPerSecond.of(1440);
  private static final AngularAcceleration TURRET_kMaxA = DegreesPerSecondPerSecond.of(1440);

  private static final double kTurretTeleopSpeed = 0.8;
  
  // TalonFX hardware + YAMS controller
  private final TalonFX m_turretMotor = new TalonFX(TURRET_MASTER_ID, kCANBus);

  @Logged(importance = Logged.Importance.CRITICAL)
  private final AnalogPotentiometer m_turretPotentiometer = new AnalogPotentiometer(TURRET_ANALOG_ID, 360.0, -180.0);

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

  private final SmartMotorController m_turretSMC = new TalonFXWrapper(m_turretMotor, DCMotor.getKrakenX60Foc(1), smc_config);

  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Inches.of(23.0))
      .withMaxRobotLength(Inches.of(34.0))
      .withRelativePosition(new Translation3d(Inches.of(0.0), Inches.of(8), Inches.of(8)));

  PivotConfig m_turretconfig = new PivotConfig(m_turretSMC)
      .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
      .withWrapping(Degrees.of(-180), Degrees.of(180)) // Wrapping enabled bc the pivot can spin infinitely
      .withSoftLimits(TURRET_MIN_DEG, TURRET_MAX_DEG) // Hard limit bc wiring prevents infinite spinning
      .withHardLimit(TURRET_MIN_DEG, TURRET_MAX_DEG) // Hard limit bc wiring prevents infinite spinning
      .withTelemetry("Turret", TelemetryVerbosity.HIGH) // Telemetry
      .withMOI(Meters.of(0.25), Pounds.of(4)) // MOI Calculation
      .withMechanismPositionConfig(robotToMechanism);

  Pivot m_turret = new Pivot(m_turretconfig);

  @Logged(importance = Logged.Importance.INFO)
  private boolean m_isTeleop = false;

  @Logged(importance = Logged.Importance.CRITICAL)
  private Angle turretTargetAngle = Degrees.of(0.0);
  @Logged(importance = Logged.Importance.CRITICAL)
  private Angle potentiometerAngle = Degrees.of(0.0);

  public TurretSubsystem(Telemetry telemetry) {
    this.telemetry = telemetry;
    seedTurretPosition();
  }

  private void seedTurretPosition() {
    this.potentiometerAngle = Degrees.of(-m_turretPotentiometer.get() + kTurretOffset);
    m_turretSMC.setEncoderPosition(this.potentiometerAngle);
  }

  @Override
  public void periodic() {
    this.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_turret.simIterate();
  }

  @Logged(importance = Logged.Importance.CRITICAL)
  public Angle getAngle() {
    return m_turret.getAngle();
  }

  public Command setAngleCommand(Angle angle) {
    Angle clampedAngle = clampToTurretLimits(angle);
    return runOnce(() -> {
      m_isTeleop = false;
      turretTargetAngle = clampedAngle;
    }).andThen(m_turret.runTo(clampedAngle, Degrees.of(0.1))).withName("TurretSetAngleCommand")
      .withTimeout(1.0);
  }

  public Command setAngleCommand(Angle angle, Angle tolerance) {
    Angle clampedAngle = clampToTurretLimits(angle);
    return runOnce(() -> {
      m_isTeleop = false;
      turretTargetAngle = clampedAngle;
    }).andThen(m_turret.runTo(clampedAngle, tolerance)).withName("TurretSetAngleWithToleranceCommand")
      .withTimeout(1.0);
  }

  public Command setAngleCommand(Supplier<Angle> angleSupplier) {
    return runOnce(() -> m_isTeleop = false)
      .andThen(m_turret.setAngle(() -> {
        Angle clampedAngle = clampToTurretLimits(angleSupplier.get());
        turretTargetAngle = clampedAngle;
        return clampedAngle;
      })).withName("TurretSetAngleSupplierCommand")
      .withTimeout(1.0);
  }

  public void setAngleDirect(Angle angle) {
    m_isTeleop = false;
    turretTargetAngle = Degrees.of(MathUtil.clamp(angle.in(Degrees), TURRET_MIN_DEG.in(Degrees), TURRET_MAX_DEG.in(Degrees)));
    m_turretSMC.setPosition(turretTargetAngle);
  }

  public Command setDutyCycleCommand(Supplier<Double> dutyCycleSupplier) {
    // Open-loop duty-cycle control for manual aiming.
    return runOnce(() -> turretTargetAngle = Degrees.of(0.0))
      .andThen(m_turret.set(dutyCycleSupplier))
      .withName("TurretSetDutyCycleCommand");
  }

  public Command setDutyCycleCommand(double dutyCycle) {
    // Open-loop duty-cycle control with a fixed value.
    return runOnce(() -> turretTargetAngle = Degrees.of(0.0))
      .andThen(m_turret.set(dutyCycle))
      .withName("TurretSetDutyCycleCommand");
  }

  public void setDutyCycleDirect(double dutyCycle) {
    // Direct motor call that bypasses the YAMS command wrappers.
    turretTargetAngle = Degrees.of(0.0);
    m_turretSMC.setDutyCycle(dutyCycle);
  }

  public Command sysIdCommand() {
    // Practice-field-only characterization helper.
    return m_turret.sysId(
                    Volts.of(4.0), // maximumVoltage
                    Volts.per(Second).of(0.5), // step
                    Seconds.of(8.0) // duration
    );
  }
  
  public void stopTurret() {
    m_turretSMC.stopClosedLoopController();
    m_turretSMC.setDutyCycle(0.0);
  }

  /**
   * Teleop controls
   *
   * @param aspeed duty-cycle request from the operator stick
   */
  public void teleop(double aspeed) {
    aspeed = MathUtil.applyDeadband(aspeed, STICK_DEADBAND);

    if (aspeed != 0.0) {
      m_isTeleop = true;
      turretTargetAngle = Degrees.of(0);
      m_turretSMC.setDutyCycle(aspeed * kTurretTeleopSpeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      this.stopTurret();
    }
  }

  private static Angle clampToTurretLimits(Angle angle) {
    return Degrees.of(
        MathUtil.clamp(angle.in(Degrees), TURRET_MIN_DEG.in(Degrees), TURRET_MAX_DEG.in(Degrees)));
  }

  private void updateTelemetry() {
    m_turret.updateTelemetry();
    // YAMS already publishes the turret mechanism angle, setpoint, and rotor state.
    telemetry.putNumber("Turret/TurretPotentiometer", -m_turretPotentiometer.get(), true);
    telemetry.putNumber("Turret/SeededTurretDeg", this.potentiometerAngle.in(Degrees), true);
  }

  // -- Commands -----------------------------------------------------------
  public Command seedTurretPositionCommand() {
    return runOnce(this::seedTurretPosition)
      .withName("SeedTurretPositionCommand");
  }

}
