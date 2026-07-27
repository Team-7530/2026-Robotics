package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.Pair;

import frc.lib.util.SystemHealthMonitor;
import frc.lib.util.SystemHealthMonitor.MotorHealthMonitor;

// Drives the main shooter flywheel.
@Logged
public class FlywheelSubsystem extends SubsystemBase {
  private static final CANBus kCANBus = CANBUS_FD;

  // CAN IDs
  private static final int FLYWHEEL_MASTER_ID = 60;
  private static final int FLYWHEEL_FOLLOWER_ID = 61;

  private static final double kFlywheelChainRatio = 24.0 / 24.0; // 24:24 ratio
  private static final double kFlywheelGearboxRatio = 1.0; // 1:1
  // Flywheel control tuning (Velocity closed-loop)
  private static final double FLYWHEEL_kS = 0.1;
  private static final double FLYWHEEL_kV = 0.12;
  private static final double FLYWHEEL_kP = 0.5;
  private static final double FLYWHEEL_kI = 0.0;
  private static final double FLYWHEEL_kD = 0.0;
  private static final AngularVelocity FLYWHEEL_kMaxV = RotationsPerSecond.of(200.0);
  private static final AngularAcceleration FLYWHEEL_kMaxA = RotationsPerSecondPerSecond.of(100);
  private static final AngularVelocity READY_TOLERANCE = RPM.of(250);

  // Motor health monitoring thresholds
  private static final double FLYWHEEL_STALL_THRESHOLD = 80.0;  // Kraken X60 warning at 80A

  private final Distance flywheelDiameter = Inches.of(4);
  private final Mass flywheelMass = Pounds.of(1);

  private static final double kFlywheelTeleopSpeed = 0.8;

  // TalonFX hardware instances
  private final TalonFX m_flywheelMasterMotor = new TalonFX(FLYWHEEL_MASTER_ID, kCANBus);
  private final TalonFX m_flywheelFollowerMotor = new TalonFX(FLYWHEEL_FOLLOWER_ID, kCANBus);

  // Health monitoring (owned by this subsystem, not central monitoring)
  private final MotorHealthMonitor masterMotorHealth;

  private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
      // PID Constants
      .withClosedLoopController(FLYWHEEL_kP, FLYWHEEL_kI, FLYWHEEL_kD)
      .withSimClosedLoopController(FLYWHEEL_kP, FLYWHEEL_kI, FLYWHEEL_kD)
      .withTrapezoidalProfile(FLYWHEEL_kMaxV, FLYWHEEL_kMaxA)
      // Gearing from the motor rotor to final shaft.
      // For example gearbox(3,4) is the same as gearbox("3:1","4:1")
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(kFlywheelChainRatio, kFlywheelGearboxRatio)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      // Telemetry name and verbosity level
      .withTelemetry("FlywheelMotor", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(FLYWHEEL_kS, FLYWHEEL_kV, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(FLYWHEEL_kS, FLYWHEEL_kV, 0))
      // Motor properties to prevent over currenting.
      .withStatorCurrentLimit(Amps.of(40))
      // Power Optimization
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      // Mass of the flywheel.
      .withMomentOfInertia(flywheelDiameter, flywheelMass)
      // Follower Motors
      .withFollowers(Pair.of(m_flywheelFollowerMotor, true))
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController m_flywheelSMC = new TalonFXWrapper(m_flywheelMasterMotor, DCMotor.getKrakenX60Foc(1), smc_config);

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(1.0),
          Volts.of(4.0),
          Seconds.of(10.0),
          state -> SignalLogger.writeString("FlywheelSysIdState", state.toString())),
      new SysIdRoutine.Mechanism(
          m_flywheelSMC::setVoltage,
          log -> log.motor("FlywheelMotor")
              .voltage(m_flywheelSMC.getVoltage())
              .angularPosition(m_flywheelSMC.getMechanismPosition())
              .angularVelocity(m_flywheelSMC.getMechanismVelocity())
              .current(m_flywheelSMC.getStatorCurrent()),
          this,
          "FlywheelMotor"));

  // Construct YAMS FlyWheel config & mechanism (use master controller for mech config)
  private final FlyWheelConfig m_flywheelConfig = new FlyWheelConfig()
      .withDiameter(flywheelDiameter)
      .withTelemetry("Flywheel", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
      .withSpeedometerSimulation(FLYWHEEL_kMaxV);

  private final FlyWheel m_flywheel = new FlyWheel(m_flywheelConfig, m_flywheelSMC);
  
  @Logged(importance = Logged.Importance.DEBUG)
  private boolean m_isTeleop = false;

  /**
   * Creates a new FlywheelSubsystem.
   * 
   * @param healthMonitor the system health monitor to register with (pass null to skip registration)
   */
  public FlywheelSubsystem(SystemHealthMonitor healthMonitor) {
    this.masterMotorHealth = healthMonitor.createMotorHealthMonitor(m_flywheelMasterMotor,
                                                                    "Flywheel",
                                                                    FLYWHEEL_STALL_THRESHOLD);
  }

  @Override
  public void periodic() {
    masterMotorHealth.update();
    this.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_flywheel.simIterate();
  }

  // YAMS Flywheel API wrappers
  @Logged(importance = Logged.Importance.CRITICAL)
  public AngularVelocity getVelocity() {
    return m_flywheel.getSpeed();
  }

  public void setVelocityDirect(AngularVelocity velocity) {
    m_flywheelSMC.setVelocity(velocity);
  }

  public void setDutyCycleDirect(double dutyCycle) {
    m_flywheelSMC.setDutyCycle(dutyCycle);
  }

  public boolean isAtSpeed(AngularVelocity targetVelocity) {
    return isAtSpeed(targetVelocity, READY_TOLERANCE);
  }

  public boolean isAtSpeed(AngularVelocity targetVelocity, AngularVelocity tolerance) {
    return getVelocity().isNear(targetVelocity, tolerance);
  }

  public Command setVelocityCommand(AngularVelocity speed) {
    return m_flywheel.run(speed)
      .withName("FlywheelSetVelocityCommand");
  }

  public Command setVelocityCommand(Supplier<AngularVelocity> speed) {
    return m_flywheel.run(speed)
      .withName("FlywheelSetVelocitySupplierCommand");
  }

  public Command setDutyCycleCommand(double dutyCycle) {
    return m_flywheel.set(dutyCycle)
      .withName("FlywheelSetDutyCycleCommand");
  }

  public Command setDutyCycleCommand(Supplier<Double> dutyCycle) {
    return m_flywheel.set(dutyCycle)
      .withName("FlywheelSetDutyCycleSupplierCommand");
  }

  public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction)
      .withName("FlywheelSysIdQuasistatic" + direction.name());
  }

  public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction)
      .withName("FlywheelSysIdDynamic" + direction.name());
  }

  /** Sets motors to constants intake speed */
  public Command flywheelStartCommand(AngularVelocity velocity) {
    return setVelocityCommand(velocity)
      .withName("FlywheelStartCommand")
      .withTimeout(0.2);
  }

  /** Sets motors to constants intake speed */
  public Command flywheelStartCommand(Supplier<AngularVelocity> velocity) {
    return setVelocityCommand(velocity)
      .withName("FlywheelStartCommand")
      .withTimeout(0.2);
  }
  public Command flywheelStopCommand() {
    return runOnce(this::flywheelStop).withName("FlywheelStopCommand");
  }

  /**
   * Run the flywheel backward for a short duration to clear jams.
   */
  public Command flywheelUnstuckCommand() {
    return setDutyCycleCommand(-0.2)
        .withName("FlywheelUnstuckCommand")
        .withTimeout(0.5)
        .finallyDo(interrupted -> flywheelStop());
  }

  /** Stops the flywheel motor immediately (open-loop stop). */
  public void flywheelStop() {
    m_flywheelSMC.stopClosedLoopController();
    m_flywheelSMC.setDutyCycle(0.0);
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
      m_flywheelSMC.setDutyCycle(aspeed * kFlywheelTeleopSpeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      this.flywheelStop();
    }
  }

  private void updateTelemetry() {
    m_flywheel.updateTelemetry();
  }
}
