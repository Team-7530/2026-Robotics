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

import frc.lib.util.SystemHealthMonitor;
import frc.lib.util.SystemHealthMonitor.MotorHealthMonitor;

// Feeds Fuel from the hopper into the shooter flywheel.
@Logged
public class FeederSubsystem extends SubsystemBase {
  private static final CANBus kCANBus = CANBUS_FD;

  // CAN IDs
  private static final int FEEDERMOTOR_ID = 50;

  private static final double kFeederChainRatio = 40.0 / 32.0; // 40:32
  private static final double kFeederGearboxRatio = 4.0; // 4:1

  // Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself
  private static final double FEEDER_KS = 0.0; // Static feedforward gain
  private static final double FEEDER_KP = 8.0; // error of 1 rps results in 8 amps output
  private static final double FEEDER_KI = 0.2; // error of 1 rps incr by 0.2 amps per sec
  private static final double FEEDER_KD = 0.001; // 1000 rps^2 incr 1 amp output
  private static final AngularVelocity FEEDER_kMaxV = RPM.of(5000);
  private static final AngularAcceleration FEEDER_kMaxA = RotationsPerSecondPerSecond.of(2500);

  private final Distance flywheelDiameter = Inches.of(4);
  private final Mass flywheelMass = Pounds.of(0.5);

  private static final AngularVelocity feederVelocity = RPM.of(3000);
  private static final AngularVelocity feederUnstuckVelocity = RPM.of(-2000);
    
  private static final double kFeederTeleopFactor = 0.8;

  // Motor health monitoring threshold
  private static final double FEEDER_STALL_THRESHOLD = 80.0;  // Kraken X60 warning at 80A

  // TalonFX hardware instance (kept for wrapper)
  private final TalonFX m_feederMotor = new TalonFX(FEEDERMOTOR_ID, kCANBus);

  // YAMS controller and mechanism (initialized at declaration to match FlywheelSubsystem style)
  private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
      // PID Constants
      .withClosedLoopController(FEEDER_KP, FEEDER_KI, FEEDER_KD)
      .withSimClosedLoopController(FEEDER_KP, FEEDER_KI, FEEDER_KD)
      .withTrapezoidalProfile(FEEDER_kMaxV, FEEDER_kMaxA)
      // Gearing from the motor rotor to final shaft.
      // For example gearbox(3,4) is the same as gearbox("3:1","4:1")
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(kFeederChainRatio, kFeederGearboxRatio)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      // Telemetry name and verbosity level
      .withTelemetry("FeederMotor", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(FEEDER_KS, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(FEEDER_KS, 0, 0))
      // Motor properties to prevent over currenting.
      .withStatorCurrentLimit(Amps.of(40))
      // Power Optimization
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      // Mass of the flywheel.
      .withMomentOfInertia(flywheelDiameter, flywheelMass)
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController m_feederSMC = new TalonFXWrapper(m_feederMotor, DCMotor.getKrakenX60Foc(1), smc_config);

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(1.0),
          Volts.of(4.0),
          Seconds.of(10.0),
          state -> SignalLogger.writeString("FeederSysIdState", state.toString())),
      new SysIdRoutine.Mechanism(
          m_feederSMC::setVoltage,
          log -> log.motor("FeederMotor")
              .voltage(m_feederSMC.getVoltage())
              .angularPosition(m_feederSMC.getMechanismPosition())
              .angularVelocity(m_feederSMC.getMechanismVelocity())
              .current(m_feederSMC.getStatorCurrent()),
          this,
          "FeederMotor"));

  private final FlyWheelConfig m_feederConfig = new FlyWheelConfig()
      .withDiameter(flywheelDiameter)
      .withTelemetry("Feeder", SmartMotorControllerConfig.TelemetryVerbosity.LOW)
      .withSpeedometerSimulation(FEEDER_kMaxV);

  private final FlyWheel m_feeder = new FlyWheel(m_feederConfig, m_feederSMC);

  @Logged(importance = Logged.Importance.DEBUG)
  private boolean m_isTeleop = false;

  // Health monitoring (owned by this subsystem, not central monitoring)
  private final MotorHealthMonitor motorHealth;

  /**
   * Creates a new FeederSubsystem.
   * 
   * @param healthMonitor the system health monitor to register with (pass null to skip registration)
   */
  public FeederSubsystem(SystemHealthMonitor healthMonitor) {
    this.motorHealth = healthMonitor.createMotorHealthMonitor(m_feederMotor,
                                                              "Feeder",
                                                              FEEDER_STALL_THRESHOLD);
  }

  @Override
  public void periodic() {
    motorHealth.update();
    this.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_feeder.simIterate();
  }

  // YAMS Flywheel API wrappers
  @Logged(importance = Logged.Importance.CRITICAL)
  public AngularVelocity getVelocity() {
    return m_feeder.getSpeed();
  }

  public void setVelocityDirect(AngularVelocity velocity) {
    m_feederSMC.setVelocity(velocity);
  }

  public void setDutyCycleDirect(double dutyCycle) {
    m_feederSMC.setDutyCycle(dutyCycle);
  }

  public Command setVelocityCommand(AngularVelocity speed) {
    return m_feeder.run(speed)
      .withName("FeederSetVelocityCommand");
  }

  public Command setVelocityCommand(Supplier<AngularVelocity> speed) {
    return m_feeder.run(speed)
      .withName("FeederSetVelocitySupplierCommand");
  }

  public Command setDutyCycleCommand(double duty) {
    return m_feeder.set(duty)
      .withName("FeederSetDutyCycleCommand");
  }

  public Command setDutyCycleCommand(Supplier<Double> dutyCycle) {
    return m_feeder.set(dutyCycle)
      .withName("FeederSetDutyCycleSupplierCommand");
  }

  public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction)
      .withName("FeederSysIdQuasistatic" + direction.name());
  }

  public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction)
      .withName("FeederSysIdDynamic" + direction.name());
  }

  /** Sets motors to constants intake speed */
  public Command feederStartCommand() {
    return setVelocityCommand(feederVelocity)
    .withName("FeederStartCommand")
    .withTimeout(1.0);
  }

  public Command feederStopCommand() {
    // scheduleable command that immediately cuts power
    return runOnce(this::feederStop).withName("FeederStopCommand");
  }

  public Command feederUnstuckCommand() {
    // run the velocity control in reverse to clear jams (negative RPM) for 500 ms,
    // then guarantee a stop via finallyDo (even if interrupted)
    return setVelocityCommand(feederUnstuckVelocity)
      .withName("FeederUnstuckCommand")
      .withTimeout(0.5)
      .finallyDo(interrupted -> feederStop());
  }

  /** Stops the feeder motor immediately (open-loop stop). */
  public void feederStop() {
    m_feederSMC.stopClosedLoopController();
    m_feederSMC.setDutyCycle(0.0);
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
      m_feederSMC.setDutyCycle(aspeed * kFeederTeleopFactor);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      this.feederStop();
    }
  }

  private void updateTelemetry() {
    m_feeder.updateTelemetry();
  }
}
