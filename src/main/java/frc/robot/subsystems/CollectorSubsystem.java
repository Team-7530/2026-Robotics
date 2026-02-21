package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import frc.robot.Telemetry;
import java.util.function.Supplier;

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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Subsystem for rollers inside of hopper
public class CollectorSubsystem extends SubsystemBase {

  public static final CANBus kCANBus = CANBUS_FD;

  // CAN IDs
  public static final int COLLECTORMOTOR_ID = 40;

  public static final double kCollectorChainRatio = 1.0; // 1:1
  public static final double kCollectorGearboxRatio = 16.0; // 16:1

  // Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself
  public static final double COLLECTOR_KS = 0.0; // Static feedforward gain
  public static final double COLLECTOR_KP = 8.0; // error of 1 rps results in 8 amps output
  public static final double COLLECTOR_KI = 0.2; // error of 1 rps incr by 0.2 amps per sec
  public static final double COLLECTOR_KD = 0.001; // 1000 rps^2 incr 1 amp output
  public static final AngularVelocity COLLECTOR_kMaxV = RPM.of(5000);
  public static final AngularAcceleration COLLECTOR_kMaxA = RotationsPerSecondPerSecond.of(2500);

  private static final Distance flywheelDiameter = Inches.of(1);
  private static final Mass flywheelMass = Pounds.of(0.5);

  public static final AngularVelocity collectorVelocity = RPM.of(3000);
  public static final AngularVelocity collectorUnstuckVelocity = RPM.of(-2000);
    
  private static final double kCollectorTeleopFactor = 0.8;

  // TalonFX hardware instance (kept for wrapper)
  private final TalonFX m_collectorMotor = new TalonFX(COLLECTORMOTOR_ID, kCANBus);

  // YAMS controller and mechanism (initialized at declaration to match FlywheelSubsystem style)
  private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // PID Constants
      .withClosedLoopController(COLLECTOR_KP, COLLECTOR_KI, COLLECTOR_KD, COLLECTOR_kMaxV, COLLECTOR_kMaxA)
      .withSimClosedLoopController(COLLECTOR_KP, COLLECTOR_KI, COLLECTOR_KD, COLLECTOR_kMaxV, COLLECTOR_kMaxA)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(COLLECTOR_KS, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(COLLECTOR_KS, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("CollectorMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // For example gearbox(3,4) is the same as gearbox("3:1","4:1")
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(kCollectorChainRatio, kCollectorGearboxRatio)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  private final SmartMotorController m_collectorSMC = new TalonFXWrapper(m_collectorMotor, DCMotor.getKrakenX60Foc(1), smc_config);

  private final FlyWheelConfig m_collectorConfig = new FlyWheelConfig(m_collectorSMC)
      // Diameter of the flywheel.
      .withDiameter(flywheelDiameter)
      // Mass of the flywheel.
      .withMass(flywheelMass)
      // Maximum speed of the shooter.
      .withUpperSoftLimit(COLLECTOR_kMaxV)
      // Telemetry name and verbosity for the arm.
      .withTelemetry("CollectorMech", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      .withSpeedometerSimulation(COLLECTOR_kMaxV);

  private final FlyWheel m_collector = new FlyWheel(m_collectorConfig);

  private boolean m_isTeleop = false;
  private final Telemetry telemetry;

  public CollectorSubsystem(Telemetry telemetry) {
    this.telemetry = telemetry;
  }

  @Override
  public void periodic() {
    this.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_collector.simIterate();
  }

  // YAMS Flywheel API wrappers
  @Logged
  public AngularVelocity getVelocity() {
    return m_collector.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return m_collector.setSpeed(speed);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return m_collector.setSpeed(speed);
  }

  public Command setDutyCycle(double duty) {
    return m_collector.set(duty);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return m_collector.set(dutyCycle);
  }

  public Command sysId() {
    return m_collector.sysId(Volts.of(10), Volts.of(1).per(Seconds), Seconds.of(5));
  }

  public Command setRPM(LinearVelocity speed) {
    return m_collector.setSpeed(RotationsPerSecond.of(speed.in(MetersPerSecond) / flywheelDiameter.times(Math.PI).in(Meters)));
  }

  public void setRPMDirect(LinearVelocity speed) {
    // directly set the motor velocity on the master based on linear speed -> rotational
    m_collectorSMC.setVelocity(RotationsPerSecond.of(speed.in(MetersPerSecond) / flywheelDiameter.times(Math.PI).in(Meters)));
  }

  /** Sets motors to constants intake speed */
  public Command collectorStartCommand() {
    return setVelocity(collectorVelocity).withName("CollectorStartCommand");
  }

  public Command collectorStopCommand() {
    return runOnce(this::collectorStop).withName("CollectorStopCommand");
  }

  public Command collectorUnstuckCommand() {
      return setVelocity(collectorUnstuckVelocity)
        .withName("CollectorUnstuckCommand")    
        .withTimeout(5.0)
        .finallyDo(() -> collectorStop());
  }

  /** Stops the collector motor immediately (open-loop stop). */
  public void collectorStop() {
    m_collectorSMC.stopClosedLoopController();
    m_collectorSMC.setDutyCycle(0.0);
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
      m_collectorSMC.setDutyCycle(aspeed * kCollectorTeleopFactor);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      this.collectorStop();
    }
  }

  private void updateTelemetry() 
  {
    m_collector.updateTelemetry();
    try {
      telemetry.putNumber("CollectorIntake RPS", getVelocity().in(RotationsPerSecond));
    } catch (Exception e) {
      telemetry.putNumber("CollectorIntake RPS", 0.0);
    }
  }

}


