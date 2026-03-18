package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

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
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Pair;

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

  private final Distance flywheelDiameter = Inches.of(4);
  private final Mass flywheelMass = Pounds.of(1);

  private static final double kFlywheelTeleopSpeed = 0.8;

  // TalonFX hardware instances
  private final TalonFX m_flywheelMasterMotor = new TalonFX(FLYWHEEL_MASTER_ID, kCANBus);
  private final TalonFX m_flywheelFollowerMotor = new TalonFX(FLYWHEEL_FOLLOWER_ID, kCANBus);

  private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // PID Constants
      .withClosedLoopController(FLYWHEEL_kP, FLYWHEEL_kI, FLYWHEEL_kD, FLYWHEEL_kMaxV, FLYWHEEL_kMaxA)
      .withSimClosedLoopController(FLYWHEEL_kP, FLYWHEEL_kI, FLYWHEEL_kD, FLYWHEEL_kMaxV, FLYWHEEL_kMaxA)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(FLYWHEEL_kS, FLYWHEEL_kV, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(FLYWHEEL_kS, FLYWHEEL_kV, 0))
      // Telemetry name and verbosity level
      .withTelemetry("FlywheelMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // For example gearbox(3,4) is the same as gearbox("3:1","4:1")
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(kFlywheelChainRatio, kFlywheelGearboxRatio)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      // Follower Motors
      .withFollowers(Pair.of(m_flywheelFollowerMotor, true));

  private final SmartMotorController m_flywheelSMC = new TalonFXWrapper(m_flywheelMasterMotor, DCMotor.getKrakenX60Foc(1), smc_config);

  // Construct YAMS FlyWheel config & mechanism (use master controller for mech config)
  private final FlyWheelConfig m_flywheelConfig = new FlyWheelConfig(m_flywheelSMC)
      // Diameter of the flywheel.
      .withDiameter(flywheelDiameter)
      // Mass of the flywheel.
      .withMass(flywheelMass)
      // Maximum speed of the shooter.
      .withSoftLimit(FLYWHEEL_kMaxV.unaryMinus(), FLYWHEEL_kMaxV)
      // Telemetry name and verbosity for the arm.
      .withTelemetry("Flywheel", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      .withSpeedometerSimulation(FLYWHEEL_kMaxV);

  private final FlyWheel m_flywheel = new FlyWheel(m_flywheelConfig);
  
  @Logged(importance = Logged.Importance.INFO)
  private boolean m_isTeleop = false;

  public FlywheelSubsystem() {}

  @Override
  public void periodic() {
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
    return m_flywheel.setSpeed(speed).withName("FlywheelSetVelocityCommand");
  }

  public Command setVelocityCommand(Supplier<AngularVelocity> speed) {
    return m_flywheel.setSpeed(speed).withName("FlywheelSetVelocitySupplierCommand");
  }

  public Command setDutyCycleCommand(double dutyCycle) {
    return m_flywheel.set(dutyCycle).withName("FlywheelSetDutyCycleCommand");
  }

  public Command setDutyCycleCommand(Supplier<Double> dutyCycle) {
    return m_flywheel.set(dutyCycle).withName("FlywheelSetDutyCycleSupplierCommand");
  }

  public Command sysIdCommand() {
    return m_flywheel.sysId(Volts.of(10), Volts.of(1).per(Seconds), Seconds.of(5));
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
