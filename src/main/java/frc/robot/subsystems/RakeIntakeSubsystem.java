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

// Powers the rake intake rollers.
@Logged
public class RakeIntakeSubsystem extends SubsystemBase {
  private static final CANBus kCANBus = CANBUS_FD;

  // CAN IDs
  private static final int RAKEINTAKEMOTOR_ID = 32;

  private static final double kRakeIntakeChainRatio = 1.0; // 1:1
  private static final double kRakeIntakeGearboxRatio = 3.0; // 3:1

  // Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself
  private static final double RAKEINTAKE_KS = 0.0; // Static feedforward gain
  private static final double RAKEINTAKE_KP = 8.0; // error of 1 rps results in 8 amps output
  private static final double RAKEINTAKE_KI = 0.2; // error of 1 rps incr by 0.2 amps per sec
  private static final double RAKEINTAKE_KD = 0.001; // 1000 rps^2 incr 1 amp output
  private static final AngularVelocity RAKEINTAKE_kMaxV = RPM.of(5000);
  private static final AngularAcceleration RAKEINTAKE_kMaxA = RotationsPerSecondPerSecond.of(2500);

  private static final Distance flywheelDiameter = Inches.of(1);
  private static final Mass flywheelMass = Pounds.of(0.5);

  private static final AngularVelocity rakeIntakeVelocity = RPM.of(3000);
  private static final AngularVelocity rakeIntakeUnstuckVelocity = RPM.of(-2000);

  private static final double kRakeIntakeTeleopFactor = 0.8;

  // TalonFX hardware instance (kept for wrapper)
  private final TalonFX m_rakeIntakeMotor = new TalonFX(RAKEINTAKEMOTOR_ID, kCANBus);

  // YAMS controller and mechanism (initialized at declaration to match FlywheelSubsystem style)
  private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // PID Constants
      .withClosedLoopController(RAKEINTAKE_KP, RAKEINTAKE_KI, RAKEINTAKE_KD, RAKEINTAKE_kMaxV, RAKEINTAKE_kMaxA)
      .withSimClosedLoopController(RAKEINTAKE_KP, RAKEINTAKE_KI, RAKEINTAKE_KD, RAKEINTAKE_kMaxV, RAKEINTAKE_kMaxA)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(RAKEINTAKE_KS, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(RAKEINTAKE_KS, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("RakeIntakeMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // For example gearbox(3,4) is the same as gearbox("3:1","4:1")
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(kRakeIntakeChainRatio, kRakeIntakeGearboxRatio)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  private final SmartMotorController m_rakeIntakeSMC = new TalonFXWrapper(m_rakeIntakeMotor, DCMotor.getKrakenX60Foc(1), smc_config);

  private final FlyWheelConfig m_rakeIntakeConfig = new FlyWheelConfig(m_rakeIntakeSMC)
      // Diameter of the flywheel.
      .withDiameter(flywheelDiameter)
      // Mass of the flywheel.
      .withMass(flywheelMass)
      // Maximum speed of the shooter.
      .withSoftLimit(RAKEINTAKE_kMaxV.unaryMinus(), RAKEINTAKE_kMaxV)
      // Telemetry name and verbosity for the arm.
      .withTelemetry("RakeIntake", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      .withSpeedometerSimulation(RAKEINTAKE_kMaxV);

  private final FlyWheel m_rakeIntake = new FlyWheel(m_rakeIntakeConfig);

  @Logged(importance = Logged.Importance.INFO)
  private boolean m_isTeleop = false;

  public RakeIntakeSubsystem() {}

  @Override
  public void periodic() {
    this.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_rakeIntake.simIterate();
  }

  // YAMS Flywheel API wrappers
  @Logged(importance = Logged.Importance.CRITICAL)
  public AngularVelocity getVelocity() {
    return m_rakeIntake.getSpeed();
  }

  public void setVelocityDirect(AngularVelocity velocity) {
    m_rakeIntakeSMC.setVelocity(velocity);
  }

  public void setDutyCycleDirect(double dutyCycle) {
    m_rakeIntakeSMC.setDutyCycle(dutyCycle);
  }

  public Command setVelocityCommand(AngularVelocity speed) {
    return m_rakeIntake.setSpeed(speed);
  }

  public Command setVelocityCommand(Supplier<AngularVelocity> speed) {
    return m_rakeIntake.setSpeed(speed);
  }

  public Command setDutyCycleCommand(double duty) {
    return m_rakeIntake.set(duty);
  }

  public Command setDutyCycleCommand(Supplier<Double> dutyCycle) {
    return m_rakeIntake.set(dutyCycle);
  }

  public Command sysIdCommand() {
    // run on the practice field to capture plant data
    return m_rakeIntake.sysId(Volts.of(10), Volts.of(1).per(Seconds), Seconds.of(5));
  }

  /** Sets motors to constants intake speed */
  public Command rakeIntakeStartCommand() {
    return setVelocityCommand(rakeIntakeVelocity)
      .withName("RakeIntakeStartCommand")
      .withTimeout(0.2);
  }

  public Command rakeIntakeStopCommand() {
    // Immediate stop command.
    return runOnce(this::rakeIntakeStop)
      .withName("RakeIntakeStopCommand");
  }

  public Command rakeIntakeUnstuckCommand() {
    // spin backward to clear jams
    return setVelocityCommand(rakeIntakeUnstuckVelocity)
      .withName("RakeIntakeUnstuckCommand")
      .withTimeout(2.0)
      .finallyDo(interrupted -> rakeIntakeStop());
  }

  /** Stops the rake intake motor immediately (open-loop stop). */
  public void rakeIntakeStop() {
    m_rakeIntakeSMC.stopClosedLoopController();
    m_rakeIntakeSMC.setDutyCycle(0.0);
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
      m_rakeIntakeSMC.setDutyCycle(aspeed * kRakeIntakeTeleopFactor);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      this.rakeIntakeStop();
    }
  }
  
  private void updateTelemetry() {
    m_rakeIntake.updateTelemetry();
  }

}
