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
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Subsystem for feeder flywheel to the shooter flywheel
@Logged
public class FeederSubsystem extends SubsystemBase {

  public static final CANBus kCANBus = CANBUS_FD;

  // CAN IDs
  public static final int FEEDERMOTOR_ID = 50;

  public static final double kFeederChainRatio = 40.0 / 32.0; // 40:32
  public static final double kFeederGearboxRatio = 4.0; // 4:1

  // Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself
  public static final double FEEDER_KS = 0.0; // Static feedforward gain
  public static final double FEEDER_KP = 8.0; // error of 1 rps results in 8 amps output
  public static final double FEEDER_KI = 0.2; // error of 1 rps incr by 0.2 amps per sec
  public static final double FEEDER_KD = 0.001; // 1000 rps^2 incr 1 amp output
  public static final AngularVelocity FEEDER_kMaxV = RPM.of(5000);
  public static final AngularAcceleration FEEDER_kMaxA = RotationsPerSecondPerSecond.of(2500);

  private final Distance flywheelDiameter = Inches.of(4);
  private final Mass flywheelMass = Pounds.of(0.5);

  public static final AngularVelocity feederVelocity = RPM.of(3000);
  public static final AngularVelocity feederUnstuckVelocity = RPM.of(-2000);
    
  private static final double kFeederTeleopFactor = 0.8;

  // TalonFX hardware instance (kept for wrapper)
  private final TalonFX m_feederMotor = new TalonFX(FEEDERMOTOR_ID, kCANBus);

  // YAMS controller and mechanism (initialized at declaration to match FlywheelSubsystem style)
  private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // PID Constants
      .withClosedLoopController(FEEDER_KP, FEEDER_KI, FEEDER_KD, FEEDER_kMaxV, FEEDER_kMaxA)
      .withSimClosedLoopController(FEEDER_KP, FEEDER_KI, FEEDER_KD, FEEDER_kMaxV, FEEDER_kMaxA)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(FEEDER_KS, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(FEEDER_KS, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("FeederMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // For example gearbox(3,4) is the same as gearbox("3:1","4:1")
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(kFeederChainRatio, kFeederGearboxRatio)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  private final SmartMotorController m_feederSMC = new TalonFXWrapper(m_feederMotor, DCMotor.getKrakenX60Foc(1), smc_config);

  private final FlyWheelConfig m_feederConfig = new FlyWheelConfig(m_feederSMC)
      // Diameter of the flywheel.
      .withDiameter(flywheelDiameter)
      // Mass of the flywheel.
      .withMass(flywheelMass)
      // Maximum speed of the shooter.
      .withSoftLimit(FEEDER_kMaxV.unaryMinus(), FEEDER_kMaxV)
      // Telemetry name and verbosity for the arm.
      .withTelemetry("Feeder", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      .withSpeedometerSimulation(FEEDER_kMaxV);

  private final FlyWheel m_feeder = new FlyWheel(m_feederConfig);

  @Logged
  private boolean m_isTeleop = false;
  private final Telemetry telemetry;

  public FeederSubsystem(Telemetry telemetry) {
    this.telemetry = telemetry; 
  }

  @Override
  public void periodic() {
    this.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_feeder.simIterate();
  }

  // YAMS Flywheel API wrappers
  @Logged
  public AngularVelocity getVelocity() {
    return m_feeder.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return m_feeder.setSpeed(speed).withName("FeederSetVelocityCommand");
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return m_feeder.setSpeed(speed).withName("FeederSetVelocitySupplierCommand");
  }

  public Command setDutyCycle(double duty) {
    return m_feeder.set(duty).withName("FeederSetDutyCycleCommand");
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return m_feeder.set(dutyCycle).withName("FeederSetDutyCycleSupplierCommand");
  }

  public Command sysIdCommand() {
    // run during practice to log system identification data
    return m_feeder.sysId(Volts.of(10), Volts.of(1).per(Seconds), Seconds.of(5));
  }

  /** Sets motors to constants intake speed */
  public Command feederStartCommand() {
    return setVelocity(feederVelocity)
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
    return setVelocity(feederUnstuckVelocity)
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
   * @param aspeed a double that sets the arm speed during teleop
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

  private void updateTelemetry() 
  {
    m_feeder.updateTelemetry();
    double velocityRpm = getVelocity().in(RPM);
    telemetry.putNumber("Feeder/VelocityRPM", Double.isFinite(velocityRpm) ? velocityRpm : 0.0, true);
  }

}

