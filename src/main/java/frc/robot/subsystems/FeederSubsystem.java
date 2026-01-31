package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.STICK_DEADBAND;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Subsystem for rollers inside of hopper

public class FeederSubsystem extends SubsystemBase {

  public static final CANBus CANBUS = new CANBus("CANFD");

  // CAN IDs
  public static final int FEEDERMOTOR_ID = 65;

  public static final double kFeederChainRatio = 24.0 / 10.0; // 24:10
  public static final double kFeederGearboxRatio = 1.0; // 1:1
  public static final double kFeederGearRatio = kFeederChainRatio * kFeederGearboxRatio;

  // Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself
  public static final double FEEDER_KS = 0.0; // Static feedforward gain
  public static final double FEEDER_KP = 8.0; // error of 1 rps results in 8 amps output
  public static final double FEEDER_KI = 0.2; // error of 1 rps incr by 0.2 amps per sec
  public static final double FEEDER_KD = 0.001; // 1000 rps^2 incr 1 amp output
  public static final AngularVelocity FEEDER_kMaxV = RPM.of(5000);
  public static final AngularAcceleration FEEDER_kMaxA = RotationsPerSecondPerSecond.of(2500);

  private final Distance flywheelDiameter = Inches.of(4);
  private final Mass flywheelMass = Pounds.of(1);

  public static final AngularVelocity feederVelocity = RPM.of(3000);
  public static final AngularVelocity feederUnstuckVelocity = RPM.of(-2000);
    
  // TalonFX hardware instance (kept for wrapper)
  private final TalonFX m_feederTalon = new TalonFX(FEEDERMOTOR_ID, CANBUS);

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
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  private final SmartMotorController m_feederMotor = new TalonFXWrapper(m_feederTalon, DCMotor.getKrakenX60Foc(1), smc_config);

  private final FlyWheelConfig m_feederConfig = new FlyWheelConfig(m_feederMotor)
      // Diameter of the flywheel.
      .withDiameter(flywheelDiameter)
      // Mass of the flywheel.
      .withMass(flywheelMass)
      // Maximum speed of the shooter.
      .withUpperSoftLimit(FEEDER_kMaxV)
      // Telemetry name and verbosity for the arm.
      .withTelemetry("FeederMech", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      .withSpeedometerSimulation(FEEDER_kMaxV);

  private final FlyWheel m_feederFlywheel = new FlyWheel(m_feederConfig);

  private boolean m_isTeleop = false;

  public FeederSubsystem() {
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    m_feederFlywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_feederFlywheel.simIterate();
  }

  // YAMS Flywheel API wrappers
  public AngularVelocity getVelocity() {
    return m_feederFlywheel.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return m_feederFlywheel.setSpeed(speed);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return m_feederFlywheel.setSpeed(speed);
  }

  public Command setDutyCycle(double duty) {
    return m_feederFlywheel.set(duty);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return m_feederFlywheel.set(dutyCycle);
  }

  public Command sysId() {
    return m_feederFlywheel.sysId(Volts.of(10), Volts.of(1).per(Seconds), Seconds.of(5));
  }

  public Command setRPM(LinearVelocity speed) {
    return m_feederFlywheel.setSpeed(RotationsPerSecond.of(speed.in(MetersPerSecond) / flywheelDiameter.times(Math.PI).in(Meters)));
  }

  public void setRPMDirect(LinearVelocity speed) {
    // directly set the motor velocity on the master based on linear speed -> rotational
    m_feederMotor.setVelocity(RotationsPerSecond.of(speed.in(MetersPerSecond) / flywheelDiameter.times(Math.PI).in(Meters)));
  }

  /** Sets motors to constants intake speed */
  public Command feederStartCommand() {
      return setVelocity(feederVelocity);
  }

  public Command feederStopCommand() {
      return setVelocity(RPM.of(0.0));
  }

  public Command feederUnstuckCommand() {
      return setVelocity(feederUnstuckVelocity);
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
      setDutyCycle(aspeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      setDutyCycle(0.0);
    }
  }
  // -- SmartDashboard ----------------------------------------------------
  private void updateSmartDashboard() {
    try {
      SmartDashboard.putNumber("FeederIntake RPS", getVelocity().in(RotationsPerSecond));
    } catch (Exception e) {
      SmartDashboard.putNumber("FeederIntake RPS", 0.0);
    }
  }

}


