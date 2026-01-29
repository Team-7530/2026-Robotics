package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.STICK_DEADBAND;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {

  public static final CANBus CANBUS = new CANBus("CANFD");

  // Placeholder CAN IDs - update to match your wiring
  public static final int FLYWHEEL_MASTER_ID = 61;
  public static final int FLYWHEEL_FOLLOWER_ID = 62;

  // Flywheel tuning
  public static final double FLYWHEEL_MAX_RPM = 6000.0; // adjust to your flywheel
  public static final double FLYWHEEL_DEFAULT_RPM = 3000.0;

  public static final double kFlywheelChainRatio = 1.0 / 1.0;
  public static final double kFlywheelGearboxRatio = 1.0; // 1:1
  public static final double kFlywheelGearRatio = kFlywheelChainRatio * kFlywheelGearboxRatio;

  // Flywheel control tuning (Velocity closed-loop)
  public static final double FLYWHEEL_kS = 0.0;
  public static final double FLYWHEEL_kP = 0.1;
  public static final double FLYWHEEL_kI = 0.0;
  public static final double FLYWHEEL_kD = 0.0;

  // flywheel (master + follower)
  private final Distance flywheelDiameter = Inches.of(4);
  private final TalonFX m_flywheelMasterTalon = new TalonFX(FLYWHEEL_MASTER_ID, CANBUS);
  private final TalonFX m_flywheelFollowerTalon = new TalonFX(FLYWHEEL_FOLLOWER_ID, CANBUS);

  private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withIdleMode(MotorMode.COAST)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
      .withClosedLoopController(FLYWHEEL_kP, FLYWHEEL_kI, FLYWHEEL_kD)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new SimpleMotorFeedforward(FLYWHEEL_kS, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(FLYWHEEL_kS, 0, 0))
      .withFollowers(Pair.of(m_flywheelFollowerTalon, true));

  private final SmartMotorController m_flywheelMotor = new TalonFXWrapper(m_flywheelMasterTalon, DCMotor.getKrakenX60(1), smc_config);

  // Construct YAMS FlyWheel config & mechanism (use master controller for mech config)
  private FlyWheelConfig m_flywheelConfig = new FlyWheelConfig(m_flywheelMotor)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withTelemetry("FlywheelMech", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      .withSoftLimit(RPM.of(-5000), RPM.of(5000))
      .withSpeedometerSimulation(RPM.of(7500));

  private FlyWheel m_flywheel = new FlyWheel(m_flywheelConfig);

  private boolean m_isTeleop = false;

  public FlywheelSubsystem() {
    // telemetry setup
    m_flywheelMotor.setupTelemetry();
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    m_flywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_flywheel.simIterate();
  }

  // YAMS Flywheel API wrappers
  public AngularVelocity getVelocity() {
    return m_flywheel.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return m_flywheel.setSpeed(speed);
  }

  public Command setDutyCycle(double dutyCycle) {
    return m_flywheel.set(dutyCycle);
  }

  public Command setVelocity(java.util.function.Supplier<AngularVelocity> speed) {
    return m_flywheel.setSpeed(speed);
  }

  public Command setDutyCycle(java.util.function.Supplier<Double> dutyCycle) {
    return m_flywheel.set(dutyCycle);
  }

  public Command sysId() {
    return m_flywheel.sysId(Volts.of(10), Volts.of(1).per(Seconds), Seconds.of(5));
  }

  public Command setRPM(LinearVelocity speed) {
    return m_flywheel.setSpeed(RotationsPerSecond.of(speed.in(MetersPerSecond) / flywheelDiameter.times(Math.PI).in(Meters)));
  }

  public void setRPMDirect(LinearVelocity speed) {
    // directly set the motor velocity on the master based on linear speed -> rotational
    m_flywheelMotor.setVelocity(RotationsPerSecond.of(speed.in(MetersPerSecond) / flywheelDiameter.times(Math.PI).in(Meters)));
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
    // Display flywheel speed in RPM
    SmartDashboard.putNumber("Shooter/FlywheelRPM", this.getVelocity().in(RPM));
  }
}
