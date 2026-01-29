package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Subsystem for rollers inside of hopper

public class FeederSubsystem extends SubsystemBase {

    public static final CANBus CANBUS = CANBus.roboRIO();
    public static final int FEEDERMOTOR_ID = 65;

    public static final double kFeederChainRatio = 24.0 / 10.0; // 24:10
    public static final double kFeederGearboxRatio = 1.0; // 1:1
    public static final double kFeederGearRatio = kFeederChainRatio * kFeederGearboxRatio;

    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    public static final double feederMotorTorqueKS = 0.0; // Static feedforward gain
    public static final double feederMotorTorqueKP = 8.0; // error of 1 rps results in 8 amps output
    public static final double feederMotorTorqueKI = 0.2; // error of 1 rps incr by 0.2 amps per sec
    public static final double feederMotorTorqueKD = 0.001; // 1000 rps^2 incr 1 amp output

    public static final double feederVelocity = -3.0;
    public static final double feederUnstuckVelocity = 3.0;
    
    // TalonFX hardware instance (kept for wrapper)
    private final TalonFX m_feederTalon = new TalonFX(FEEDERMOTOR_ID, CANBUS);

    // YAMS controller and mechanism (initialized at declaration to match FlywheelSubsystem style)
    private final SmartMotorControllerConfig smc_config = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withIdleMode(MotorMode.COAST)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
        .withWheelDiameter(Inches.of(1.5))
        .withClosedLoopController(feederMotorTorqueKP, feederMotorTorqueKI, feederMotorTorqueKD)
        .withFeedforward(new SimpleMotorFeedforward(feederMotorTorqueKS, 0, 0))
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("Feeder", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    private final SmartMotorController m_feederMotor = new TalonFXWrapper(m_feederTalon, edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1), smc_config);

    private final FlyWheelConfig m_feederConfig = new FlyWheelConfig(m_feederMotor)
        .withDiameter(Inches.of(1.5))
        .withMass(Pounds.of(0.5))
        .withTelemetry("FeederMech", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withSoftLimit(RPM.of(-5000), RPM.of(5000))
        .withSpeedometerSimulation(RPM.of(5000));

    private final FlyWheel m_feederFlywheel = new FlyWheel(m_feederConfig);

  public FeederSubsystem() {
    // telemetry setup
    m_feederMotor.setupTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_feederFlywheel.simIterate();
    m_feederMotor.simIterate();
  }
    

  /**
   * Sets the motor Target velocity
   *
   * @param Fvelocity left motors target velocity
   */
    public void setFeederVelocity(double Fvelocity) {
    // Fvelocity is rotations per second; set on YAMS controller
    m_feederMotor.setVelocity(RotationsPerSecond.of(Fvelocity));
  }

  /**
   * Sets feeder motor speed
   *
   * @param speed double
   */
    public void setFeederSpeed(double speed) {
    m_feederMotor.setDutyCycle(speed);
  }

  /** Activates motor brakes */
    public void feederStop() {
    m_feederMotor.stopClosedLoopController();
    m_feederMotor.setDutyCycle(0.0);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    m_feederFlywheel.updateTelemetry();
  }

  // YAMS wrappers
  public edu.wpi.first.units.measure.AngularVelocity getVelocity() {
    return m_feederFlywheel.getSpeed();
  }

  public Command setVelocity(edu.wpi.first.units.measure.AngularVelocity speed) {
    return m_feederFlywheel.setSpeed(speed);
  }

  public Command setDutyCycle(double duty) {
    return m_feederFlywheel.set(duty);
  }

  /** Sets motors to constants intake speed */
    public void feederIn() {
    this.setFeederVelocity(feederVelocity);
  }

  /** Sets motors to constants intake speed */
  public void feederUnstuck() {
      this.setFeederVelocity(feederUnstuckVelocity);
  }

  /** Updates the Smart Dashboard */
    private void updateSmartDashboard() {
    try {
      SmartDashboard.putNumber("FeederIntake RPS", m_feederFlywheel.getSpeed().in(RotationsPerSecond));
    } catch (Exception e) {
      SmartDashboard.putNumber("FeederIntake RPS", 0.0);
    }
  }

    public Command feederCommand() {
        return run(() -> this.feederIn())
            .withName("FeederCommand")
            .withTimeout(5.0)
            .finallyDo(() -> this.feederStop());
  }

    public Command feederUnstuckCommand() {
      return run(() -> this.feederUnstuck())
          .withName("UnstuckCommand")
          .withTimeout(5.0)
          .finallyDo(() -> this.feederStop());
  }
}


