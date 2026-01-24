package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Subsystem for rollers inside of hopper

public class FeederSubsystem extends SubsystemBase {

    public static final CANBus CANBUS = CANBus.roboRIO();
    public static final int FEEDERMOTOR_ID = 65;

    public static final InvertedValue kFeederInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kFeederNeutralMode = NeutralModeValue.Coast;
    public static final double peakForwardVoltage = 10.0; // Peak output of 8 volts
    public static final double peakReverseVoltage = -10.0; // Peak output of 8 volts
    public static final double peakForwardTorqueCurrent = 40.0; // Peak output of 40 amps
    public static final double peakReverseTorqueCurrent = -40.0; // Peak output of 40 amps

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
    
    private final TalonFX m_FeederMotor =
      new TalonFX(FEEDERMOTOR_ID, CANBUS);

    private final VelocityTorqueCurrentFOC m_velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final DutyCycleOut m_manualRequest = new DutyCycleOut(0);

     private final NeutralOut m_brake = new NeutralOut();

    public FeederSubsystem() {
        initFeederConfigs();
}

    private void initFeederConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.Inverted = kFeederInverted;
        configs.MotorOutput.NeutralMode = kFeederNeutralMode;
        configs.Voltage.PeakForwardVoltage = peakForwardVoltage;
        configs.Voltage.PeakReverseVoltage = peakReverseVoltage;
        configs.TorqueCurrent.PeakForwardTorqueCurrent = peakForwardTorqueCurrent;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = peakReverseTorqueCurrent;

        configs.Slot0.kS = feederMotorTorqueKS;
        configs.Slot0.kP = feederMotorTorqueKP;
        configs.Slot0.kI = feederMotorTorqueKI;
        configs.Slot0.kD = feederMotorTorqueKD;

        StatusCode status = m_FeederMotor.getConfigurator().apply(configs);
        if (!status.isOK()) {
        System.out.println("Could not apply top configs, error code: " + status.toString());
        }
}
    @Override
    public void periodic() {
    updateSmartDashboard();
  }

  /**
   * Sets the motor Target velocity
   *
   * @param Fvelocity left motors target velocity
   */
    public void setFeederVelocity(double Fvelocity) {
        m_FeederMotor.setControl(
            m_velocityRequest.withVelocity(Fvelocity * kFeederGearRatio));
  }

  /**
   * Sets feeder motor speed
   *
   * @param speed double
   */
    public void setFeederSpeed(double speed) {
        m_FeederMotor.setControl(m_manualRequest.withOutput(speed));
  }

  /** Activates motor brakes */
    public void feederStop() {
        m_FeederMotor.setControl(m_brake);
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
        SmartDashboard.putNumber("FeederIntake Speed", m_FeederMotor.getVelocity().getValueAsDouble());
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


