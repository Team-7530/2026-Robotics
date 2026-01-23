package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Subsystem for rollers inside of hopper

public class FeederSubsystem extends SubsystemBase {

    private final TalonFX m_FeederMotor =
      new TalonFX(FeederConstants.FEEDERMOTOR_ID, FeederConstants.CANBUS);

    private final VelocityTorqueCurrentFOC m_velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final DutyCycleOut m_manualRequest = new DutyCycleOut(0);

     private final NeutralOut m_brake = new NeutralOut();

    public FeederSubsystem() {
        initFeederConfigs();
}

    private void initFeederConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.Inverted = FeederConstants.kFeederInverted;
        configs.MotorOutput.NeutralMode = FeederConstants.kFeederNeutralMode;
        configs.Voltage.PeakForwardVoltage = FeederConstants.peakForwardVoltage;
        configs.Voltage.PeakReverseVoltage = FeederConstants.peakReverseVoltage;
        configs.TorqueCurrent.PeakForwardTorqueCurrent = FeederConstants.peakForwardTorqueCurrent;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = FeederConstants.peakReverseTorqueCurrent;

        configs.Slot0.kS = FeederConstants.feederMotorTorqueKS;
        configs.Slot0.kP = FeederConstants.feederMotorTorqueKP;
        configs.Slot0.kI = FeederConstants.feederMotorTorqueKI;
        configs.Slot0.kD = FeederConstants.feederMotorTorqueKD;

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
            m_velocityRequest.withVelocity(Fvelocity * FeederConstants.kFeederGearRatio));
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
        this.setFeederVelocity(FeederConstants.feederVelocity);
  }

  /** Sets motors to constants intake speed */
  public void feederUnstuck() {
      this.setFeederVelocity(FeederConstants.feederUnstuckVelocity);
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


