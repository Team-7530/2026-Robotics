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

//Subsystem for colector rollers on the rake

public class CollectorSubsystem extends SubsystemBase {

    private final TalonFX m_CollectorMotor =
      new TalonFX(CollectorConstants.COLLECTORMOTOR_ID, CollectorConstants.CANBUS);

    private final VelocityTorqueCurrentFOC m_velocityRequest =
      new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final DutyCycleOut m_manualRequest = new DutyCycleOut(0);

     private final NeutralOut m_brake = new NeutralOut();

    public CollectorSubsystem() {
        initCollectorConfigs();
}

    private void initCollectorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.Inverted = CollectorConstants.kCollectorInverted;
        configs.MotorOutput.NeutralMode = CollectorConstants.kCollectorNeutralMode;
        configs.Voltage.PeakForwardVoltage = CollectorConstants.peakForwardVoltage;
        configs.Voltage.PeakReverseVoltage = CollectorConstants.peakReverseVoltage;
        configs.TorqueCurrent.PeakForwardTorqueCurrent = CollectorConstants.peakForwardTorqueCurrent;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = CollectorConstants.peakReverseTorqueCurrent;

        configs.Slot0.kS = CollectorConstants.collectorMotorTorqueKS;
        configs.Slot0.kP = CollectorConstants.collectorMotorTorqueKP;
        configs.Slot0.kI = CollectorConstants.collectorMotorTorqueKI;
        configs.Slot0.kD = CollectorConstants.collectorMotorTorqueKD;

        StatusCode status = m_CollectorMotor.getConfigurator().apply(configs);
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
   * @param Cvelocity left motors target velocity
   */
    public void setCollectorVelocity(double Cvelocity) {
        m_CollectorMotor.setControl(
            m_velocityRequest.withVelocity(Cvelocity * CollectorConstants.kCollectorGearRatio));
  }

  /**
   * Sets collector motor speed
   *
   * @param speed double
   */
    public void setCollectorSpeed(double speed) {
        m_CollectorMotor.setControl(m_manualRequest.withOutput(speed));
  }

  /** Activates motor brakes */
    public void collectorStop() {
        m_CollectorMotor.setControl(m_brake);
  }

  /** Sets motors to constants intake speed */
    public void collectorIn() {
        this.setCollectorVelocity(CollectorConstants.collectorVelocity);
  }

  /** Updates the Smart Dashboard */
    private void updateSmartDashboard() {
        SmartDashboard.putNumber("CollectorIntake Speed", m_CollectorMotor.getVelocity().getValueAsDouble());
  }

    public Command feederCommand() {
        return run(() -> this.collectorIn())
            .withName("CollectorCommand")
            .withTimeout(5.0)
            .finallyDo(() -> this.collectorStop());
  }
}


