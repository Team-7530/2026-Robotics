package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;

import static frc.robot.Constants.STICK_DEADBAND;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ShooterSubsystem controls a turret (absolute encoder + position control) and a two-motor
 * flywheel (master + follower). This is a lightweight, conservative implementation that uses
 * percent open-loop control for the flywheel and position control for the turret. 
 */
public class FlywheelSubsystem extends SubsystemBase {

  public static final CANBus CANBUS = new CANBus("CANFD");

  // Placeholder CAN IDs - update to match your wiring
  public static final int FLYWHEEL_MASTER_ID = 61;
  public static final int FLYWHEEL_FOLLOWER_ID = 62;

  // Flywheel tuning
  public static final double FLYWHEEL_MAX_RPM = 6000.0; // adjust to your flywheel
  public static final double FLYWHEEL_DEFAULT_RPM = 3000.0;

  public static final InvertedValue kFlywheelInverted = InvertedValue.CounterClockwise_Positive;
  public static final NeutralModeValue kFlywheelNeutralMode = NeutralModeValue.Brake;

  public static final double kFlywheelChainRatio = 1.0 / 1.0;
  public static final double kFlywheelGearboxRatio = 1.0; // 1:1
  public static final double kFlywheelGearRatio = kFlywheelChainRatio * kFlywheelGearboxRatio;

  // Flywheel control tuning (Velocity closed-loop)
  public static final double FLYWHEEL_kS = 0.0;
  public static final double FLYWHEEL_kP = 0.1;
  public static final double FLYWHEEL_kI = 0.0;
  public static final double FLYWHEEL_kD = 0.0;

  public static final double kFlywheelPeakForwardVoltage = 8.0; // Peak output of 8 volts
  public static final double kFlywheelPeakReverseVoltage = -8.0; // Peak output of 8 volts

  // flywheel
  private final TalonFX m_flywheelMaster = new TalonFX(FLYWHEEL_MASTER_ID, CANBUS);
  private final TalonFX m_flywheelFollower = new TalonFX(FLYWHEEL_FOLLOWER_ID, CANBUS);
  private final VelocityTorqueCurrentFOC m_flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final DutyCycleOut m_flywheelPercentRequest = new DutyCycleOut(0);
  private double lastFlywheelPercent = 0.0;

  private boolean m_isTeleop = false;

  public FlywheelSubsystem() {
    initFlywheelConfigs();

    if (RobotBase.isSimulation()) initSimulation();
  }

  private void initFlywheelConfigs() {
    // Configure flywheel PID slot
    TalonFXConfiguration flyConfigs = new TalonFXConfiguration();
    flyConfigs.MotorOutput.Inverted = kFlywheelInverted;
    flyConfigs.MotorOutput.NeutralMode = kFlywheelNeutralMode;
    flyConfigs.Voltage.PeakForwardVoltage = kFlywheelPeakForwardVoltage;
    flyConfigs.Voltage.PeakReverseVoltage = kFlywheelPeakReverseVoltage;

    flyConfigs.Slot0.kS = FLYWHEEL_kS;
    flyConfigs.Slot0.kP = FLYWHEEL_kP;
    flyConfigs.Slot0.kI = FLYWHEEL_kI;
    flyConfigs.Slot0.kD = FLYWHEEL_kD;

    StatusCode s = m_flywheelMaster.getConfigurator().apply(flyConfigs);
    if (!s.isOK()) System.out.println("Could not apply flywheel configs: " + s);

    s = m_flywheelFollower.getConfigurator().apply(flyConfigs);
    if (!s.isOK()) System.out.println("Could not apply flywheel configs: " + s);
    m_flywheelFollower.setControl(new Follower(m_flywheelMaster.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  private void initSimulation() {
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  // -- Flywheel control --------------------------------------------------
  /**
   * Set flywheel speed as percent output (-1.0..1.0).
   */
  public void setFlywheelPercent(double pct) {
    pct = MathUtil.clamp(pct, -1.0, 1.0);
    m_flywheelMaster.setControl(m_flywheelPercentRequest.withOutput(pct));
    // mirror to follower (no high-level follow helper used)
    m_flywheelFollower.setControl(m_flywheelPercentRequest.withOutput(pct));
    lastFlywheelPercent = pct;
  }

  /**
   * Set flywheel velocity by RPM. This implementation maps RPM to percent based on
   * `FLYWHEEL_MAX_RPM`. For closed-loop control replace this method with
   * a velocity closed-loop set using appropriate sensor configuration.
   */
  public void setFlywheelVelocityRPM(double rpm) {
    // Convert desired flywheel RPM to motor rotations per second (sensor units)
    double desiredFlywheelRps = rpm / 60.0;
    double motorRps = desiredFlywheelRps * kFlywheelGearRatio;
    m_flywheelMaster.setControl(m_flywheelVelocityRequest.withVelocity(motorRps));
    // mirror to follower
    m_flywheelFollower.setControl(m_flywheelVelocityRequest.withVelocity(motorRps));
    lastFlywheelPercent = MathUtil.clamp(rpm / FLYWHEEL_MAX_RPM, -1.0, 1.0);
  }

  public void stopFlywheel() {
    setFlywheelPercent(0.0);
  }

  public double getFlywheelPercent() {
    return lastFlywheelPercent;
  }

  /** Returns flywheel speed in RPM (based on master sensor). */
  public double getFlywheelRPM() {
    double motorRps = m_flywheelMaster.getVelocity().getValueAsDouble();
    double flywheelRps = motorRps / kFlywheelGearRatio;
    return flywheelRps * 60.0;
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
      setFlywheelPercent(aspeed);
    } else if (m_isTeleop) {
      m_isTeleop = false;
      stopFlywheel();
    }
  }
  // -- SmartDashboard ----------------------------------------------------
  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Shooter/FlywheelPercent", this.getFlywheelPercent());
  }

  // -- Commands -----------------------------------------------------------
  public Command shooterToVelocityCommand(double velocity) {
    return run(() -> this.setFlywheelVelocityRPM(velocity))
        .withName("ShooterToVelocityCommand")
        .withTimeout(5.0);
  }

  public Command shooterToPercentCommand(double pct) {
    return run(() -> this.setFlywheelPercent(pct))
        .withName("ShooterToPercentCommand")
        .withTimeout(5.0);
  }
}
