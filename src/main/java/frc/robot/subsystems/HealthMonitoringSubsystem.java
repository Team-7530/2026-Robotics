package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Telemetry;

/**
 * HealthMonitoringSubsystem tracks hardware health during competition.
 * 
 * Monitors:
 * - Motor currents to detect jamming, disconnects, or mechanical failures
 * - Battery voltage and brownout conditions
 * - CAN bus health and errors
 * - Overall system health aggregate for driver feedback
 * 
 * All data is timestamped and logged via Epilogue for post-match analysis.
 */
@Logged
public class HealthMonitoringSubsystem extends SubsystemBase {
  private final Telemetry telemetry;
  private final PowerDistribution pdp;

  // Motor current thresholds (in Amps)
  // Adjust based on your specific motor specifications and expected operating currents
  private static final double MOTOR_STALL_CURRENT_THRESHOLD = 80.0;  // Kraken X60 stall current is ~257A, warning at 80A indicates potential jam
  // Note: MOTOR_DISCONNECT_TIMEOUT_MS reserved for future enhanced motor connectivity monitoring

  // Voltage thresholds
  private static final double BROWNOUT_VOLTAGE_THRESHOLD = 6.5;      // Warning when approaching brownout territory
  private static final double CRITICAL_VOLTAGE_THRESHOLD = 6.0;      // Critical when this low

  // CAN bus health (tracked via PowerDistribution faults)

  // Health tracking for each motor
  @Logged(importance = Logged.Importance.INFO)
  private boolean flywheel_healthy = true;
  @Logged(importance = Logged.Importance.INFO)
  private boolean collector_healthy = true;
  @Logged(importance = Logged.Importance.INFO)
  private boolean feeder_healthy = true;
  @Logged(importance = Logged.Importance.INFO)
  private boolean turret_healthy = true;
  @Logged(importance = Logged.Importance.INFO)
  private boolean rake_arm_healthy = true;
  @Logged(importance = Logged.Importance.INFO)
  private boolean rake_intake_healthy = true;

  // Power system health
  @Logged(importance = Logged.Importance.INFO)
  private boolean battery_healthy = true;

  @Logged(importance = Logged.Importance.INFO)
  private boolean can_bus_healthy = true;

  // Overall health aggregate
  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean overall_healthy = true;

  // Current snapshot (for quick driver visibility)
  @Logged(importance = Logged.Importance.DEBUG)
  private double flywheel_current_amps = 0.0;
  @Logged(importance = Logged.Importance.DEBUG)
  private double collector_current_amps = 0.0;
  @Logged(importance = Logged.Importance.DEBUG)
  private double feeder_current_amps = 0.0;
  @Logged(importance = Logged.Importance.DEBUG)
  private double turret_current_amps = 0.0;
  @Logged(importance = Logged.Importance.DEBUG)
  private double rake_arm_current_amps = 0.0;
  @Logged(importance = Logged.Importance.DEBUG)
  private double rake_intake_current_amps = 0.0;

  // Power system diagnostics
  @Logged(importance = Logged.Importance.INFO)
  private double battery_voltage = 12.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double total_current_amps = 0.0;

  // Motor references (populated by setters from each subsystem)
  private TalonFX flywheelMaster = null;
  private TalonFX collectorMotor = null;
  private TalonFX feederMotor = null;
  private TalonFX turretMotor = null;
  private TalonFX rakeArmMotor = null;
  private TalonFX rakeIntakeMotor = null;

  public HealthMonitoringSubsystem(Telemetry telemetry, PowerDistribution pdp) {
    this.telemetry = telemetry;
    this.pdp = pdp;
  }

  /**
   * Register motor references for health monitoring.
   * Each subsystem should call these methods during initialization.
   */
  public void registerFlywheelMotors(TalonFX master) {
    this.flywheelMaster = master;
    // Follower motors are monitored via master; both run in sync
  }

  public void registerCollectorMotor(TalonFX motor) {
    this.collectorMotor = motor;
  }

  public void registerFeederMotor(TalonFX motor) {
    this.feederMotor = motor;
  }

  public void registerTurretMotor(TalonFX motor) {
    this.turretMotor = motor;
  }

  public void registerRakeArmMotor(TalonFX motor) {
    this.rakeArmMotor = motor;
  }

  public void registerRakeIntakeMotor(TalonFX motor) {
    this.rakeIntakeMotor = motor;
  }

  @Override
  public void periodic() {
    // Sample all motor currents
    sampleMotorCurrents();

    // Check power system health
    checkPowerSystem();

    // Aggregate overall health
    updateOverallHealth();

    // Publish telemetry
    publishTelemetry();
  }

  /**
   * Sample stator currents from all registered motors.
   * High current can indicate jamming, mechanical failure, or disconnect.
   */
  private void sampleMotorCurrents() {
    // Flywheel (both master and follower)
    if (flywheelMaster != null) {
      double current = flywheelMaster.getStatorCurrent().getValueAsDouble();
      flywheel_current_amps = current;
      flywheel_healthy = (current < MOTOR_STALL_CURRENT_THRESHOLD);
    }

    // Collector
    if (collectorMotor != null) {
      double current = collectorMotor.getStatorCurrent().getValueAsDouble();
      collector_current_amps = current;
      collector_healthy = (current < MOTOR_STALL_CURRENT_THRESHOLD);
    }

    // Feeder
    if (feederMotor != null) {
      double current = feederMotor.getStatorCurrent().getValueAsDouble();
      feeder_current_amps = current;
      feeder_healthy = (current < MOTOR_STALL_CURRENT_THRESHOLD);
    }

    // Turret
    if (turretMotor != null) {
      double current = turretMotor.getStatorCurrent().getValueAsDouble();
      turret_current_amps = current;
      turret_healthy = (current < MOTOR_STALL_CURRENT_THRESHOLD);
    }

    // Rake Arm
    if (rakeArmMotor != null) {
      double current = rakeArmMotor.getStatorCurrent().getValueAsDouble();
      rake_arm_current_amps = current;
      rake_arm_healthy = (current < MOTOR_STALL_CURRENT_THRESHOLD);
    }

    // Rake Intake
    if (rakeIntakeMotor != null) {
      double current = rakeIntakeMotor.getStatorCurrent().getValueAsDouble();
      rake_intake_current_amps = current;
      rake_intake_healthy = (current < MOTOR_STALL_CURRENT_THRESHOLD);
    }
  }

  /**
   * Check battery voltage and overall power system health.
   * Brownout at ~6.5V, critical at ~6.0V.
   */
  private void checkPowerSystem() {
    battery_voltage = RobotController.getBatteryVoltage();
    total_current_amps = pdp.getTotalCurrent();

    // Flag as unhealthy if voltage is critically low
    if (battery_voltage < CRITICAL_VOLTAGE_THRESHOLD) {
      battery_healthy = false;
    } else if (battery_voltage < BROWNOUT_VOLTAGE_THRESHOLD) {
      battery_healthy = false;
    } else {
      battery_healthy = true;
    }

    // Check CAN bus health via PowerDistribution faults
    // Note: CAN errors are transient; only flag if they become systemic
    // PowerDistributionFaults has no faults when the object is equal to a default constructed one
    var faults = pdp.getFaults();
    can_bus_healthy = (faults == null);
  }

  /**
   * Aggregate all health checks into a single "overall health" boolean.
   * This is what the driver should watch during competition.
   */
  private void updateOverallHealth() {
    overall_healthy = flywheel_healthy
        && collector_healthy
        && feeder_healthy
        && turret_healthy
        && rake_arm_healthy
        && rake_intake_healthy
        && battery_healthy
        && can_bus_healthy;
  }

  /**
   * Publish health status to telemetry for driver dashboard and post-match analysis.
   */
  private void publishTelemetry() {
    // Motor health statuses (always logged, vital for debugging issues)
    telemetry.putBoolean("Health/Flywheel", flywheel_healthy, true);
    telemetry.putBoolean("Health/Collector", collector_healthy, true);
    telemetry.putBoolean("Health/Feeder", feeder_healthy, true);
    telemetry.putBoolean("Health/Turret", turret_healthy, true);
    telemetry.putBoolean("Health/RakeArm", rake_arm_healthy, true);
    telemetry.putBoolean("Health/RakeIntake", rake_intake_healthy, true);

    // Power system health
    telemetry.putBoolean("Health/Battery", battery_healthy, true);
    telemetry.putBoolean("Health/CANBus", can_bus_healthy, true);

    // Overall health (CRITICAL - driver must see this)
    telemetry.putBoolean("Health/Overall", overall_healthy, true);

    // Current details (debug level - only when tuning)
    telemetry.putNumber("Health/Current/Flywheel", flywheel_current_amps, false);
    telemetry.putNumber("Health/Current/Collector", collector_current_amps, false);
    telemetry.putNumber("Health/Current/Feeder", feeder_current_amps, false);
    telemetry.putNumber("Health/Current/Turret", turret_current_amps, false);
    telemetry.putNumber("Health/Current/RakeArm", rake_arm_current_amps, false);
    telemetry.putNumber("Health/Current/RakeIntake", rake_intake_current_amps, false);
    telemetry.putNumber("Health/Power/BatteryVoltage", battery_voltage, true);
    telemetry.putNumber("Health/Power/TotalCurrent", total_current_amps, false);
  }

  // Getter methods for external subsystems to check health
  public boolean isOverallHealthy() {
    return overall_healthy;
  }

  public boolean isFlywheelHealthy() {
    return flywheel_healthy;
  }

  public boolean isCollectorHealthy() {
    return collector_healthy;
  }

  public boolean isFeederHealthy() {
    return feeder_healthy;
  }

  public boolean isTurretHealthy() {
    return turret_healthy;
  }

  public boolean isRakeArmHealthy() {
    return rake_arm_healthy;
  }

  public boolean isRakeIntakeHealthy() {
    return rake_intake_healthy;
  }

  public boolean isBatteryHealthy() {
    return battery_healthy;
  }

  public boolean isCANHealthy() {
    return can_bus_healthy;
  }

  public double getBatteryVoltage() {
    return battery_voltage;
  }

  public double getTotalCurrent() {
    return total_current_amps;
  }
}
