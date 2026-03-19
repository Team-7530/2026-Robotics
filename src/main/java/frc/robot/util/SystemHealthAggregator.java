package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;

/**
 * SystemHealthAggregator is a subsystem that aggregates health checks from throughout
 * the robot and provides a single "overall health" status for the driver.
 *
 * This subsystem collects health from:
 * - Individual motor monitors registered by each subsystem
 * - Power system checks (battery voltage, CAN errors)
 * - Custom health suppliers from any subsystem
 *
 * ARCHITECTURE NOTES:
 * Rather than having a centralized "HealthMonitoringSubsystem" that knows about all motors,
 * health monitoring is distributed: each subsystem owns its own MotorHealthMonitor instance(s).
 * The aggregator simply queries all registered monitors to compute system health.
 *
 * This provides better:
 * - Scalability: New subsystems don't require changes to the aggregator
 * - Testability: Each subsystem can be tested independently
 * - Maintainability: Each subsystem is responsible for its own diagnostics
 * - Encapsulation: Motor references live where they're used, not in a central location
 *
 * USAGE:
 * In RobotContainer:
 *   SystemHealthAggregator aggregator = new SystemHealthAggregator(telemetry, pdp);
 *   aggregator.registerMonitor(flywheelMonitor);
 *   aggregator.registerMonitor(turretMonitor);
 *   // ... register other monitors ...
 *   addPeriodic(aggregator::periodic, 0.02);
 *
 * The aggregator will periodically query all monitors and publish overall health.
 */
@Logged
public class SystemHealthAggregator extends SubsystemBase {
  private final Telemetry telemetry;
  private final PowerDistribution pdp;

  // Registered health monitors
  private final List<MotorHealthMonitor> motorMonitors = new ArrayList<>();
  private final List<Supplier<Boolean>> customHealthChecks = new ArrayList<>();

  // Power system health
  @Logged(importance = Logged.Importance.INFO)
  private boolean battery_healthy = true;

  @Logged(importance = Logged.Importance.INFO)
  private boolean can_bus_healthy = true;

  // Overall aggregate health
  @Logged(importance = Logged.Importance.CRITICAL)
  private boolean overall_healthy = true;

  // Power diagnostics for telemetry
  @Logged(importance = Logged.Importance.INFO)
  private double battery_voltage = 12.0;

  @Logged(importance = Logged.Importance.DEBUG)
  private double total_current_amps = 0.0;

  // Power system thresholds
  private static final double BROWNOUT_VOLTAGE_THRESHOLD = 6.5;
  private static final double CRITICAL_VOLTAGE_THRESHOLD = 6.0;

  public SystemHealthAggregator(Telemetry telemetry, PowerDistribution pdp) {
    this.telemetry = telemetry;
    this.pdp = pdp;
  }

  /**
   * Register a motor health monitor for aggregation.
   * Typically called during subsystem initialization.
   *
   * @param monitor MotorHealthMonitor instance from a subsystem
   */
  public void registerMonitor(MotorHealthMonitor monitor) {
    motorMonitors.add(monitor);
  }

  /**
   * Register a custom health check supplier (e.g., for vision system, pneumatics, etc.).
   * The supplier will be called during each periodic cycle to check health.
   *
   * @param name Display name for the health check
   * @param healthCheck Supplier returning true if system is healthy
   */
  public void registerHealthCheck(String name, Supplier<Boolean> healthCheck) {
    customHealthChecks.add(healthCheck);
  }

  @Override
  public void periodic() {
    // Check power system health
    checkPowerSystem();

    // Aggregate all health checks
    updateOverallHealth();

    // Publish telemetry
    publishTelemetry();
  }

  /**
   * Check battery voltage and CAN bus health.
   * Aggregates power system diagnostics for driver visibility.
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
    // PowerDistributionFaults returns null when healthy, object when faults present
    var faults = pdp.getFaults();
    can_bus_healthy = (faults == null);
  }

  /**
   * Aggregate all health checks into a single "overall health" boolean.
   * Returns true only if ALL motors, power system, and custom checks are healthy.
   */
  private void updateOverallHealth() {
    // All motors must be healthy
    boolean allMotorsHealthy = motorMonitors.stream()
        .allMatch(MotorHealthMonitor::isHealthy);

    // All custom checks must pass
    boolean allCustomChecksHealthy = customHealthChecks.stream()
        .allMatch(Supplier::get);

    // Overall: everything must be healthy
    overall_healthy = allMotorsHealthy
        && allCustomChecksHealthy
        && battery_healthy
        && can_bus_healthy;
  }

  /**
   * Publish health status to telemetry for driver dashboard and post-match analysis.
   */
  private void publishTelemetry() {
    // Power system health
    telemetry.putBoolean("Health/Battery", battery_healthy, true);
    telemetry.putBoolean("Health/CANBus", can_bus_healthy, true);

    // Overall health (CRITICAL - driver must see this)
    telemetry.putBoolean("Health/Overall", overall_healthy, true);

    // Power diagnostics (for tuning/debugging)
    telemetry.putNumber("Health/Power/BatteryVoltage", battery_voltage, true);
    telemetry.putNumber("Health/Power/TotalCurrent", total_current_amps, false);
  }

  // ========== PUBLIC QUERY METHODS ==========
  // These allow external code (commands, SmartDashboard, etc.) to query health status

  /**
   * Query overall system health.
   * @return true if all motors, power, and custom checks are healthy
   */
  public boolean isOverallHealthy() {
    return overall_healthy;
  }

  /**
   * Query power system health (battery + CAN).
   * @return true if both battery and CAN bus are healthy
   */
  public boolean isPowerSystemHealthy() {
    return battery_healthy && can_bus_healthy;
  }

  /**
   * Query if all registered motors are healthy.
   * @return true if all motor monitors report healthy status
   */
  public boolean areAllMotorsHealthy() {
    return motorMonitors.stream().allMatch(MotorHealthMonitor::isHealthy);
  }

  /**
   * Query battery health specifically.
   * @return true if battery voltage is above brownout threshold
   */
  public boolean isBatteryHealthy() {
    return battery_healthy;
  }

  /**
   * Query CAN bus health specifically.
   * @return true if no CAN errors detected
   */
  public boolean isCANBusHealthy() {
    return can_bus_healthy;
  }

  /**
   * Get current battery voltage.
   * @return Voltage in volts
   */
  public double getBatteryVoltage() {
    return battery_voltage;
  }

  /**
   * Get total current draw.
   * @return Current in amps
   */
  public double getTotalCurrent() {
    return total_current_amps;
  }

  /**
   * Get number of registered motor monitors.
   * Useful for debugging/verification that all subsystems registered.
   * @return Number of monitors
   */
  public int getMonitorCount() {
    return motorMonitors.size();
  }
}
