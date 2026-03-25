package frc.lib.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;

/**
 * SystemHealthMonitor is a comprehensive health monitoring subsystem that aggregates
 * health checks from throughout the robot and provides a single "overall health" status
 * for the driver.
 *
 * This subsystem collects health from:
 * - Individual motor monitors (via nested MotorHealthMonitor.create() factory)
 * - Power system checks (battery voltage, power distribution comms)
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
 *   SystemHealthMonitor monitor = new SystemHealthMonitor(telemetry);
 *
 * In a subsystem:
 *   private final MotorHealthMonitor flywheelMonitor = MotorHealthMonitor.create(
 *       master, "Flywheel", telemetry, 80.0, monitor);
 *
 *   @Override
 *   public void periodic() {
 *     flywheelMonitor.update();
 *     // ... existing motor control logic ...
 *   }
 *
 * The monitor will periodically query all registered motor monitors and publish
 * overall health to the dashboard and logs.
 */
@Logged
public class SystemHealthMonitor extends SubsystemBase {
    public final Telemetry telemetry;
    public final PowerDistribution pdp = new PowerDistribution();

    // Registered health monitors
    private final List<MotorHealthMonitor> motorMonitors = new ArrayList<>();
    private final List<Supplier<Boolean>> customHealthChecks = new ArrayList<>();

    // Power system health
    @Logged(importance = Logged.Importance.INFO)
    private boolean battery_healthy = true;

    @Logged(importance = Logged.Importance.INFO)
    private boolean power_distribution_healthy = true;

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

    public SystemHealthMonitor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Create a health monitor for a motor and automatically register it with the system monitor.
     *
     * @param motor The TalonFX motor controller to monitor
     * @param motorName Display name for telemetry (e.g., "Flywheel", "Turret", "Collector")
     * @param stallCurrentThreshold Current in amps above which the motor is considered unhealthy
     *                             (e.g., 80A for Kraken X60 as a jam indicator)
     * @return A new MotorHealthMonitor instance registered with the system monitor
     */
    public MotorHealthMonitor createMotorHealthMonitor( TalonFX motor,
                                                        String motorName,
                                                        double stallCurrentThreshold) {
        return this.registerMonitor(new MotorHealthMonitor(motor, motorName, this.telemetry, stallCurrentThreshold));
    }

    /**
     * Create a health monitor for a motor without automatic registration.
     * Useful if you want to manage registration manually.
     *
     * @param motor The TalonFX motor controller to monitor
     * @param motorName Display name for telemetry (e.g., "Flywheel", "Turret", "Collector")
     * @param stallCurrentThreshold Current in amps above which the motor is considered unhealthy
     * @return A new MotorHealthMonitor instance
     */
    public MotorHealthMonitor createMotorHealthMonitorUnregistered( TalonFX motor,
                                                                    String motorName,
                                                                    double stallCurrentThreshold) {
        return new MotorHealthMonitor(motor, motorName, this.telemetry, stallCurrentThreshold);
    }

    /**
     * Register a motor health monitor for aggregation.
     * Typically called when creating a MotorHealthMonitor via the factory method.
     *
     * @param monitor MotorHealthMonitor instance from a subsystem
     */
    public MotorHealthMonitor registerMonitor(MotorHealthMonitor monitor) {
        motorMonitors.add(monitor);
        return monitor;
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
     * Check battery voltage and power distribution communications health.
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

        // This is only the power distribution device's CAN warning bit, not a
        // robot-wide CAN bus health indicator.
        PowerDistributionFaults faults = pdp.getFaults();
        power_distribution_healthy = !faults.CanWarning;
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
            && power_distribution_healthy;
    }

    /**
     * Publish health status to telemetry for driver dashboard and post-match analysis.
     */
    private void publishTelemetry() {
        // Power system health
        telemetry.putBoolean("Health/Battery", battery_healthy, true);
        telemetry.putBoolean("Health/PowerDistribution/CAN", power_distribution_healthy, true);

        // Overall health (CRITICAL - driver must see this)
        telemetry.putBoolean("Health/Overall", overall_healthy, true);

        // Power diagnostics (for tuning/debugging)
        telemetry.putNumber("Health/Power/BatteryVoltage", battery_voltage, false);
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
     * Query power system health (battery + power distribution communications).
     * @return true if both battery voltage and power distribution comms are healthy
     */
    public boolean isPowerSystemHealthy() {
        return battery_healthy && power_distribution_healthy;
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
     * Query power distribution communication health specifically.
     * @return true if the power distribution device is not reporting a CAN warning
     */
    public boolean isPowerDistributionHealthy() {
        return power_distribution_healthy;
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

    // ========== NESTED MotorHealthMonitor CLASS ==========
    /**
     * MotorHealthMonitor tracks the health of a single motor controller.
     *
     * This nested helper class is designed to be owned by and instantiated within a subsystem,
     * allowing each subsystem to be responsible for monitoring its own motors. This follows
     * the Single Responsibility Principle: each subsystem owns its hardware and diagnostics.
     *
     * USAGE:
     * In your subsystem constructor:
     * ```
     * public class FlywheelSubsystem extends SubsystemBase {
     *   private final TalonFX master;
     *   private final MotorHealthMonitor masterMonitor;
     *
     *   public FlywheelSubsystem(Telemetry telemetry, SystemHealthMonitor monitor) {
     *     master = new TalonFX(...);
     *     masterMonitor = MotorHealthMonitor.create(master, "Flywheel", telemetry, 80.0, monitor);
     *   }
     *
     *   @Override
     *   public void periodic() {
     *     masterMonitor.update();
     *   }
     *
     *   public MotorHealthMonitor getHealthMonitor() { return masterMonitor; }
     * }
     * ```
     *
     * The aggregator (SystemHealthMonitor) periodically queries all registered monitors
     * to compute system-wide health without needing each subsystem to report back to a
     * central authority.
     */
    public class MotorHealthMonitor {
        private final TalonFX motor;
        private final String motorName;
        private final Telemetry telemetry;
        private final double stallCurrentThreshold;

        // Current health state
        private boolean healthy = true;
        private double currentAmps = 0.0;

        /**
         * Public constructor for direct use by subsystems.
         * Subsystems can instantiate monitors directly and register them manually, or use the factory
         * methods create() and createUnregistered() for convenience.
         */
        public MotorHealthMonitor(TalonFX motor, String motorName, Telemetry telemetry, double stallCurrentThreshold) {
            this.motor = motor;
            this.motorName = motorName;
            this.telemetry = telemetry;
            this.stallCurrentThreshold = stallCurrentThreshold;
        }

        /**
         * Sample the motor's stator current and update health status.
         * Call this from your subsystem's periodic() method (typically alongside updateTelemetry()).
         *
         * High current can indicate:
         * - Mechanical jam or obstruction
         * - Motor stall condition
         * - Loss of power (internal fault)
         * - Disconnected motor
         */
        public void update() {
            if (motor == null) {
                healthy = false;
                return;
            }

            currentAmps = motor.getStatorCurrent().getValueAsDouble();
            healthy = (currentAmps < stallCurrentThreshold);

            // Always log current (helps with competition diagnostics)
            telemetry.putNumber("Health/Motors/" + motorName + "/Current", currentAmps, false);
            telemetry.putBoolean("Health/Motors/" + motorName + "/OK", healthy, false);
        }

        /**
         * Query whether this motor is currently healthy.
         *
         * @return true if current is below threshold, false otherwise
         */
        public boolean isHealthy() {
            return healthy;
        }

        /**
         * Get the last sampled stator current in amps.
         *
         * @return Motor stator current (only valid after calling update())
         */
        public double getCurrentAmps() {
            return currentAmps;
        }

        /**
         * Get the motor name for display/logging.
         *
         * @return Display name (e.g., "Flywheel")
         */
        public String getMotorName() {
            return motorName;
        }

        /**
         * Get the stall current threshold.
         *
         * @return Threshold in amps
         */
        public double getThreshold() {
            return stallCurrentThreshold;
        }
    }
}
