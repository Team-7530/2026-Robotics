package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Telemetry;

/**
 * MotorHealthMonitor tracks the health of a single motor controller.
 *
 * This helper class is designed to be owned by and instantiated within a subsystem,
 * allowing each subsystem to be responsible for monitoring its own motors. This follows
 * the Single Responsibility Principle: each subsystem owns its hardware and diagnostics.
 *
 * Usage (in your subsystem):
 * ```
 * public class FlywheelSubsystem extends SubsystemBase {
 *   private final TalonFX master;
 *   private final MotorHealthMonitor masterMonitor;
 *
 *   public FlywheelSubsystem(Telemetry telemetry) {
 *     master = new TalonFX(...);
 *     masterMonitor = new MotorHealthMonitor(master, "Flywheel", telemetry, 80.0);
 *     SystemHealthAggregator.registerMonitor(masterMonitor);
 *   }
 *
 *   @Override
 *   public void periodic() {
 *     // ... existing motor control logic ...
 *     masterMonitor.update();  // Call from periodic to sample current
 *   }
 *
 *   public boolean isHealthy() { return masterMonitor.isHealthy(); }
 * }
 * ```
 *
 * The aggregator (SystemHealthAggregator) periodically queries all registered monitors
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
   * Create a health monitor for a motor.
   *
   * @param motor The TalonFX motor controller to monitor
   * @param motorName Display name for telemetry (e.g., "Flywheel", "Turret", "Collector")
   * @param telemetry Telemetry instance for logging health data
   * @param stallCurrentThreshold Current in amps above which the motor is considered unhealthy
   *                             (e.g., 80A for Kraken X60 as a jam indicator)
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
    telemetry.putNumber("Health/Motors/" + motorName + "/Current", currentAmps, true);
    telemetry.putBoolean("Health/Motors/" + motorName + "/OK", healthy, true);
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
