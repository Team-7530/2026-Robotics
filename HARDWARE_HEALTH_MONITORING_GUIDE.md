# Hardware Health Monitoring Implementation Guide

**Originally Implemented:** March 18, 2026  
**Latest Refactor:** March 18, 2026  
**Status:** Production-Ready for Competition  
**Architecture:** Distributed Monitoring with Centralized Aggregation

---

## Overview

The health monitoring system provides real-time hardware diagnostics for your FRC robot through a **distributed architecture** where each subsystem monitors its own motors, while a centralized `SystemHealthMonitor` aggregates all health checks into a single **"Overall Health"** boolean that drivers can monitor during competition.

**Key Architecture:** Each subsystem owns a `MotorHealthMonitor` instance; `SystemHealthMonitor` queries all registered monitors to compute system health.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│ SystemHealthMonitor (frc.lib.util)                          │
│ - Extends SubsystemBase                                     │
│ - Aggregates health from all subsystems                     │
│ - Monitors power system (battery, CAN bus)                  │
│ - Publishes overall health to dashboard                     │
└─────────────────────────────────────────────────────────────┘
  ▲         ▲         ▲         ▲         ▲         ▲
  │         │         │         │         │         │
  └─────────┴─────────┴─────────┴─────────┴─────────┘
    Registers Monitors
    
┌─────────────┬─────────────┬─────────────┬─────────────┬──────────────┬──────────────┐
│ Flywheel    │ Turret      │ Collector   │ Feeder      │ RakeArm      │ RakeIntake   │
│ Subsystem   │ Subsystem   │ Subsystem   │ Subsystem   │ Subsystem    │ Subsystem    │
├─────────────┼─────────────┼─────────────┼─────────────┼──────────────┼──────────────┤
│ Motor       │ Motor       │ Motor       │ Motor       │ Motor        │ Motor        │
│ │           │ │           │ │           │ │           │ │            │ │            │
│ └─Monitor   │ └─Monitor   │ └─Monitor   │ └─Monitor   │ └─Monitor    │ └─Monitor    │
│   (owned)   │   (owned)   │   (owned)   │   (owned)   │   (owned)    │   (owned)    │
└─────────────┴─────────────┴─────────────┴─────────────┴──────────────┴──────────────┘
```

---

## What It Monitors

### 🔌 **Motor Health** (Per-Subsystem)
- **Flywheel** — Primary shooter motor current tracking
- **Collector** — Intake roller current
- **Feeder** — Magazine feed motor current  
- **Turret** — Rotational mechanism current
- **Rake Arm** — Deployment arm current
- **Rake Intake** — Secondary intake roller current

**Health Status:** GREEN (✓) if current < 80A, RED (✗) if exceeding threshold  
**Threshold Logic:** Detects jamming, mechanical failure, or disconnects via stator current

### 🔋 **Battery Health**
- **Voltage Monitoring:**
  - 🟢 **Healthy:** ≥ 6.5V
  - 🟡 **Warning:** 6.0V – 6.5V (approaching brownout)
  - 🔴 **Critical:** < 6.0V (brownout territory)

### 📡 **CAN Bus Health**
- Monitors PowerDistribution faults
- Flags if CAN communication errors detected
- Helps identify loose connectors or electrical issues

---

## Telemetry Output

### Driver Dashboard (Always Visible)

```
Health/Overall              → [TRUE/FALSE]     Main aggregate indicator (CRITICAL)
Health/Battery              → [TRUE/FALSE]     Battery voltage status (INFO)
Health/CANBus               → [TRUE/FALSE]     CAN communication health (INFO)
Health/Motors/{Name}/OK     → [TRUE/FALSE]     Individual motor health (INFO)
Health/Power/BatteryVoltage → [12.0 – 6.0]     Actual voltage reading (INFO)
```

### Debug/Tuning (DEBUG mode only)

```
Health/Motors/{Name}/Current → [0.0 – 257.0]   Stator current in Amps (DEBUG)
Health/Power/TotalCurrent    → [0.0 – 400.0]   Total system current (DEBUG)
```

**Available Motor Names:** Flywheel, Turret, Collector, Feeder, RakeArm, RakeIntake

---

## Usage Examples

### In Dashboard

Add this to your Shuffleboard dashboard to give drivers a quick health overview:

```
Layout: "Status"
├─ Health/Overall              (Boolean indicator, BIG RED/GREEN)
├─ Health/Battery              (Boolean + analog voltage)
├─ Health/Flywheel             (Boolean)
├─ Health/Collector            (Boolean)
├─ Health/Feeder               (Boolean)
├─ Health/Turret               (Boolean)
├─ Health/RakeArm              (Boolean)
├─ Health/RakeIntake           (Boolean)
└─ Health/CANBus               (Boolean)
```

### In Code

Check health before executing critical operations:

```java
// In a targeting command or autonomous routine:
public void execute() {
    if (robotContainer.healthMonitor.isOverallHealthy()) {
        // Safe to operate
        shooter.aimAtHub();
    } else {
        // Health issue detected - handle gracefully
        System.err.println("⚠️ Hardware issue detected! Check diagnostics.");
        
        // Can drill down to specific subsystems
        if (!robotContainer.healthMonitor.areAllMotorsHealthy()) {
            DriverStation.reportWarning("Motor issue detected - check telemetry", false);
        }
        if (!robotContainer.healthMonitor.isBatteryHealthy()) {
            DriverStation.reportWarning("Battery voltage critical - brownout risk", false);
        }
    }
}
```

### Post-Match Analysis

After a match, the SignalLogger file will contain timestamped health data:

```
// Log analysis (from .wpilog file):
t=5.23s   Health/Overall = false
t=5.24s   Health/Turret = false      // Issue started here
t=5.24s   Health/Current/Turret = 92.3A
         ↑ Turret jammed during endgame
```

---

## Configuration

Each subsystem's motor health threshold is a static constant in the subsystem class:

```java
// In FlywheelSubsystem
private static final double FLYWHEEL_STALL_THRESHOLD = 80.0;

// In TurretSubsystem
private static final double TURRET_STALL_THRESHOLD = 80.0;

// In CollectorSubsystem
private static final double COLLECTOR_STALL_THRESHOLD = 80.0;

// ... etc for each subsystem
```

Power system thresholds are in `SystemHealthMonitor`:

```java
private static final double BROWNOUT_VOLTAGE_THRESHOLD = 6.5;      // Warning
private static final double CRITICAL_VOLTAGE_THRESHOLD = 6.0;      // Critical
```

### Tuning During Practice

1. Enable `DEBUG_LOGGING = true` in `Constants.java`
2. Run robot through all subsystem motions
3. Watch `Health/Current/` telemetry entries
4. Note maximum normal current for each motor:
   - If motor never exceeds 50A, set threshold to 70A
   - If collector stalls at 90A when jamming, set to 120A for margin
5. Adjust thresholds based on observed data
6. Set `DEBUG_LOGGING = false` before competition

### Kraken X60 Current Specifications

- **Continuous Stall Current:** ~257A
- **Recommended Warning Threshold:** 80A (31% of stall)
- **Typical Operating Current:** 10-40A (depending on load)
- **Peak Current (brief collision):** Can spike to 100-150A momentarily

---

## How It Works

### Architecture: Distributed Monitoring

Rather than a centralized subsystem knowing about all motors, each subsystem monitors its own motors:

```
1. Subsystem Initialization (RobotContainer constructor):
   ├─ Create SystemHealthMonitor aggregator
   ├─ Create subsystems (Flywheel, Turret, Collector, Feeder, RakeArm, RakeIntake)
   │  └─ Each subsystem creates its own MotorHealthMonitor during __init__
   └─ Register all monitors with aggregator
      healthMonitor.registerMonitor(shooter.flywheel.getHealthMonitor());
      healthMonitor.registerMonitor(shooter.turret.getHealthMonitor());
      // ... etc

2. Per-Loop Periodic (50Hz):
   
   a) Subsystem periodic() runs first (in any order):
      ├─ Flywheel: calls masterMotorHealth.update() → samples current
      ├─ Turret: calls motorHealth.update() → samples current
      ├─ Collector: calls motorHealth.update() → samples current
      ├─ Feeder: calls motorHealth.update() → samples current
      ├─ RakeArm: calls motorHealth.update() → samples current
      └─ RakeIntake: calls motorHealth.update() → samples current
   
   b) SystemHealthMonitor.periodic() runs (via CommandScheduler):
      ├─ checkPowerSystem()
      │  ├─ Read battery voltage
      │  ├─ Read CAN bus faults
      │  └─ Set battery_healthy, can_bus_healthy
      │
      ├─ updateOverallHealth()
      │  ├─ Query all motors: motorMonitors.stream().allMatch(m -> m.isHealthy())
      │  └─ Aggregate: overall_healthy = allMotorsHealthy && battery_healthy && can_bus_healthy
      │
      └─ publishTelemetry()
         ├─ Health/Overall → overall_healthy
         ├─ Health/Battery → battery_healthy
         ├─ Health/CANBus → can_bus_healthy
         └─ Health/Power/BatteryVoltage → battery_voltage
```

### Class Responsibilities

| Class | Location | Responsibility |
|-------|----------|-----------------|
| `SystemHealthMonitor` | `frc.lib.util` | Aggregates health, monitors power system, publishes telemetry |
| `SystemHealthMonitor.MotorHealthMonitor` | (nested static) | Monitors single motor current, detects stalls |
| Each Subsystem | `frc.robot.subsystems` | Creates and updates its own MotorHealthMonitor |

### Initialization (RobotContainer)

```java
public class RobotContainer {
    // ... other fields ...
    
    public final SystemHealthMonitor healthMonitor;
    public final ShooterSubsystem shooter;
    public final RakeArmSubsystem rakeArm;
    public final RakeIntakeSubsystem rakeIntake;
    
    public RobotContainer() {
        // 1. Create the system aggregator
        this.healthMonitor = new SystemHealthMonitor(logger, power);
        
        // 2. Create subsystems (each creates its own motor monitor internally)
        this.shooter = new ShooterSubsystem(drivetrain, logger);
            // Internally creates monitors for Flywheel, Turret, Collector, Feeder
            // but does NOT auto-register (passed null)
        this.rakeArm = new RakeArmSubsystem(logger, healthMonitor);
            // Creates monitor and auto-registers with aggregator
        this.rakeIntake = new RakeIntakeSubsystem(logger, healthMonitor);
            // Creates monitor and auto-registers with aggregator
        
        // 3. Manually register shooter subsystem motors
        //    (ShooterSubsystem contains nested subsystems)
        healthMonitor.registerMonitor(shooter.flywheel.getHealthMonitor());
        healthMonitor.registerMonitor(shooter.turret.getHealthMonitor());
        healthMonitor.registerMonitor(shooter.collector.getHealthMonitor());
        healthMonitor.registerMonitor(shooter.feeder.getHealthMonitor());
    }
}
```

### Subsystem Pattern

Each subsystem creates and owns a `MotorHealthMonitor`:

```java
public class MySubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final MotorHealthMonitor motorHealth;
    
    // Constructor signature: accepts optional healthMonitor aggregator
    public MySubsystem(Telemetry telemetry, SystemHealthMonitor healthMonitor) {
        this.motor = createMotor();  // CTRE Phoenix setup
        
        // Create monitor for this motor
        this.motorHealth = new SystemHealthMonitor.MotorHealthMonitor(
            motor,
            "MyMotorName",
            telemetry,
            80.0  // stall threshold in Amps
        );
        
        // Auto-register with aggregator (if provided)
        if (healthMonitor != null) {
            healthMonitor.registerMonitor(motorHealth);
        }
    }
    
    @Override
    public void periodic() {
        // Update motor health (samples stator current each loop)
        motorHealth.update();
        
        // ... rest of subsystem logic ...
    }
    
    public MotorHealthMonitor getHealthMonitor() {
        return motorHealth;
    }
}
```

---

## Troubleshooting

### Motor shows UNHEALTHY but seems to work

**Cause:** Threshold too sensitive for your load  
**Fix:** 
1. Enable DEBUG_LOGGING
2. Run the problematic subsystem
3. Note the peak current under normal operation
4. Increase threshold by ~20% margin
5. Disable DEBUG_LOGGING and redeploy

### Battery shows UNHEALTHY during heavy power draw

**Cause:** Normal voltage sag under load  
**Fix:**
1. Verify battery is charged before each match
2. If voltage drops below 6.5V during operation, your battery may be weak
3. Consider replacing battery if this happens frequently
4. Brownout threshold (6.0V) indicates critical battery condition

### Health/CANBus shows FALSE

**Cause:** CAN communication error detected  
**Fix:**
1. Check that all CAN connector are seated firmly
2. Verify terminating resistors on CAN bus (should be 2, one on each end)
3. Check for CAN line contention (multiple devices on same address)
4. Run CAN diagnostics via CTRE Phoenix Tuner

### Overall Health stays FALSE even though individual subsystems show TRUE

**Cause:** Power system check failing (battery or CAN bus)  
**Fix:**
1. Call `robotContainer.healthMonitor.isBatteryHealthy()` to check battery specifically
2. Call `robotContainer.healthMonitor.isCANBusHealthy()` to check CAN bus
3. If battery is failing:
   - Verify battery voltage with `robotContainer.healthMonitor.getBatteryVoltage()`
   - Charge battery if below 6.5V
4. If CAN bus is failing:
   - Check all CAN connector seating
   - Verify CAN terminating resistors (2 total, one on each end)

---

## During Matches

### Pre-Match Checklist

- [ ] Verify `Health/Overall` shows TRUE on driver station
- [ ] Check `Health/Battery` shows TRUE (voltage should be ~12.2V)
- [ ] Verify all subsystem health indicators show TRUE
- [ ] Confirm motor speeds are responsive during test mode
- [ ] Programmatically verify with: `robotContainer.healthMonitor.isOverallHealthy()`

### During Match

- **Watch** the `Health/Overall` indicator on the dashboard
- **If it goes RED** during a match:
  1. Note the timestamp
  2. Drill down using query methods:
     - `robotContainer.healthMonitor.areAllMotorsHealthy()` — Check if motor issue
     - `robotContainer.healthMonitor.isBatteryHealthy()` — Check battery voltage
     - `robotContainer.healthMonitor.isCANBusHealthy()` — Check CAN communication
  3. Adjust strategy if needed (e.g., avoid shooting if flywheel unhealthy)
  4. Post-match, review the log to understand what happened

### Post-Match Analysis

1. Connect laptop and download logs to the `logs/` folder
2. Open `.wpilog` file with AdvantageScope
3. Search for `Health/` entries on the dashboard
4. Review timeline of when health indicators changed
5. Use `Health/Motors/{Name}/Current` to identify stall conditions or mechanical failures

---

## Integration with Autonomous

Example: Safe autonomous that degrades gracefully

```java
@Override
public void autonomousInit() {
    // Query the health monitor to decide which auto to run
    if (!robotContainer.healthMonitor.isOverallHealthy()) {
        System.err.println("⚠️ Hardware issue detected before auto!");
        
        // Diagnose which subsystem is failing
        if (!robotContainer.healthMonitor.areAllMotorsHealthy()) {
            DriverStation.reportWarning("Motor health issue - running minimal auto", false);
        }
        if (!robotContainer.healthMonitor.isBatteryHealthy()) {
            DriverStation.reportWarning("Battery critically low - running safe auto", false);
        }
        
        // Run minimal safe auto instead
        autonomousCommand = getMinimalAuto();
    } else {
        // All systems nominal - run full, aggressive autonomous
        autonomousCommand = getFullAuto();
    }
}

@Override
public Command getAutonomousCommand() {
    // Autonomous chooser from PathPlanner
    Command autoCommand = robotContainer.getAutonomousCommand();
    
    // Wrap it with a health check that cancels if hardware fails mid-auto
    return new ConditionalCommand(
        autoCommand,  // if healthy
        new PrintCommand("Auto canceled - hardware failure detected"),  // if not healthy
        () -> robotContainer.healthMonitor.isOverallHealthy()
    );
}
```

---

## Performance Impact

- **CPU Time:** < 1ms per loop (minimal, just current reads)
- **Memory:** ~500 bytes (booleans and telemetry caches)
- **Network:** ~50 bytes per loop to dashboard (< 0.1% of bandwidth)

---

## Future Enhancements

- [ ] Temperature monitoring via motor firmware
- [ ] Encoder error tracking for position-based subsystems
- [ ] Brownout event logging with event timestamps
- [ ] Motor response time tracking (detect sluggish behavior)
- [ ] CAN message latency monitoring
- [ ] Power draw trending (predict battery depletion)

---

## Summary

The Health Monitoring system provides:
✅ Per-motor current tracking  
✅ Battery voltage monitoring  
✅ CAN bus diagnostics  
✅ Real-time driver feedback  
✅ Post-match analysis capability  
✅ Minimal performance overhead  

This is critical infrastructure for understanding hardware failures during competition and post-match analysis.

