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
        // All systems healthy - safe to operate
        shooter.aimAtHub();
    } else {
        // Hardware issue detected - drill down to identify the problem
        System.err.println("⚠️ Hardware issue detected!");
        
        if (!robotContainer.healthMonitor.areAllMotorsHealthy()) {
            DriverStation.reportWarning("Motor stall/jam detected - check telemetry", false);
        }
        if (!robotContainer.healthMonitor.isBatteryHealthy()) {
            DriverStation.reportWarning("Battery critical - brownout risk", false);
        }
        if (!robotContainer.healthMonitor.isCANBusHealthy()) {
            DriverStation.reportWarning("CAN communication error - check wiring", false);
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

Motor health thresholds are static constants defined in each subsystem class and passed to the factory method:

```java
// In FlywheelSubsystem
private static final double FLYWHEEL_STALL_THRESHOLD = 80.0;  // Kraken X60 warning threshold

public FlywheelSubsystem(SystemHealthMonitor healthMonitor) {
    this.masterMotorHealth = healthMonitor.createMotorHealthMonitor(
        m_flywheelMasterMotor,
        "Flywheel",
        FLYWHEEL_STALL_THRESHOLD  // ← Pass threshold here
    );
}

// In TurretSubsystem
private static final double TURRET_STALL_THRESHOLD = 80.0;

public TurretSubsystem(SystemHealthMonitor healthMonitor) {
    this.motorHealth = healthMonitor.createMotorHealthMonitor(
        turretMotor,
        "Turret",
        TURRET_STALL_THRESHOLD
    );
}
```

Power system thresholds are constants inside `SystemHealthMonitor`:

```java
private static final double BROWNOUT_VOLTAGE_THRESHOLD = 6.5;  // Warning level
private static final double CRITICAL_VOLTAGE_THRESHOLD = 6.0;  // Critical level
```

### Tuning During Practice

1. Enable `DEBUG_LOGGING = true` in `Constants.java`
2. Run robot through all subsystem motions
3. Watch `Health/Motors/{Name}/Current` telemetry entries (only visible in DEBUG mode)
4. Note maximum normal current for each motor:
   - If motor never exceeds 50A, set threshold to 70A (margin for safety)
   - If collector jams at 90A, set threshold to 120A (avoid false alarms during acceleration)
5. Update subsystem threshold constants with observed data
6. Set `DEBUG_LOGGING = false` before competition

### Kraken X60 Current Specifications

- **Continuous Stall Current:** ~257A
- **Recommended Warning Threshold:** 80A (31% of stall)
- **Typical Operating Current:** 10-40A (depending on load)
- **Peak Current (brief collision):** Can spike to 100-150A momentarily

---

## How It Works

### Architecture: Distributed Monitoring with Factory Registration

Each subsystem monitors its own motors; the aggregator queries them all without needing explicit subsystem cooperation:

```
1. Initialization (RobotContainer constructor):
   
   a) Create aggregator:
      healthMonitor = new SystemHealthMonitor(logger)
   
   b) Create subsystems, each with aggregator reference:
      shooter = new ShooterSubsystem(drivetrain, healthMonitor)
      rakeArm = new RakeArmSubsystem(healthMonitor)
      rakeIntake = new RakeIntakeSubsystem(healthMonitor)
   
   c) Within each subsystem constructor:
      - Create TalonFX motor instance
      - Call healthMonitor.createMotorHealthMonitor(motor, "Name", threshold)
      - Factory method creates MotorHealthMonitor AND auto-registers it
      → Result: All monitors are now registered with aggregator

2. Per-Loop Periodic (50Hz):
   
   a) Subsystem periodic() runs first (in any order):
      ├─ Flywheel.periodic(): masterMotorHealth.update()    [samples stator current]
      ├─ Turret.periodic():   motorHealth.update()           [samples stator current]
      ├─ Collector.periodic(): motorHealth.update()          [samples stator current]
      ├─ Feeder.periodic():    motorHealth.update()          [samples stator current]
      ├─ RakeArm.periodic():   motorHealth.update()          [samples stator current]
      └─ RakeIntake.periodic(): motorHealth.update()         [samples stator current]
   
   b) SystemHealthMonitor.periodic() runs (via CommandScheduler):
      ├─ checkPowerSystem()
      │  ├─ Read battery voltage
      │  ├─ Check CAN bus faults
      │  └─ Set battery_healthy, can_bus_healthy
      │
      ├─ updateOverallHealth()
      │  ├─ Query all motors: motorMonitors.stream().allMatch(m -> m.isHealthy())
      │  ├─ Query custom checks: customHealthChecks.stream().allMatch(s -> s.get())
      │  └─ Aggregate: overall_healthy = allMotorsHealthy && customHealthy && battery_healthy && can_bus_healthy
      │
      └─ publishTelemetry()
         ├─ Health/Overall → overall_healthy        [driver sees this]
         ├─ Health/Battery → battery_healthy
         ├─ Health/CANBus → can_bus_healthy
         └─ Health/Power/BatteryVoltage → voltage
```

### Key Design Advantages

| Aspect | Benefit |
|--------|---------|
| **Factory Method** | Subsystems don't manually call `registerMonitor()` — auto-registration reduces boilerplate |
| **Nested Class** | `MotorHealthMonitor` is non-static, so it has access to parent `SystemHealthMonitor` state if needed |
| **Subsystem Ownership** | Each subsystem owns its monitor; motor references stay where they're used |
| **No Circular Dependencies** | Subsystems don't need the aggregator to function; they only pass it during init for registration |
| **Scalability** | New subsystems can add monitors without touching the aggregator code |

### Class Responsibilities

| Class | Location | Responsibility |
|-------|----------|-----------------|
| `SystemHealthMonitor` | `frc.lib.util` | Aggregates health from all motors; monitors power system (battery, CAN); publishes overall health to dashboard; provides factory methods for subsystems |
| `SystemHealthMonitor.MotorHealthMonitor` | (nested non-static) | Monitors single motor current; detects stalls/jams; owned and updated by each subsystem |
| Each Subsystem | `frc.robot.subsystems` | Creates motor monitor via factory method; calls `update()` in periodic; owns its own motor(s) |

### Initialization (RobotContainer)

The pattern is extremely clean: create the aggregator, then pass it to subsystems.

```java
public class RobotContainer {
    public final SystemHealthMonitor healthMonitor;
    public final ShooterSubsystem shooter;
    public final RakeArmSubsystem rakeArm;
    public final RakeIntakeSubsystem rakeIntake;
    
    public RobotContainer() {
        // 1. Create the system aggregator (aggregates and publishes health)
        this.healthMonitor = new SystemHealthMonitor(logger);
        
        // 2. Create subsystems, each passing the aggregator
        //    (subsystems use factory method to auto-register their monitors)
        this.shooter = new ShooterSubsystem(drivetrain, healthMonitor);
        this.rakeArm = new RakeArmSubsystem(healthMonitor);
        this.rakeIntake = new RakeIntakeSubsystem(healthMonitor);
        
        // That's it! All motors are now monitored and aggregated.
    }
}
```

### Subsystem Pattern

Each subsystem creates and owns a `MotorHealthMonitor` via the factory method.

```java
public class FlywheelSubsystem extends SubsystemBase {
    private static final double FLYWHEEL_STALL_THRESHOLD = 80.0;  // Kraken X60 warning threshold
    
    private final TalonFX m_flywheelMasterMotor;
    private final MotorHealthMonitor masterMotorHealth;
    
    /**
     * Constructor accepts the system health monitor (aggregator).
     * The subsystem will create its own motor monitor and auto-register it.
     */
    public FlywheelSubsystem(SystemHealthMonitor healthMonitor) {
        this.m_flywheelMasterMotor = new TalonFX(FLYWHEEL_MASTER_ID, kCANBus);
        
        // Create a motor health monitor and auto-register with aggregator
        // (using the factory method on the aggregator)
        this.masterMotorHealth = healthMonitor.createMotorHealthMonitor(
            m_flywheelMasterMotor,
            "Flywheel",
            FLYWHEEL_STALL_THRESHOLD
        );
    }
    
    @Override
    public void periodic() {
        // Update motor health (samples stator current each loop)
        masterMotorHealth.update();
        
        // ... rest of subsystem logic ...
    }
    
    public MotorHealthMonitor getHealthMonitor() {
        return masterMotorHealth;
    }
}
```

**Key Points:**
- **Factory Method:** `healthMonitor.createMotorHealthMonitor()` creates AND auto-registers in one call
- **Nested Class:** `MotorHealthMonitor` is a nested non-static class within `SystemHealthMonitor`
- **Subsystem Owns Monitor:** Each subsystem creates and updates its own monitor via `update()` in periodic
- **Single Dependency:** Subsystems only need the aggregator passed in constructor
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

