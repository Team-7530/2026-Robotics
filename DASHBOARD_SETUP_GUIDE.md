# Driver Dashboard Setup Guide - Health Monitoring

**Purpose:** Quick reference for setting up Shuffleboard tabs to display health monitoring data during competition.

---

## Recommended Dashboard Layout

### Tab 1: "Status" (Critical - Always Visible)

This is your "at a glance" health check during competition.

```
┌─────────────────────────────────────────┐
│          ROBOT STATUS DASHBOARD         │
├─────────────────────────────────────────┤
│                                         │
│  ⚫ Health/Overall           [TRUE ✓]  │
│                            (Very Large) │
│                                         │
│  Battery: 12.2V  [Health/Battery ✓]    │
│  Total Current: 45A                     │
│                                         │
│  Motor Health:                          │
│  ✓ Flywheel  ✓ Collector  ✓ Feeder    │
│  ✓ Turret    ✓ RakeArm    ✓ RakeInt   │
│                                         │
│  CAN Bus: [Health/CANBus ✓]            │
│                                         │
│  Vision: [Vision/IsHealthy ✓]          │
│                                         │
└─────────────────────────────────────────┘
```

**Add these entries to your dashboard (in this order):**

1. `Health/Overall` — **HUGE boolean indicator** (make this 4x normal size; this is what drivers watch)
2. `Health/Power/BatteryVoltage` — Analog gauge (6.0V - 13V range)
3. `Health/Battery` — Boolean (next to voltage)
4. `Health/Flywheel` — Boolean
5. `Health/Collector` — Boolean  
6. `Health/Feeder` — Boolean
7. `Health/Turret` — Boolean
8. `Health/RakeArm` — Boolean
9. `Health/RakeIntake` — Boolean
10. `Health/CANBus` — Boolean
11. `Vision/IsHealthy` — Boolean (from VisionSubsystem watchdog)

---

### Tab 2: "Diagnostics" (Tuning/Debug - Visible in Practice)

Use this during practice and tuning days to understand current draw.

```
┌─────────────────────────────────────────┐
│         HARDWARE DIAGNOSTICS            │
├─────────────────────────────────────────┤
│                                         │
│  Current Draw (Amps):                   │
│  • Flywheel:       Health/Current/Flywheel      │
│  • Collector:      Health/Current/Collector    │
│  • Feeder:         Health/Current/Feeder       │
│  • Turret:         Health/Current/Turret       │
│  • Rake Arm:       Health/Current/RakeArm      │
│  • Rake Intake:    Health/Current/RakeIntake   │
│  ─────────────────────────────────────  │
│  • Total Current:  Health/Power/TotalCurrent   │
│                                         │
│  Battery Voltage: Health/Power/BatteryVoltage  │
│                                         │
│  Targeting (from ShooterSubsystem):     │
│  • Angle to Target: Shooter/TurretAngleToTargetDeg │
│  • Distance:        Shooter/DistanceToTargetM      │
│  • RPM:             Shooter/FlywheelVelocityRPM    │
│  • Profile:         Shooter/ActiveShotProfile      │
│                                         │
└─────────────────────────────────────────┘
```

**Add these entries (only visible when DEBUG_LOGGING=true):**

1. `Health/Current/Flywheel`
2. `Health/Current/Collector`
3. `Health/Current/Feeder`
4. `Health/Current/Turret`
5. `Health/Current/RakeArm`
6. `Health/Current/RakeIntake`
7. `Health/Power/TotalCurrent`
8. `Shooter/TurretAngleToTargetDeg`
9. `Shooter/DistanceToTargetM`
10. `Shooter/FlywheelVelocityRPM`
11. `Shooter/ActiveShotProfile`

---

### Tab 3: "Field" (Vision - Always Updated)

Standard field visualization with robot pose.

```
┌─────────────────────────────────────────┐
│         FIELD VISUALIZATION             │
├─────────────────────────────────────────┤
│                                         │
│    [Field2d from Telemetry]             │
│    Shows robot pose, target points      │
│                                         │
│  Vision Pipeline: Vision/Camera/limelight/Pipeline │
│  Active Mode:     Vision/ActiveMode                │
│                                         │
└─────────────────────────────────────────┘
```

**Add these entries:**

1. `Field` (Field2d visualization)
2. `Vision/Camera/limelight/Pipeline`
3. `Vision/ActiveMode`

---

## Interpreting the Dashboard

### Health/Overall Indicator

| Condition | Meaning | Action |
|-----------|---------|--------|
| 🟢 **TRUE** | All systems healthy | Proceed normally; all subsystems ready |
| 🔴 **FALSE** | At least one system unhealthy | Investigate which subsystem failed; adapt strategy |

### When Health Indicator Goes RED During Match

1. **Look at individual motor indicators** to identify which failed
2. **Check battery voltage** — if low, reduce power draw
3. **Check CAN bus** — if unhealthy, may have electrical connection issue
4. **Note the timestamp** for post-match analysis
5. **Adapt your strategy** — if flywheel is down, focus on other mechanisms

### Battery Voltage Guide

| Voltage | Status | Action |
|---------|--------|--------|
| 12.0 – 12.6V | 🟢 Optimal | Run full power |
| 11.5 – 12.0V | 🟡 Good | Normal operation |
| 10.5 – 11.5V | 🟡 Warning | Approaching concerns |
| 6.5 – 10.5V | 🟠 Brownout Risk | Reduce power; avoid stressing motors |
| < 6.0V | 🔴 Critical | Mission abort; potential brownout |

**Note:** Voltage sags under load. This is normal. Concern if it stays below 6.5V for more than a few seconds.

### Current Draw Interpretation

**Normal Operating Currents:**
- Idle (no load): 0 – 2A per motor
- Normal operation: 10 – 40A per motor
- Stall/jam: > 80A (threshold for health indicator)

**What high current means:**
- 60 – 80A: Heavy load, possibly approaching jam
- 80A+: Likely jam or mechanical failure; immediate attention needed

---

## Pre-Match Verification Sequence

**5 minutes before match:**

```
Driver/Coach performs this check:

1. [ ] Health/Overall shows TRUE
2. [ ] Health/Battery shows TRUE
3. [ ] Battery voltage is 12.0V or higher
4. [ ] All 6 motor health indicators show TRUE
5. [ ] Health/CANBus shows TRUE
6. [ ] Vision/IsHealthy shows TRUE
7. [ ] Field2d shows robot at correct position

If ANY indicator is FALSE:
  → Investigate and resolve before match
  → If unfixable, inform coaching staff
  → Prepare degraded strategy
```

---

## During Match Monitoring

**Live Watch (continuous):**
- Primary focus: `Health/Overall` indicator
- Secondary: `Health/Battery` voltage
- Tertiary: Individual subsystem health if overall goes RED

**Sample reactions:**
```
Scenario 1: Health/Overall goes RED during auto
└─ Check which motor failed
   ├─ If Flywheel: Can still aim turret and operate collector
   ├─ If Turret: Can't aim; focus on pickup strategy
   └─ If Battery: Reduce all power draw; switch to conservative play

Scenario 2: Battery voltage drops to 6.5V during endgame
└─ This is expected under heavy load
   ├─ If it recovers to 8V+: Safe to continue
   ├─ If it stays below 6.5V for 2+ seconds: Brownout risk
   └─ React: Reduce motor power, finish climb if safe, park robot

Scenario 3: CAN Bus indicator goes RED
└─ This suggests loose CAN connector or ID conflict
   ├─ Likely to be transient
   ├─ Continue operating; may self-recover
   └─ Note for post-match troubleshooting
```

---

## Post-Match Analysis

**Within 3 minutes of match end:**

1. Connect laptop to robot (via USB or Ethernet)
2. Download `.wpilog` file (usually in `/home/lvuser/logs/`)
3. Open in FRC log viewer tool
4. Search for `Health/` entries
5. Review timeline:
   - When did each indicator change?
   - What was the sequence of failures?
   - Did motor current spike before failure?
   - What was battery voltage at time of failure?

**Example log review:**
```
t=42.3s   Health/Overall = TRUE  (everything OK)
t=93.5s   Health/Flywheel = FALSE  (current spike to 95A)
          Health/Current/Flywheel = 95.2A
          → Flywheel jammed picking up a piece
          
t=93.7s   Health/Overall = FALSE  (aggregated failure)

t=94.2s   Health/Flywheel = TRUE  (cleared jam, recovered)
t=94.3s   Health/Overall = TRUE  (fully recovered)
```

**Actions after review:**
- [ ] Document mechanical issues found
- [ ] Plan maintenance repairs
- [ ] Adjust game strategy if needed
- [ ] Share findings with mechanics team

---

## Configuration for Competition

### Shuffleboard Setup

1. **Create new layout named "Competition"**
2. **Add "Status" tab** — visible from kickoff
3. **Add "Field" tab** — for vision/strategy review
4. **Hide "Diagnostics" tab** — only enable during practice
5. **Save layout** to team folder

### WPILib Preferences

File: `~/wpilib/preferences.json` or set via LiveWindow

```json
{
  "team": "7530",
  "teamNumber": 7530,
  "autoOpenDiagnostics": false,
  "preferencesTabLocation": "TOP_LEFT"
}
```

---

## Troubleshooting Dashboard Issues

### Entries not appearing

**Cause:** Fields not being logged  
**Fix:**
1. Verify `HealthMonitoringSubsystem` is scheduled (instantiated in RobotContainer)
2. Check that telemetry is being called (should be in `periodic()`)
3. Restart dashboard and robot code

### All health indicators RED at startup

**Cause:** Motors not yet initialized or telemetry lagging  
**Fix:**
1. Wait 2 seconds for full startup
2. Check that `health.registerMotors()` calls completed
3. Look for ERROR messages in RioLog

### Battery voltage showing 0V

**Cause:** RobotController not initialized  
**Fix:**
1. Verify robot is powered and radio initialized
2. Check USB/Ethernet connection to RoboRIO
3. Restart dashboard

### Vision health always FALSE

**Cause:** Vision measurements not being accepted  
**Fix:**
1. Verify Limelight is powered and connected
2. Check `Vision/Camera/limelight/Pipeline` is correct index
3. Verify April tags are visible on field
4. Check vision standard deviation thresholds in VisionSubsystem

---

## Quick Command Reference

### Enable Debug Telemetry (Practice Only)
```java
// In Constants.java:
public static final boolean DEBUG_LOGGING = true;  // Change to false for competition
```

### Check Health from Code
```java
if (robotContainer.health.isOverallHealthy()) {
    shooter.aimAtHub();
} else {
    System.err.println("Hardware issue - checking details...");
    if (!robotContainer.health.isFlywheelHealthy()) {
        DriverStation.reportWarning("Flywheel current spike", false);
    }
}
```

### Manually Test Health Monitoring
```java
// In test mode, jam each mechanism and verify health indicator goes RED
// Then release and verify it goes GREEN again

// To simulate brownout:
// Spin all motors at full power and watch battery voltage drop
// Verify Health/Battery goes RED if voltage drops below 6.5V
```

---

## Summary

**Driver's job:**
- Watch `Health/Overall` during match
- If it goes RED, identify which subsystem failed
- Adapt strategy accordingly
- Note timestamp for mechanics team

**Coaching staff's job:**
- Monitor battery voltage
- If below 6.5V, reduce power draw
- Plan endgame strategy based on health status

**Mechanics team's job:**
- Post-match, review health logs
- Identify mechanical failures
- Plan repairs/maintenance
- Communicate findings to programming team

**Programming team's job:**
- Ensure health monitoring is accurate
- Adjust current thresholds based on tuning data
- Respond to hardware failures in auto/teleop code

---

**Dashboard Setup Complete! ✅**

Your team now has real-time hardware diagnostics throughout competition.

