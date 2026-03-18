# FRC 2026 Logging Level Audit Report

**Date:** March 18, 2026  
**Status:** Competition-Focused Review  
**Mode:** DEBUG_LOGGING = false (Competition)

---

## Executive Summary

Your logging infrastructure is well-designed with good use of the `debugMode` parameter to gate expensive operations. However, there are several opportunities to optimize for competition:

- **Current State:** Good separation of DEBUG vs. CRITICAL telemetry
- **Issue Found:** Some non-critical fields marked as CRITICAL; some DEBUG-level data logged unnecessarily
- **Risk:** Minimal performance impact, but disk logging overhead could accumulate in long events
- **Recommendations:** Reclassify 3-5 fields and optimize 2 hot loops

---

## Detailed Audit Results

### ✅ **Well-Designed Logging** (No Changes Needed)

#### Telemetry.java
- ✅ `m_poseX, m_poseY, m_poseRotation` → **CRITICAL** — Essential for driver feedback and post-match analysis
- ✅ `m_odometryPeriod` → **CRITICAL** — Diagnostic for odometry health
- ✅ `m_lastPose, lastTime` → **DEBUG** — Only used for velocity calculations, debug-level is correct
- ✅ `field` → **CRITICAL** — Displayed in dashboard and logs
- ✅ SignalLogger properly configured with `enableAutoLogging(false)` — Avoids spam

#### ShooterSubsystem.java
- ✅ `activeShotProfile` → **CRITICAL** — Determines shot accuracy; critical for post-match review
- ✅ `turretAngleToTarget, distanceToTarget` → **CRITICAL** — Direct impact on shooting success
- ✅ `turretFieldPosition, vectorToTarget, fieldAngleToTarget, turretRelativeAngle` → **CRITICAL** — Intermediate calculations logged for debugging why shots missed
- ✅ `lastAppliedFlywheelVelocity` → **@NotLogged** — Correct; internal tracking not needed in logs

#### VisionSubsystem.java
- ✅ Telemetry calls use `false` parameter to gate behind DEBUG mode — Excellent practice
- ✅ `Vision/Pipeline` and `Vision/ActiveMode` logged with `true` (always) — Correct; pipeline changes are critical diagnostics

#### TurretSubsystem.java
- ✅ `turretTargetAngle, potentiometerAngle` → **CRITICAL** — Encoder health is critical for competition

---

### 🟡 **Questionable Classifications** (Review These)

| Field | Current | File | Issue | Recommendation |
|-------|---------|------|-------|-----------------|
| `m_isTeleop` | **DEBUG** | CollectorSubsystem, FeederSubsystem, FlywheelSubsystem, RakeArmSubsystem, RakeIntakeSubsystem, TurretSubsystem | Logs when operator manually controls vs. auto-controlled. Useful during matches? | ⚠️ **Consider INFO** if tracking operator engagement is valuable; **DEBUG** if purely for development |
| `m_maxSpeed` | **DEBUG** | Telemetry.java | Max speed changes (cruise/slow/max). During competition, you want to know when speed limits changed. | ⚠️ **Recommend: INFO or CRITICAL** |

---

### 🔴 **Performance Issues** (Potential Optimization)

#### 1. **ShooterSubsystem Telemetry in `updateTelemetry()` (Lines 118–123)**

**Current Code:**
```java
private void updateTelemetry() {
    telemetry.putNumber("Shooter/TurretFieldX", turretFieldPosition.getX(), true);
    telemetry.putNumber("Shooter/TurretFieldY", turretFieldPosition.getY(), true);
    telemetry.putNumber("Shooter/TurretAngleToTargetDeg", turretAngleToTarget.in(Degrees), true);
    telemetry.putNumber("Shooter/DistanceToTargetM", distanceToTarget.in(Meters), true);
    telemetry.putNumber("Shooter/FlywheelVelocityRPM", getFlywheelVelocity().in(RPM), true);
    telemetry.putString("Shooter/ActiveShotProfile", activeShotProfile, true);
}
```

**Issue:** All 6 telemetry calls marked `true` (always log) are called **every 50ms during competition**. The intermediate values (TurretFieldX/Y, vectorToTarget components) are recalculated even when not targeting.

**Impact:** Minimal for 1 subsystem, but across all subsystems this adds overhead. Under high load (during endgame), CPU time could matter.

**Optimization:**
```java
private void updateTelemetry() {
    // Always log critical targeting data (needed for post-match analysis)
    telemetry.putNumber("Shooter/TurretAngleToTargetDeg", turretAngleToTarget.in(Degrees), true);
    telemetry.putNumber("Shooter/DistanceToTargetM", distanceToTarget.in(Meters), true);
    telemetry.putNumber("Shooter/FlywheelVelocityRPM", getFlywheelVelocity().in(RPM), true);
    telemetry.putString("Shooter/ActiveShotProfile", activeShotProfile, true);
    
    // Intermediate calculations: only during targeting or debug mode
    telemetry.putNumber("Shooter/TurretFieldX", turretFieldPosition.getX(), false);
    telemetry.putNumber("Shooter/TurretFieldY", turretFieldPosition.getY(), false);
}
```

---

#### 2. **VisionSubsystem `shouldAcceptVisionEstimate()` (Lines 389–404)**

**Current Code:**
```java
private boolean shouldAcceptVisionEstimate(PoseEstimate est, String telemetryPrefix) {
    String rejectionReason = getVisionEstimateRejectionReason(est);
    if (rejectionReason.isEmpty() && !isFreshVisionEstimate(est)) {
      rejectionReason = "staleTimestamp";
    }

    boolean accepted = rejectionReason.isEmpty();
    telemetry.putBoolean(telemetryPrefix + "/Accepted", accepted, false);                    // DEBUG
    telemetry.putString(telemetryPrefix + "/RejectReason", accepted ? "accepted" : rejectionReason, false);  // DEBUG
    telemetry.putBoolean(telemetryPrefix + "/PoseOnField", ..., false);                      // DEBUG
    telemetry.putNumber(telemetryPrefix + "/TagCount", est.tagCount, false);                 // DEBUG
    // ... more telemetry
    return accepted;
}
```

**Issue:** **Every vision update attempt** calls 5+ telemetry operations, even if rejected. During startup or with poor vision, this runs 50 times/second.

**Impact:** Low but unnecessary during competition. Debug telemetry correctly gated by `false` parameter.

**Current Status:** Already optimized (gated by `false`); no change needed but worth monitoring.

---

### ✅ **Good Practices Observed**

1. ✅ **Telemetry.putNumber/String/Boolean with alwaysLog parameter** — Excellent practice
2. ✅ **Consistent use of `true` for critical data, `false` for debug** — Well-structured
3. ✅ **@NotLogged for internal fields** — Avoids unnecessary serialization
4. ✅ **Field2d visualization marked CRITICAL** — Driver needs real-time pose feedback
5. ✅ **Vision debug details only logged with `false`** — Prevents spam

---

## Recommendations by Priority

| Priority | Action | File(s) | Impact | Effort |
|----------|--------|---------|--------|--------|
| 🟢 P1 | Consider reclassifying `m_maxSpeed` from DEBUG → INFO | Telemetry.java | Low | 1 min |
| 🟡 P2 | Audit `m_isTeleop` usage: is it needed for post-match analysis? | All subsystems | Very low | 5 min |
| 🟠 P3 | Move intermediate shooter calcs to DEBUG telemetry | ShooterSubsystem.java | Very low | 3 min |
| 🟠 P4 | Consider adding motor current/CAN error logging | All subsystems | Low | 30 min |
| 🔵 INFO | Continue monitoring telemetry output during practice | All | N/A | Ongoing |

---

## Proposed Changes

### Change 1: Upgrade `m_maxSpeed` logging importance

**File:** `Telemetry.java` (Line 28)

**Rationale:** Driver's speed limit changes (cruise/slow/max) are critical diagnostics. When analyzing why a robot missed a shot or collision, knowing if the robot was artificially speed-limited is essential.

**Current:**
```java
@Logged(importance = Logged.Importance.DEBUG)
private double m_maxSpeed;
```

**Recommendation:**
```java
@Logged(importance = Logged.Importance.INFO)
private double m_maxSpeed;  // Speed limit changes are significant events
```

---

### Change 2: Optimize ShooterSubsystem intermediate telemetry

**File:** `ShooterSubsystem.java` (Lines 118–123)

**Current:**
```java
private void updateTelemetry() {
    telemetry.putNumber("Shooter/TurretFieldX", turretFieldPosition.getX(), true);
    telemetry.putNumber("Shooter/TurretFieldY", turretFieldPosition.getY(), true);
    telemetry.putNumber("Shooter/TurretAngleToTargetDeg", turretAngleToTarget.in(Degrees), true);
    // ...
}
```

**Optimized:**
```java
private void updateTelemetry() {
    // Critical targeting data (always logged for post-match analysis)
    telemetry.putNumber("Shooter/TurretAngleToTargetDeg", turretAngleToTarget.in(Degrees), true);
    telemetry.putNumber("Shooter/DistanceToTargetM", distanceToTarget.in(Meters), true);
    telemetry.putNumber("Shooter/FlywheelVelocityRPM", getFlywheelVelocity().in(RPM), true);
    telemetry.putString("Shooter/ActiveShotProfile", activeShotProfile, true);
    
    // Intermediate geometric calculations (debug only; not essential for match analysis)
    telemetry.putNumber("Shooter/TurretFieldX", turretFieldPosition.getX(), false);
    telemetry.putNumber("Shooter/TurretFieldY", turretFieldPosition.getY(), false);
}
```

**Rationale:** TurretFieldX/Y are intermediate values used for debugging geometry. During competition, the final angle-to-target is what matters. This reduces unnecessary logging while keeping debugging capability via DEBUG_LOGGING flag.

---

### Change 3: Clarify `m_isTeleop` intent (No code change needed)

**Finding:** Six subsystems log `m_isTeleop` at DEBUG level. This tracks when operators manually control a subsystem.

**Questions:**
- Is this useful for post-match review? (e.g., "Did the operator ever touch the flywheel?")
- Or is it purely for development debugging?

**Recommendation:** If useful for analysis, keep at DEBUG. If not needed, consider removing (saves serialization overhead). Current implementation is correct either way.

---

## Summary: Competition Readiness

| Aspect | Status | Notes |
|--------|--------|-------|
| **Annotation Syntax** | ✅ Fixed | All CRITICAL/DEBUG/INFO correct |
| **Gate Strategy** | ✅ Good | Proper use of `alwaysLog` parameter |
| **Log Redundancy** | ⚠️ Minor | Some intermediate values logged unnecessarily |
| **CAN/Hardware Health** | ⚠️ Missing | No motor current/error monitoring |
| **Vision Watchdog** | ✅ Added | Detects camera disconnect |
| **Overall** | ✅ Ready | Logging optimizations are nice-to-have, not blocking |

---

## During-Match Monitoring Checklist

- [ ] Verify SignalLogger file size does not grow > 100 MB per 2.5-minute match
- [ ] Check that no unexpected telemetry entries appear (indicates code path not exercised in testing)
- [ ] Monitor robot performance: no brownouts or CAN timeouts
- [ ] Confirm vision health indicator working (Vision/IsHealthy or similar)
- [ ] Review post-match: can you explain why each shot succeeded/failed from telemetry?

---

## Next Steps

1. **Immediately:** Apply recommended changes from "Proposed Changes" section (5 min)
2. **Before Competition:** Run practice match with logging enabled; review log file size and contents
3. **Post-Match:** Examine SignalLogger output (.wpilog files) to verify no unexpected growth
4. **Tuning Days:** Enable DEBUG_LOGGING to capture detailed intermediate values for troubleshooting

