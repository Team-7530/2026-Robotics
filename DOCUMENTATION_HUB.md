# Robotics Documentation Hub

**Last Updated:** July 27, 2026

This is the current entry point for robot documentation. It keeps the active guides in one place and captures the process patterns we used during tuning, integration, and season prep.

## Start Here

1. `HARDWARE_HEALTH_MONITORING_GUIDE.md`
   - Driver-facing health indicators
   - Motor current thresholds
   - Post-match troubleshooting

2. `DASHBOARD_SETUP_GUIDE.md`
   - Shuffleboard layout
   - What to watch during a match
   - How to verify health data before a match

3. `src/main/java/frc/robot/operator_interface/TestControllerBindings.java`
   - Test controller layout for tuning
   - SysId button map for subsystems

## SysId Cheat Sheet

Current test-controller map:

| Modifier | Direction | Buttons | Mechanism |
|---|---|---|---|
| `Start` + `Left Bumper` | Quasistatic forward | `A` | Turret |
| `Start` + `Left Bumper` | Quasistatic forward | `B` | Rake arm |
| `Start` + `Left Bumper` | Quasistatic forward | `X` | Flywheel |
| `Start` + `Left Bumper` | Quasistatic forward | `Y` | Feeder |
| `Start` + `Left Bumper` | Quasistatic forward | Left stick button | Collector |
| `Start` + `Left Bumper` | Quasistatic forward | Right stick button | Rake intake |
| `Start` + `Right Bumper` | Quasistatic reverse | Same buttons as above | Same mechanisms |
| `Back` + `Left Bumper` | Dynamic forward | Same buttons as above | Same mechanisms |
| `Back` + `Right Bumper` | Dynamic reverse | Same buttons as above | Same mechanisms |

Notes:
- `Start` selects quasistatic SysId.
- `Back` selects dynamic SysId.
- `Left Bumper` selects forward.
- `Right Bumper` selects reverse.
- A binding only runs when the matching modifiers and mechanism button are held together.
- These commands are for tuning only and should stay disabled in competition builds unless you are actively characterizing hardware.

## Active References

- `HARDWARE_HEALTH_MONITORING_GUIDE.md`
- `DASHBOARD_SETUP_GUIDE.md`

## Recommended Process

These are the high-level practices that worked well during performance-tuning and integration sessions:

1. Start with one clear problem or subsystem.
   - Define what is changing and what should stay untouched.
   - Capture the baseline behavior before making edits.

2. Make the smallest useful change.
   - Prefer focused refactors over broad rewrites.
   - Keep mechanical, control, and documentation changes separate when possible.

3. Verify in layers.
   - Confirm the project still builds.
   - Check the subsystem in isolation when possible.
   - Test on hardware only after the local behavior is understood.

4. Measure and compare.
   - Use logs, dashboard values, and SysId runs to confirm the effect.
   - Record what improved, what regressed, and what remains unknown.

5. Keep the repo self-explanatory.
   - Add short comments for non-obvious tuning choices.
   - Update this hub when a workflow or reference becomes the new normal.
   - Retire or delete one-off season notes once they stop teaching anything new.

## Historical Docs

These are the kinds of season-session documents we used to generate when tuning a robot or evaluating a subsystem. Most of them are intentionally removed now; if you need the original discussion, check Git history.

- Performance tuning walkthroughs
- Memory allocation investigations
- Logging audits
- End-of-season status summaries
