## Quick orientation — what this project is

This is a WPILib Java robot project (GradleRIO) using the Command-based pattern and CTRE/PathPlanner vendor libraries. The robot entry points are in `src/main/java/frc/robot/Robot.java` and `RobotContainer.java` (subsystems, default commands, and button bindings live in `RobotContainer`).

## Big picture architecture

- Command-based design: subsystems live under `frc.robot.subsystems` and commands under `frc.robot.commands`. `RobotContainer` wires subsystems, OI, autonomous chooser and default commands. `Robot` is a thin TimedRobot wrapper that delegates to `RobotContainer`.
- Tuning/constants: static configuration lives in `src/main/java/frc/robot/Constants.java`. Per-build/generated constants live in `frc.robot.generated.TunerConstants` (used to create hardware instances). Prefer adding robot-wide constants to `Constants` and vehicle/tuning values into the generated tuner where appropriate.
- Telemetry: `Telemetry.java` publishes NetworkTables + SignalLogger entries (Field2d, module visuals). Use it as an example for adding telemetry.
- Simulation: lightweight physics harness exists (`frc.robot.sim.PhysicsSim`, `Mechanisms`, RoboRioSim usage). Resource files (paths, autos, apriltag maps) are under `src/main/deploy` and are read via `Robot.RESOURCES_PATH` (switches between deploy dir on real robot and source deploy dir when simulated).

## Key files to inspect (examples)
- `src/main/java/frc/robot/Robot.java` — robot lifecycle and resources path handling.
- `src/main/java/frc/robot/RobotContainer.java` — subsystem instantiation, OI selection (`OISelector`), button bindings, default commands, and auto registration (`NamedCommands`, PathPlanner AutoBuilder).
- `src/main/java/frc/robot/Constants.java` — nested static classes group DriveTrain/Arm/Wrist/Intake/Climber constants.
- `src/main/java/frc/robot/Telemetry.java` — how telemetry is published (NetworkTables, SignalLogger) and how module visuals are built.
- `src/main/deploy` — contains PathPlanner paths, autos, apriltag maps and other static files that are copied to the robot image.

## Developer workflows (build / test / simulate / deploy)
- Build: use the Gradle wrapper at repo root. Typical commands:
  - `./gradlew build` — compiles and creates the fat JAR (see `jar` task in `build.gradle`).
  - `./gradlew test` — runs JUnit 5 tests (project uses JUnit 5).
  - `./gradlew deploy` — deploys artifacts to the RoboRIO target (configured in `build.gradle` via GradleRIO). Team number comes from WPILib preferences or CLI.
  - Simulation: the GradleRIO plugin exposes simulation tasks and the project config enables `wpi.sim` GUI. Use the WPILib VS Code plugin for an integrated simulate/run experience, or run the provided Gradle simulate task from CLI if you prefer.

Note: `build.gradle` uses Java 17 and creates a fat jar; the manifest points to the robot main class defined in the file.

## Conventions & patterns specific to this repo
- Constants are grouped in nested static classes (e.g., `DriveTrainConstants`, `ArmConstants`). Follow the same grouping for new subsystems.
- OI is dynamic: `OISelector` allows swapping operator interface implementations at runtime. When adding controller mappings, update `OperatorInterface` implementations and ensure `updateOI()` in `RobotContainer` is used to rebind buttons.
- Autonomous paths & routines are registered by name via `NamedCommands.registerCommand(...)` and `AutoBuilder.buildAutoChooser(...)`. Add new auto commands as NamedCommands so PathPlanner can reference them.
- Hardware wiring and IDs live in `Constants` or generated tuner classes — do not hardcode IDs elsewhere.
- Telemetry (SignalLogger + NetworkTables) is centralized in `Telemetry.java` and registered by subsystems (e.g., `drivetrain.registerTelemetry(logger::telemeterize)`). Use the same API for consistent outputs.

## Integration points / external dependencies
- Vendor libraries and JNI are declared in `build.gradle` (CTRE Phoenix, PathPlanner, limelight wrappers). Vendordep files are in `vendordeps/`.
- Static files under `src/main/deploy` are deployed to `/home/lvuser/deploy` on the RoboRIO and used at runtime (paths, autos, apriltag maps).
- NetworkTables is used for telemetry and dashboard integration; SignalLogger is used for logged telemetry.

## Small examples to copy/paste
- Get the autonomous command selected by the chooser:
  - `Command auto = robotContainer.getAutonomousCommand();` (see `Robot.autonomousInit()`)
- Schedule a PathPlanner warmup command (already used in `configureAutoCommands()`):
  - `CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());`

## When to ask the team / missing info
- Hardware mapping: check physical wiring vs `Constants` before changing IDs.
- CI / deployment to field: this repo assumes WPILib / GradleRIO defaults — if your team uses custom deployment credentials or scripts, ask for them.

If anything is unclear or you'd like a shorter or longer variant (more examples, templates for new subsystems/commands, or CI snippets), tell me which sections to expand. 
