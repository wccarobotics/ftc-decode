# Copilot Instructions — FTC DECODE 2025-2026

## Build

This is an Android Gradle project (AGP 8.13.1, Gradle 8.13, FTC SDK 11.1.0). Source/target compatibility is Java 8.

```shell
# Build the project
./gradlew assembleDebug

# Build only TeamCode
./gradlew :TeamCode:assembleDebug

# Deploy to a connected robot controller
./gradlew :TeamCode:installDebug
```

There are no unit tests or lint configurations.

## Architecture

### Package layout (TeamCode)

```
org.firstinspires.ftc.teamcode/
├── commands/       # Command pattern classes for autonomous sequencing
├── mechanisms/     # Hardware abstractions (drivetrain, scoring, vision, odometry)
├── opmodes/        # OpMode implementations (teleop, autonomous, tests)
└── pedroPathing/   # Pedro Pathing follower configuration and tuning
```

Root-level files include utility classes (`LoopTimer`) and the `GoBildaPinpointDriver` hardware driver.

### Command framework

Autonomous routines use a WPILib-style command pattern. All commands extend the abstract `Command` class with lifecycle methods: `initialize()` → `execute()` (looped) → `isFinished()` → `end(interrupted)`. The `CommandScheduler` manages scheduling and lifecycle.

Composite commands: `SequentialCommand`, `ParallelCommand`, `ParallelRaceCommand`. Pedro Pathing is integrated via `FollowPathCommand`, `FollowPathChainCommand`, and `LineToCommand`.

### Main robot ("Jeff") hierarchy

`JeffBase` (abstract, extends `OpMode`) → `JeffAuto` (@Autonomous) and `JeffTeleOp` (@TeleOp).

JeffBase handles shared initialization: Lynx hub bulk caching (MANUAL mode), Pedro `Follower`, `JeffScoring`, `LimelightVision`, `Lights`, and Panels telemetry. It persists robot pose and alliance color across OpModes via `SharedPreferences`.

### Mechanisms

Each mechanism wraps hardware devices and is initialized from the `HardwareMap`:

- **MechanumDrive** — 4-motor holonomic drive with field-relative mode via `PinpointOdometry`
- **JeffScoring** — Dual flywheels, intake, diverter servo, feeder servos, 4 color sensors. Contains a nested `Flywheel` class for dual-motor velocity PID
- **JeffLauncher** — Per-launcher state machine (IDLE → SPIN_UP → LAUNCH → LAUNCHING) with feed servo timing and ball detection
- **LimelightVision** — Limelight 3A AprilTag localization. Converts FTC center-origin coordinates to Pedro bottom-left origin. Goal tag IDs: Blue=20, Red=24, Obelisk motifs=21-23
- **PinpointOdometry** — GoBILDA Pinpoint 2-wheel odometry wrapper (forward pod Y=17mm, strafe pod X=-200mm)

### Key libraries

| Library | Version | Purpose |
|---------|---------|---------|
| Pedro Pathing | 2.0.6 | Autonomous path planning and following |
| Pedro Telemetry | 1.0.0 | Path telemetry |
| Bylazar Panels | 1.0.9 | Real-time field visualization |
| Limelight | 3A | AprilTag detection and robot localization |
| GoBILDA Pinpoint | custom driver | Odometry with IMU fusion |

## Conventions

### Hardware names

Hardware devices are referenced by string name from the robot configuration. Motor and servo names use `snake_case`:
`left_front_drive`, `right_back_drive`, `left_flywheel`, `right_flywheel`, `intake`, `left_feeder`, `right_feeder`, `diverter`, `odo`, `imu`, `limelight`.

### OpMode annotations

Active competition OpModes use `@Autonomous` or `@TeleOp` with a `name` and `group`. Experimental, student, and test OpModes are annotated `@Disabled`.

### State machines

Complex mechanism behaviors (launcher sequencing, scoring cycles) are implemented as state machines using enums (`LaunchState`, `IntakeState`). State transitions happen in an `update()` method called each loop.

### Pedro Pathing configuration

All follower configuration lives in `pedroPathing/Constants.java`. The `createFollower()` factory method returns a fully configured `Follower`. PID gains, velocity constraints, motor mappings, and pinpoint offsets are defined here.

### Coordinate systems

Pedro Pathing uses a bottom-left origin coordinate system. Limelight returns FTC center-origin poses. `LimelightVision` handles the conversion internally. When writing autonomous paths, use Pedro coordinates.

### Performance

Lynx hub bulk reads are set to MANUAL mode in `JeffBase.init()` with cache cleared each loop. `LoopTimer` tracks rolling-window loop times. JVM heap is capped at 1024MB.

### Copilot skills

This project has Copilot skills available for `decode` (game rules/field), `pedro-pathing` (path building), and `limelight` (vision integration). Use these when working in those domains.
