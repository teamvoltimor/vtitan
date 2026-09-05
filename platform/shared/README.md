# platform/shared

Shared Python constants, enums, domain models, and utilities for the vTitan robot stack.

## Consumers

**`platform/robot` only.** No other project in this monorepo imports `shared` — the Go
backend (`platform/backend`), the Go simulation generator (`platform/robot-go`), and
the frontend (`platform/frontend`) each maintain their own parallel type/constant
definitions in their own language, rather than depending on this Python package. `robot.toml`
(see below) is the one exception that *is* shared across languages, via code generation
rather than a runtime import.

## What's here

- `shared.config.constants` — physical/hardware constants (`RobotSpecs`, `TrackDimensions`,
  `WallSpecs`, `TrafficSignSpecs`). The chassis/Ackermann/LIDAR/camera-mount fields are sourced
  from `shared.config.robot_constants.RobotConstants`, which reads
  `platform/config/robot.toml` directly at runtime (the actual cross-language source of
  truth — the Go/xacro consumers are now hand-maintained copies (the regenerator was removed
  2026-09-03), but Python reads the TOML itself, no codegen step); everything else in
  `RobotSpecs` (LIDAR/IMU/camera simulation parameters) is hand-maintained since it isn't
  duplicated in Go or xacro.
- `shared.config.navigation_tuning` — `NavigationTuning`, the runtime-tunable navigation
  parameter set (pursuit, clearance, escape, sign routing, parking, etc.), loadable from the
  per-group TOML files under `platform/config/navigation/`.
- `shared.config.enums` — `Section`, `Direction`, `ScenarioType`, `RiskLevel`, `RobotState`,
  `NodeHealth`.
- `shared.domain.models` — `Pose`, `Velocity`, `Detection`, `LidarClearances`,
  `ScenarioMetadata`, and other cross-module data shapes.
- `shared.io` — `JsonlReader`/`JsonlWriter`/`JsonlValidator` for session/replay files.
- Coordinate transform and steering-angle conversion helpers.

## Installing

`platform/robot`'s `pixi.toml` depends on this package via a local path. There's no separate
install step — it's part of the robot's pixi environment.
