# src/python/shared

Shared Python constants, enums, domain models, and utilities for the vTitan robot
stack. Installed as its own package (`shared`) so the ROS2 nodes, the simulation
harness and the diagnostic scripts all import one copy.

## Consumers

**`src/python` only.** Nothing else in this monorepo imports `shared`: the Go
backend (`other/apps/backend`), the Go robot stack (`src/go`) and the frontend
(`other/apps/frontend`) each keep their own type and constant definitions in
their own language rather than depending on this package.

The one thing that genuinely crosses languages is the TOML tree under
`src/config/` at the repo root. Python and Go both read those files directly at
runtime; there is no shared runtime object and no generated bridge between the
two stacks. See [ADR 0020](../../../other/docs/adr/0020-config-loaded-at-runtime.md).

## Where the configuration lives

The TOML is **not** in this package. It sits at the repo root, outside
`src/python/` entirely:

| Tree | Holds |
|---|---|
| `src/config/robot.toml` | Cross-language physical constants (chassis, Ackermann, wheel, LIDAR, camera mount) |
| `src/config/navigation/*.toml` | Navigation tuning, grouped by subsystem |
| `src/config/hardware/*.toml` | One file per driver |
| `src/config/profiles/*/` | Swappable hardware profiles, composed at `VTITAN_HARDWARE_PROFILE` |
| `src/model/*.schema.json` | The JSON Schemas that validate all of the above |

This package is the **Python loader** for that tree, not a copy of it.

## What's here

- `shared.config.robot_constants` - reads `src/config/robot.toml` at runtime
  (`DEFAULT_CONFIG_PATH`), applies the active hardware-profile overlays, and
  exposes the result as typed constants.
- `shared.config.constants` - `RobotSpecs`, `TrackDimensions`, `WallSpecs`,
  `TrafficSignSpecs`, `CompetitionSpecs`, `ParkingLotSpecs` and the identifier
  tables (`TfFrames`, `NodeNames`, `FolderNames`, ...). The chassis, Ackermann,
  LIDAR and camera-mount fields come from `robot_constants`; the simulation-only
  parameters are hand-maintained here, because nothing else duplicates them.
- `shared.config.navigation_tuning` - `NavigationTuning`, the runtime-tunable
  parameter set (pursuit, clearance, escape, sign routing, parking, ...), loaded
  from the per-group files under `src/config/navigation/`.
- `shared.config.generated` - DTOs generated from the JSON Schemas in
  `src/model/`. Do not hand-edit; they carry only the fields and their types.
- `shared.config.hardware_profile` - profile discovery and overlay composition.
- `shared.domain.enums` - `Section`, `Direction`, `Corridor`, `ScenarioType`,
  `RiskLevel`, `RobotState`, `NodeHealth`, `NavigatorPhase` and friends.
- `shared.domain.models` - `Pose`, `Velocity`, `Detection`, `LidarClearances`,
  `ScenarioMetadata` and the other cross-module data shapes.
- `shared.domain.steering`, `shared.config.coordinate_transform` - steering-angle
  conversion and frame transforms.
- `shared.io` - `JsonlReader` / `JsonlWriter` / `JsonlValidator` for
  session and replay files.

## A note on code generation

Two different things used to be called "codegen" here, and only one still exists:

- **Alive:** `shared.config.generated.*`, the DTOs built from the JSON Schemas in
  `src/model/`. Regenerate them rather than editing them.
- **Removed 2026-09-03:** the regenerator that pushed `robot.toml` into the Go
  and xacro consumers. Those are now **hand-maintained copies**, so a change to
  `robot.toml` has to be mirrored into them by hand. Python does not need it; it
  reads the TOML itself.

## Installing

`src/python`'s `pixi.toml` depends on this package by local path. There is no
separate install step; it is part of the robot's pixi environment.
