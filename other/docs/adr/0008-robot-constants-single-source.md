# 0008. Robot constants live in one source and are read at runtime

- Status: accepted
- Date: 2026-09-03
- Commit: 24cb052f

## Context

`src/config/robot.toml` is the single source of truth for the robot's physical
constants (custom chassis, counter-phase four-wheel steering), measured
2026-07-11 (see `src/python/docs/robot-physical-constants.md`). The Go/xacro
consumers were once produced by a generator, but that generator was removed
2026-09-03 and the Go consumer no longer exists as a file at all: the sim now
loads robot.toml at runtime.

## Options considered

- (a) Keep a generator that rewrites the Go and xacro consumers.
- (b) Treat the TOML as the source and hand-update the consumers.
- (c) Read the TOML at runtime in each language, leaving only the xacro as a
      checked-in copy.

## Decision

(c), selected 2026-09-13 and superseding (b). Editing `robot.toml` means:

- Go loads it at runtime through `internal/config/profile.LoadRobotConfig`
  (with the active hardware-profile overlays) and the sim pipeline builds
  `simconfig.Robot` (`src/go/internal/simgen/simconfig`) from those values.
  There is no `robot_constants.gen.go`.
- Python reads the file directly at runtime via
  `shared.config.robot_constants.RobotConstants`, merging hardware-profile
  overlays, nothing to regenerate.
- The xacro at
  `other/apps/gazebo/runtime/robot_description/robot_properties.gen.xacro`
  (`xacro:include`d from `wro_robot.urdf.xacro`) remains a checked-in copy that
  must be kept in step by hand when a chassis or sensor value changes.

All lengths are meters, masses kilograms, angles radians unless a field is
explicitly named `_deg` (for example `lidar.mount_yaw_offset_deg`). The general
runtime-load decision is ADR 0020.

## Consequences

- The Go and Python consumers cannot drift from the TOML, because they read it.
- The xacro can still drift if an edit forgets it; the TOML header names it so an
  editor sees it in place.
