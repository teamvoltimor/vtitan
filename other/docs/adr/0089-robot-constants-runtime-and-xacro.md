# 0089. Robot constants are read at runtime and the xacro is the one hand-synced copy

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0008

## Context

Robot constants were once hand-duplicated across Python, Go and xacro, and had
already drifted: the chassis length and width were corrected in Python
(0.28 x 0.15 to 0.30 x 0.20) but never updated in the Go Gazebo generator or the
xacro, which silently kept the old numbers for months. A generator that emitted
the Go and xacro consumers existed, but it was unused and emitted malformed Go
source.

## Options considered

- (a) Keep a generator that rewrites the Go and xacro consumers.
- (b) Read the TOML at runtime in each language, leaving only the xacro
      hand-synced.

## Decision

(b). `src/config/robot.toml` is the single source of the physical constants. Go
loads it at runtime through `LoadRobotConfig` (with overlays); Python reads it at
runtime through `RobotConstants` (with overlays). There is no
`robot_constants.gen.go`; it was deleted. The xacro at
`other/apps/gazebo/runtime/robot_description/robot_properties.gen.xacro` remains a
checked-in copy that must be kept in step BY HAND when a chassis or sensor value
changes; its `.gen.` name and `DO NOT EDIT` header are stale.

Units are metres, kilograms and radians unless a field is explicitly named `_deg`.
The concrete consumers are `shared.config.robot_constants` (Python), the
`simconfig` package (Go) and the URDF xacro.

## Consequences

- The Go and Python consumers cannot drift from the TOML, because they read it.
- The xacro can still drift if an edit forgets it; that is the accepted residual.
- `robot_physical_constants.md` still instructs editing two consumers and calls
  them generated, which contradicts the removal; the doc is stale.

## History

- b3280218 2026-07-11: centralize into `robot.toml` and add the Go generator.
- 31fc8eb2 2026-08-08: load the constants at runtime; delete the generated Python
  file.
- 3f6696c1 and 24cb052f 2026-09-03: move simgen, then remove
  `generate-robot-constants` (unused, malformed Go output); the xacro becomes a
  hand-maintained copy.
- 39d8e679 2026-09-10: dissolve `platform/` into `src/` and `other/`.
- 3b6456d5 2026-09-13: delete `robot_constants.gen.go` and `track_constants.gen.go`
  in the config-schema single-sourcing.
- b982e109 2026-09-13: write ADR 0008 with the runtime-read decision.

## Cross-references

- 0008 is carried here; the general governance and precedence order is 0069.
- 0076 owns the drivetrain constants that are also read at runtime.
