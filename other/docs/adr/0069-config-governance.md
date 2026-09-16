# 0069. Config values live in TOML, descriptions in schemas, rationale in ADRs

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0003, 0004, 0008, 0016, 0017, 0018, 0020

## Context

Behavior constants were scattered across Python literals, Go literals and a
code generator that emitted per-language constant files. A generated artifact can
go stale, and a constant with no traceable justification can ship. The same
physical fact had to be edited in several places to reach both stacks, which is
how the two drifted (a robot copy had drifted the LIDAR mount z by +0.02 against
the TOML's -0.02).

## Options considered

- (a) Keep generating constants into each language from a source of truth.
- (b) Read TOML at runtime in each language, with the shape and short description
      in JSON Schemas and the long rationale in ADRs.

## Decision

(b). `src/config/*.toml` is the single source of values, read at runtime by both
Python and Go. There is no generated values artifact; a TOML edit reaches every
consumer without a regeneration step. `src/model/**.schema.json` (52 schemas) is
the single source of each key's shape and short `description`; each schematized
TOML opens with `#:schema`. Long rationale lives in these ADRs, which a schema key
points at through `x-journal`.

`configgen.py` `check` fails if a TOML key has no described schema entry or if an
`x-journal` reference does not resolve, so a constant without traceable
justification does not pass verification. DTOs are generated from the schemas and
hold no values; they are never edited by hand. Precedence is model default, then
base TOML, then the active hardware profiles, then the challenge layer.

The specific consolidated facts this carries: track constants in `track.toml` (one
source, runtime-read); corridor division lines `[0.40, 0.60]` defined once with
the starting-square bands and sign-grid width derived; robot constants in
`robot.toml` read at runtime (no `robot_constants.gen.go`; the xacro is a
hand-synced checked-in copy); sensor specs consolidated
into `robot.toml` (`lidar.min_range`, `lidar.max_range`, `imu.mount_z_offset`);
ROS topic names centralized in `ros_topics.toml`; competition rules in
`competition_specs.toml` (`round_time_limit_s = 180.0`, `open_challenge_laps = 3`,
`obstacle_challenge_laps = 3`). `ros_topics.toml` is loaded through
`RosTopicConfig.load_default()`; track consumers are
`shared.config.track_constants` (Python) and the Go generator `simgen`
(`trackconfig`).

There is no environment variable override for navigation tuning. The only variable
read is `VTITAN_HARDWARE_PROFILE`, and it selects profiles, not values. A key
declared in the base TOML wins over any Pydantic default; the only way a model
default reaches the robot is if the key is absent from the TOML.

## Robot constants and the xacro copy

Robot constants were once hand-duplicated across Python, Go and the xacro, and
had already drifted: the chassis length and width were corrected in Python
(0.28 x 0.15 to 0.30 x 0.20) but never updated in the Go Gazebo generator or the
xacro, which silently kept the old numbers for months. A generator that emitted
the Go and xacro consumers existed, but it was unused and emitted malformed Go
source.

`src/config/robot.toml` is the single source of the physical constants. Go loads
it at runtime through `LoadRobotConfig` (with overlays); Python reads it at
runtime through `RobotConstants` (with overlays). There is no
`robot_constants.gen.go`; it was deleted. The xacro at
`other/apps/gazebo/runtime/robot_description/robot_properties.gen.xacro` remains
a checked-in copy that must be kept in step BY HAND when a chassis or sensor
value changes; its `.gen.` name and `DO NOT EDIT` header are stale.

Units are metres, kilograms and radians unless a field is explicitly named
`_deg`. The concrete consumers are `shared.config.robot_constants` (Python), the
`simconfig` package (Go) and the URDF xacro.

This section absorbs 0089, which superseded 0008; the drift and the residual
hand-synced xacro are the accepted cost of reading TOML at runtime.

## Consequences

- A value edit reaches both stacks with no regeneration and no stale artifact.
- An unjustified constant cannot pass `config:check`.
- Editing a Pydantic default does nothing when the key is already in TOML.
- The xacro (and any remaining hand-maintained consumer) must be kept in step by
  hand until a replacement lands.

## History

- 617e1905 2026-08-01: give the mat, the robot and the tuning one source each.
- 40b79421 2026-09-05: make the code-only tuning constants TOML-driven (59 fields
  written out at shipped values); closes a Go/Python bay-exit drift.
- 39d8e679 2026-09-10: dissolve `platform/` into `src/python` + `src/go`, `apps/`,
  `ml/`, `contracts/`.
- 5671099b 2026-09-10: move shared TOML out of `src/python` into `src/config`; fix
  about 19 stale Go path constants.
- 3b6456d5 2026-09-13: single-source config schemas with generated Python and Go
  DTOs; create the 52 schemas and `configgen.py`; delete
   `track_constants.gen.go` and `robot_constants.gen.go`.
- b3280218 2026-07-11: centralize into `robot.toml` and add the Go generator.
- 31fc8eb2 2026-08-08: load the constants at runtime; delete the generated Python
  file.
- 3f6696c1 and 24cb052f 2026-09-03: move simgen, then remove
  `generate-robot-constants` (unused, malformed Go output); the xacro becomes a
  hand-maintained copy.
- b982e109 2026-09-13: move value rationale into ADRs and link schemas; 62 keys
  linked via `x-journal`.
- 1d0cf606 2026-09-13: consume generated DTOs directly in Go and Python; field
  names become lowercase TOML keys.
- 61bb6df4 2026-09-14: drop the navigator's redundant TOML decode structs.
- d6f01332 2026-09-14: source every Go config from TOML, not code defaults.
- b7f62bfa 2026-09-14: source Python config from TOML with one cached loader; a
  partial tree raises rather than falling back.

## Cross-references

- 0003, 0004, 0016, 0017, 0018 and 0020 are superseded; their decisions are
  carried above.
- 0008 was superseded by 0089, and 0089 is now merged into this ADR; edit this
  one, not that.
- 0076 owns the drivetrain constants that are also read at runtime.
- 0019 (simulation robot-model topic split) and 0046 (repo layout) stay separate.
- 0070 owns the profile overlay mechanism this precedence order names.
- 0074 owns `control_hz`, the loop rate these consumers read.

## Evidence

- All environment-variable config goes through pydantic-settings (Python) or
  Viper (Go); never `os.getenv` or the legacy `EnvVar` helper.
- Silent config loaders are a class bug: a loader can return success while never
  reading a key it needs. Print the production loader's resolved value before any
  A/B, and use `profile.Load` rather than `LoadRobotConfig` for robot.toml-only
  reads.
- The fail-loud loader check is justified by a native corpus sweep given a config
  root but no hardware profiles: it scored 640 of 640 STUCK at max speed 0.000,
  reading as a navigation failure rather than a config error.
- A model default can leak past the TOML: `CorridorWidthEntry` defaulted to
  `width_mm=500` against `type=wide`, and that reached `ScenarioMetadata` through
  two layers of model defaults.
- A stale description is a documentation defect even when the value is right: the
  ARC_RADIUS floor was documented as 0.329 m against an actual 0.034 m, 10x off.
