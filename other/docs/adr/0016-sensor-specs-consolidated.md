# 0016. Sensor specs are consolidated into robot.toml

- Status: superseded by 0069
- Superseded by: 0069
- Date: 2026-07-30
- Commit: f6d53ac9

## Context

Several sensor specs were hand-maintained in more than one language or file.
`lidar.min_range` sat in a hand-maintained Python constant that stated 0.05
(measured ~45 mm); `lidar.max_range` was a hand-maintained Python constant
(`LIDAR_MAX_RANGE`). `imu.mount_z_offset` (0.01 m) was duplicated as
`simconfig.RobotImuZOffset` (Go, correct, since removed: `simconfig.Robot` now
carries it loaded from `robot.toml`), a hardcoded 0.01 in
`src/python/ros2_ws/src/vtitan_bringup/launch/static_tfs.launch.py` (correct),
and the URDF xacro's `imu_link` joint origin
(WRONG, hardcoded to 0, i.e. the IMU was not actually placed there). The LIDAR
z-offset was similarly triplicated (see ADR 0014).

## Options considered

- (a) Leave each consumer's constant in place and keep them aligned by hand.
- (b) Consolidate the hardware description in `robot.toml`.

## Decision

(b). A sensor spec belongs with the rest of the hardware description, not beside
the simulation parameters or duplicated per consumer. `lidar.min_range = 0.045`
(measured ~45 mm), `lidar.max_range = 12.0`, and `imu.mount_z_offset = 0.01` live
here, consolidated 2026-07-30.

## Consequences

- The xacro's wrong `imu_link` origin (0) is fixed by reading the one value.
- The distinction that matters for `min_range` is that anything nearer is
  unmeasurable rather than clear, which matters when the chassis works within a
  few centimetres of a surface (parking, wall contact, escape).

## Superseded by 0069

This decision was replaced by [0069](0069-config-governance.md). Its content is reproduced below so this file stays self-contained; edit only the successor.

### Context

Behavior constants were scattered across Python literals, Go literals and a
code generator that emitted per-language constant files. A generated artifact can
go stale, and a constant with no traceable justification can ship. The same
physical fact had to be edited in several places to reach both stacks, which is
how the two drifted (a robot copy had drifted the LIDAR mount z by +0.02 against
the TOML's -0.02).

### Options considered

- (a) Keep generating constants into each language from a source of truth.
- (b) Read TOML at runtime in each language, with the shape and short description
      in JSON Schemas and the long rationale in ADRs.

### Decision

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
hand-synced checked-in copy, see 0089); sensor specs consolidated
into `robot.toml` (`lidar.min_range`, `lidar.max_range`, `imu.mount_z_offset`);
ROS topic names centralized in `ros_topics.toml`; competition rules in
`competition_specs.toml` (`round_time_limit_s = 180.0`, `open_challenge_laps = 3`,
`obstacle_challenge_laps = 3`).

There is no environment variable override for navigation tuning. The only variable
read is `VTITAN_HARDWARE_PROFILE`, and it selects profiles, not values. A key
declared in the base TOML wins over any Pydantic default; the only way a model
default reaches the robot is if the key is absent from the TOML.

### Consequences

- A value edit reaches both stacks with no regeneration and no stale artifact.
- An unjustified constant cannot pass `config:check`.
- Editing a Pydantic default does nothing when the key is already in TOML.
- The xacro (and any remaining hand-maintained consumer) must be kept in step by
  hand until a replacement lands.

### History

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
- b982e109 2026-09-13: move value rationale into ADRs and link schemas; 62 keys
  linked via `x-journal`.
- 1d0cf606 2026-09-13: consume generated DTOs directly in Go and Python; field
  names become lowercase TOML keys.
- 61bb6df4 2026-09-14: drop the navigator's redundant TOML decode structs.
- d6f01332 2026-09-14: source every Go config from TOML, not code defaults.
- b7f62bfa 2026-09-14: source Python config from TOML with one cached loader; a
  partial tree raises rather than falling back.

### Cross-references

- 0003, 0004, 0016, 0017, 0018 and 0020 are superseded; their decisions are
  carried above.
- 0008 is superseded by 0089 (runtime constants and the xacro copy).
- 0019 (simulation robot-model topic split) and 0046 (repo layout) stay separate.
- 0070 owns the profile overlay mechanism this precedence order names.
- 0074 owns `control_hz`, the loop rate these consumers read.

