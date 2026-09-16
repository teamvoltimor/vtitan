# 0020. Config is read at runtime from TOML, not generated into consumers

- Status: superseded by 0069
- Superseded by: 0069
- Date: 2026-09-13
- Commit: 3b6456d5

## Context

Config values were once duplicated into each language: a generator rewrote Go
const blocks and xacro documents from the TOMLs. Every regeneration was a
separate artifact that could lag the TOML, and two of them had already drifted.
The hand-maintained `robot_constants.gen.go` read a LIDAR mount z of +0.02
against robot.toml's -0.02, and carried the retired 180 degree servo's steering
values. The `track_constants.gen.go` was a second, bespoke copy of the same mat
numbers. The generator itself was removed 2026-09-03.

## Options considered

- (a) Keep a generator and regenerate the Go/xacro consumers after each TOML
      edit.
- (b) Read the TOML at runtime in every language, through one generated DTO per
      file plus a hand-written behaviour wrapper.

## Decision

(b). `src/config/*.toml` stays the single source. Each consumer loads the TOML
live:

- Python wraps the generated DTO (`shared.config.generated.*`) with the
  behaviour and profile merge in `shared.config.robot_constants` and
  `shared.config.track_constants`.
- Go loads it through `internal/config/profile` (`LoadRobotConfig`,
  `Load[TrackConfig]`), and the sim pipeline builds `simconfig.Track` /
  `simconfig.Robot` from those values at runtime.
- The generated DTOs under `internal/config/generated` exist so the field names
  cannot drift from the schemas; they hold no values.

The JSON Schemas under `src/model/` now carry only short descriptions. Long
rationale belongs in these ADRs, which a schema key points at through
`x-journal`.

## Consequences

- A TOML edit reaches every consumer without a regeneration step; there is no
  generated artifact to go stale.
- The loaders, not a generator, are the only place a consumer can disagree with
  the TOML, and `LoadRobotConfig` / `LoadEncoderConfig` fail loudly when a
  required hardware-profile key is missing.
- The old `track_constants.gen.go` and `robot_constants.gen.go` are gone, which
  is why ADRs 0003 and 0008 no longer describe a generator.

## Superseded by 0069

Replaced by [0069](0069-config-governance.md): Config values live in TOML, descriptions
in schemas, rationale in ADRs.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
