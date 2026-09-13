# 0003. Track constants live in one source

- Status: accepted
- Date: 2026-09-13
- Commit: 3b6456d5

## Context

The WRO 2026 Future Engineers mat geometry was hand-maintained twice: once in the
Python classes of `src/python/shared/src/shared/config/` and once in the Go const
blocks of `simconfig/constants.go`, whose comment asked future editors to keep the
two in sync by hand. That is the same hazard `robot.toml` had already removed for
the robot's own dimensions.

## Options considered

- (a) Keep the two hand-maintained copies and rely on a comment to keep them aligned.
- (b) Make one TOML the source and have each language read or regenerate from it.

## Decision

(b). `src/config/track.toml` is the single source of truth for the mat geometry.
Both languages read it directly at runtime. Python goes through
`shared.config.track_constants.TrackConstants`; Go loads it through
`internal/config/profile.Load[TrackConfig]` and the sim pipeline builds
`simconfig.Track` (package `internal/simgen/simconfig`) from the generated DTO in
`src/go/internal/config/generated`. There is no generated copy and no
regeneration step; see ADR 0020. All lengths are metres and colours are
normalized RGB triples in [0, 1].

Values that genuinely belong to one side only stay out: competition rules (lap
counts, round time), simulator-only tuning (lighting ranges, sensor noise) and
rendering z-order live in their own language's config.

## Consequences

- One edit updates both languages; the manual sync hazard is gone.
- Both languages read the file at load time, so there is no regeneration step to
  run and no generated artifact that can lag a TOML edit.
