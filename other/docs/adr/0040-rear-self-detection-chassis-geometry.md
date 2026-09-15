# 0040. The rear self-detection filter follows chassis geometry

- Status: superseded by 0056
- Superseded by: 0056
- Date: 2026-09-06

## Context

One scalar self-detection threshold cannot describe the rear: the chassis
boundary runs from 0.137 m at the rear sector's edges to 0.2722 m straight back,
and 0.08 m is inside the body everywhere in between. Measured on
`run_20260906_192424` the rear minimum was the ROBOT on 100 percent of scans
(-157 deg / 0.125 m, -172 deg / 0.187 m), so `most_constrained_side` read BACK on
84 percent of driving ticks. There is no BACK escape branch, so no maneuver was
ever generated: 212 ticks of `escape_risk=critical` across the five pillar
contacts produced zero escapes.

## Options considered

- (a) Filter the rear sector's self-detection with the single scalar.
- (b) Gate it by the chassis geometry at each bearing.

## Decision

(b). `rear_self_detection_from_chassis` ships true. It is a `default` tag, so a
TOML that omits the key reads the shipped value rather than silently reverting
the filter to the scalar through the zero value.

## Consequences

- The rear sector stops mistaking the robot's own body for an obstacle, so the
  escape machinery can see real rear threats.
- Mirrors the same trap class as `corner_arc_assume_wide` and
  `min_history_for_distance`: an omitted key must not silently change behaviour.
