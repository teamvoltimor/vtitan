# 0048. The forward lane corroborates a short return across adjacent rays

- Status: superseded by 0056
- Superseded by: 0056
- Date: 2026-09-13

## Context

The forward lane's short-return test originally took the bare minimum over the
sweep. That is an extreme-value statistic, not a clearance: with sigma = 0.03 m
of range noise across ~500 rays, the smallest reading in the lane sits 2-3
sigma below the nearest real surface, so a single noisy ray can read as an
obstacle that is not there. Setting the window to 1 restores that bare
minimum.

## Options considered

- (a) Keep the bare minimum over the forward lane.
- (b) Require several ADJACENT rays in the forward lane to corroborate the
      short return before it counts as an obstacle.

## Decision

(b). The window is a count of adjacent rays that must agree. A value of 1
restores the previous bare-minimum behaviour.

## Consequences

- A lone low return from range noise no longer reads as an obstacle, so the
  reactive clearance gate fires on corroborated geometry rather than on an
  extreme value of the noise distribution.
- The window is a ray count, not a distance: it trades sensitivity to a genuine
  narrow obstacle against false positives, and needs re-tuning if the LIDAR's
  ray count or noise changes.

## Superseded by 0056

Replaced by [0056](0056-raw-and-masked-scan.md): The scan is read twice, raw for speed
and masked for the escape.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
