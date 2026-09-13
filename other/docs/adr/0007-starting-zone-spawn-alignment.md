# 0007. Spawn hugs one band edge, never centred

- Status: accepted
- Date: 2026-09-13
- Commit: 3b6456d5

## Context

The robot is never centred in its starting band. The two band edges are not
alike: one is a painted division line the robot can sit flush against harmlessly,
the other may be a physical obstacle. Splitting the slack evenly spends half the
margin on the line and leaves only half against the thing it can actually hit.

Band 0 hugs its inner edge because its outer edge IS the outer wall. Bands 1 and 2
hug their outer edge because the surface beyond them is the inner block: in a
narrow corridor band 1 ends exactly at the block, so this is the difference
between 3 mm and 6 mm of clearance from it.

## Options considered

- (a) Centre the chassis in its band.
- (b) Push it flush against one named edge per band.

## Decision

(b). `spawn_alignment = ["inner", "outer", "outer"]`. The resulting offsets are
absolute metres, not fractions of the corridor width, and are derived from
`spawn_alignment` and the chassis width in `robot.toml`, so they follow a
re-measurement instead of going stale. A narrow corridor simply stops after the
first two bands, because the third lies under the centre square where it cannot be
a start.

## Consequences

- The full margin is kept against the obstacle side.
- The offsets depend on the chassis width and must be re-derived after a width
  change.
