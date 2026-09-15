# 0041. The LIDAR valid-range floor sits below the rated minimum

- Status: superseded by 0056
- Superseded by: 0056
- Date: 2026-09-10

## Context

The LIDAR invalid-reading filter is `r > min_valid_range_m`, and the Slamtec C1
REPORTS 0.045 m for anything closer than it can measure. `min_valid_range_m`
held 0.05 until 2026-09-10 while its comment claimed to "match" 0.045. Every
floor reading was therefore discarded as invalid, so a chassis nosed into a
corner had its whole forward cone thrown away and `AssessRisk` returned the
unconditional `RiskCritical` for a fully-invalid lane, a wedge it could not read
its way out of.

## Options considered

- (a) Keep the floor at 0.05 and treat reported floor readings as invalid.
- (b) Put the floor strictly below the rated minimum so the reported 0.045 is
      valid.

## Decision

(b). `min_valid_range_m` must sit strictly below `RobotSpecs.LIDAR_MIN_RANGE`
(0.045). A floor reading is a measurement (something is there, nearer than the
sensor can resolve), not an absent one.

## Consequences

- A nosed-into-a-corner chassis keeps a usable forward cone instead of a
  fully-invalid lane.
- Any change to `lidar.min_range` in robot.toml must not cross this floor, or
  the same wedge returns.
