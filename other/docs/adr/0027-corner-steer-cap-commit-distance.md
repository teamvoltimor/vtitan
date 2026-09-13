# 0027. The corner-turn steering cap scales with commit distance

- Status: accepted
- Date: 2026-08-31

## Context

`max_corner_steer_deg` is the angle the corner-turn and back-off branches steer
AT, sized by GEOMETRY: the corner branch commits its turn at `turn_clearance_m`,
so its arc has to fit inside that clearance. The 21.25 deg value is anchored on
`turn_clearance_m = 0.60`. Two other branches commit closer and inherited the
same angle, so the arc they drive cannot fit:

| branch | commit distance | fits? |
|---|---|---|
| corner, wide corridor (the anchor) | 0.60 m | fits |
| corner, narrow_turn_clearance_m | 0.40 m | does NOT fit |
| back-off, min_forward_clearance_m | 0.30 m | 1.5x too wide |

On a rear-free chassis the back-off branch drives forward under this angle; its
comment calls it "full lock" and it was a quarter of it. That branch also spends
59 percent of colliding runs' creep ticks (3943 of 6657, against 676 in the
corner branch).

## Options considered

- (a) Apply `max_corner_steer_deg` to all three branches.
- (b) Scale the cap by the distance each branch actually commits at.

## Decision

(b). Holding radius proportional to the commit distance gives
`tan(cap) = tan(max_corner_steer_deg) * turn_clearance_m / d`. It returns 21.25
deg at 0.60 m -- so the one measured case does not move -- and 30.26 deg at
0.40 m, 37.87 deg at 0.30 m. The ratio form cancels
`(1 + rear_steer_ratio) * yaw_gain`, so it inherits the anchor's calibration
rather than depending on those two separately.

Measured 2026-08-31 over the full 640-case Open space: 596 to 612 ok,
collisions 18 to 3, zero new collisions, sim time mean -0.01 s with 587 of 596
cases identical. The 3 surviving collisions are exactly the 3 runs that logged
zero corner-branch and zero back-off ticks, so the fix repaired every collision
routed through the branches it touches.

## Consequences

- The cap is commandable: `max_wheel_angle_deg` is 85.0 on the current servo.
- A new branch that commits at a different distance inherits a fitting arc
  instead of the anchor's angle.
