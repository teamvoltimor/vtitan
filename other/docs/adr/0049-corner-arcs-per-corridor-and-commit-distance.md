# 0049. Corner arcs are sized per corridor and the steering cap from the commit distance

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0027

## Context

The corner-turn and back-off branches steer at a cap, and that cap only works if
the arc it drives fits the clearance the branch commits at. Two separate values
carried that job and both were wrong in the same way: one global `arc_radius`
(0.45 m) for all four corners, and one `max_corner_steer_deg` (21.25 deg) applied
to all three steering branches.

The global radius is correct for every corner except narrow-to-narrow, where the
arc bulges past the corridor centreline toward the inner block: 0.070 m of chassis
clearance where 0.153 m was available, less than half. The global cap is anchored
on `turn_clearance_m = 0.60`, so the branches that commit closer inherit an angle
whose arc cannot fit: the narrow branch commits at 0.40 m and the back-off branch
at 0.30 m, where the same 21.25 deg is 1.5x too wide. The back-off branch is
where 59 percent of colliding runs' creep ticks are spent (3943 of 6657, against
676 in the corner branch).

A third failure is blind-specific. A blind round starts believing every corridor
narrow, so a narrow-to-wide corner plans a 0.300 m entry where the true geometry
wants 0.450 m and the robot commits 0.15 m late (0.38 s at the medium tier).
Confirming wide needs 12 aligned readings, about 0.5 m of travel, so the
correction generally arrives after the entry point has already passed.

## Options considered

- (a) One global `arc_radius` and one `max_corner_steer_deg` for every corner
      and every branch.
- (b) Size each corner from the two corridors it joins, and scale the cap by the
      distance each branch commits at.
- (c) Lower `arc_radius` globally to buy clearance.

## Decision

(b). `arc_radius` is a ceiling, not the radius:
`corner_arc_radius = min(arc_radius, max(W_entry, W_exit) / 2 - center_bias)`.
`max` rather than `min`: the arc has to reach the wider corridor's centreline to
be tangent to it, and forcing it to the narrow side pulls the arc off that tangent
and into the corner (0.05 m clearance on a mixed corner instead of 0.25 m).

`corner_arc_assume_wide = true` sizes every corner as if both corridors were
wide, so the arc and its entry point do not depend on a width belief that starts
wrong. This is Open-only without a gate: every Obstacles corridor is 1.0 m by
rule, so the substitution is the identity there (verified byte-identical).

The cap is scaled by commit distance:
`tan(cap) = tan(max_corner_steer_deg) * turn_clearance_m / d`. The ratio form
cancels `(1 + rear_steer_ratio) * yaw_gain`, so it inherits the anchor's
calibration instead of depending on those two separately. Outputs: 21.25 deg at
0.60 m (unchanged), 30.26 deg at 0.40 m, 37.87 deg at 0.30 m. All commandable,
since `max_wheel_angle_deg` is 85.0 on the current servo.

With the shipped biases (wide 0.10, narrow 0.0) the true radii are: narrow 0.45
(the ceiling binds), wide 0.40 (the bias binds), Obstacles 0.35 (the bias binds).
Because the width is forced wide while the bias is not, the wide corridor receives
the tighter arc of the two, 0.40 against 0.45.

(c) is refuted. Clearance is flat below the per-corner value, so a smaller radius
buys nothing and costs lap length: for a rounded rectangle `dP/dr = 2*pi - 8 =
-1.717`, so a wider arc is a shorter lap. Dropping `arc_radius` to 0.30 globally
was measured +5.8 s mean over 24 scenarios, 23 of 24 slower, and changed no
verdict.

## Consequences

- A new branch that commits at a different distance inherits a fitting arc
  instead of the anchor's angle.
- The cap is commandable at every branch; the physical linkage is not the limit.
- The narrow-to-narrow arc is the only one that reaches the ceiling; raising the
  bias widens it, lowering the radius tightens it, and that is the single lever
  for sharper corners.
- `unconfirmed_width_inner_bias_m` does NOT touch the arc: `calculate_waypoints`
  passes the CONFIRMED bias, so the arc is invariant to it. The earlier claim that
  it tightened the lap-1 arc was wrong and wrongly excluded the 0.10 arm.

## History

- 013c3ebf (twin 98586b45) 2026-08-08: size each corner arc from the corridors it
  joins; make `ARC_RADIUS` a ceiling. Narrow-to-narrow clearance 0.167 to 0.250 m
  (chassis margin +0.070 to +0.153 m); Open 24/24 ok, +1.57 s mean, a quarter of
  the +5.80 s that shrinking the radius globally to 0.30 cost. 7 new tests.
- 81741f89 2026-08-08: raise the centreline bias 0.05 to 0.10 now the corner
  allows it. 24/24 ok, no verdict changed, -5.49 s mean, 23 of 24 faster; leaves
  0.103 m of margin per side against a 0.16 m median hardware crosstrack.
- 9efbbe92 2026-08-06: (predecessor, pursuit) arm the short lookahead on the
  corner ahead, not the error behind. `corner_preview_distance_m = 0.40`,
  `corner_turn_threshold_rad = 0.35`.
- ee3ccfcb 2026-08-31: Open speed ladder and belief-independent corner arcs.
  `corner_arc_assume_wide` added. balanced128 seed 0: 94 to 121, about 12.5 s
  faster per run at equal reliability; narrow-to-narrow clearance traded
  0.153 to 0.070 m for never being late.
- 1cc37ba5 2026-08-31: derive the steering cap from each branch's commit distance.
  640-case Open: 596 to 612 ok, collisions 18 to 3, zero new collisions, sim time
  mean -0.01 s with 587 of 596 cases identical. The 3 survivors logged zero
  corner-branch and zero back-off ticks.
- c94e8a31 2026-08-31: correct the unconfirmed-bias ceiling, it is clearance not
  the arc. Refutes the arc-coupling premise; 0.15 loses 16 cases on balanced128
  (18 worse, 2 better) by leaving a genuinely narrow corridor 0.053 m of inner
  margin against ~0.07 m of measured inward drift.
- 643f8982 2026-09-13: split the corner preview by corridor width class, override
  unset. Screened at 0.57: uniform wide n=48 -2.04 s (48 faster, 0 slower);
  uniform narrow n=32 +3.63 s (3 faster, 29 slower). Three arms rejected:
  `wide_center_bias_m` 0.10 to 0.05 (+1.83 s, 0 faster, 48 slower), to 0.15
  (+0.16 s, 17/25), and `lookahead_short` 0.16 to 0.22 (+0.88 s, 1 faster,
  46 slower).
- 2026-08-28: refuted the "mixed narrow-to-narrow wedge is the max(entry,exit)
  arc" theory. Both outer walls never drop below the straights' clearance; only
  the inner block corner gets closer. Bag run_20260828_231522.

## Cross-references

- 0027 is superseded: its anchor, cap formula and the 640-case measurement are
  carried above verbatim.
- The corner preview and corner latch belong to the pursuit target-selection
  story, not here; only the `corner_preview_distance_m` interaction is noted.
- 0028 (waypoint centre bias split by corridor class) is the bias this arc
  consumes; it stays separate for now.
