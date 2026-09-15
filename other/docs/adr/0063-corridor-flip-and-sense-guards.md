# 0063. The corridor flip and the sense guards are temporal, not geometric

- Status: accepted
- Date: 2026-09-15

## Context

At a corner boundary the corridor label flipped EAST/NORTH every tick, swapping
the deformation axis. A geometric dead-band cannot fix it: the corner tie-break
picks the NEAREST inner face, so a point 0.40 m deep in the east band but 0.003 m
past the north one classifies NORTH, and nudging the position toward the label you
want to keep can make that label LESS likely. Distance to the decision surface is
not usable as a margin.

Separately, two sense guards were added to catch a target the chassis would only
reach by going round the loop the wrong way.

## Options considered

- (a) A geometric dead-band on the corridor label; rely on the target search's
      arc-length bound to stop wrong-way targets.
- (b) Require consecutive agreement before committing the label; gate the target
      search on path SENSE.

## Decision

(b). `corridor_flip_ticks = 1` (the mechanism inert) requires the robot's corridor
to change on consecutive ticks before the label commits. It is deliberately
temporal, not geometric. The code and the sweep arm stay so it can be re-tested on
hardware: the sim commands steering with infinite bandwidth, so a 20 Hz axis flip
costs it almost nothing, where a real servo must physically slew between the two
commands.

`target_sense_gate = false` (off) rejects a candidate the chassis would only reach
by going the wrong way, by projecting the pose-to-candidate bearing on the path's
outgoing bearing. The arc-length bound CONVERTED this failure rather than closing
it: at 0.45 m the correctly chosen point sits behind a chassis that has turned
round, and nothing in the tree turns a chassis round. The sense gate is
PREVENTION, not recovery; once fully rotated every candidate is wrong-sense and
the gate empties.

`sign_deform_sense_guard = false` is INERT on this tree: `sign_lane_planner` and
`sign_lane_suppress_deform` are both true and `sign_lane_deform_fallback_m` is
0.0, so the navigator never reassigns `steer_target` to the deformed point. It is
kept so the deform cannot be re-enabled without it. Its motivating evidence is a
COUNTERFACTUAL: `sign_deform_magnitude_m` is computed unconditionally, applied or
not, so the wrong-sense separation measured on it does not say what it first
appeared to.

`sign_lane_corner_entry_m = 0.5` suppresses lane entry within 0.5 m of a corner,
because 1211 of 1282 signs sit at a section boundary with no near-side runway and
corner arcs are guaranteed free (0 of 1282 signs in a corner). A 0.25 value halves
collisions sighted but inverts blind.

## Consequences

- The label cannot oscillate on the tie-break's shape.
- The two guards are prevention, measured never-worse but unvalidated on track,
  and they never rescue a reversal.
- The sense guards share a symptom with two origins (wrong-way search and wrong-way
  deformation) and are measured on different runs; neither covers the other.

## History

- 869dc7a1 2026-08-15: correct the inverted sign/park collision split and
  re-measure the arrive-square premise. Adds `corridor_flip_ticks` default 1.
  Traced on go_obstacles_0000: the estimate wobbled +/-5 mm at y=2.00, flipping
  EAST/NORTH every tick, steering swinging -0.068 to -0.172. Damping flat:
  collisions 252/254/252 and laps>=3 4/2/4 at ticks 1/5/10.
- c1e26857: adds `robot_corridor_flip_ticks` for discovery.
- 59c926a1 2026-09-10: the corridor-gate axis must be BLIND. Sighted arms were
  byte-identical and void; run properly, flip_ticks 5 leads on all four headline
  metrics (10 collisions / 214 laps>=1 / 196 laps>=3 / 168 in-time against 12/209/190/164
  at 10, 11/213/189/166 at 20, 11/211/195/165 at 40).
- e52bca88 2026-09-12: gate the target search on path SENSE, retract the deform
  half. `target_sense_gate`: Obstacles sighted in-time 98 to 100, blind 99 to 102,
  collisions 16 to 16 and 20 to 17; Open 128/128, mean -0.12 s. Prelaunch
  wrong-sense 23.0 percent. Retracts `sign_deform_sense_guard` as inert.

## Cross-references

- 0045 and 0051 own the sign router and lane; this story owns the corridor label
  and the sense guards.
- 0064 owns the depth-based corridor decision that complements the temporal flip.
