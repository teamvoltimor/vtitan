# 0064. A corner sign's corridor is decided by depth, and the clearance budget is thin

- Status: accepted
- Date: 2026-09-15

## Context

`corridor_for_position` resolved a corner by NEAREST FACE, which is the wrong
axis. A sign at depth exactly 2.00 m needs a millimetre of estimate error to tip
past `CORNER_MAX`; it is then millimetres from the perpendicular face and 0.6 m
from its own, so nearest-face hands it the perpendicular one. There the sign's
LATERAL offset becomes its depth, the lane target is computed on the wrong axis,
and the clamp bounds it against the wrong face. Measured with the belief offset
removed, 42.1 percent of published specs were filed on the perpendicular face,
with spikes at 0.40 m (568) and 0.60 m (483) past the corner; boundary signs were
44.9 percent wrong at depth 1.00 and 44.8 percent at 2.00, against 0 of 153
mid-straight.

The clearance at a sign pass is a thin, shared budget, which is why most lane
tuning is zero-sum.

## Options considered

- (a) Resolve a corner by nearest face.
- (b) Decide on depth, which is a legal value (1.00/1.50/2.00) and answers the tie
      because 0.40 m is not.

## Decision

(b). `sign_lane_depth_consistent_corridor = true`: the candidate face whose depth
falls least outside the corner band wins, and a genuine tie keeps nearest-face.
The value sets are disjoint by 0.40 m (depth 1.00/1.50/2.00 against lateral
0.40/0.60/2.40/2.60), so the depth is a valid discriminant.

The verified layout invariants across all 256 scenarios and 1282 signs: depth is
only 1.00, 1.50 or 2.00; lateral only 0.40, 0.60, 2.40 or 2.60; a section holds 0,
1 or 2 signs, never 3; a middle sign is always alone; two signs are always 1.00 m
apart; 1211 of 1282 signs sit at a section boundary and 0 of 1282 in a corner.
These are strong enough to plan against: corners are guaranteed free runway.

The clearance budget at a pass: a squeezed plateau leaves a 0.1814 m lane-to-sign
gap. The simulator collides by exact SAT on the oriented chassis, so the required
gap is `0.15*|sin yaw| + 0.097*|cos yaw| + 0.025`. At zero yaw the budget is
5.94 cm and yaw alone consumes 5.05 cm (85 percent) at the median collision. Only
ONE lever is not zero-sum: moving the lane sideways trades sign clearance against
wall clearance exactly (gap-centring saved 31 signs and lost 58 to the wall);
straightening the chassis at the pass widens BOTH margins. A step change needs a
narrower chassis.

## Consequences

- A corner sign is filed under the corridor whose depth it actually has, so the
  lane target is computed on the right axis. The independent metric (assigned axis
  disagreeing with the invariant) fell 42.1 to 0.0 percent (0/2777).
- Pass-side 121 to 73 (-40 percent), laps>=3 42 to 56, in-time 29 to 40; raw
  collision rises are survivorship (laps driven 229 to 301, collisions/lap flat).
  Unvalidated on hardware (1.42x understeer invisible in sim).
- Do not attribute a sign collision to a single term; check the joint budget.
  Moving the lane sideways is zero-sum and should not be retried.

## History

- 877ecee6 2026-08-16: route Obstacles past signs by moving the path, not the
  carrot. Full 256 sighted: collisions 234 to 70, laps>=3 22 to 186.
- 66fa5f2e and 640dd86b 2026-08-20: gap-centre a squeezed plateau, REFUTED and
  reverted. Wall 3 to 61, laps>=3 56 to 32; whole adjustable range 3.1 cm against
  a 6.3 to 6.6 cm crosstrack shortfall.
- 7d376325 2026-08-25: relabel a sign whose lane target is unsatisfiable. Pass-side
  140 to 121, dual-corridor 22 to 14.
- 3b36bc49 and 8f535d10 2026-08-25: skip-unsatisfiable and split-overlap, both
  refuted and off. Split-overlap pass-side 121 to 110 but collisions 94 to 106;
  the 4.3x lift was correlation from 2.50x discovery duplication.
- a67345a9 (twin 585ce6f7) 2026-08-26: decide a corner sign's corridor by depth.
  The core measurement above.
- 16a881aa (twin 913ae1fc) 2026-08-27: sweep under real sensor errors. Laps>=3
  42/56/42/55 (perfect OFF/ON, real OFF/ON); in-time gain larger under real
  sensing; wall collisions 24 to 34 per lap.
- 0047a808 2026-08-27: repair sign crosstrack; pass yaw is corner-driven. Boundary
  yaw 16.03 deg against middle 6.46; 79.8 percent of collisions on the arc.
- d5cd8ac6 2026-09-11: widen the lane plateau (0.25 to 0.40).
- 7618f32b and 4ef36c7b 2026-09-12: make the inner-wall rule a sweep arm and
  restore gap-centring; the 2026-08 refutation is obsolete, not wrong.
- 855c319b 2026-09-12: revert the plateau to 0.25, 0.40 was never tested on
  hardware.
- 56d8d9a1: correct the corner-exit over-claim.

## Cross-references

- 0051 owns the lane; 0049 owns the corner arc, which sets the corner runway these
  signs sit beside.
- 0004 and 0005 stay separate; their division lines and pillar tolerance are the
  geometry the invariants rest on.
- 0035 is superseded into 0059.
