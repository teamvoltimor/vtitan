# 0059. The pass-side rule is travel-relative, and a scorer must not share the scored system's convention

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0034, 0035, 0043

## Context

WRO rule 9.19 is travel-relative: the vehicle passes to its own RIGHT of a red
pillar and to its own LEFT of a green one, in the direction the round is driven.
The vehicle's right is the OUTER wall counterclockwise and the INNER square
clockwise, so the same rule names opposite world directions in the two travel
directions and cannot be evaluated without knowing the direction.

Between 2026-07-05 and 2026-09-03 the routing table was ABSOLUTE: red always
outward, the clockwise rows identical to the counterclockwise ones. That is right
for counterclockwise and backwards for every clockwise round. It was invisible
because the simulator's scorer used the same absolute convention the router drove,
so the simulator graded the router against the router's own mistake. The test
suite had been rewritten from the same misreading and asserted red-outward for
both directions, so it passed on the bug it was covering. Two months of pass-side
figures were scored on the absolute rule and mean nothing under the real one.

The scoring model had its own version of the same error. Wrong-side passes were
read from the router's believed-frame record, which flagged 46 percent of passes
against a true 21 percent and ended 45 of 64 runs where 38 genuinely offended.

## Options considered

- (a) Keep the absolute table and score from the router's belief.
- (b) Make the rule travel-relative and score from true layout and true pose in an
      independent scorer.

## Decision

(b). The routing table's CW rows are the exact sign-flip of the CCW rows, and
`pass_side_lateral_axis(corridor, color, direction)` returns `None` when the
direction is unsettled. The router's `_record_pass_side` is keyed on the travel
direction, not a hardcoded clockwise.

Wrong-side passes are scored by an independent scorer over the TRUE layout and
TRUE pose, never read from the router's record. A violation is terminal only when
the chassis FOOTPRINT completely crosses the sign's radius while on the forbidden
side (rule 9.24.5); a partial crossing is the permitted fix-it window and ends
nothing, and a violation is never cleared by a lap boundary. The router's record is
still kept as a measure of discovery quality; it must not be what ends a run. The
judging helpers take the round's TRUE direction, the delivery helpers the router's.

The cross-cutting rule: a scorer must not share its convention with the thing it
scores. Independence today is achieved through the FRAME (true layout and true
pose, per tick, footprint test) and through the rule now being the correct
travel-relative one; the scorer still imports the production rule table, so a
future regression in `ROUTING_TABLE` would again be shared by scorer and router.

Rule 9.21: the vehicle may drive against the round direction for two sections
only, the section where the direction changed and its neighbour, and the round
stops once the projection is completely out of that window. Travel direction is
taken from velocity, not heading, because the rules permit back-to-front driving
while moving in the round direction. Two simplifications are documented and
chosen stricter than the rules.

`obstacles_inner_wall_terminal = true` keeps the strict scoring every figure was
measured under. The actual rule set is: OPEN, the outer wall may not be touched;
BOTH, a wall may not be MOVED; OBSTACLES, the parking lot may not be touched
(9.24.7). Terminal regardless of the flag: the parking lot, displacing a sign out
of its 85 mm circle (9.20), and the wrong pass side. A result must state which
scoring it used; one taken under each and compared is meaningless. The inner-wall
flag is not a lever: its whole corpus effect is 3 collisions and 2 laps, exactly
zero at the shipped placement.

## Consequences

- The rule can no longer be driven correctly counterclockwise and backwards
  clockwise.
- Every pass-side figure recorded before 2026-09-03 is void.
- The scorer shares the routing table's axis rule; only the FRAME and the
  direction source keep it independent. This residual coupling is known.
- The pass-side deficit is tracking, not routing: the planned path passes 642/642
  signs on the required side and clips zero, cross-track median 4.63 cm against a
  5.6 cm plan margin. Moving the lane sideways trades sign against wall exactly
  (gap-centring saved 31 signs, lost 58 to the wall); only straightening the
  chassis widens both margins at once, and a step change needs a narrower chassis.

## History

- 360770c5 2026-07-05: flip all four CCW rows to the absolute rule (red outward,
  direction-independent). The bug that stood for two months.
- 53a6f746 2026-08-22: flag red/green signs passed on the wrong side.
- cef75f2a 2026-08-24: record pass-side violations on `SimResult`; make wrong-side
  terminal.
- d7eec942 2026-08-24: score the rule from truth, not the robot's belief. Believed
  verdict flagged 46 percent against a true 21 percent.
- 0f5e446c 2026-08-24: attribute Obstacles wrong-side passes to plan, chassis or
  perception (91 routing / 95 tracking of n=188).
- 585ce6f7 (twin a67345a9) 2026-08-26: decide a corner sign's corridor by depth.
  Pass-side 121 to 73, laps>=3 42 to 56; independent metric 42.1 to 0.0 percent.
- 879198f7 2026-09-03: the pass-side rule is travel-relative, not absolute.
  `outward_lateral_axis` to `pass_side_lateral_axis`; timeouts 27 to 18,
  escapes/lap 59.25 to 51.55, clean 83 to 88, in-time 76 to 83. Two West/CW demos
  now fail honestly. "Every pass-side figure recorded before this commit is void."
- b331c1fb 2026-09-03: the Go port.
- 9a181319 2026-09-03: score the rule as written, by footprint radius-crossing
  instead of closest approach. Pass-side 115 to 113, clean 88 to 91, escapes/lap
  51.55 to 44.67. Direction split CCW 47/122 (38.5 percent), CW 66/134 (49.3).
  Recovery window refuted (about 2/256).
- a044dee6 2026-09-04: a judge must not score a travel-relative rule under the
  direction the robot merely believed; that shared convention is exactly what hid
  the bug.
- 8a72f98a 2026-09-04: enforce rule 9.21. 14/256 runs were driving illegally;
  direction from velocity, footprint test.
- 6945d9cb 2026-09-04: order rule 9.21 against U-turns; the causal reading
  (violations ARE corner U-turns) refuted.
- a68f3e76 2026-09-06: score rule 9.24.5 from truth in Go. Router 53/256 against
  truth 0/256.
- 49f77bf3 2026-09-12: the 8-cell sign-pair taxonomy; `sign_lane_hold_m` back to
  0.25 (25 to 21 collisions, 191 to 218 in-time).
- 4ef36c7b 2026-09-12: gap-centre the squeezed lane; the 2026-08 refutation is
  obsolete, not wrong.
- 5342f125 / 83f181ae 2026-09-15: ground-truth pass-side judge, first failing its
  own control then passing it. Wrong-side passes are per-pillar and total: two
  green pillars at the southern end of the west and east corridors fail every lap
  while the other five are perfect.

## Cross-references

- 0034, 0035 and 0043 are superseded; their decisions are carried above.
- 0049 and 0064 own the corridor geometry the pass-side rule keys on; 0051 owns
  the lane.
- 0062 owns the inner-wall scoring flag's place in the sim contact model.

## Evidence

- `known_start = true` is refuted: pose error collapses 1.477 m to 0.011 m, yet
  every wrong-side bucket moves by at most one count, so the believed frame is
  innocent.
- The reproducible wrong-side split is 91 routing / 95 tracking of n=188; the
  2026-08-24 122/68 headline is an unrecoverable environment artifact.
- `plan-wrong` is a 5x risk factor, not a cause: plan-wrong violates 51 percent,
  but 89 clean passes are also plan-wrong.
- Hardware pass-side is a runway and distance problem, not authority: crossings
  fail 67.8 percent against 6.9 percent for already-legal passes, and crossings
  under 0.25 m fail 89.5 percent while holding 52 percent of all crossings.
- Escapes HELP crossings (any manoeuvre 58.0 percent fail against 89.3 percent
  with none), so suppressing them must not be proposed.
- `obstacles_inner_wall_terminal = false` scores the real rule (9.18 permits
  touching an unmoved wall) and the 82-test battery is insensitive to it, so
  pricing needs the 256 corpus under both scorings.
