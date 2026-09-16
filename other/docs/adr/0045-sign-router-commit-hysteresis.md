# 0045. The sign router holds its committed sign

- Status: superseded by 0051
- Superseded by: 0051
- Date: 2026-09-07

## Context

With two signs in play -- the WRO grid spaces them 0.50 m apart along a 1.0 m
corridor, so this is common -- a pure per-tick nearest-wins race can flip the
winner mid-approach and jump the commanded lateral line from one sign's required
value to the other's with no runway left to track it.

Measured 2026-08-01 over the corpus it read flat (sighted 229 collisions either
way, blind 227 off vs 228 on) and was dismissed as not earning its place.
Re-run 2026-09-07 the corpus still reads flat: in-time 59 = 59, laps>=3 71 to
70, collided 18 to 19.

What changed is the instrument, not the number. This holds the aim point still
when two tracks of the SAME pillar compete, and duplicate tracks exist in both
worlds at the same rate (sim 2.0x, hardware 2.2x) but NOT at the same
separation:

| duplicate nearest-neighbour | p50 |
|---|---|
| sim | 0.012 m |
| hardware | 0.21 m |

So a winner-switch moves the commanded line 1.2 cm in the corpus and 21 cm on
the mat, a factor of 17. The corpus was pricing a defect it barely has. Replayed
over recorded detections from the 09-07 runs (`diag_bag_sign_target_churn.py`),
holding the commitment cuts aim-point jumps over 0.15 m from 38 to 21, with the
committed-tick count unchanged at 1538.

## Options considered

- (a) Re-run the nearest-wins race every tick.
- (b) Once engaged, keep routing around that sign until it is cleared.

## Decision

(b). `commit_hysteresis` is ON since 2026-09-07. Reported from the track first:
the robot lines up correctly to pass a sign, keeps correcting, and arrives badly
placed; the target was moving, not the controller.

## Consequences

- The aim point cannot jump to a second sign mid-approach with no runway left.
- Corpus sweeps must be read with the duplicate-separation factor in mind: a
  flat corpus result does not clear a mechanism that matters 17x more on
  hardware.

## Superseded by 0051

Replaced by [0051](0051-sign-lane-planner.md): Sign avoidance rewrites the path into a
lane.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
