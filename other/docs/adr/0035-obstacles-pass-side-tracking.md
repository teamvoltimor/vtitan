# 0035. The Obstacles pass-side deficit is tracking, not routing

- Status: superseded by 0059
- Superseded by: 0059
- Date: 2026-09-01

## Context

The 2026-08-24 investigation reported 122 routing / 68 tracking of n=190 and
made routing the target. Re-running the control at `2c8cbfb` in a worktree
reproduces `ffc3dedd` to every digit (188 violations, 91/95/2, 4319
retirements), so that split differed by ENVIRONMENT, not code. The reproducible
split is 91 routing / 95 tracking of n=188, near-even.

Measured 2026-09-01 on ground-truth widths and sign positions, one canonical lap,
no belief and no simulation: the planned path passes 642/642 signs on the
required side and clips zero. The planner is exonerated. The deficit is a
TRACKING failure: cross-track error at sign passes runs a median 4.63 cm against
a plan margin of 5.6 cm at tight signs.

Two measurements that seemed to disagree by an order of magnitude are both
correct and measure different points: delivery at the sign's own in-span waypoint
is 97 percent, but the chassis follows the polyline, and the polyline comes
closest to the sign on another segment, the corner arc, where the lane is not
applied. 1211 of 1282 signs sit at a section boundary, and 194 of 195 collisions
there.

## Options considered

- (a) Treat pass-side as a routing/planning failure and tune placement.
- (b) Treat it as a tracking/runway failure.

## Decision

(b). The planner already passes every sign on the required side; the lever is
tracking error and the runway available for the lane step, not placement. The
maximin placement lever that follows from the routing reading was independently
refuted (`routing.py:90-96`: sign collisions 199 to 168 but wall collisions 3 to
61).

## Consequences

- Retires every conclusion attributing pass-side to routing in
  ADR 0059.
- The truth-scoring method and the refuted-candidate list in that document
  remain valid; its pass-side numbers are void because they were scored on an
  absolute convention by a scorer that shared it (see the 2026-09-04 banner in
  `obstacles-pass-side-rule-and-runway.md`).

## Superseded by 0059

Replaced by [0059](0059-pass-side-travel-relative-and-scorer-independence.md): The
pass-side rule is travel-relative, and a scorer must not share the scored system's
convention.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
