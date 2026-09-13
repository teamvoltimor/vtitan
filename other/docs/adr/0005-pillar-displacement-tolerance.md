# 0005. Pillars are displaced, not scored as first contact

- Status: accepted
- Date: 2026-09-13
- Commit: d7a31bf3

## Context

Touching a traffic pillar is NOT a failure. The pillar may be nudged and the run
stays valid so long as ANY corner of its 50 mm square is still inside the
placement circle; only pushing it fully out counts against the team.

The tolerance that follows from the geometry is generous: 59.4 mm of displacement
pushing along an axis, and 77.9 mm diagonally. That is why the simulator must
model displacement rather than end the run on first contact.

## Options considered

- (a) End the run on first contact with a pillar.
- (b) Model displacement and fail only when the pillar leaves the placement circle.

## Decision

(b). `placement_circle_diameter = 0.085` m is the circle each pillar is placed
within, and the simulator models pillar displacement. Only pushing the pillar
fully out counts against the team.

## Consequences

- A nudge is survivable, matching the official geometry.
- The simulator must track pillar pose, not just a contact flag.
