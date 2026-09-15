# 0039. The Obstacles contact zone is per-challenge

- Status: superseded by 0061
- Superseded by: 0061
- Date: 2026-08-31

## Context

The contact threshold fires the reversing escape (`assess_risk` returns CRITICAL
below it), and the two challenges present different things to escape FROM. A
wall 0.10 m ahead in Open is a genuine emergency. Obstacles additionally has
signs the router deliberately routes PAST at about 0.175 m from their surface,
so 0.10 fires on geometry the planner chose on purpose: the robot escapes from
clearances it was aimed at. Measured 2026-08-31 on subset128: about 180 escapes
per run while colliding with a sign only 4-5 times in 128 runs.

Worth, on the full 256 corpus (2026-09-01, raw in
`src/.corpus/contactdist_256_fixed.txt`):

| metric | 0.10 | obstacle zone |
|---|---|---|
| laps>=3 | 11 | 82 |
| laps>=1 | 41 | 136 |
| in-time | 3 | 52 |
| timeouts | 126 | 48 |
| stuck | 19 | 10 |
| escapes/lap | 685 | 44 |
| sign collisions/lap | 0.132 | 0.009 |

Wall collisions stay FLAT at 17, the check that the extra laps are not bought by
driving harder at walls.

## Options considered

- (a) One contact distance for both challenges.
- (b) A per-challenge zone resolved once at construction.

## Decision

(b). `obstacles_contact_dist` supersedes `contact_dist` whenever a sign router
is attached. There is deliberately no `open_contact_dist`: nothing measured
wants Open to differ, and an unset knob nothing has ever moved reads as tuning
that exists.

## Consequences

- The threshold is genuinely optional (a pointer): absent means "leave
  contact_dist alone", not "escape never fires".
- Verify on 256, never a subset: this plumbing check passed on 16 fixtures and
  128 scenarios and failed at 256, because the escape gates never bound in the
  smaller sets.
- NOT hardware-validated; the stopping-distance bench is still unrun.
