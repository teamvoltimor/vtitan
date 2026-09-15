# 0047. The contact reverse ships disabled

- Status: superseded by 0088
- Superseded by: 0088
- Date: 2026-09-13

## Context

The contact reverse backs the chassis off when it has closed to `contact_dist`,
instead of creeping forward into it. It is ~0.4 s / 25 mm at creep, with a
cooldown that only counts down while the path is CLEAR so the back-off and the
approach cannot oscillate against the same obstacle.

It was believed (premise, later refuted) that a trail gate -- only reversing
when the pose trail confirms enough covered ground -- would rescue the
behaviour by stopping it firing against a surface the chassis had not actually
reached. The hardware case it was written for is real: `run_20260906_121254`
spent ten seconds at +0.152 m/s against a green pillar. That case is still
unaddressed, and this mount has no rear sensing to make a seeing reverse
possible.

Three back-to-back passes over the 256 corpus, 2026-09-06, differing only in
this value and the gate:

| metric | off(0) | ungated(8) | trail-gated(8) |
|---|---|---|---|
| in-time | 149 | 150 | 150 |
| laps>=3 | 150 | 151 | 151 |
| stuck | 26 | 15 | 24 |
| collisions | 8 | 13 | 11 |
| of which wall | 4 | 10 | 9 |
| timeouts | 65 | 70 | 66 |
| rev-run | 5 | 4 | 2 |

## Options considered

- (a) Enable the contact reverse ungated.
- (b) Enable it gated on the pose trail.
- (c) Ship it disabled.

## Decision

(c). Gating gave back the timeouts and most of the extra collisions, and gave
back the stall benefit with them -- the trail rarely confirms 6.1 cm of covered
ground at the moment the chassis is against something, which is precisely when
it has stopped moving and stopped laying trail. What survives is +5 wall
contacts against a FLAT headline, so the behaviour is not earned in either
form. The value stays 0.

## Consequences

- The code stays, measured and documented, so the behaviour can be revisited
  without rediscovering it.
- The hardware case that motivated it (`run_20260906_121254`, ten seconds of
  +0.152 m/s against a green pillar) remains real and still unaddressed: the
  sim says a blind reverse is not the answer, and this mount has no rear
  sensing to make a seeing one.
