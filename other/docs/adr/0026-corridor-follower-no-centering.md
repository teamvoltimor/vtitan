# 0026. The corridor follower does not centre

- Status: superseded by 0057
- Superseded by: 0057
- Date: 2026-08-22

## Context

Position error and heading error are 90 deg out of phase in a steered chassis,
so a proportional-on-position centring term is an oscillator. Measured on
hardware 2026-08-07: 112 steering sign flips in 177 s, 45 percent of ticks
pinned at `max_centering_steer_deg`, heading 30 deg off axis at the median. That
starves the direction gate, which needs the chassis square to a corridor at the
moment one side opens.

Traced on a failing scenario: starting 0.303 m from the outer wall of a 1.0 m
corridor, the ~0.2 m lateral correction swung the heading 0 to 32 deg and
crossed the estimator's 25 deg alignment gate about one second before the
corridor end arrived. The robot reached the corner already forbidden to look,
never settled, never got a plan, and sat there until the no-progress bailout. A
passing run held 6-8 deg flat and voted the instant a side opened.

## Options considered

- (a) Keep centring and rely on it to hold the middle of the corridor.
- (b) Zero the centring term and keep only the heading damping term.

## Decision

(b). `centering_gain_deg_per_m = 0.0` since 2026-08-22. The creep does not need
to centre: it exists so the direction estimator can settle, and centring is what
stops it settling. Heading error remains damped by `heading_gain` and clamped by
`max_centering_steer_deg`; the corner and back-off branches use
`max_corner_steer_deg` instead (ADR 0027).

## Consequences

- Sitting off-centre for a metre costs nothing; oscillating costs the round.
- The direction gate is no longer swung past its alignment tolerance by the
  centring loop.
