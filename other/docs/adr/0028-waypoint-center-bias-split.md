# 0028. Waypoint centre bias is split by corridor class

- Status: accepted
- Date: 2026-08-29

## Context

The path is a loop around the inner block, so each centimetre nearer it comes
off all four sides: 8 cm of lap per cm of bias. An absolute bias costs a much
larger FRACTION of a 0.6 m corridor than of a 1.0 m one, so one value has to be
safe narrow and therefore gives up lap time wide.

Measured inward vs outward at 0.05: the inward path finished 24/24 against 21/24
and was faster on all 21 comparable scenarios (+18.2 s mean for outward). The
value was raised to 0.10 once per-corner arc radius landed; with per-corner arcs
the corner margin equals the straight margin at every bias (0.203 -
center_bias_m). Measured at 0.10: 24/24 ok, -5.5 s mean over 24 scenarios, 23 of
24 faster. About 0.103 m of inner margin remains, which is already below the
0.16 m median hardware crosstrack.

Narrow corridors are where the INNER block binds: `run_20260829_020308` measured
a median 0.113 m to the inner wall against 0.435 m outer over 3 hardware laps,
with p05 0.069 m. Tracking error ADDS to the commanded bias instead of averaging
out, so the narrow corridor was spending margin twice. A shared side can only
say "narrow hugs the same boundary, less far"; the useful direction to tune
narrow is "outer", which a shared inner side can only reach through zero.

## Options considered

- (a) One `center_bias_m` and one side for every corridor.
- (b) Split the magnitude and the side by corridor class.

## Decision

(b). `wide_center_bias_m = 0.10` (inner), `narrow_center_bias_m = 0.0` (plans
the true centreline), and `unconfirmed_width_inner_bias_m = 0.05` for a narrow
corridor still on the blind prior. The unconfirmed bias shrinks the single-tick
0.30 m lateral shift of the width-belief update and hands the margin back the
moment the width resolves; it does not touch the corner arc, which reads the
CONFIRMED bias.

Measured 2026-08-31 over the full 640-case Open space with the unconfirmed bias:
618 to 631 ok, collisions 1 to 0, incompletes 21 to 9, sim time -12.80 s mean.
21 fixed against 8 regressed, every gain and residual failure in a mixed-width
layout.

## Consequences

- The two classes can be tuned apart without one costing the other.
- `narrow_center_bias_m = 0.0` widens the narrow-to-narrow arc and reduces its
  steering demand; separation and corner sharpness are the same knob.
