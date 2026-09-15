# 0013. The turn-radius floor is speed-dependent

- Status: superseded by 0086
- Superseded by: 0086
- Date: 2026-09-10
- Commit: 35d0429d

## Context

The tightest turn the chassis can make had been treated as a constant. The bicycle
term has no floor: at the shipped 85 deg lock it gives
`L_eff / (tan(85) * yaw_gain)` = 1.5 cm of radius, which a 30 x 19.4 cm
four-wheeled car cannot do.

A first measurement (2026-09-07, `/joint_states` drive-wheel travel against pose
yaw over five hardware bags) showed the real radius SATURATES, buying almost
nothing past ~30 deg of lock:

| |steer| | effective R | model R | ratio |
|---|---|---|---|---|
| 15-30 | 66.0 cm | 41.7 cm | 1.6x |
| 30-45 | 38.2 cm | 22.5 cm | 1.7x |
| 75-90 | 28.9 cm | 2.3 cm | 12.7x |

Those numbers were taken against POSE yaw. Measured on one time base, pose yaw
sees 0.66x the rotation the IMU does, so a radius taken against it is ~1.5x
overstated; treat that table as the shape, not the scale.

`BayExit` also dead-reckons its own pose from the same bicycle model, and while
that copy had no floor it over-read the bay ratchet's outward travel by 31x
(measured 2026-09-09, `scripts/sim/diag_bay_guard.py`).

## Options considered

- (a) Keep one constant minimum radius (`min_turn_radius_m = 0.29`).
- (b) Model the floor as a linear function of speed.

## Decision

(b). Re-measured 2026-09-10 (`scripts/bag/diag_bay_slip.py`, 33 bags, free space,
lock >= 30 deg, IMU yaw), stable across `--window-s 0.02..0.20` AND
`--settled-s 0..0.8`, which is what makes these four points the finding:

| speed m/s | 0.02 | 0.08 | 0.13 | 0.16 |
|---|---|---|---|---|
| R m | 0.090 | 0.205 | 0.287 | 0.349 |

i.e. `R = 0.053 + 1.86 * v`. So `min_turn_radius_intercept_m = 0.053`,
`min_turn_radius_slope_s = 1.86`, and `min_turn_radius_m = 0.29` is what the curve
reads at ~0.127 m/s. The bay exit runs at CREEP end to end, where the constant is
nearly 2x too large, which is why `72e7172b` took the in-bay exit from 16/16 to
0/16 while hardware kept getting out.

THE CAP IS NOT MEASURED. Above ~0.17 m/s the bins fall to n=20-37 and go
non-monotonic (0.349 at 0.16 m/s, then 0.285 at 0.22), and that is structural
rather than a sampling accident: at full lock the robot is SLOW by definition,
because it slows down to turn. The saturation cannot be read from these bags at
all. `min_turn_radius_cap_m = 0.35` is the largest value the measured range
supports, carried as a bound so the linear term cannot run away, not a measurement.
The OPEN challenge runs at 0.26-0.50 m/s, entirely outside the measured range, so
for the corridor this curve is EXTRAPOLATION. It is inside it for the bay.

Two corrections are already baked in, both found by testing the instrument: the
numbers are NET yaw over a window (summing |yaw| at the IMU's 166 Hz accumulates
noise and read 0.088 where this reads 0.29), and they require the steering COMMAND
to be steady, because the servo slews and its feedback is the command echoed back
(unfiltered, corner-entry transients inflated the top of the curve from 0.35 to
0.43).

## Consequences

- `min_turn_radius_m` is one sample of the curve, not a global constant.
- The cap is a safety bound, not a measurement; the OPEN speed range is
  extrapolation.
- Re-measure after a tyre, weight or linkage change, the same way `yaw_gain` is.
