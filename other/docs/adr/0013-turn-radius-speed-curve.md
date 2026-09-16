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

## Superseded by 0086

This decision was replaced by [0086](0086-simulator-realism.md). Its content is reproduced below so this file stays self-contained; edit only the successor.

### Context

Until 2026-09-14 the entire 256-scenario corpus ran with a PERFECT pose, perfect
start placement, a zero-lag camera that spoke on 54.5 percent of ticks, a LIDAR
with a uniform 1 percent dropout and no chassis occlusion, a chassis that stopped
dead on any contact, and a servo that slewed at the cornering policy rate. The
simulator was also turning 1.83 times sharper than the real car (3512 deg of yaw
against the IMU's 1918) because `yaw_gain` was implicitly 1.0. Every conclusion
drawn on the easy side of those assumptions was suspect.

### Options considered

- (a) Keep the idealised model and treat the differences as known.
- (b) Model the measured physics and error budget, and mark the results that cross
      a calibration as incomparable.

### Decision

(b). `yaw_gain = 0.55` is a measured fraction of the kinematic yaw rate; the
shortfall is flat (about 0.49) from 5 to 30 deg of commanded wheel angle, which is
what makes it a scale rather than large-angle saturation, and it is a property of
these tyres on this surface. The turn-radius floor is speed-dependent:
`min_turn_radius_m = 0.29` is the curve's value at one speed, and the floor is
`min(cap, intercept + slope * |v|)` with `intercept 0.053`, `slope 1.86` and
`cap 0.35` (a safety bound, not a measurement; the Open range is extrapolation).

The measured sensor error budget (`sensor_start_pos_error_m = 0.05`,
`sensor_yaw_bias_rad = 0.03`, `sensor_imu_drift_rad_per_s = 0.000145`,
`sensor_gyro_scale_error = 0.005`, `sensor_imu_noise_rad = 0.005`) is on by
default. The camera gets the hardware's detection rate and latency
(`vision_latency_s = 0.85`, `vision_frame_miss_rate = 0.79`, emulated 11.0 percent
of ticks against hardware's 11.6 percent); `vision_color_flip_rate = 0.0` ships at
an honest zero because it is unmeasured, and the magenta-barrier failure cannot be
emitted by the simulator at all. The LIDAR models chassis occlusion (two blind
front-corner bands, `lidar_occlusion_min_deg = 25`, `max 60`, with measured
dropout and self-return) and `lidar_invalid_ray_rate = 0.095`. A blocked
translation slides along the surface (`contact_slides_along_surfaces = true`)
instead of stopping dead; the old model measured 56 times less progress at a 20
deg approach.

### Consequences

- Every simulator number predating the 2026-09-14 fidelity batch is not comparable.
  Commits that break comparability carry `!` in the type (see 0087).
- `vision_through_pinhole = false` means the corpus still does not exercise the
  real bbox decode path, pending an Obstacles re-baseline.
- `scrub_yaw_gain` (code default 0.0) is the suspected home of the remaining escape
  yaw gap; free-space replay cannot generate it.
- `min_turn_radius_cap_m` is not measured, and terminal-surface contact still ends
  the run on the first tick.

### History

- bfd644d4 2026-08-29: calibrate `AckermannKinematics` against a bag; add
  `yaw_gain = 0.55` and `speed_response_tau_s = 0.35` (1.83x to 1.03x).
- 544958be 2026-09-02: a sliding contact model and cycle bay exit, both off, both
  refuted at the time (254/256 no-slide against 0/256 with slide).
- 35d0429d 2026-09-10: `min_turn_radius_tracks_speed` added inert with the measured
  curve.
- 7cd402d2 2026-09-09/10: the curve honours servo slew; `cap = 0.35` is not
  measured.
- ced51207 2026-09-14: model the C1's chassis occlusion and real dropout rate over
  5,763,600 real rays (no-return 25.4 percent against sim 1 percent).
- 8c503c6c 2026-09-14: slew the simulated servo at the hardware rate (2.4 rad/s),
  not the cornering policy (1.2).
- ac2b6637 2026-09-14: run the corpus on the measured sensor error budget (BREAKING
  comparability).
- 0e088ada 2026-09-14: give the emulated camera the hardware's detection rate and
  latency (54.5 to 11.0 percent).
- 86bee47f 2026-09-14: slide a blocked chassis along a surface.
- 34c327b8 2026-09-14: floor curvature on the measured speed curve (0.62 to 0.67x
  worst run).
- 95ed97cb 2026-09-14: drop `vision_false_positive_rate`, which nothing read.

### Refuted

- The implicit zero-slip `yaw_gain = 1.0`; the perfect pose and start as adequate;
  the constant turn radius as global; a non-sliding contact; the guessed 1 percent
  LIDAR dropout; `min_turn_radius_cap_m` as a measurement; `vision_false_positive_rate`.

### Cross-references

- 0012 and 0013 are superseded; their decisions are carried above.
- 0084 owns the localizer; 0087 owns the test methodology and the `!` convention;
  0078 owns the camera lens; 0080 owns the LIDAR mount.

