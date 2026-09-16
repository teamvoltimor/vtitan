# 0054. Absolute heading is read off the Manhattan walls, mod 90

- Status: accepted
- Date: 2026-09-15

## Context

Yaw is the only unbounded state the robot carries. The localizer takes yaw as
given and the BNO085 is run in UART-RVC 6-axis mode with no magnetometer and no
absolute reference, so its error is a ramp, not a bound. Measured on 2026-08-06,
0.1 deg/s of drift takes 28 fixtures from 28/28 to 6/28. A robot that is
confidently turned the wrong way is worse than one that knows it is lost.

The track is rectilinear, so every wall segment is axis-aligned and a single
sweep observes absolute heading mod 90.

## Options considered

- (a) Bound heading with the IMU alone.
- (b) Estimate absolute heading from the walls each scan and fuse it into the
      IMU ramp.
- (c) Overwrite yaw with the wall estimate on every scan.

## Decision

(b). `estimate_yaw_from_walls` builds segments from pairs of returns separated by
`baseline_rays = 15`, splits them at a `max_segment_jump_m = 0.30` range step,
discards segments under `min_segment_m = 0.02` and returns at or beyond
`near_max_range_m = 11.0`, and requires at least `min_returns = 3`. It takes a
segment-length-weighted circular mean of `4 * theta`, which folds the four
90-degree candidates together, and only trusts the estimate when the fitted
directions are aligned enough (`min_concentration = 0.55`). The correction is a
complementary filter (`yaw_correction_gain = 0.05` in `state_estimator.toml`),
not a hard assignment.

The baseline is 15 rays because adjacent rays land 12 mm apart at a typical 0.7 m
wall distance against 30 mm of range noise; the first implementation using
adjacent rays returned nothing on every one of 384 samples and looked like a
broken threshold rather than a sampling error. 15 rays gives about 190 mm, six
times the noise.

The concentration test is what rejects a corner, a sign or an open side, where
the returns disagree about where the wall runs and the heading reference is worse
than none. A skipped scan costs only that scan, because the IMU carries heading
between corrections.

(c) is rejected: overwriting yaw would inject the estimate's per-scan noise
straight into steering at 10 Hz and discard the half of the pair worth keeping.

The estimator reads the LIDAR in the robot frame (`0 rad = forward`); the 180
degree mount offset is applied upstream. Getting that frame wrong swaps
front/rear and left/right and is invisible in simulation, which synthesizes
correct robot-frame angles.

## Consequences

- The wall estimate is insurance, not an improvement. At the BNO085's datasheet
  figures and at twice them, the blind difference is within run-to-run variation
  (26 to 27 of 28 either way); the honest headline is blind.
- It is capped at 45 degrees of prior error: beyond that the mod-90 ambiguity
  resolves to the wrong quadrant and the caller cannot detect it.
- It must not be conflated with `yaw_gain` (ADR 0012). `yaw_gain` scales
  predicted yaw rate in the kinematic and sim model and is re-measured after a
  tyre, weight or mat change; the wall estimate is the absolute correction. Do
  not retune one to mask the other.
- Sensor-error tests run with the correction off, because it would mask exactly
  the perturbation those tests exist to measure.
- `yaw_correction_gain` is the dead-reckoning blend, not the LIDAR pose-search
  tuning in `localization.toml`, and it runs whether or not localization is
  enabled. It is deliberately small because `corridor_estimator` files every
  width reading by heading: a large blend would move the heading that decides
  which corridor a reading belongs to.
- A gyro-scale perturbation of 0.5 percent drops the fixture tally from 28/28
  to 25/28, gauging how tight the gyro scale error is as an axis.

## History

- 1a49b076 2026-07-26: bound heading against the walls. Creates `wall_heading.py`
  and the constants; 384 samples over 12 fixtures, every section, both directions,
  4 heading offsets: 0.35 deg mean, 0.90 deg p95, 1.40 deg worst, one declined.
  Sighted drift 0.1 deg/s 13/28 to 28/28, 0.5 deg/s 0/28 to 28/28; blind at
  0.1 deg/s 9/28 to 26/28. 22 new tests plus 3 interaction.
- a9bc70d1 2026-07-28: apply the LIDAR mount yaw offset to real navigation and
  OLED clearances. Raw angle-zero pointed opposite robot-front, swapping
  front/rear and left/right; invisible in every simulated test.
- b1977a32 2026-08-30: port `wall_heading` to Go; the wrong-quadrant lock is
  asserted by a Go test.
- Refactor-only commits moved the values into `wall_heading.toml` (617e1905
  2026-08-01, 001cac4a / 30b11a9c / b730da2b 2026-08-07, 0908d579 2026-08-08)
  without changing the algorithm.

## Cross-references

- 0012 stays separate and must be preserved: `yaw_gain = 0.55` is a measured
  scale of the kinematic model (run_20260829_140424), flat from 5 to 30 deg of
  commanded wheel angle, re-measured like `max_speed_mps`.
