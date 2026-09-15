# 0012. Yaw gain is a measured fraction of the kinematic model

- Status: superseded by 0086
- Superseded by: 0086
- Date: 2026-08-29
- Commit: bfd644d4

## Context

The fraction of the yaw rate the kinematic model predicts that the chassis
ACTUALLY delivers is 1.0 in the zero-slip textbook model; anything less is the
tyre slip and linkage compliance that model has no term for. Before this was
measured the simulator ran an implicit 1.0 and turned 1.83x sharper than the real
car over the same command stream: 3512 deg of yaw against the IMU's 1918, so every
corner the sim cleared was a corner the hardware would have run wide.

## Options considered

- (a) Keep the implicit 1.0 zero-slip model.
- (b) Measure the delivered yaw and scale the model.

## Decision

(b). `yaw_gain = 0.55`, measured 2026-08-29 from `run_20260829_140424` (a clean
3-lap Open run) via `scripts/bag/diag_bag_sim_fidelity.py --replay`. The shortfall
is flat (~0.49) from 5 to 30 deg of commanded wheel angle, which is what makes it
a scale factor rather than large-angle saturation.

## Consequences

- Sim yaw matches the hardware over the same command stream.
- It is a property of THESE tyres on THIS surface: re-measure after a tyre, weight
  or mat change, the same way `max_speed_mps` is re-measured after a motor swap.

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

