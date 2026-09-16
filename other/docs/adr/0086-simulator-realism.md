# 0086. The simulator runs on a measured error budget, not a perfect world

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0012, 0013

## Context

Until 2026-09-14 the entire 256-scenario corpus ran with a PERFECT pose, perfect
start placement, a zero-lag camera that spoke on 54.5 percent of ticks, a LIDAR
with a uniform 1 percent dropout and no chassis occlusion, a chassis that stopped
dead on any contact, and a servo that slewed at the cornering policy rate. The
simulator was also turning 1.83 times sharper than the real car (3512 deg of yaw
against the IMU's 1918) because `yaw_gain` was implicitly 1.0. Every conclusion
drawn on the easy side of those assumptions was suspect.

## Options considered

- (a) Keep the idealised model and treat the differences as known.
- (b) Model the measured physics and error budget, and mark the results that cross
      a calibration as incomparable.

## Decision

(b). `yaw_gain = 0.55` is a measured fraction of the kinematic yaw rate; the
shortfall is flat (about 0.49) from 5 to 30 deg of commanded wheel angle, which is
what makes it a scale rather than large-angle saturation, and it is a property of
these tyres on this surface. It is solved from an open-loop replay of the real
command stream through `AckermannKinematics`. The turn-radius floor is speed-dependent:
`min_turn_radius_m = 0.29` is the curve's value at one speed, and the floor is
`min(cap, intercept + slope * |v|)` with `intercept 0.053`, `slope 1.86` and
`cap 0.35` (a safety bound, not a measurement; the Open range is extrapolation).

The measured sensor error budget (`sensor_start_pos_error_m = 0.05`,
`sensor_yaw_bias_rad = 0.03`, `sensor_imu_drift_rad_per_s = 0.000145`,
`sensor_gyro_scale_error = 0.005`, `sensor_imu_noise_rad = 0.005`) is on by
default. The camera gets the hardware's detection rate and latency
(`vision_latency_s = 0.85`, `vision_frame_miss_rate = 0.79`, emulated 11.0 percent
of ticks against hardware's 11.6 percent); `vision_color_flip_rate = 0.051` now
ships the measured marginal colour-flip rate (111 of 2,162 detections, 5.1
percent, 2026-09-15), applied i.i.d. even though the
real errors are concentrated per pillar; it was 0.0 while unmeasured when this ADR
was written, and the magenta-barrier failure still cannot be emitted by the
simulator at all. The LIDAR models chassis occlusion (two blind
front-corner bands, `lidar_occlusion_min_deg = 25`, `max 60`, with measured
dropout and self-return) and `lidar_invalid_ray_rate = 0.095`. A blocked
translation slides along the surface (`contact_slides_along_surfaces = true`)
instead of stopping dead; the old model measured 56 times less progress at a 20
deg approach.

## Consequences

- Every simulator number predating the 2026-09-14 fidelity batch is not comparable.
  Commits that break comparability carry `!` in the type (see 0087).
- `vision_through_pinhole = false` means the corpus still does not exercise the
  real bbox decode path, pending an Obstacles re-baseline.
- `scrub_yaw_gain` (code default 0.0) is the suspected home of the remaining escape
  yaw gap; free-space replay cannot generate it.
- `min_turn_radius_cap_m` is not measured, and terminal-surface contact still ends
  the run on the first tick.

## History

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

## Refuted

- The implicit zero-slip `yaw_gain = 1.0`; the perfect pose and start as adequate;
  the constant turn radius as global; a non-sliding contact; the guessed 1 percent
  LIDAR dropout; `min_turn_radius_cap_m` as a measurement; `vision_false_positive_rate`.

## Cross-references

- 0012 and 0013 are superseded; their decisions are carried above.
- 0084 owns the localizer; 0087 owns the test methodology and the `!` convention;
  0078 owns the camera lens; 0080 owns the LIDAR mount.

## Evidence

- Dead reckoning had no turn-radius floor and over-read outward travel by 31x, so
  the guard's model of the chassis pose had almost no relation to reality.
- Do NOT revert the radius fix: `72e7172b` took the sim exit 16/16 to 0/16, which
  looks like a regression and is not, because the sim exit was passing on a wrong
  radius.
- In the pocket the true turn radius is about 0.075 m, not the 0.29 m the chassis
  saturates at during Open-speed driving; the 0.29 m figure is an extrapolation to
  Open speeds and does not describe the bay.
- The escape path is a shared confound: every run emits "Reverse escape refused:
  rear sector measured nothing" because `compute_rear_clearance` fails open, then
  falls through to a full-lock pivot at an 8 mm radius, which is correct geometry
  (`0.095 / tan(85 deg)` from counter-phase 4WS, `L_eff = wheelbase/2`).
- A control tick can act on a scan one period old, up to 1.6 cm of travel at full
  speed at a 20 Hz loop.
- The steering servo delivers 2.84 N-m (29 kg-cm at 5 V), which is 35 to 70x the
  moment to scrub a wheel in place and 10 to 30x the force to slide the 1.5 kg
  chassis sideways, in a pocket whose entire margin is 6 mm; `scrub_yaw_gain` is
  only the suspected gap.
- The all-or-nothing contact refusal left a robot commanding 0.15 m/s with 0.76 m
  clear ahead travelling 0.00 m for the round; narrow-corridor middle-band starts
  failed 23/23, and 6 mm of lateral clearance allowed only 2.3 deg of yaw.
- Effective against model radius: 66.0/41.7 cm at 15 to 30 deg (1.6x), 38.2/22.5
  at 30 to 45 deg (1.7x), 28.9/2.3 at 75 to 90 deg (12.7x); over 33 bags the
  achieved radius was 0.105 m at 0.025 m/s, 0.298 at 0.132 m/s, and 0.43 above
  0.22 m/s.
- Inside the occlusion bands 68.9 percent of rays are non-finite; self-returns
  have p50 0.0207 m and p5-p95 0.0108-0.0280 m; dropout outside the bands is 9.5
  percent; the +-3 sigma self-return envelope is 0.0051-0.0363 m against a real
  0.0047-0.0435 m.
- LIDAR sub-floor (< 0.044 m): 6.9 percent of the whole sweep; within the 25-60
  deg bands 31.1 percent were sub-floor and 99.7 percent of the returns there were
  sub-floor; outside the bands sub-floor was 0.13 percent. 471 of 5,763,600 real
  rays landed on the old 0.045 clip, against zero sub-floor before the model.
- `vision_color_flip_rate` reconciliation: the ADR originally recorded the knob
  shipping at 0.0 because unmeasured; the shipped config now carries 0.051 (111
  of 2,162 detections, 5.1 percent, measured 2026-09-15) and the emulator applies
  it i.i.d. per observation. The measured errors are concentrated (most pillars
  near 0 percent, one at 47 percent), so the marginal rate is honest but its
  structure is not; detections matching no pillar within 0.35 m are excluded, so
  the magenta barrier remains unscreenable.
- The 2026-09-15 hardware-vs-sim audit found the vision frame-miss rate 2 to 4x
  PESSIMISTIC in sim and the terminal-push rule pessimistic too, while the
  occlusion band, escape rotation, creep gain, start pose and detection range were
  all optimistic. A k_turn burst can exceed 180 deg in one episode (measured 275),
  so escape yaw must be unwrapped over the whole run before differencing.
- The occlusion band sat on the wrong side; correcting it moved the corpus 28 to
  22, which is a new baseline rather than a fix.
- `vision_frame_miss_rate = 0.79` is 2 to 4x pessimistic against the 21 to 50
  percent of nav ticks that carry a fresh detection.
- The simulator rotates about half as much per escape (`allowed_step` scales but
  does not induce yaw) while the real chassis reverses as if on a 0.21 m radius.
- Displacing a pillar is terminal in sim; on hardware one round scored three laps
  with five pillars pushed 8 to 19 cm.
- The three-change set (16 fixtures x 6 seeds x 2 arms, blind) moved before
  56/56/10/7/20 (in_time/laps3/collided/stuck/timed) to 59/70/19/0/0: 27 runs that
  previously failed to finish now complete. The turn-radius floor alone raised
  collisions 5 to 8 on a smaller set, so roughly half the collision rise is the
  chassis losing an impossible dodge.
- The real detector's range distribution is p50 0.70 m and p90 1.06 to 1.31 m; the
  emulated camera at `CAMERA_FAR_CLIP` sees roughly ten times further.
- The vision-range A/B (16 fixtures x 6 seeds x 2 arms, blind) ran off
  59/69/21/1 to on 59/71/18/0 (in_time/laps3/collided/pass_side): the cost is
  absent, so it is free rather than a gain, likely because cross-corridor phantom
  detections are the known about 2.7x track duplication.
- The pocket adds 1.3 to 1.7x on top of the free-space radius, and summing |yaw| at
  the raw IMU rate understates the radius by about 19 percent against the same
  windows scored net.
- A pre-shared-table sim-vs-hardware divergence audit failed on exactly the 120 to
  160 / 160 to 180 boundary, because the hardware band table lumped them and the
  sim split them.
- The "1.42x understeer invisible in sim" is stale. Measured 2026-09-16 with the
  same angle-bucketed table on both sides (`diag_bag_sim_fidelity.py --yaw` against
  a ground-truth probe of `AckermannKinematics`), the yaw-gain curve matches bucket
  by bucket: 0.57-0.60 vs 0.57-0.58 at 10-20 deg, 0.38-0.46 vs 0.33-0.39 at 30-45,
  0.21-0.29 vs 0.23-0.26 above 45. `yaw_gain` plus the speed-tracking floor IS the
  understeer model. The one bucket that differs (2-5 deg, hardware 0.75-1.04) is the
  +1.5..+3 road-wheel deg trim, not gain.
- `known_start` belief-offset isolation: blind assumes the canonical South start,
  so a run beginning elsewhere carries a rigid belief offset (p50 1.58 m over the
  corpus) for its whole length; `known_start` seeds from ground truth to isolate
  it.
