# 0085. The speed envelope is absolute m/s with the drivetrain as a clamp

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0031, 0032, 0033

## Context

Speed tiers were once fractions of the top speed, so a motor swap silently
rescaled policy, and an earlier absolute ladder collapsed MEDIUM and FAST onto the
same ceiling. The heading correction used a four-rung ladder whose intermediate
rungs taxed every corner. Clearance thresholds were fixed metres that shrank in
time terms as speed rose. The finish-line slowdown was added because the robot
coasted past the section on hardware.

## Options considered

- (a) Fractions of top speed; a multi-rung heading ladder; fixed clearance metres;
      no finish cap.
- (b) Absolute m/s clamped by the drivetrain; one heading threshold; per-challenge
      ladders; a finish approach cap.

## Decision

(b). Speed tiers are absolute m/s (`min_mps = 0.0499` is the friction floor,
`creep_mps = 0.1014`, `slow_mps = 0.117`, `medium_mps = 0.1326`, `fast_mps =
0.156`). The drivetrain ceiling (`max_speed_mps`) is applied by the accessors as a
CLAMP, not a multiplier, so recalibrating the motor does not move a tier below.

Per-challenge ladders are symmetric overrides resolved from the motor profile:
Open sets `open_slow_mps = 0.26`, `open_medium_mps = 0.38`, `open_fast_mps = 0.50`,
`open_max_mps = 0.55`; Obstacles shares the base ladder. Absent (falls back) and
zero (a tier a `gt>0` constraint rejects) mean different things, and a drivetrain
with no headroom declares neither prefix so both challenges share one ladder.
Obstacles degrades monotonically with speed (in-time 38 of 256 at 0.156, 16 at
0.50, 9 at 0.60), so it keeps the base ladder; Open tolerates more but is not
clock-bound.

Heading correction is a single binary threshold, `crawl = 1.0` rad (about 57 deg):
at or above it the speed floors to `creep_mps`, otherwise `fast_mps`. The
intermediate rungs were deleted because the graduated version cost 33 percent of
lap time (CW 134.9 to 179.3 s, CCW 161.7 to 200.9 s, both past the limit), since
ordinary cornering sits at 23 to 45 deg.

Clearance thresholds stay in metres for now (`contact_dist 0.10`, `slow_dist 0.25`,
`medium_dist 0.50`, `fast_dist 1.00`); the proposed time-to-collision refactor is
not implemented, because shipping the literals leaves the same latent bug for the
next speed change and the same ladder feeds the Obstacles risk and deformation
caps. Raising `fast_mps` shrinks reaction runway with no compensation (0.35 to
0.42 cuts `medium_dist` runway 1.43 to 1.19 s).

`finish_approach_m = 0.40` caps the last-lap approach to `slow_mps`, cutting the
coast from about 0.6 m (measured on bags) to about 0.09 m, inside the 0.50 m the
finish straight offers. The simulator cannot demonstrate the benefit; it is a
hardware observation.

## Consequences

- A motor swap is a calibration change, never a policy edit.
- One heading threshold, tuned to its narrow optimum near 17 deg, does the work
  only at speed.
- Clearance thresholds are still metres; the TTC refactor is owed.
- A tier above its challenge cap is rejected at load as silently inert.

## History

- 17ddf3fd, a5245e77 2026-08-09: tiers become fractions of the real ceiling, then
  restore the top tiers; the graduated ladder cost 33 percent lap time.
- 990e9d62 2026-08-09: corners run at the ceiling; the heading ladder keeps only
  its crawl cutoff (ADR 0032).
- a33a50ad 2026-08-21: express speed and steering tuning in physical units.
- 8ef8ec97 and bbd6803f 2026-08-28/30: lower then raise the ladder for the new
  motor (0.20/0.28/0.35 to 0.22/0.32/0.42).
- df342945 (twin 0956c33e) 2026-08-30: per-challenge speed ladders (ADR 0033).
- ee3ccfcb and a98bcbc4 2026-08-31/09-01: Open ladder to 0.26/0.38/0.50 cap 0.55;
  640-case 638 to 639, zero collisions either arm.
- 654e57e6 2026-09-02: `max_speed_mps = 0.58` measured, not the feel-based 1.0.
- 703403ab 2026-09-04: `finish_approach_m = 0.40`.
- 9260f162 2026-09-09: `crawl_ramp_start` added, ships 0.0.
- 42926df1 2026-09-09: min target radius filter measured and shipped off (see 0052).
- 2026-09-10: `crawl_ramp_start` written out at its shipped value; until then it
  was a Python literal unreachable from the schema, and pytest's shipped-tree
  completeness check now requires every concrete default to appear in the file.

## Refuted

- The four-rung heading ladder (33 percent lap time); fractions of top speed; the
  feel-based `max_speed_mps = 1.0`; the "~0.45 m/s ceiling"; raising `max_duty` to
  go faster; the stale Open lap-time justification (Open is not clock-bound); the
  x1.6 clearance literals; `crawl_ramp_start` on; `MIN_TARGET_RADIUS_M` nonzero.

## Cross-references

- 0031, 0032 and 0033 are superseded; their decisions are carried above.
- 0052 owns the pursuit lookahead and target radius; 0070 owns the per-challenge
  overlays and the profile mechanism.

## Evidence

- The noise floor is +/-4 cases from genuine chaotic sensitivity at scenario
  boundaries; any difference of 4 cases or fewer is not evidence, and only three
  results in the whole study clear it.
- `speed 0.6` was inert: `navigator.py:922` clamps the selected tier to
  `speed.max_mps()` and the sweep set only `fast_mps`, so every arm above
  `max_mps` was byte-identical to `max_mps`.
- `localization.max_speed_mps` is an implausible-jump guard shipped at 0.25, sized
  0.156 x 1.6 in `9e91981f` and never raised when the profile went to 1.0; above
  about 0.25 m/s it defers scan-match corrections, so any speed arm must raise it
  in step or measure the guard.
- `steer_kp` is dead config: `compute_steering` no longer consumes it, so sweeping
  it is a silent no-op, and it still ships.
- Braking distance is not the issue: `max_accel_mps2 = 2.0`, so stopping from
  0.60 m/s takes 0.09 m; the distance thresholds buy STEERING runway, not braking
  runway.
- 97.8 percent of the ticks that reach the creep floor arrive through the heading
  error term rather than the contact zone: the floor this term drops to is
  separable from the contact zone's speed, and the contact jobs that share the
  constant fail as collisions rather than as slow laps.
- The cruise speed captured BEFORE the heading limiter, the envelope clamp and the
  risk cap is required for attribution: reporting the post-min value made
  `final <= heading_speed` true by construction, so the heading limiter looked
  innocent on 100 percent of ticks while it was the binding constraint on most of
  them.
- `min_mps`: the friction floor was measured no-load and in a straight line; the
  true floor under full steering lock is higher (tyre scrub) and has never been
  measured.
- `creep_mps`: raised 2026-08-09 from 0.050, which was exactly the stiction floor,
  commanded precisely when steering is near full lock and tyre scrub worst;
  hardware spent 16 percent (CW) / 21 percent (CCW) of driving ticks there, all
  from the heading limiter. The servo slews at 2.0 rad/s, so full lock from centre
  takes 0.61 s covering 6.2 cm against 3.1 cm, versus a 0.103 m lateral margin;
  that budget is in METRES, so this tier must not be re-scaled by a motor change.
  Legacy fractional names: `min_frac = 0.32`, `max_frac = 1.0`, `creep_frac =
  0.65`, `medium_frac = 0.85`.
- `finish_approach_m`: scoring rule 1.3 pays 3 points for stopped in the finish
  section; the section straight is 1 m and the line sits at its along-track
  centre, leaving 0.50 m past the line. The drivetrain decays with
  `speed_response_tau_s = 0.35`, so 0.40 m is roughly triple the ~0.13 m needed to
  shed fast to slow. 2026-09-01 bags showed ~0.6 m between the start square and
  the resting place, landing past the boundary.
- The blind corridor-follow phase, before travel direction settles, actually ran
  at 0.150 m/s; the medium tier (0.1326) is its closest shipped match.
- `angle_error_rad` read p50 65 deg on the 2026-09-08 hardware rounds with the
  crawl cut binding 44 to 64 percent of an Open round, far above the 23 to 45 deg
  ordinary cornering the threshold assumes; the field is recomputable from its own
  inputs (pose plus `steer_target_x/y`) and near-target geometry inflates the
  bearing, so a large bearing next to a small command means the field and
  `pure_pursuit_steer` disagree about the geometry.
- The heading crawl is the majority state at 44 to 63 percent of Open ticks, so a
  run's own median speed IS the crawled value and a slowdown cannot be detected
  against it; two inspected runs crawled for 96 s and 45 s.
- `current_corridor` flapped 37 to 39 times against 12 real corners on a 3-lap
  round, so the transition count cannot classify corners.
