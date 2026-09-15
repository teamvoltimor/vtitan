# 0031. Clearance thresholds are distance, not time

- Status: superseded by 0085
- Superseded by: 0085
- Date: 2026-08-28

## Context

There are seven independent speed caps, composed by `min()` with no
arbitration. The one that caps speed in practice is the clearance ladder:

```
contact 0.10 . slow 0.25 . medium 0.50 . fast 1.00 m
```

These do not scale with speed. Converted to reaction time, `medium_dist = 0.50 m`
is 3.2 s at the 0.156 m/s ceiling they were tuned against, 1.0 s at the shipped
0.50, 0.83 s at 0.60, and 0.63 s at 0.80. Every speed increase silently cuts the
runway the thresholds were designed to provide.

That predicts the 0.80 collapse directly: plain 53/128 against the 0.50
baseline's 114/128. Scaling the ladder with the ceiling (x1.6 at 0.80: contact
0.16 / slow 0.40 / medium 0.80 / fast 1.60) recovers almost the entire collapse:
53 to 111, and with `CRAWL 0.3` to 117, which beats the 0.50 baseline while
running a 60 percent higher ceiling. At 0.60 the thresholds are only about 17
percent off, so scaling adds nothing beyond `CRAWL` alone; the dose-response is
the strongest evidence the mechanism is dimensional rather than incidental.

Braking distance is not the issue: `max_accel_mps2 = 2.0` stops from 0.60 m/s
in 0.09 m. What the thresholds buy is steering runway, not braking runway.

## Options considered

- (a) Ship the x1.6 literals and leave the ladder distance-based.
- (b) Derive the thresholds from current speed (a time-to-collision budget) so
      the envelope travels with the speed profile.

## Decision

(b), not yet implemented. The refactor is the decision because shipping the
literals would leave the same latent bug for the next speed change, and the same
distance-based ladder feeds the `risk` cap and the sign-deformation cap on the
Obstacles path.

## Consequences

- Until the refactor lands, raising `fast_mps` shrinks reaction time with no
  compensating change (raising it 0.35 to 0.42 cuts `medium_dist`'s runway from
  1.43 s to 1.19 s).
- The clearance-ladder result is Open-only; the correction has not been tested
  on Obstacles, where the ladder stacks with the risk and deformation caps.
- The +/-4-case noise floor and the Obstacles "slower is better" data are in
  ADR 0085.

## Superseded by 0085

This decision was replaced by [0085](0085-speed-envelope.md). Its content is reproduced below so this file stays self-contained; edit only the successor.

### Context

Speed tiers were once fractions of the top speed, so a motor swap silently
rescaled policy, and an earlier absolute ladder collapsed MEDIUM and FAST onto the
same ceiling. The heading correction used a four-rung ladder whose intermediate
rungs taxed every corner. Clearance thresholds were fixed metres that shrank in
time terms as speed rose. The finish-line slowdown was added because the robot
coasted past the section on hardware.

### Options considered

- (a) Fractions of top speed; a multi-rung heading ladder; fixed clearance metres;
      no finish cap.
- (b) Absolute m/s clamped by the drivetrain; one heading threshold; per-challenge
      ladders; a finish approach cap.

### Decision

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

### Consequences

- A motor swap is a calibration change, never a policy edit.
- One heading threshold, tuned to its narrow optimum near 17 deg, does the work
  only at speed.
- Clearance thresholds are still metres; the TTC refactor is owed.
- A tier above its challenge cap is rejected at load as silently inert.

### History

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

### Refuted

- The four-rung heading ladder (33 percent lap time); fractions of top speed; the
  feel-based `max_speed_mps = 1.0`; the "~0.45 m/s ceiling"; raising `max_duty` to
  go faster; the stale Open lap-time justification (Open is not clock-bound); the
  x1.6 clearance literals; `crawl_ramp_start` on; `MIN_TARGET_RADIUS_M` nonzero.

### Cross-references

- 0031, 0032 and 0033 are superseded; their decisions are carried above.
- 0052 owns the pursuit lookahead and target radius; 0070 owns the per-challenge
  overlays and the profile mechanism.

