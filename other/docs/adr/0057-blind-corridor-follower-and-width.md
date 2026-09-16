# 0057. The blind corridor follower does not centre, and width is classified not measured

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0026, 0028

## Context

The blind creep exists so the direction estimator can settle. Centring is what
stops it settling: a position error and a heading error are 90 degrees out of
phase in a steered chassis, so a proportional-on-position correction is an
oscillator. Measured on hardware 2026-08-07: 112 steering sign flips in 177 s, 45
percent of ticks pinned at `max_centering_steer_deg`, heading 30 deg off axis at
the median. Traced, a robot 0.303 m from the outer wall of a 1.0 m corridor swung
from 0 to 32 deg of heading on a 0.2 m correction and crossed the estimator's 25
deg alignment gate at about t=4.0 s.

The old corner turn committed at `turn_clearance_m = 0.60`, which is numerically
identical to `NARROW = 0.60`, so in a narrow corridor the near edge sat exactly at
the commit distance and the settling window was zero. Direction never settled and
the whole round was spent in blind creep with no plan and no lap counting.

Corridor width is one of two known values (0.60 m or 1.0 m), 0.40 m apart against
0.03 m of LIDAR noise, so it is a decision between two values, not a measurement.
Attributing a reading by position is circular: position comes from matching a wall
model built from the widths being estimated, so a wrong belief mis-attributes the
reading that would correct it and the error locks in.

## Options considered

- (a) Centre in the corridor; commit the corner at one clearance; attribute width
      by position.
- (b) Do not centre; give narrow corridors their own turn clearance; attribute by
      heading and classify between the two legal widths.

## Decision

(b). `centering_gain_deg_per_m = 0.0`. Heading error is still damped by
`heading_gain = 0.767945` and clamped by `max_centering_steer_deg = 13.75`. The
corner and back-off branches use `max_corner_steer_deg` (ADR 0049), not the
centring clamp. Sitting off-centre for a metre costs nothing; oscillating costs
the round.

`narrow_turn_clearance_m = 0.40` commits the blind corner earlier once the
corridor classifies narrow, keeping a 0.10 m margin above `min_forward_clearance_m
= 0.30`. `turn_clearance_m = 0.60` must stay strictly below
`direction_estimator.corner_clearance_m`, enforced at load.

Width is classified between NARROW and WIDE at `decision_boundary_m = 0.80`
(better than 4 sigma either way) after `min_samples = 12` heading-attributed
readings; unknown reports NARROW because planning a 1.0 m corridor as 0.6 m stays
inside it while the converse clips the inner block mid-turn. Obstacles is fixed
wide (`fixed = True`) because every Obstacles corridor is 1.0 m by rule, and a
sign-hugging ray must not vote it falsely narrow.

`section_from_heading` attributes each reading by snapping the IMU heading to the
nearest axis; heading comes from the IMU and owes nothing to the map.

`defer_current_corridor_replan = true` holds a width change until the robot has
LEFT the corridor it describes; a width update for the corridor under the chassis
moves the line it is actively tracking (a 0.30 m crosstrack step in one tick).
Width and confirmed-ness are gated as one unit. This is Open-only by construction
(Obstacles builds no gate).

The centre bias is split by corridor class: `wide_center_bias_m = 0.10` (inner),
`narrow_center_bias_m = 0.0` (the true centreline), `unconfirmed_width_inner_bias_m
= 0.05` for a narrow corridor still on the blind prior. The unconfirmed bias does
NOT touch the corner arc, which reads the CONFIRMED bias; that coupling was a
mistake and it wrongly excluded the 0.10 arm.

## Consequences

- The direction gate can settle during creep because the centring loop no longer
  swings the heading past its alignment tolerance.
- A narrow corridor gets the same settling window a wide one has.
- A width belief cannot mis-file its own correcting reading, and cannot move the
  tracked line under the chassis.
- The dead zone tried on the turn comparison is not shipped: even sized to real
  LIDAR noise it ate the same margin the deadlock back-off depends on. Do not
  re-attempt without re-measuring the full Open battery.
- `replan_blend_ticks = 0` is kept configurable but is refuted twice as a lever.

## History

- 55e5a6de 2026-08-03: make `replace_path`'s reseek heading-aware. Adds
  `replan_heading_tie_margin_m = 0.15`. Also records the dead zone tried and
  reverted (real hardware showed a 193 deg rotation where 90 was needed).
- 31a8ff12 2026-08-06: only turn the blind corner when the corridor has ended.
  `turn_arc_half_fov_deg = 15.0`, `turn_open_range_m = 1.00`. run_20260806_162008
  zero laps in 305 s; corner branch 53 percent of the round at 47 percent
  precision against a 45 percent base.
- 8d70bbe3 2026-08-06: the bag diagnostics that isolated the CW blind-creep
  failure; the 15 deg / 1.00 m pair was chosen from that table.
- 9c0097b9 2026-08-08: damp corridor centring; single-source the alignment gate.
  Adds `heading_gain`.
- 990e9d62 2026-08-09: the heading ladder keeps only its crawl cutoff (ADR 0032).
  The intermediate rungs cost 33 percent of lap time: CW 134.9 to 179.3 s, CCW
  161.7 to 200.9 s.
- eed906f5 2026-08-15: commit the blind corner turn earlier in narrow corridors.
  `narrow_turn_clearance_m = 0.40`. NARROW South/CW 0/3 to 2/3 laps.
- 5c1e8379 2026-08-21: promote `min_reverse_clearance_m` and
  `min_forward_clearance_m` from `RobotSpecs.LENGTH` aliases to real fields.
- cdf9497f 2026-08-22: stop the blind creep centring itself out of a direction
  fix. `centering_gain_deg_per_m = 0.0`; 128 blind scenarios 11 failures to 0;
  creep 6.7 to 3.4 s.
- 38d38724 2026-08-29: split centreline bias by corridor width.
  run_20260829_020308 three hardware laps: narrow median 0.113 m inner, p05 0.069
  m, outer 0.435 m. Narrow straight clearance 0.103 to 0.203 m.
- 90bdfcfa 2026-08-29: give narrow and wide corridors their own bias side (no-op,
  both inner).
- d0352e5e 2026-08-30: give the blind creep's corner turn its own steering angle.
  `max_corner_steer_deg = 21.25` split from `max_centering_steer_deg`.
- 50c83322 and ed088467 2026-08-30: port `corridor_estimator` and
  `corridor_follower` to Go; pins centring at zero, unreadable rear is not
  permission to reverse, the dead zone stays out.
- 2a0e9e28 2026-08-31: pre-position the blind width prior.
  `unconfirmed_width_inner_bias_m = 0.05`; 640-case 618 to 631 ok, collisions 1 to
  0, sim time -12.80 s mean.
- dd0cb0c6 2026-08-31: defer a width replan until the robot has left the
  corridor. 640-case 634 to 638 ok, failures 6 to 2, no collisions either arm,
  -4.74 s mean; closes cluster {38, 64, 71, 207, 264, 368, 407, 426}.
- ca8ac1aa 2026-09-04: write the width-belief flags into `waypoints.toml` with a
  text-asserting test so a deleted key fails locally.
- a84e5b2f 2026-09-05: let the blind path see the shipped config. Blind Open
  laps>=3 56 to 112, collisions 20 to 0, stuck 52 to 16.

## Cross-references

- 0026 and 0028 are superseded; their decisions are carried above.
- 0032 (heading single crawl threshold) stays separate but is the speed cut this
  follower's heading term feeds.
- 0049 owns the corner arc and the steering cap; 0051 owns the sign lane and
  inherits the planned centreline this story biases.

## Evidence

- The blind corridor-width prior must start NARROW and only widen on confirmation;
  never start wide and narrow later.
- On wide corridors bias the path toward the INNER wall, not the outer (the outer
  is what gets hit); fix the anchoring and keep the narrow prior.
- `replan_heading_tie_margin_m = 0.15` re-ranks near-tied waypoints by heading
  agreement; picking purely by position handed pursuit a 193 deg correction where
  the corner needed 90. `narrow_width_threshold_m = 0.8` is the midpoint between
  the two rule widths, a threshold not an interpolation. `first_lap_corner_caution`
  is on; `corner_caution_all_laps` and `first_lap_corner_caution_narrow_only` off.
- The corner-end second opinion (`turn_arc_half_fov_deg = 15`,
  `turn_open_range_m = 1.00`) asks the complementary question as a MAXIMUM over a
  wider arc, because `turn_clearance_m` alone reads a MINIMUM over the +/-8 deg
  cone and cannot separate an ended corridor from an oblique chassis
  (run_20260806_162008 fired the corner branch 53 percent of a round at 47 percent
  precision against a 45 percent base). `corner_leak_margin_m = 0.35` is added to
  WIDE for the side that has leaked past the inner block; `corner_speed_scale` and
  `reverse_speed_scale` are 0.6.
- `replace_path`'s backward-step guard: measured on hardware 2026-09-08 as an
  19 -> 16 index step in `normal_drive`, corridor north, lap 0, in BOTH 3-lap runs
  (run_20260908_001541 t=12.81 s, run_20260908_003041 t=10.85 s) while the yaw was
  swinging through the corner, the same index pair twice. Against the measured
  0.29 m minimum turn radius, the target-behind full-lock branch is a U-turn
  attempt inside a corridor.
- `validate_path_feasibility`'s prior form added `arc_radius` to a lateral
  half-extent, which are not commensurable (a path curvature against a width),
  and ignored `center_bias_m`, so it scored a centred path and a biased one
  identically while the bias was what actually spent the margin.
