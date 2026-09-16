# 0053. Direction is inferred from the corridor span and the start pose from the scan

- Status: accepted
- Date: 2026-09-15

## Context

A blind round has to answer two questions the rules do not: which end of the
corridor it was placed at, and which way round the loop it must travel. Both were
once assumed, and both assumptions are the one value that can make the robot
score zero while everything looks healthy.

The assumed start pose sat at 1.5 m along the corridor, exactly on the boundary
between the two half-metre starting cells, which is the one legal along-corridor
position the robot can never occupy. On 2026-08-05 a round set down near the far
end planned about 1.5 m of runway where the true clearance was 0.69 m and drove
into the wall in six seconds.

Direction was inferred by comparing the two side ranges directly, which measures
which wall is nearer, not which side is open: drifted toward the inner block a
robot reads 0.27 m to the block on its left and 0.72 m to the outer wall on its
right, and "the larger side is open" picks the outer wall, exactly wrong. It cost
two fixtures a confident wrong direction inside six seconds.

A LIDAR dropout is substituted with max range (12 m), which reads as "this side
is wide open", the precise signal the estimator hunts for. Scans arrive at 10 Hz
against a 20 Hz loop, so one bad sweep can supply a whole vote block.

## Options considered

- (a) Assume the start pose and the direction.
- (b) Infer direction from the left plus right span with voted, gated side-ray
      observations, and measure the start pose from the four cardinal rays.

## Decision

(b). Direction inference uses four gates on a sweep: the heading must align with
the nearest track axis within `alignment_tolerance_rad = 25 deg`
(`0.4363323129985824`), no side ray may exceed `max_in_track_range_m = 4.5`
(a dropout would read as open), `left + right` must not exceed
`plausible_span_threshold_m = 1.25` (the widest legal corridor is 1.0 m and the
LIDAR sits at chassis centre, so the two side rays sum to corridor width wherever
the robot sits), and the left/right difference must exceed
`min_asymmetry_m = 0.20`. `min_votes = 5` coincident observations settle it.
`corner_clearance_m = 1.00` opens the window in which the robot can read which
side is open, deliberately larger than `turn_clearance_m = 0.60` with the
ordering enforced across the two files: turning swings the heading past the
alignment gate, so a robot that turns the instant the comparison is decisive
rotates through its only measurement window. `alignment_tolerance_rad` is read by
both `infer_direction` and `measure_corridor_width` so a scan cannot be trusted
for width and rejected for direction.

An in-bay start reads direction straight off the track
(`direction_from_parking_bay`): the lot is always against the outer wall with its
opening toward the inner block and a lap always turns toward the block, so open
side, inner side and turn side coincide. Checked 256/256 corpus scenarios. It
exists because the alternative is a deadlock: in-bay, 8 of 8 rounds never moved.

The start pose is measured from the four cardinal rays (`measure_start_pose`),
with `closing_tolerance_m = 0.15` as the free validity check: opposite rays along
a corridor must span the mat, and good data summed to 2.978 and 2.971 m against a
nominal 3.0, so the honest error is 2 to 3 cm and the 0.15 m default is five
times that while the failure it rejects misses by a metre. A refusal means "do
not race", not "use the old assumption"; `retry_window_s = 8.0` retries a refused
measurement rather than racing on the assumption.

Standing directive: do NOT relax the gates. Relaxing them produces a confident
wrong answer.

## Consequences

- Direction is a reflection of the world, not a rotation, so it must come from
  outside: no width learning recovers a wrong direction. The starting section may
  be assumed because declaring a corridor south only rotates the robot's private
  frame, but the direction may not.
- The start pose is observed, not assumed, and a refused measurement blocks the
  round rather than falling back.
- The estimator's votes are not fully independent (a cached scan is observed on
  consecutive ticks), so `min_votes` does not mean what its docstring says; the
  gates, not the vote count, are the defence.
- `alignment_tolerance_rad` is deliberately not one of the `heading.toml` zones:
  those modulate speed, and retuning speed must not move this gate.

## History

- 3c8262b4 2026-07-26: run with no scenario file. Creates `start_conditions.py`;
  section assumed, direction must come from outside.
- 05e95912 2026-07-26: infer travel direction from LIDAR, opt-in. Off: 28/28, 0
  collisions; forced-wrong correct 23/28. The 224/224 standalone probe was
  measured from centred starts and predicted none of the closed-loop failures.
- a02ee9c4 2026-07-26: give the blind follower somewhere to go at a corner.
- 662a15ce 2026-07-26: decide direction on the corridor span, not the nearer wall.
  Inference 21/28 to 26/28; two fixtures confident wrong inside six seconds
  (go_open_0023).
- 0e1ce5b0 2026-08-01: stop discarding the corner window that already decided the
  direction. `min_asymmetry_m` 0.30 to 0.20; 28/28 correct; go_open_0000 settle
  step 696 to 134, 196 s to 146.0 s.
- fda64a0e 2026-08-01: reject LIDAR dropouts. `max_in_track_range_m = 4.5`;
  direction 24/28 to 27/28, zero wrong. go_open_0010 took all five votes from one
  scan.
- 626a0112 2026-08-06: measure the start pose off the scan. Creates
  `start_measurement.py`; rounds within 3.6 to 4.8 cm where the assumption was
  out 0.35 to 0.80 m; the 2026-08-05 round had 0.69 m ahead against about 1.5 m.
- ca69e71e 2026-08-07: make an undetermined direction distinct from a told one.
  run_141814 drove 2.11 laps CW in 177 s and scored 0 (0 of 1763 scans passed all
  four gates); the silent `cw` default was right twice wrong twice, and the two
  right scored zero.
- 9c0097b9 2026-08-08: single-source the alignment gate; damp corridor centring.
  Adds `alignment_tolerance_rad = 25 deg`, `plausible_span_threshold_m`. Raising
  the centring gain cannot fix the oscillation and makes it worse.
- 0908d579 2026-08-08: move `ray_half_width_deg`, `closing_tolerance_m`,
  `min_votes` into TOML.
- 88312eaf 2026-08-09: retry a refused start-pose measurement instead of racing
  on the assumption. 2 of 4 rounds on 2026-08-08 failed; the blocking ray cleared
  0.6 s and 1.5 s after commit.
- 5bd18568 2026-08-29: break the in-bay start deadlock. `direction_from_parking_bay`;
  in-bay 8/8 never moved, dist 0.00 m; checked 256/256.
- 3378ab7c 2026-09-05: default Obstacles to an in-bay start (`ASSUME_BAY_START`).
- d72c72e1 2026-09-07: the sign router kept the placeholder direction for the
  whole race. `SignRouter._direction` set once in `__init__` was never updated by
  `_commit_direction`; adds `adopt_direction`. 22 of 28 illegal passes mirrored
  the command.

## Cross-references

- 0007 (starting zone spawn alignment) stays separate: it owns the band-edge
  spawn geometry in `track.toml`; this story observes the actual pose and must not
  reintroduce a centred or assumed start.
- 0012 (yaw gain measured) is the model scale, not the absolute heading
  correction; 0054 owns the latter.

## Evidence

- The navigator never runs until direction settles (no path following, no
  `LapDetector`, no waypoint advancement), and inference has no timeout or
  fallback, so an unsettled estimator costs the whole round (case 300: 1 of 640, a
  free-space creep deadlock where the escape code is unreachable).
- `corner_clearance_m` is validator-only and read by nothing at runtime, so the
  documented ordering is aspirational. The narrow-corridor settle failure had zero
  window because `turn_clearance_m` (0.60) numerically coincided with `NARROW`
  (0.60), fixed by `narrow_turn_clearance_m = 0.40`.
- Refuted: opening the gates yields a confident wrong answer. `max_range` 12.5
  settles CCW at 19.8 s and `align_tol` doubled settles CCW at 6.6 s against a
  clockwise truth, with 272 of 360 side readings implying CCW (noise).
- Refuted: raising the corridor-follower gain or cap. At gain 2.0, 12 of 28
  fixtures lost their direction and 9 went into a wall; authority was never the
  limit, damping was.
- A CW round planned as CCW came from `model_copy(update={"direction": str(...)})`
  not validating, leaving the field a plain `str` so identity tests read False;
  invisible on CCW.
- `corner_clearance_m` and `turn_clearance_m` measured together at 0.75 m: three
  fixtures never settled at all and two settled wrong after 20+ s of wandering,
  which is why the shipped pair is 1.00 against 0.60.
- `assume_bay_start` is the in-bay start (the 7-point start); `false` is the
  recognition-only control arm, not a competing start policy.
- Without the parallel-start guard a parallel-start run went 22.40 m -> 3.42 m.
- The canonical start seed is (1.500, 0.400); the localizer landed at (1.781,
  0.494) against a true start of (1.775, 0.500), so using the seed cost 29 cm.
  Over the 16 Obstacles fixtures, discovered signs sat a median 0.200 m from
  world truth with 107/190 beyond the 10 cm tolerance, and a median 0.010 m with
  0/190 beyond it once mapped through the believed frame.
- All three real starts on 2026-08-06 landed in the NEIGHBOURING corridor
  ((2.099, 0.484) and (2.101, 0.487) as EAST, (0.656, 0.596) as WEST, none as
  SOUTH); run 180154 crossed the line four times, every crossing labelled east,
  and scored 0 laps while a replay against the assumed origin counts 4.
