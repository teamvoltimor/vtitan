# 0052. Pursuit selects its target by arc length and path sense, and arms the lookahead on the corner ahead

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0029

## Context

Three failures shared one root: the controller looked at the wrong point, or at
the wrong quantity, to decide where to go.

The steering law itself was a P term on the bearing. It was stable only below
about 0.07 m/s and saturated the servo at race speed, and its gain had been tuned
against a front-steer model while the chassis is counter-phase four-wheel-steer
with double the yaw rate. That was replaced by curvature pure pursuit on the
planned path with effective length `wheelbase/2`; `steer_kp` remains accepted but
unread.

On top of that, three target-selection defects:

- The lookahead switched between long and short on a bare `value > threshold`
  step. Hardware `run_20260829_104641` shows 0.320, 0.160, 0.320, 0.160 at about
  2.5 Hz, and because pure-pursuit curvature goes as `2y/L^2`, each flip swings
  commanded curvature 4x.
- Crosstrack error lags a corner: it cannot rise until the turn has already been
  missed. The robot held 0.9 rad (51 deg) of heading error for three seconds
  while commanding 0.23.
- The forward target search could wrap a full lap and return the first waypoint
  merely geometrically in front, which once the chassis has turned is on the far
  side of the ring. The three rounds lost 2026-09-11 selected targets at p50
  2.08 to 2.50 m, 97 to 140 deg backwards, on 60 to 91 percent of ticks.

A fourth defect was diagnosed but its fix was refused. 57 to 59 percent of Open
ticks aim at a circle tighter than the chassis's measured 0.29 m minimum radius,
so the target bearing cannot be steered away and just re-fires the heading speed
cut.

## Options considered

- (a) Keep the step lookahead, arms on crosstrack error, and an unbounded target
      search.
- (b) Ramp the lookahead, arm it from a geometric corner preview held until the
      turn is driven, bound the search by arc length, and gate the candidate on
      path sense.
- (c) Filter candidates below the minimum turn radius.

## Decision

(b). `lookahead_blend_start = 0.70` ramps long to short instead of switching;
1.0 restores the bare step. A ramp rather than hysteresis, because hysteresis
stops the chatter but keeps the 4x jump.

`corner_preview_distance_m = 0.80` arms the short lookahead on the corner ahead.
0.40 gave that preview no lead at all (0.000 m); 0.80 gives 0.257 m (0.73 s) and
is the smallest value that leads. `corner_turn_threshold_rad = 0.35` is the
angle gate; it is insensitive over 0.21 to 0.58 and is not the lever for turn
timing. `CornerLatch` holds the previewed value until the chassis has turned 0.8
of the previewed heading change (not 1.0, to avoid a latch that never releases),
with a one-revolution backstop.

`target_search_span_m = 1.0` bounds the search by arc length walked, with a
5-candidate floor because waypoint spacing is not uniform. A clean 3-lap round
never selected a target beyond 0.91 m in 2533 ticks; the lost rounds ran 27 to
30 percent beyond 1.0 m. Lookahead is 0.16 to 0.32 m, so 1.0 m is 3x anything
the search can legitimately need.

`target_sense_gate = false` (off) rejects a candidate the chassis would only
reach by going round the loop the wrong way, by projecting the pose-to-candidate
bearing on the path's outgoing bearing. It is prevention, not recovery: once the
chassis is fully turned round every candidate is wrong-sense and the gate empties,
so it never rescues a reversal. Ships off and unvalidated on track.

(c) is refuted. Skipping an unreachable candidate takes a farther one, and
curvature divides by squared distance, so the filter weakens the correction:
128 Open cases went 128/128 to 127/128 with one new collision, +26.61 s mean, 0
faster and 127 slower. `min_target_radius_m = 0.0` disables the floor and is
bit-identical to not having it. The real harm is the heading cut reading the
unreachable bearing as tracking error, and that is what the crawl threshold
(ADR 0032) governs.

`wall_margin_safety_m = 0.03` caps the crosstrack threshold at what the path's
own distance to the outer wall affords, not the fixed 0.30 that under the blind
narrow prior could not fire before the wall arrived.

## Consequences

- The target cannot come from the far side of the loop, and crosstrack, the
  lookahead gate and the target search now agree on the same path.
- The corner is armed before it is missed, and the latch keeps it armed through
  the turn.
- `min_target_radius_m` and `target_sense_gate` are measured and off; both are
  kept so the ideas are not re-proposed from the same reasoning.
- Three mid-turn steering-bleed fixes are refuted and left out (raise the steer
  rate 1.2 to 1.8, hold the angle on CornerLatch, or the same slowed to 25
  percent): on a constant-radius arc the correct steering is constant and
  moderate, and holding it oversteers through the corner.

## History

- 9efbbe92 2026-08-06: arm the short lookahead on the corner ahead, not the error
  behind. Adds `path_turn_ahead` and `corner_preview_distance_m = 0.40`,
  `corner_turn_threshold_rad = 0.35`. The robot never commanded more than 0.61 of
  full lock; on wide corridors p90 was 0.21 to 0.27.
- 6742a441 2026-08-20: screen yaw-reduction knobs and sweep the pursuit lookahead.
  Shipped 0.20/0.40 gave 202 collisions and 56 laps>=3; 0.16/0.32 gave 196 and
  64. Non-monotonic below 0.14.
- bbfd700c 2026-08-29: ramp the lookahead instead of switching. Worst change per
  5 mm of crosstrack 0.160 m to 0.015 m; mean peak steer 0.306 to 0.398.
- 97117b8d 2026-08-29: give the corner preview an actual lead. 0.40 gave 0.000 m
  lead (27/38 percent armed); 0.80 gives 0.257 m (38/64); 1.00 gives 0.514 m
  (49/73) but is rejected as a permanently higher gain.
- 9fdfa264 2026-08-30: hold the corner preview open until the turn has been
  driven. Adds `corner_latch.py`. Open 128: without 125 ok / 1 collision, with
  126 ok / 0 collision.
- 7ce081ee 2026-09-03: give the Open Challenge its own `open_lookahead_long = 0.24`.
  Open 640 mean -5.39 s (530 faster, 94 slower); Obstacles 256 worse (clean 21 to
  19, escapes/lap 59.6 to 66.4).
- afe948e3 2026-09-05: compensate the plant's under-turn on Obstacles only,
  shipped off. `obstacles_yaw_gain_compensation = 0.55`; Obstacles wrong-side
  46/301 to 6/311, rule 9.21 9/64 to 0/64, collisions 14 to 15.
- 42926df1 2026-09-09: filter unreachable aim points, measure it, ship it off.
  57 to 59 percent of Open ticks aim tighter than 0.29 m; 128 cases one new
  collision, +26.61 s mean, 0 faster, 127 slower.
- f5e8f705 2026-09-10: write `min_target_radius_m = 0.0` into TOML so operators
  can reach it; `TestShippedTreeIsComplete` guards the tree.
- 80562e60 2026-09-11: bound how far the target search may walk. `target_search_span_m
  = 1.0`; runs 171915, 172334, 172543 lost at p50 2.08 to 2.50 m against a clean
  run at 0.44 m; clean farthest 0.91 m.
- e52bca88 2026-09-12: gate the target search on path sense, retract the deform
  half. Pre-reversal wrong-sense 23.0 percent; the bound dropped selected range
  to 0.43 to 0.47 m but the car still drove 0.23 of a lap backwards. Also
  retracts `sign_deform_sense_guard` as inert and its evidence as a counterfactual.
- 643f8982 2026-09-13: split the corner preview by width class, override unset.
  Uniform wide n=48 -2.04 s; uniform narrow n=32 +3.63 s; the narrow harm is
  bigger than the wide gain, so lowering the shared value is a net loss.
- f3ca48b1 2026-09-15: three refuted attempts at the mid-turn steering bleed,
  recorded beside the knob. 14 to 29 percent of turns bleed; 82 to 88 percent
  release with median 1.19 to 1.39 rad owed.

## Cross-references

- 0029 is superseded: its Open lookahead split is the `open_lookahead_long` entry
  above.
- 0032 (heading single crawl threshold) stays separate; the unreachable-target
  harm routes through it.
- 0027/0049 own the corner arc; 0051 owns the path the pursuit tracks.

## Evidence

- `path_turn_ahead` measures heading change ahead of the current waypoint, so the
  corner preview decays to zero once the chassis is inside the arc, un-arming the
  short lookahead mid-turn.
- `CornerLatch` is live-verified on hardware (3 runs per side): forward-clearance
  p05 rose 0.11 to 0.14 m to 0.23 to 0.24 m, and heading-error p90 fell 0.82 to
  1.22 rad to 0.56 to 0.66 rad.
- `CornerLatch` does not help narrow corridors: the preview peaks at 0.197 rad and
  never reaches the 0.35 arm threshold, left open as a `corner_turn_threshold_rad`
  question.
- Refuted: short-lookahead occupancy as the cause of the steering weave. A
  four-point latch-release sweep over a 21-point occupancy range was flat, and the
  S-paths predate the latch.
- Method rule: never classify corner vs straight with `path_turn_ahead_rad`; it
  looks ahead, peaks on approach and reads about 0 inside a corner (median |turn|
  during corners is 0.000).
- The per-width-class `wide_corner_preview_distance_m` split was screened at 0.57
  (uniform wide n=48 mean -2.04 s, 48 faster / 0 slower; uniform narrow n=32 mean
  +3.63 s, 3 faster / 29 slower), both unanimous and opposite, so the narrow harm
  is bigger than the wide gain; the key was later retired and the shipped tree
  keeps one preview value. The wide corridor receives the tighter arc (0.40
  against narrow's 0.45), so its straight is 0.86 m against 1.29 m.
- Waypoint advance past a passed point. Measured on the 2026-08-06
  counterclockwise round: crosstrack ran 0.26 to 0.51 m against the 0.20 m
  reached-radius, the index froze on the last waypoint at t=90 s, and the robot
  circled the mat for a further 7 minutes with the lap count stuck at 0.
  STALE_TARGET_RESCUE was root-caused on go_obstacles_0042 under the `wideonly`
  (85 deg steering) profile, subset64.
- run_20260831_224600, first corner: the index froze on waypoint 8 at (0.35, 1.00)
  for FOUR SECONDS while the chassis swept past it; pure pursuit turned back for
  it and heading ran 172 -> 151 -> 130 -> 107 -> 66 -> 14 deg, about 160 deg where
  the corner needed 90, with forward clearance collapsing 0.21 -> 0.11 m into the
  wall. The index then jumped 8 -> 11 once the overshoot was unrecoverable. The
  distance test cannot catch this by construction: overshoot a waypoint and both
  it and its successor recede every tick, so `next_closer` never closes.
  STALE_TARGET_RESCUE was measured on Obstacles at 2 to 5 percent and refuted
  there.
- Path-sense gate measured 2026-09-12 inside the known reversal windows, against a
  clean 3-lap control reading 0.7 percent: wrong-sense targets ran 56 to 91 percent
  of ticks before the span bound shipped and 2.8 percent after, while
  target-behind-the-chassis went from 0.0 percent exactly to 35.4 percent. The
  bound converted the failure rather than closing it; see
  `scripts/bag/diag_bag_target_loop_sense.py`.
- Lookahead arming anchors: on hardware crosstrack ran 0.09 -> 0.15 through a
  corner and never crossed 0.30, so the lookahead stayed long and the curvature
  stayed weak; the moment crosstrack reached 0.13 the short lookahead armed and
  steering jumped to 0.52.
- `obstacles_yaw_gain_compensation` paired on 2026-09-05: Obstacles runs with a
  violation 33/64 to 6/64 and laps credited 131 to 196. Open was the downside:
  640-case 638 to 615, rule 9.21 terminations 1 to 15, nine more timeouts, and the
  128-case screen read only 127 to 124, an eightfold understatement of the
  Obstacles cost. `diag_base.load_tuning` sets both yaw fields together because
  this knob CAN shadow the base on an Obstacles sweep, unlike `open_lookahead_long`.
  UNSETTLED ON HARDWARE: the sim is exact because the plant IS `yaw_gain` 0.55; on
  the robot the same 1.8x is unresolved between `rear_steer_ratio` and
  `linkage_ratio` and needs a bench test.
- `open_lookahead_long` Open verdict: the one change was case 590 ok to incomplete,
  and case 300 fails in both arms (the known free-space creep deadlock). Its
  Obstacles cost: 256 timeouts 35 to 40; sign-pass crosstrack IDENTICAL at 10.32 cm
  median, so the Open tracking gain does not reproduce.
- `target_sense_gate`: pre-reversal wrong-sense bursts ran up to 117 ticks against
  a longest of 14 on the clean control.
- `corner_preview_distance_m`: waypoint spacing is 0.117 to 0.258 m, so the preview
  must clear the straight remainder AND reach into the arc before any heading change
  registers.
- In `run_20260830_014612` the first corner (west to south, 6.1 s) ran preview
  1.373 -> 0.980 -> 0.590 -> 0.197 -> 0.000 rad while the lookahead went long two
  seconds before the corner; `run_20260830_013702` previewed 0.197 rad.
- The shortened straight of the Open zigzag chain leaves the short lookahead
  spanning 82 percent of straight ticks, a loop gain of x2.25.
- Inside the known reversal windows target-behind-the-chassis rose from 0.0 to
  35.4 percent as the span bound shipped, but crosstrack stays about 0.142 m p50,
  so no crosstrack guard fires on the converted failure.
- On the reversal rounds the target sits behind the chassis AND ahead along the
  loop; the path's travel sign is read from index-advancing ticks, so an early
  start manoeuvre cannot flip the measurement.
- Steering effectiveness measured as wheel angle in, yaw rate out: a left/right
  split is a steering trim, and it is why CCW drifted about 3x further outward
  than CW. The 2026-08-09 bags first showed the chassis cornering wide with the
  command obeyed.
- `diag_bag_open_oscillation.py` counts per-rung speed occupancy and
  lookahead-ramp occupancy to name the ladder in force; the motor profile
  overrides the 0.24 m Open ladder.
