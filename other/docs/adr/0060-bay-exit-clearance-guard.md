# 0060. The bay exit is bounded by predicted fin clearance, not by contact

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0038

## Context

Touching the parking-lot fins ends the round (rule 9.24.7). The old exit ended
each leg on the stall backstop, which fires because a fin STOPPED the chassis, so
the leg-end signal was itself the violation. The manoeuvre was also a pendulum:
in the pre-guard exit the forward and reverse legs held the same steering sign,
so consecutive legs cancelled, giving 0 flips in 111 and 140 legs, 815 to 1070 deg
of yaw spent for 2 to 7 deg kept, and 0 of 2 exits.

Two more hardware facts shaped the fix. The pocket wall sits inside the LIDAR's
minimum range, so forward clearance reports nothing exactly when the obstacle is
most present; rotation is the only signal that works there. And a commanded
0.10 m/s sits inside the motor deadband: encoder-zero on 56.1 percent of bay
ticks with delivered p50 zero.

## Options considered

- (a) End each leg on stall or contact, and keep holding the forward lock on the
      reverse leg.
- (b) Bound each leg by PREDICTED fin clearance, mirror the reverse-leg steering,
      end the exit on accumulated rotation, and set the exit speed above the motor
      deadband.

## Decision

(b). `bay_exit_clearance_guard = true` bounds each leg by predicted clearance from
dead reckoning; a leg that IMPROVES an already-violated gap may run
(`bay_exit_guard_overlap_recovery`), because `run_20260906_192358` stood still for
14.2 s of a 16.6 s exit at a frozen 44 mm overlap. It supersedes both older exits:
with the guard on, `bay_exit_cycle` and the reverse-then-swing exit are
unreachable.

`bay_exit_guard_mirrors_reverse = true` mirrors the reverse-leg steering so the
arcs curve opposite ways and yaw accumulates instead of cancelling. With it: 0
held / 4 flipped, about 75 deg of rotation for 71 net (95 percent efficient), 3/3
out in 2.8, 13.4 and 21.1 s.

`bay_exit_target_yaw_deg = 70` ends the exit on accumulated rotation; `bay_exit_latch_direction
= true` decides the open side once, by a `bay_exit_open_side_sector_deg = 15`
sector with `bay_exit_open_side_votes = 5`, because once the chassis rotates the
+/-90 deg comparison is noise and a flip turns the escape into a re-entry. Single
ray scored 70.6 to 72.9 percent of 383 hardware bay scans; the sector scores 100
percent.

`bay_exit_speed_mps = 0.15`, raised from 0.10 on 2026-09-10: at 0.15 the wheel
stops stalling outright (0.4 percent against 56.1 percent) and the bay phase went
62.2 s to 24.1 s. `bay_exit_arc_steer_norm = 1.0` is a cliff (0.3 to 0.9 all give
0/8 out) and `bay_exit_speed_scale = 0.35` leaves 9.0 mm of fin margin. The coast
budget is taken from MEASURED speed, not commanded (`bay_exit_guard_measured_coast`),
which took the exit to 10.4 s with one reversal and 96 percent efficiency.

`solid-walls` is not an admissible evaluation model: it lets the chassis grind
along a fin, and any manoeuvre that needs sustained wall contact is modelling an
infraction. The exit must be evaluated against the default contact model.

## Consequences

- A leg can no longer end by touching a fin; the 9.24.7 violation is removed by
  construction.
- Yaw accumulates, so the exit leaves instead of shuffling in place.
- `bay_exit_clearance_tolerance_m = 0.0` (inert; tolerating overlap risks fin
  contact), `bay_exit_contact_recovery_ticks = 0` (ships disabled; the recovery
  reverses blind into a fin) and `bay_exit_dr_uses_measured_yaw = false`
  (refutation recorded, inert) stay off.
- When a banner voids a body of work, re-examine the flags that body of work
  turned off: the pre-guard mirror refutation was itself voided and mirroring was
  then shipped.
- Not hardware-validated in full; the stopping-distance bench is still unrun.

## History

- cf1d65be 2026-09-03: bound the bay exit by predicted clearance, not by contact.
  Guard default OFF; guard0 min predicted gap -0.0075, guard1 -0.0036.
- 768c7d0b 2026-09-03: an exit that never touches the lot, TOUCHED 0/16. Guard0
  16/16 TOUCHED and 16 collided; guard1 +0.0189, 0/16.
- b557633e 2026-09-03: the leg bounds were dead on the first leg of every round
  (falsy-zero `or`).
- 52a108f4 2026-09-05: ratchet out against the wall legally. 210/256 out, TOUCHED
  0/254.
- d723bbc5 2026-09-05: switch the exit onto the wall ratchet by default. Arc 1.0:
  3/4 in 127 ticks against 0/4 at 0.3/0.6; speed 0.35: 7/8 with 9.0 mm clearance.
- 37198e18 2026-09-06: give the exit its own absolute speed. run_20260906_181613:
  commanded 0.067 on 876 of 882 ticks, encoder 0 deg/s on 92 to 97 percent.
- ef78a913 2026-09-06: end the exit on rotation, the one thing the pocket can
  measure. Adds `bay_exit_target_yaw_deg`, `bay_exit_contact_dist_m`,
  `max_frames`.
- 8e8fe374 2026-09-06: leave only when turned out AND the way out is open.
- 5e848788 2026-09-06: the exit latched the open side from a dropout, then jammed
  on its own guard. Sector 15 deg, votes 5, margin 0.005 to 0.001.
- 413d3f51 2026-09-09: bound the guarded leg in TIME, because a stalled leg cannot
  end itself. Adds `bay_exit_leg_max_s`; median distance 0.66 to 0.97 m.
- ec649f98 2026-09-10: ship the mirrored bay reverse, the ratchet was a pendulum.
  Held 111 / flipped 0 to held 0 / flipped 4; net 0.000 m 0/2 to 11 to 20 m 3/3.
- 7d6c4865 2026-09-10: raise `bay_exit_speed_mps` out of the motor deadband, 0.10
  to 0.15. Encoder-zero 56.1 to 0.4 percent; bay 62.2 s to 24.1 s.
- 9a578a86 2026-09-10: ship the measured-speed coast budget; bay leaves in 10.4 s,
  one reversal, efficiency 75 to 96 percent.
- 9e80162c 2026-09-10: `BAY_EXIT_DR_USES_MEASURED_YAW` inert; both mirror verdicts
  voided.
- 3d3ef5e9 2026-09-10: budget the coast from MEASURED speed. run_20260910_210503:
  0.10 stall 56.1 percent, coast 35.0 mm real about 0; 0.15 delivered p50 0.027.
- e5677704 2026-09-10: let the guard tolerate predicted overlap, because it
  arbitrates below its own error (predicted 4 mm against a 1 mm margin, pose about
  29 mm wrong). Tolerance ships 0.0.
- 5a430568 2026-09-10: port the exit fixes to Go and wire the missing TOML loader.
- 73693f53 2026-09-11: the servo is at least twice as fast as assumed, 2.4 rad/s
  measured; bay exit 30.5 s to 5.3 s.

## Cross-references

- 0038 is superseded; its mirror decision is carried above.
- 0037 (lot from in-bay start) stays separate and is carried in 0062.
- 0030 (servo slew rate) is the measured rate the exit rides on.

## Evidence

- `MAX_STEERING_RATE` had never been measured: 1.2 rad/s matched an unmeasured
  TOML and doubles as both the software command limiter and the sim's physical
  slew. The loaded bench datum is 150 deg in 0.90 s = 2.91 rad/s; 2.4 ships as the
  conservative end.
- `servo_slew_rate_rad_s` was split out (`0fecf09a`) precisely so the bay could be
  fixed without re-heating cornering; `MAX_STEERING_RATE` stays 1.2 as cornering
  policy (lowered 2.0 to 1.2 on 2026-08-28, still binding 8.2 to 13.1 percent of
  driving ticks).
- Re-measuring slew needs only `diag_servo_slew.py`: park at one lock, command the
  other, hold, ask if the wheel REACHED the far stop, bisect. Above the boundary
  the wheel visibly DWELLS, and reading `Publisher count: 0` once is not a check.
- Before the direction latch the reverse-gate chatter WAS the escape (about 300
  reverse ticks against 290 forward, progress pinned at 0.045 to 0.054 m, 187/256
  out); latching the reverse scored 0/64, so "chatter is a bug destroying the
  escape" is backwards.
- Width is not the lever: 15 cm of along-wall travel needs theta >= 42 deg, at
  which the length term alone is 0.201 m, the entire pocket depth. The binding
  dimension is chassis LENGTH against pocket DEPTH, so narrowing cannot fix it.
- `max_steering_rate` was lowered 2.0 to 1.2 on 2026-08-28 after
  run_20260828_220533 showed the controller saturating at hard corners (steering
  swinging exactly 0.6 rad); past sweeps only tested RAISING it, both of which
  scored worse (107 against 114).
- `joint_states` is recorded because it is the guard's ONLY state input, the
  drive-wheel position the pocket pose is dead-reckoned from (`drive_speed` is a
  smoothed estimate with a different bias). It had to be added to the bags: an
  offline replay of the 14.2 s deadlock from the 2026-09-06 bags could not be
  made faithful, because reconstructions that reproduced the stall destroyed the
  runs that escaped and no single reconstruction exceeded 90 percent agreement on
  all three runs.
- The reverse-then-swing and cycle exits are complementary under the sim contact
  model: each leaves the bay 254/256 where the other leaves 0/256, so a fallback
  between them (`bay_exit_fallback_frames`) is cheap insurance while which one
  matches the real robot is unknown.
- `bay_exit_clearance_margin_m`: at 0.005 m a band of outward positions refuses
  BOTH legs while the modelled pose is still CLEAR, so nothing moves and the
  refusal is permanent; the band opens at `dr_out = 0.0365 m` and the ratchet
  drives through it. Unit fixture: 0.005 stalls 795 of 900 ticks, 0.003 for 786,
  and at 0.001 the longest block is 1. True fin clearance drops 9.0 to 5.6 mm,
  still 16/16 with TOUCHED 0/16.
- `bay_exit_guard_overlap_recovery`: `_predicted_gap` takes the min over BOTH
  fins, so once the modelled body overlaps one, the fin being moved AWAY vetoes
  the leg as hard as the one ahead; the recovery flag is what lets it run.
- `bay_exit_guard_block_ticks`: try 40 (2 s) first, reading TOUCHED alongside.
- `bay_exit_speed_scale` is a cliff: 1.0 never moves (the coast alone exceeds the
  along-wall slack), 0.5 collides, and 0.2 leaves only 3.5 mm of fin margin
  against the shipped 9.0 mm.
- `bay_exit_open_side_sector_deg`: the near-pocket-wall ray drops out on 21-37
  percent of ticks against 0-5 percent for the open-space ray, which made 12 m
  beat 0.84 m and sent `run_20260906_192424` into the wall. Widening to +/-30 deg
  reaches the pocket end walls, so do not widen past about +/-15 deg.
- `bay_exit_open_side_votes`: recording began 1.9-2.6 s after the exit in two of
  three runs, so a first-scan (tick-1) latch rests on a frame no bag can show.
- `bay_exit_max_frames`: nothing else bounds the manoeuvre; `run_20260906_105056`
  held the chassis for 1832 of 1834 ticks (91.7 s, -420 deg of yaw) and was
  stopped by the operator. `CoreNavigator` never steps while the exit owns the
  tick.
- `bay_exit_speed_mps`: the simulator has no deadband, moves at any commanded
  speed, and collides 32/32 at 0.10 m/s over 0.27 m of travel, but its failure
  mode is OVERRUN under an instant-delivery assumption; 0.067 commanded read
  encoder 0 deg/s on 92-97 percent of ticks. Each mirrored reversal swings
  170 deg = 2.97 rad at 1.2 rad/s, so a reversal costs 2.47 s and 9 reversals are
  22.2 s of the 24.1 s bay phase (92 percent is the servo).
- `bay_exit_contact_recovery_ticks`: on `diag_bay_start --corpus --limit 32`,
  disabling it moved ticks 12 to 0, moved-distance median 0.06 to 23.52 m,
  laps>=1 0 to 22, collided 32 to 2, and fins TOUCHED 32/32 to 0/32. Re-enabling
  requires routing the reverse through the same fin-gap prediction
  `_guarded_command` uses.
- `bay_exit_guard_measured_coast`: at a commanded 0.15 the wheel never stalls
  (0.0 percent against 56.1 at 0.10) but delivers 0.027, so the guard budgets
  52.5 mm of coast for a real 9.5 and vetoed 39-71 percent of ticks (306 and 195
  reversals, zero net travel, 0/2 out); inert pending a hardware trial with
  corrected stopping-distance data.
- `bay_exit_clearance_tolerance_m`: the guard refuses on a predicted 4 mm gap
  against a 1 mm margin while its dead-reckoned pose is about 29 mm wrong, giving
  324 legs of 0.06 s in 39.3 s, 814 deg of rotation for 5.3 net and zero travel;
  trial 0.010-0.020 only with a hand on the chassis.
- Low-arc ratchet cost: at arc 0.3 the ratchet shuffled 8.09 m for 0.03 m outward,
  while arc 1.0 escaped in 127 ticks; the unlatched gate flapped about 600 times a
  run, and the first cycle exit backed 6.5 cm onto the rear fin and held there for
  174 ticks.
- Shipping the real turn radius took the exit from 16/16 out (`72e7172b~1`) to 0/16
  (`72e7172b`); the HOLD variant burned 267 forward and 272 reverse legs for 1 cm of
  net progress, and commanding 0 between legs reached only 30.9 deg of the 85 deg
  asked (36 percent of full lock).
- Free-space exchange rate: the pocket grants about 0.032 m of along-wall slack each
  way at a placement of `a <= 1.15 deg`, about 0.7 mm of the 78.6 mm that frees the
  rotation; the guard reach back-solved to 0.0600 m on all six checked refusals with
  residual 0.000000, and a clean raycast gives 0.215 m against a live pipeline
  reading of 0.05-0.13 m, so gated on it the arc got one tick per cycle and turned
  0.1 deg in 57 ticks.
- First-leg distance bound: `BAY_EXIT_FORWARD_M` and `BAY_EXIT_CYCLE_REVERSE_M`
  swept byte-identical at 0.02 and 0.04, and `rev_m` measured 0.041 against the 0.09
  asked.
- Releasing on rotation alone drove back into a marker 0.24-0.30 m every run; the
  stale-plan handover showed the same, and `run_20260906_094342` returned only
  self-detection at 0.050-0.052 m for seconds before dropping out for one tick.
- `BAY_EXIT_MAX_FRAMES` (900, 45 s) fired in none of the 2026-09-06 hardware runs;
  one run stood still 14.2 s of a 16.6 s exit.
- Measured-coast stale baseline: `run_20260911_152714` and `_152819` rolled 12-34 mm
  per commanded-zero settle, reported by the stale baseline as 0.24-0.68 m/s.
- Direction latch: steering hold alone took the exit 0 to 187 of 256, and latching
  the open side took it to 254 of 256; the inverted reverse-steer knob was refuted,
  every non-zero value collapsing to 0.02 m.
- Cycle turn radius (removed from `_cycle_command`, possibly 0013): at 85 deg lock
  the radius is 17 mm and the chassis pivots about itself; at 45 deg it is about
  0.19 m.
- No-budget release: with `is_clear` as the only release the exit held for 600/600
  ticks and `CoreNavigator` never stepped, so no escape was reachable from an in-bay
  start; `run_20260906_112613` stopped without ever reporting clear, then drove
  forward at 0.26 m/s into a wall 0.02 m away.
- Settling guard: without it the in-bay probe travelled 0.33-14.06 m falling to
  0.18 m and every run collided; leaving the bay-start state un-cleared made the
  second and every later race of a session skip the in-bay start entirely.
- Held lock: 87 percent of consecutive legs cancelled, 0 of 2 runs out of the bay.
- `run_20260906_192358`: `_dr_along` reached 0.106 m, 63 percent past the whole
  65 mm of along-wall slack.
- The handover moves 85 mm and spends nearly all of it along the wall.
- The shipped margin frees the ratchet 7.35x the frozen displacement (asserted at
  5 in the test), against an old held-lock shape that measured 0.06 m of travel
  for no net gain.
