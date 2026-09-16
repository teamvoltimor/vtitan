# 0055. The escape manoeuvre fails closed on an unread rear, pivots when wedged, and re-anchors at its end

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0023

## Context

The rear of this chassis cannot be trusted from a single range. The LIDAR's rear
sector is mount-occluded, and `compute_rear_clearance` reports the same
`no_data_range_m` (10 m) for "nothing behind me" and "I cannot see behind me", so
a reverse gate reading only the distance clears the manoeuvre precisely when the
robot can see nothing. That is a gate that fails open.

The escape manoeuvre also picked its own success. A K-turn reverses 0.117 m
median, so when the escape sequence was anchored at the latch point the manoeuvre
satisfied its own movement test with its own travel: every escape certified
itself, the count never reached `escalate_after_attempts`, and escalation could
not fire. 59 of 149 K-turns were repeats within 0.40 m and 30 s, and
run_20260914_000926 spent 179 s of a 292 s round in one limit cycle.

When the robot was wedged front and rear with no rear sensor, the old code held
(speed 0) and re-armed the same failed command forever: run_20260804_161650 froze
for 27 s, with `rear_clearance_m` 0.089 m just below `contact_dist`.

Finally, an escape reverse chosen from FRONT severity alone could not fit the
space behind: at `rev_speed 0.20 m/s` the two K-turn durations are 10.8 cm and
21.6 cm of reverse against a rear gap measured at p50 17 cm / p10 7 cm over 46
escape episodes (09-10 bags), so the reverse did not fit in 35 percent of them.

## Options considered

- (a) Read only the rear distance and hold when wedged.
- (b) Branch rear logic on whether the sector is measured, fit the reverse to the
      measured rear room, pivot forward when both ends are blocked, and re-anchor
      the escalation at the manoeuvre end.

## Decision

(b). Every rear gate branches on `rear_sector(...).measured`, never on the range
number. An unmeasured rear refuses the reverse; the pose trail may vouch for it
instead, but only if it covers the WHOLE manoeuvre with `contact_dist` to spare,
and an empty trail refuses. `_fit_reverse_to_rear_gap` caps the reverse above by
the measured rear room, as a CEILING not a replacement: a reverse that fits is
untouched, and an unmeasured rear is left alone rather than capped to zero.

`STUCK_FORWARD` pivots forward at low speed toward the more open side when both
ends are blocked and the rear is free, instead of holding. It is gated on the
side being genuinely open so a true dead end still holds, and it never reverses.
The stuck K-turn's base steering side is seeded once per sequence from a left/right
clearance comparison (`_stuck_escape_base_sign`), falling back to the committed
side on a tie or an unreadable side; the base used to be hardcoded to 1.0 and
opposed the clearer side 57 percent of the time against 11 percent for side
correction.

`_maybe_escalate` re-anchors the sequence at the manoeuvre's END, so the
escalation asks whether the robot made progress SINCE the escape, not during it.

`retrace_escape = false` (refuted): replacing the reverse arc with a retrace of
the pose trail halves wall strikes (13 to 7) but gives back most of the sign gain
(41 to 53), because backing straight out does not reposition the chassis and the
forward re-approach repeats the line that just failed. Retrace is a steering
substitution inside an already-decided 21.6 cm reverse leg, re-aimed at a
breadcrumb 0.25 m back; it arranges no second attempt and does not extend the
manoeuvre. The cheapest correct sub-floor fall-through is a straight reverse
(steering 0.0), not retrace.

## Durations

Escape durations are stored in SECONDS, not frames (this supersedes ADR 0023 and
carries its rationale). 40 frames is 2 s at the shipped 20 Hz and 0.8 s at 50 Hz,
so raising the control rate would have shifted every duration together with
nothing raising. `profile.Frames(seconds, controlHz)` converts at the point of
use, rounding rather than truncating and flooring at one tick. `k_turn_min_s =
0.54`, `k_turn_max_s = 1.08`, `max_escape_s = 1.8`. The 1.8 is the shortest
duration clearing timeouts at the measured 0.29 m turn radius: 1.0 gave 26
in-time / 28 laps>=3 / 9 timed out; 1.8 gave 29 / 34 / 0; 2.3 gave 31 / 33 / 0;
3.0 gave 22 / 22 / 1.

## Consequences

- A reverse can no longer pass an unmeasured rear, and cannot overshoot the room
  it has.
- A wedged robot walks its nose clear instead of deadlocking.
- The escalation fires on genuine progress, so a limit cycle is broken rather
  than re-armed.
- The escape changes must not be screened on the corpus: the sim's contact model
  never slides along a wall (56x less progress at 20 deg) and has no motor
  deadband, so it is not admissible for this class of change.

## History

- e1f9ab9c 2026-08-04: stuck-escape forces forward when only reverse is blocked.
  run_20260804_161650 froze 27 s. Adds `STUCK_FORWARD`.
- da7c1515 2026-08-04: the escape escalation counter survives a normal_drive tick
  without progress. run_20260804_213147 pinned 34+ s.
- 032a4c24 2026-08-17: refuse to reverse blind; the rear gate fails closed via
  `SectorRanges.measured`. Retrace rejected: wall 13 to 7, sign 41 to 53.
- a9383218 2026-08-22: authorize a rear escape via pose-trail evidence when the
  rear is blind. 128/128 blind Open clean (was 127 ok / 1 collision); an empty
  trail still refuses.
- 95d7a050 2026-08-27: pivot instead of holding when forward is blocked and the
  rear is unreadable. In-bay 0.00 m to 0.33-14.06 m, one completed lap.
- bda6b10c 2026-08-31: don't escape FORWARD into a wall the robot cannot see.
  `forward_no_data_is_degraded` true on a zero-false-positive bag replay.
- 1edd3dc2 2026-09-04: make escape durations rate-independent in both stacks.
- 9eb7c3cb 2026-09-07: run sim diagnostics on every core; find the escape-duration
  knee at radius 0.29 (table above).
- 72e7172b 2026-09-07: give the chassis its measured turn radius (0.29) and an
  escape long enough to use it. Timeouts 9 to 0, laps>=3 28 to 34, collided 8 to
  11; 90 escape episodes median 27.4 deg, 39 percent under 20 deg.
- d2599906 2026-09-10: cap the escape reverse at the rear room it can measure.
  46 episodes, p50 17 cm / p10 7 cm; reverse fits 30/46 against an expected 46/46.
- 33be7da7 2026-09-11: seed the stuck K-turn's steering side from measured
  clearance. 30/30 escape tests.
- 6c727c87 2026-08-21 / 2026-08-22: read bumper gaps from the bumper frame, not
  the sensor mount. Forward readings moved 12.2 cm closer and the CRITICAL gate
  fired 30811 times with zero reachable in the previous frame; once the rear
  frame was fixed the rear contact distance could fire, wall collisions rose 1 to
  36 as escapes went 4 to 54 per lap.
- 328d514a 2026-09-14: stop the escape manoeuvre certifying its own success.
  Re-anchor at the end; sim 28 failed to 21.
- 63c042bb / c1945efd: turning-escape re-seek reverted as inert and wrongly
  modelled: the 90 deg threshold never fires (k_turn p50 54.3, max 68.3 deg).
- 2026-08-22: `pose_trail_min_step_m` written out from the `core_navigator`
  module constant.
- 2026-09-05: `min_history_for_distance` written out from a Python literal; the
  shipped value was unchanged, so naming it changed nothing.
- 2026-09-16: decline the locked K-turn when the TAIL would sweep into
  something (`k_turn_tail_clearance_m`, Obstacles 0.10 m of lateral reach beyond
  the flank, Open off). In reverse the tail curves toward the steer side while
  the nose swings away; the wanted-side gate in 0050 checks the nose side only.
  A swept STRIP (rear bumper back by the reverse distance, flank out by the
  reach), not a quadrant: a quadrant at 0.30 m declined on parallel corridor
  walls and broke 0001 and 0014. Sources are the raw scan with self-returns
  removed AND the mapped sign positions, because the pillar beside the tail sits
  in the rear occlusion band. Kept locked when the rear room is under one
  minimum K-turn: the straight leg would be cut to a stutter and the stuck nudge
  then drove FORWARD into the pillar ahead (0014). Straight reverse is the same
  answer the K-turn already gives to a shut wanted side.

## Cross-references

- 0023 is superseded; its seconds-not-frames rationale is carried above.
- 0040 (rear self-detection follows chassis geometry) is the sensing prerequisite
  that makes `.measured` and the bumper-gap conversion trustworthy; it is carried
  in 0056.
- 0050 owns the escape steering units and the committed side; this story owns the
  manoeuvre choice and the durations.
- 0047 (contact reverse disabled) stays separate.

## Evidence

- `contact_dist` was compared to a raw LIDAR range although the sensor sits
  0.1222 m ahead of chassis centre, so front gaps are `range - 0.0278` and rear
  gaps `range - 0.2722`. In the old body-centred frame 0 of 30,811 CRITICAL ticks
  fired, and the rear reverse-guard (0.2722 against 0.10) could never fire.
- Keep the rear logic branching on `rear_sector(...).measured`; do not re-tune
  `min_reverse_clearance_m`, because tuning a threshold on a sector that does not
  exist is pointless.
- The frame fix alone regressed (57 to 146 collisions with both ends converted),
  because the broken rear gate was load-bearing; restore correctness first, tune
  second.
- Use an early-window control (first 20 s, before runs diverge) as the only honest
  causality test for escape-to-timeout; escape rate predicts failure from the
  start, it is not a symptom.
- `obstacles_contact_dist = 0.05` would give laps>=3 11 to 82 and timeouts 126 to
  48 with wall collisions flat at 17, but pass-side violations rise 82 to 122; it
  ships unset pending a stopping-distance bench check.
- Open shares this escape ladder, its rear sector is gone too, and its 128/128 was
  partly a phantom rear sensor.
- `tick_router_during_maneuver` (off) addresses the same loop from the other end:
  over 105 escape episodes on five rounds the escape gains a median 9.8 cm of
  forward clearance and only 15 percent gain nothing, yet 62 percent are followed
  by another escape within two seconds because 97 percent are handed back the
  SAME target (median movement 0 cm). The escape works and the frozen plan undoes
  it. Motivating stats measured 2026-09-12 over three hardware rounds: 183
  manoeuvre episodes covered 22.3 percent of all ticks and 179 of 183 (97.8
  percent) held one steering value for up to 44 ticks.
- `pose_trail_min_step_m` is a per-tick threshold coupled to speed: at 0.156 m/s
  and 20 Hz the chassis advances about 0.008 m per tick and the trail thins,
  while at 0.234 m/s it advances about 0.012 m and nothing thins. Re-check it
  with any speed-profile change.
- The older rear guard only checks the gap at the START of the manoeuvre, so it
  cannot catch a reverse that is too long part-way through; that is what the
  rear-gap cap exists to catch.
- The rear-gap cap's evidence is all Obstacles (the object behind is a pillar);
  Open escapes in corners against walls, where a shortened reverse under-rotates
  and re-triggers, feeding the corner-escape loop that costs about 20 percent of
  runs. The shared flag extends it to Open, unmeasured there.
- Never cover ground backwards: a robot reversing down a corridor is going the
  wrong way regardless of which way it points, which is why `min_reverse_clearance_m`
  is a safety floor and not a tuning lever.
- SIDE_CORRECTION reverses on 99.4 percent of its ticks.
- On 2026-08-05 four escalating escapes over 40 s rocked the yaw and translated
  the robot nowhere.
- Escapes are counted as EPISODES (contiguous latched-manoeuvre runs separated by
  a 1.0 s gap), not by `escape_count`: `escape_count` reads 1 on 96 percent of
  triggers because it resets about 3 to 4 cm of ordinary driving AFTER the
  manoeuvre ends (the end-of-manoeuvre re-anchor, p50 0.039 to 0.042 m), not on
  the escape's own reverse, so it cannot segment episodes.
- Trigger bearings are published on a 0..2*pi convention and must be wrapped to
  +/-180 deg before the front/rear split.
- Rotation and duration transfer from the simulator almost exactly (sim 22.2 deg /
  1.05 s against hardware 19.0 deg / 0.97 s), but the OUTCOME does not: raising
  `max_escape_s` 1.0 to 1.8 made the simulator need FEWER escapes at flat total
  cost and the robot need MORE, doubling time spent reversing to 31 percent of the
  round.
- Predicted from the pre-change bags: room for the 0.20 m reverse on 94 percent of
  escape triggers, but for a 0.36 m reverse on only 73 percent. Reverse has never
  been live-verified on this chassis.
- Between two escapes the robot covered 2-7 cm on the 2026-09-10 evening runs,
  with half a metre of clear space ahead, so the interval is the robot failing to
  drive rather than a navigation failure.
- Fifteen 2026-09-15 Obstacles rounds reported three laps often enough to look
  healthy, but only one was scoreable inside the time limit; every overtime round
  turned on a single 37-73 s wedge inside one lap, while a cruise lap costs 41-53 s.
- On the 2026-09-11 Obstacles rounds two of three runs wedged at the same physical
  point about 5 cm apart; the chassis commanded and the wheel turned on 97-100
  percent of ticks, absolute wheel travel ran 3.7x the signed, and a sign was
  committed on 62/67 percent of wedge ticks against 26/53 percent over the whole
  run. The sign lane saw it, committed, commanded around it, and the chassis still
  could not get past: an execution failure with the plan already in hand.
- The bay exit wins a "held still longest" search and buries the wedge being looked
  for (369 ticks, 41 percent of a run on run_20260911_110734).
- Terminating on achieved yaw is refuted: a 60 deg target is byte-identical to off
  because the time cap binds first.
- A single k_turn burst reached about 275 deg of chassis yaw, so wrapping a
  whole-escape delta reports the short way round.
- Counter census over the 2026-09-15 rounds, one row per bag as triggers / count=1
  / count=2 / count>=3 and post-escape travel at reset: 140358 49/47/2/0, p50
  0.039 m in 0.20 s; 140852 25/22/2/1, p50 0.039 m in 0.17 s; 102714 189/159/24/6,
  p50 0.042 m in 0.20 s. 84 to 96 percent of triggers are attempt number one, and
  the counter clears about two tenths of a second after the escape ends.
- The anchor check: not one reset fired under the threshold, minimum 0.034 m
  against a floor of 0.03 m, which is what shows the measurement shares
  production's anchor rather than approximating it.
- The cases reaching two or more are the ones whose reverse leg was squeezed by
  the rear-gap fit, so the ladder counts a BLOCKED escape and forgets a free one;
  it escalates on the robot that cannot move, not the robot losing the rounds.
- Tail swing, 2026-09-16, corpus at baseline 25: of the six pillar-push failures
  five accrued most of the displacement IN REVERSE during a locked K-turn fired by
  the wall ahead (trigger bearing -26 to +29 deg), the pillar being passed sitting
  at 100-135 deg of bearing and 0.19-0.24 m from the chassis centre, shoved 6-57
  mm per manoeuvre; the same pose reversed straight clears it by about 8 cm. The
  four scenarios `dfdb23dc` broke (0005, 0006, 0010, 0012) and 0004 complete with
  the gate on; 0009 does not, its pillar is unmapped at 0.35 m inside the blind
  band and an ESCALATED (doubled) K-turn sweeps 72 deg into it.
- K-turn trap, 2026-09-14: the escalated manoeuvre feeds the K-turn, and the
  K-turn was measured turning against the plan on 57 percent of episodes, so make
  the response right before making the ladder reachable.
