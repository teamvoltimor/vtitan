# 0023. Escape durations are stored in seconds, not frames

- Status: superseded by 0055
- Superseded by: 0055
- Date: 2026-09-13

## Context

Every escape length, the stuck timeout and the parking give-up were once stored
as a frame count. A stored frame count silently means a different duration at a
different loop rate: 40 frames is 2 s at the shipped 20 Hz and 0.8 s at 50 Hz.
Changing the control loop would have shifted every one of those durations
together, with nothing raising and no config edited.

## Options considered

- (a) Store frame counts and accept that they are loop-rate dependent.
- (b) Store seconds and convert to ticks at the point of use.

## Decision

(b). The TOMLs hold seconds. `profile.Frames(seconds, controlHz)` converts each
one, rounding rather than truncating and flooring at a single tick: a duration
shorter than one tick is still a maneuver the caller asked for, and zero frames
would skip it entirely. `core_navigator`'s own escape fields convert the same
way at load.

## Consequences

- A control-rate change rescales every escape duration consistently instead of
  silently retuning them.
- Frame counts remain the runtime currency, so the consumers are unchanged.

## Superseded by 0055

This decision was replaced by [0055](0055-escape-maneuver-selection.md). Its content is reproduced below so this file stays self-contained; edit only the successor.

### Context

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

### Options considered

- (a) Read only the rear distance and hold when wedged.
- (b) Branch rear logic on whether the sector is measured, fit the reverse to the
      measured rear room, pivot forward when both ends are blocked, and re-anchor
      the escalation at the manoeuvre end.

### Decision

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

### Durations

Escape durations are stored in SECONDS, not frames (this supersedes ADR 0023 and
carries its rationale). 40 frames is 2 s at the shipped 20 Hz and 0.8 s at 50 Hz,
so raising the control rate would have shifted every duration together with
nothing raising. `profile.Frames(seconds, controlHz)` converts at the point of
use, rounding rather than truncating and flooring at one tick. `k_turn_min_s =
0.54`, `k_turn_max_s = 1.08`, `max_escape_s = 1.8`. The 1.8 is the shortest
duration clearing timeouts at the measured 0.29 m turn radius: 1.0 gave 26
in-time / 28 laps>=3 / 9 timed out; 1.8 gave 29 / 34 / 0; 2.3 gave 31 / 33 / 0;
3.0 gave 22 / 22 / 1.

### Consequences

- A reverse can no longer pass an unmeasured rear, and cannot overshoot the room
  it has.
- A wedged robot walks its nose clear instead of deadlocking.
- The escalation fires on genuine progress, so a limit cycle is broken rather
  than re-armed.
- The escape changes must not be screened on the corpus: the sim's contact model
  never slides along a wall (56x less progress at 20 deg) and has no motor
  deadband, so it is not admissible for this class of change.

### History

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
- 328d514a 2026-09-14: stop the escape manoeuvre certifying its own success.
  Re-anchor at the end; sim 28 failed to 21.
- 63c042bb / c1945efd: turning-escape re-seek reverted as inert and wrongly
  modelled: the 90 deg threshold never fires (k_turn p50 54.3, max 68.3 deg).

### Cross-references

- 0023 is superseded; its seconds-not-frames rationale is carried above.
- 0040 (rear self-detection follows chassis geometry) is the sensing prerequisite
  that makes `.measured` and the bumper-gap conversion trustworthy; it is carried
  in 0056.
- 0050 owns the escape steering units and the committed side; this story owns the
  manoeuvre choice and the durations.
- 0047 (contact reverse disabled) stays separate.

