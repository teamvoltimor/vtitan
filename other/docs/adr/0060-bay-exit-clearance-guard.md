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
