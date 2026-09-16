# 0036. Parking is not pursued after the final lap

- Status: superseded by 0062
- Superseded by: 0062
- Date: 2026-09-05
- Commit: 4e061f6f

## Context

The parking bay is geometrically unreachable for this chassis: 0.194 m of
chassis width against a 0.20 m pocket is 6 mm of total slack, +/-3 mm on the
centre, and a maximum heading error of 1.16 deg against the 6.0 deg the rule
allows. Chasing it after the laps are already banked only spends clock and
contact.

Measured blind over the 256 corpus, pursue-OFF against pursue-ON: in-time 158 vs
62, collisions 4 vs 51, timeouts 61 vs 110, with laps>=3 identical at 159.

## Options considered

- (a) Continue to pursue the bay after the final lap.
- (b) Stop in the finish section.

## Decision

(b). `attempt_after_final_lap = false`. The bay is unreachable, so the pursuit
spends the round for no return.

## Consequences

- A round that would have chased the bay stops in the finish section instead.
- This was fixed in Python at `4e061f6f` but Go had never read the key until it
  was mirrored 2026-09-06, so every Go corpus number taken before that was
  measured against the pursuing behaviour.

## Superseded by 0062

This decision was replaced by [0062](0062-sim-contact-model-and-parking.md). Its content is reproduced below so this file stays self-contained; edit only the successor.

### Context

The simulator graded itself against a model that rewarded infractions. An exit
scoring 254/256 under `--solid-walls` was disqualified, because `--solid-walls`
lets the chassis grind along a fin: the escape needs sustained contact, and that
contact is the 9.24.7 round-ender. A result taken under that model is not an
evaluation.

Three maskers hid the same class of penetration: parking fins inherited the sign
leniency, the start-collision grace forgave fin contact during the 22-tick exit,
and the solid surface set excluded fins. The parking lot was arithmetically
unreachable until the wall was modelled at its real thickness: a 0.194 m chassis
in a 0.20 m bay needs its centre within 0.10 m of the wall, but the inflated
collision stopped it at 0.14 m.

### Options considered

- (a) Score with `--solid-walls`; inflate the wall; treat pillars as first-contact
      failures; pursue parking after the final lap.
- (b) Score the default (contact ends the run) model; model the wall at its visual
      thickness; model pillar displacement; stop in the finish section; derive the
      lot from the in-bay start.

### Decision

(b). `wall.collision_thickness = 0.10` equals the visual wall with no inflation,
paired with the LIDAR mounted where it really is; this is what makes parking not
arithmetically blocked (0.097 vs 0.100 m, 3 mm per side).

`sign.placement_circle_diameter = 0.085`: pillars are DISPLACED, not scored as
first contact. Only pushing a pillar fully out counts against the team, and the
conservative tolerance is 59.4 mm axially and 77.9 mm diagonally, so the simulator
must track pillar pose, not just a contact flag. `parking.spacing_factor = 1.5`
multiplies the robot LENGTH from `robot.toml` (1.5 x 0.30 = 0.45 m fin spacing);
without the length the factor is meaningless.

`attempt_after_final_lap = false`: the round ends in the finish section (rule
1.3). Blind, 256 corpus, OFF against ON: in-time 158 against 62, collisions 4
against 51, timeouts 61 against 110, laps>=3 identical. `derive_lot_from_in_bay_start
= true`: "it never tried" is a worse failure than "it tried and could not", even
though it will probably still fail on geometry (6 mm of depth slack, 1.16 deg of
heading tolerance against the rule's 6.0).

Terminal surfaces: OPEN is the outer wall; OBSTACLES is the inner wall, the
obstacles, and the parking lot. `obstacles_inner_wall_terminal = true` keeps the
strict scoring every figure was measured under. Parking-lot contact has no grace
(`_UNFORGIVABLE_SURFACES`) and the fins are solid with a touch epsilon (905a9b15).

The state machine has NO PARKING state. `RobotState` has exactly four values
(BOOT_CHECK, READY, RACING, FINISHED) and `_VALID_TRANSITIONS` has no edge to a
parking state; parking was always an internal navigator phase within RACING.
(`NavigatorPhase.PARKING` exists as an internal debug phase, not a top-level
state.)

### Consequences

- A manoeuvre that "works" only on sustained wall contact cannot pass; `--solid-walls`
  must never be used to evaluate the exit.
- Parking is not blocked by geometry alone, but it is still expected to fail on
  the 6 mm depth slack; the flag records the honest attempt.
- `contact_slides_along_surfaces = true` (2026-09-14) re-bases all in-bay A/Bs
  measured before that date.

### History

- 617e1905 2026-08-01: introduce `spacing_factor` into TOML.
- d7a31bf3 2026-08-01: score pillar displacement, not first contact. 194 collisions
  / 62 in-time to 182 / 74 over the 256 corpus; tolerance 59.4 / 77.9 mm.
- 8d3ccd16 and 6c727c87 2026-08-21: stop inflating wall collision past the visual
  mesh (0.18 to 0.10); model the LIDAR where it is (0.1222 m forward). Compatible
  at 0.097 against 0.100 m.
- 2ab86743 2026-08-23: single source for the parking block geometry.
- e6a88982 2026-08-30: port the parking manoeuvre to Go.
- 335fe021 2026-09-01: give-up budget and `--solid-walls` in the bay probe.
- 0f277aa1 and 544958be 2026-09-02: the 254/256 `--solid-walls` exit and the 64/64
  `--slide` exit; both disqualified (with slide 0/256 under the real model).
- c594e37d 2026-09-03: parking fins inherited sign leniency. Splits
  `ContactSurface.PARKING_LOT`; 48/240 parking-sweep runs touch a fin.
- d358edad 2026-09-03: no grace may forgive parking-lot contact. Before 8/8 out,
  after 0/8, all 8 collided on the parking lot.
- 905a9b15 2026-09-03: fins are solid; resting counts as touching. Guard0 16/16
  stuck on a fin; guard1 0/16.
- 4e061f6f 2026-09-05: stop in the finish section.
- 97c0720b and 818275e6 2026-09-06: Go reads the parking-deferral flag; every Go
  figure before this was against pursuing behaviour.
- 287b8507 2026-09-11: score rule 9.18 as written, off by default;
  `obstacles_inner_wall_terminal = true`.
- 996c7415 2026-09-11: derive the lot from the in-bay start. 0 of 227 bags ever
  constructed a `ParkController`; 67 reached 3 laps.
- 5ea0f524 2026-09-10: removed the non-existent PARKING state from the state
  diagram.
- 86bee47f 2026-09-14: `contact_slides_along_surfaces = true`, re-baselining the
  in-bay A/Bs.

### Cross-references

- 0001, 0005, 0006, 0036 and 0037 are superseded; their decisions are carried above.
- 0059 owns the rulebook scoring rules and the inner-wall terminal flag.
- 0060 owns the bay exit; 0058 owns the barrier belief.

