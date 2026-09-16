# 0011. Swappable servo and motor limits live in the hardware profile

- Status: superseded by 0070
- Superseded by: 0070
- Date: 2026-08-11
- Commit: d2f821ce

## Context

`servo_max_angle_deg`, `max_wheel_angle_deg`, `max_speed_mps`, `max_accel_mps2`
and `speed_response_tau_s` describe a specific servo or motor, not the chassis.
They used to be declared in `robot.toml` (the Injora's 90/55, and one shared
accel/lag value of 2.0), which meant a run with no profile set silently modelled
whichever servo happened to be checked in rather than the one on the robot. That
is not hypothetical: a whole night of motor tests once ran on the unmodified base
ceiling because `VTITAN_HARDWARE_PROFILE` was blank, and there is no way to tell
from behaviour alone which config a run used.

`max_speed_mps` is a hard ceiling, not a tuning knob: `AckermannKinematics` clamps
to it, which is why a speed profile capping commands at 0.30 m/s once turned out
to be completely inert (both 0.30 and 0.50 saturate against it). Since 2026-08-21
the navigation speed ladder is absolute m/s, so raising this value only clamps and
delivers nothing until the motor profile's own `motion/speed.toml` raises the
tiers too. `max_accel_mps2` and `speed_response_tau_s` were removed from here for
the same reason: a bag showed the current motor nowhere near the shared 2.0.

## Options considered

- (a) Keep defaults in `robot.toml` for the components on hand.
- (b) Make the fields absent there and require a named profile.

## Decision

(b). The fields live in the selected component profile under `config/profiles/`
(`180deg-injora-14kg` or `270deg-hiwonder-35kg` for servos), and one must be named
in `VTITAN_HARDWARE_PROFILE`. Loading fails with a message listing the candidates
if none is. What stays in `robot.toml` is everything true of the CHASSIS
regardless of which servo drives it. Everything upstream (`/ackermann_cmd`, the
navigator, the simulator) speaks WHEEL angles per the ROS convention; only the
servo speaks servo angles, and `linkage_ratio` is derived from the two profile
fields rather than declared.

## Consequences

- A missing profile is a loud load-time error instead of a silent wrong model.
- Raising `max_speed_mps` alone does nothing until the speed tiers rise too.
- `max_accel_mps2` and `speed_response_tau_s` moved out after the 2026-08-29 bag.

## Superseded by 0070

This decision was replaced by [0070](0070-hardware-profiles-and-challenge-overlays.md). Its content is reproduced below so this file stays self-contained; edit only the successor.

### Context

A robot without a profile selected silently modelled whichever servo happened to
be checked in, not the one on the robot. That is not hypothetical: a whole night
of motor tests ran against the unmodified base ceiling because
`VTITAN_HARDWARE_PROFILE` was blank, and there is no way to tell from behaviour
alone which config a run used. A profile folder was also misnamed
(`servo270` against the active name), and the loader skipped a missing overlay
silently, so the 270 degree servo range never applied and every drive test that
night ran on the base ceiling. A silently inactive or silently misnamed profile
looks identical to a robot correctly running on unmodified defaults.

The chassis facts and the component facts were also mixed in one file, so
swapping a servo risked silently recomputing the tuning that had been
hand-validated for the old geometry.

### Options considered

- (a) One chassis file with component facts inline; profiles as full files.
- (b) Component facts only in selected component profiles, combined as ordered
      stackable partial overlays, with a loud load-time error on a missing or
      malformed profile.

### Decision

(b). `robot.toml` holds only chassis-true values and refuses to guess component
facts (`_require_component_facts`). The required facts (`max_speed_mps`,
`max_accel_mps2`, `speed_response_tau_s` for the drivetrain; `servo_max_angle_deg`,
`max_wheel_angle_deg` for steering) live only in the selected profile under
`src/config/profiles/`.

`VTITAN_HARDWARE_PROFILE` is an ordered, comma-separated list; each profile is a
partial tree deep-merged left to right onto base, later wins, base untouched.
Component combinations are just multiple names (for example
`270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm`). Profiles whose numbers are not
yet benchmarked are seeded empty or verbatim from base and marked
`# TODO(...): needs bench/track validation`, never with invented values. Omitting
a required fact is a loud load-time error listing candidates, not a silent zero.
Upstream speaks wheel angles; only the servo speaks servo angles; `linkage_ratio`
is derived, not declared.

`max_speed_mps` is a hard clamp ceiling, not a tuning knob: raising it alone does
nothing until the speed tiers rise too.

The per-challenge tuning layer uses the same idea: overrides under
`navigation-challenges/<challenge>/` merge last, and Open's stays empty by
construction so `load_default(challenge=OPEN) == load_default()` is a meaningful
tested invariant, not true by omission. No value is added there without a
measurement; an earlier per-challenge profile was removed after being found not to
move the outcome it was meant to fix.

### Consequences

- A missing, malformed or misnamed profile is a loud error instead of a silent
  wrong model.
- Swapping a servo cannot silently recompute the tuning validated for the old
  geometry.
- The driver-level `hardware/motors/profiles/` tree still skips a missing overlay
  file silently (the loud-fail lives in the `config/profiles/` tree), which is a
  known open gap.
- `open_contact_dist` deliberately does not exist: an unset knob nothing has ever
  moved reads as tuning that exists.

### History

- 9ce05142 2026-07-25: remove `NavigationTuning.for_obstacles()`. FAST_SPEED 0.30
  sat above the 0.156 ceiling so the cap was inert; the lookahead change improved
  cross-track p90 12.9 to 5.1 cm without moving the sign-collision rate (16/16 at
  every value 0.10 to 0.40).
- b8ad653d 2026-08-11: add stackable hardware-profile overlays; seed `servo270`.
- a6e8ea1e 2026-08-11: resolve challenge mode at runtime, lock corridor width for
  Obstacles; add the challenge overlay and the byte-identical Open test.
- d2f821ce 2026-08-21: per-component profiles; delete the old combined profiles;
  base refuses to guess. Byte-equivalence verified against the old fastwide.
- 3a7d7cbe (twin 5f396624) 2026-08-27: rename `servo270` to match the active
  profile name; the 270 degree range had never applied (67.5 deg 2250 to 2000 us).
  The misnaming lasted 16 days, not months.
- bfd644d4 2026-08-29: move `max_accel_mps2` and `speed_response_tau_s` out of
  base into motor profiles.
- df342945 (twin 0956c33e) 2026-08-30: per-challenge speed ladders resolved from
  the motor profile.

### Cross-references

- 0011 and 0021 are superseded; their decisions are carried above.
- 0069 owns the governance and precedence order; 0073 owns the challenge resolved
  at runtime; 0085 owns the speed units.

