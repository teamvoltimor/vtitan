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
