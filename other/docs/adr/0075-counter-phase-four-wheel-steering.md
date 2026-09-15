# 0075. Four-wheel counter-phase steering replaces Ackermann

- Status: accepted
- Date: 2026-09-15

## Context

The chassis must approach a 90 degree per-wheel turn to make the parking-bay exit
viable and to take the tight Obstacles corners. A car-like Ackermann linkage was
the alternative. Steering a four-wheel-drive chassis also loads the servo far
more than a two-wheel front steer, which is why the steering servo was upgraded
from 14 kg-cm to 35 kg-cm.

## Options considered

- (a) Ackermann front steering.
- (b) Counter-phase four-wheel steering: both axles steer, opposite directions,
      equal angle.

## Decision

(b). Both axles steer in opposite directions by the same 1:1 angle
(`rear_steer_ratio = 1.0`). The effective wheelbase for the kinematic model
becomes `wheelbase / (1 + rear_steer_ratio) = wheelbase / 2`, because the vehicle
yaws about twice as fast as a front-steer car at the same steering angle. The
chassis delivers about 0.55 of the yaw a bicycle model predicts, which is
compensated separately (`obstacles_yaw_gain_compensation = 0.55`). Measured
minimum turn radius is 0.29 m.

The 270 degree servo (Hiwonder HPS-3527SG) replaced the 180 degree Injora because
counter-phase steering needs the extended travel to approach the 90 degree
per-wheel turn.

## Consequences

- The turning radius is roughly halved and the bay exit becomes feasible.
- Any gain fitted against a front-steer model is too hot for the real chassis; the
  simulator had to be corrected to counter-phase before its numbers meant anything.
- Pre-4WS measurements are void: old sim minimum turn 0.329 m against the actual
  0.165 m, collisions 7/16 to 16/16, timeouts 9/16 to 0/16.
- The explicit quantitative Ackermann-versus-counter-phase comparison is still
  missing from the shipped docs; only the geometry diagram of the rejected
  alternative is catalogued.

## History

- 8eb3c38e 2026-07-25: model the chassis as counter-phase four-wheel steer, not
  front only. Moves the ICR to the chassis centre; 20 deg steer gives 0.261 m
  radius against the old 0.522 m.
- 77962720 2026-07-25: flag the sign-avoidance log as stale after the model change.
- d7579db7 2026-08-15: pin the counter-phase geometry; 6 tests, reverting
  `_turn_reference_len` fails 2 of 6.
- 6a89c45f 2026-08-22: make the Gazebo URDF a counter-phase 4WS car (rear links
  mimic `x-1.0` of front).
- 7a96dd6a 2026-09-05: compensate pure pursuit for the delivered yaw; Go had no
  compensation and turned about 1.8x too wide. Obstacles sighted laps>=3 60 to 111.
- 72e7172b 2026-09-07: ship the measured turn radius 0.29 m.
- 3b330a1c and 878b8485 2026-09-14: document the trade-offs and catalogue the
  rejected Ackermann diagram.

## Cross-references

- 0076 owns the drivetrain and steering hardware; 0052 owns the pursuit law that
  consumes the effective wheelbase; 0054 owns the yaw model.
