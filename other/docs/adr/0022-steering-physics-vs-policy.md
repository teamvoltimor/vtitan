# 0022. Steering physics and steering policy are separate fields

- Status: superseded by 0050
- Superseded by: 0050
- Date: 2026-08-21

## Context

The road-wheel angle the linkage produces at full servo lock and the angle the
navigator is allowed to command were one field. With the 180 degree servo both
coincided at 55 deg, so nothing distinguished them. A 270 degree servo made the
distinction load-bearing: `linkage_ratio` is derived from the physical angle, so
capping the command by lowering the physical number would have recomputed
`linkage_ratio` as 55/135 = 0.407 instead of the true 85/135 = 0.630. The one
consumer that converts a wheel angle back to a servo angle,
`ackermann_motor_node`, would then have driven the servo 1.55x too far -- the
wheels reaching 85 deg when 55 was asked for.

The simulator would not have caught it: it reads `max_steering_angle` directly
and never performs the servo conversion, so the error is invisible in every
sweep and appears only on hardware.

## Options considered

- (a) Keep one field and lower it to mean "steer more gently".
- (b) Keep `max_wheel_angle_deg` as the measured physics and add
      `steering_limit_deg` as the commanded policy.

## Decision

(b). `max_wheel_angle_deg` is the bench-measured linkage travel and the only
input to `linkage_ratio`; it must never be lowered as a tuning action.
`steering_limit_deg` is how much of that travel the navigator may command, and
may be lowered freely. A validator rejects a `steering_limit_deg` above the
physical maximum instead of silently clamping, because a limit above the linkage
means somebody believes the car steers harder than it does.

## Consequences

- The servo conversion stays correct when the command limit moves.
- `max_steering_angle` is `steering_limit_deg` when set, else the full travel,
  so the physical and policy angles can diverge without a code branch.
- Extends ADR 0010: the wheel angle is derived from the hardware, and now the
  used fraction of it is a separate, freely tunable value.

## Superseded by 0050

Replaced by [0050](0050-escape-steering-degrees-and-committed-side.md): Escape steering
is physical road-wheel degrees and follows the committed pass side.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
