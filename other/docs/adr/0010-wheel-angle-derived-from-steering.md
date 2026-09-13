# 0010. Road-wheel angle is derived from the steering hardware

- Status: accepted
- Date: 2026-09-13
- Commit: aac9c358

## Context

The road-wheel angle at full lock is NOT a free parameter: it is whatever the
steering hardware produces. It used to sit in `[ackermann]` as a hand-computed
1.2253 rad whose comment pointed at `src/.env.example`, which by then pointed at
`motors.toml`, where the two factors actually lived. Three files, one number, and
only prose holding them together. Before that it was 0.5236 (~30 deg), which
understated the chassis by more than half.

Because `ackermann_motor_node` clamps against the SERVO limit, the navigator's own
decode of `steering_norm` through this constant was the binding limit: a full-lock
command reached only 38.5 of the servo's 90 degrees and the clamp never fired to
say so.

## Options considered

- (a) Keep a hand-computed road-wheel angle in `[ackermann]`.
- (b) Derive it from the steering section (`max_wheel_angle_deg`).

## Decision

(b). The angle is derived from `[steering] max_wheel_angle_deg`, a measured
hardware property, and is not declared in `[ackermann]`.

## Consequences

- The angle tracks the servo/linkage that actually produces it.
- A stale hand-computed value can no longer silently bind the steering.
