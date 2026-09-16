# 0074. The control loop rate has a single source

- Status: accepted
- Date: 2026-09-15

## Context

The loop period was stated twice, as 0.05 s in the waypoint controller and 20 Hz
in the simulated gateway, and the two agreed only by maintenance. The controller
rate-limits steering with dt, so the two drifting apart would not raise anything:
it would tune the robot against a cadence the simulator never ran at.

## Options considered

- (a) Leave the loop period stated independently in each consumer.
- (b) One `control_hz` in the navigation motion config, read by every consumer.

## Decision

(b). `control_hz = 20.0` in `src/config/navigation/motion/control.toml` is the
single source for the loop period. The waypoint controller derives `dt = 1 /
control_hz` for its steering rate limit; the simulated gateway steps at
`control_hz`; escape durations (stored in seconds, ADR 0023 to 0055) convert to
ticks through it; the Go stack reads `ControlHz`. Comments beside a TOML field
are not a mechanism; the value is read.

## Consequences

- Changing the loop rate rescales every derived duration and rate consistently,
  with nothing silently retuning the robot against a different cadence.
- A loop-rate change is a deliberate action with one edit point.

## History

- 617e1905 2026-08-01: give the mat, the robot and the tuning one source each.
  `control.toml` created; waypoint controller and gateway both read it.
- b730da2b 2026-08-07: gateway defaults come from the config, with a backward
  compatible export.
- 4da75303 2026-08-29: the Go stack consumes the config rate.
- 1edd3dc2 2026-09-04: make escape durations rate-independent in both stacks;
  removes a hardcoded `ControlHz = 20.0` in the harness and a literal 4000-tick
  budget.
- 5c2df515 2026-09-09: document `control_hz` in the navigation config reference.
- 5671099b and 39d8e679 2026-09-10: path moves to `src/config/navigation/motion/`.
- 3b6456d5, 1d0cf606, b7f62bfa 2026-09-13/14: schema plus generated DTOs; field
  becomes lowercase `control_hz`.
- 2026-09-01: the loop's catch-all handler logged with `exc_info=True`, which
  rclpy rejects with TypeError, turning every unnamed exception into a fatal one;
  an AttributeError in the escape path (`run_20260901_075151`) killed the node
  mid-race, `/nav_debug` stopping at 3.47 s against 21.2 s for every other node.

## Cross-references

- 0069 owns the config governance; 0055 owns the escape durations that convert
  through this rate; 0066 owns the processes that run the loop.
