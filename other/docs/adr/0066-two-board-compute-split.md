# 0066. Compute is split Pi 5 perception and Pi Zero real-time control

- Status: accepted
- Date: 2026-09-15

## Context

Vision inference is heavy and latency-variable; the motor loop must be constant.
If both compete for the same processor, a slow frame becomes a late steering
correction, so no vision delay may be able to stall the control loop. The SoC of
the Pi Zero 2 W also has exactly two hardware PWM generators, which matters to
which services can live where.

## Options considered

- (a) Run everything on one board.
- (b) Split: Pi 5 for perception and planning, Pi Zero for real-time motor and
      servo control; merge the low-rate Zero peripherals into one process and keep
      the motor service separate.

## Decision

(b). The Pi 5 runs LIDAR, camera, AI HAT+ inference and planning; the Pi Zero 2 W
runs motor and servo control exclusively. Software is five board-agnostic ROS2
packages (`vtitan_bringup`, `vtitan_drivers`, `vtitan_navigation`,
`vtitan_state_machine`, `vtitan_vision`) and 29 topics declared in
`ros_topics.toml`.

On the Zero there are TWO processes: `ackermann_motor_node` alone, and
`pi_zero_peripherals_node` (button, OLED, challenge mode) under a
`SingleThreadedExecutor`. The peripherals were merged from three processes into
one to save about 60 to 100 MB of RAM and collapse three interpreters and DDS
participants into one; the executor was made single-threaded because strace put
61.6 percent of process CPU in `futex` (GIL contention), not in I2C or GPIO. True
multicore parallelism is deliberately traded away because one Python process is
enough, and the motor was split back out so it is not sharing executor threads
with the lower-rate peripherals.

Feedback publishes at `publisher_rate_hz = 20.0`; motor diagnostics moved to their
own 2 Hz timer.

## Consequences

- No vision delay can stall the motor loop.
- 30 Hz was tested and reverted: it cut round-trip latency about 31 percent
  (41.15 ms to 28.45 ms) but raised CPU to about 78 percent of one core, leaving
  too little headroom alongside the button and OLED callbacks and the 500 ms
  watchdog.
- The latent next step is splitting the motor node further, which the two-process
  layout already anticipates.

## History

- 30dfe1b3 2026-04-06: the two-board launch origin.
- 4d3bb66b 2026-07-09: split the monolith into five ROS2 packages, all driver
  nodes as `LifecycleNode`.
- 677c7245 2026-07-24: merge the three Pi Zero nodes into one `pi_zero_node`,
  shared `MultiThreadedExecutor`; about 60 to 100 MB of RAM saved.
- fed274ea 2026-07-25: split `ackermann_motor_node` back into its own process;
  rename the merged process to `pi_zero_peripherals_node`.
- 5130306b 2026-07-25: add `challenge_mode_node` to the peripherals process.
- c5d6d572 and 40a00586 2026-07-28: cut peripherals CPU (batched I2C, skip
  unchanged frames, cap executor threads; then `SingleThreadedExecutor` after the
  futex measurement).
- 1377870d 2026-07-28: feedback rate 100 Hz to 20 Hz; diagnostics to 2 Hz.
- ade65103 2026-07-25: raw ping 0.22 ms avg; ROS2 round-trip at 20 Hz avg 41.15 ms,
  max 60.57 ms; 30 Hz gave 28 ms at about 22 percent headroom.

## Cross-references

- 0067 owns the board link and provisioning; 0071 owns the per-round recording.
- 0074 owns the control loop rate these processes run at.
