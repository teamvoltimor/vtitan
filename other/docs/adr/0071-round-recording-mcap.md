# 0071. Every round is recorded as an MCAP bag and analyzed offline

- Status: accepted
- Date: 2026-09-15

## Context

A round lasts at most 180 seconds and cannot be paused or repeated. If something
goes wrong, watching the robot does not reveal the cause, and a fault is not
solved by repeating the round and hoping it manifests again.

## Options considered

- (a) Observe the robot live and reproduce faults by re-running.
- (b) Record every round tick by tick and replay the exact instant offline.

## Decision

(b). Every round is recorded as an MCAP bag, gated on `/robot_state`: a bag opens
when the state machine enters RACING and closes when it leaves (finish or E-STOP),
so one bag is exactly one round. The recorded topics are `/scan`, `/imu/data`,
`/vision/detections`, `/ackermann_cmd`, `/robot_state`, `/race_metrics`,
`/nav_debug`, `/system_status`, `/motor/drive_speed`,
`/motor/steering_position`, `/joint_states`, plus `/tf` and `/tf_static`. Bags are
pulled off the robot and reopened in Foxglove for visual inspection and replayed
through `diag_bag_*.py` diagnostics.

`/camera/image_raw` is deliberately excluded (measured 63 MB/s, more than
everything else combined). Retention prunes oldest-first before each new bag.
`provenance.json` is written beside the mcap at race start (commit, branch, dirty,
`VTITAN_HARDWARE_PROFILE`).

## Consequences

- A fault can be replayed with the same data as many times as needed.
- The recorder is race-gated, so it does not fill the card with stationary-robot
  bags.
- Analysis traps are documented: a crashing diagnostic exits 0 (read its output,
  not its code); `travelled_m` is signed and cancels on oscillation;
  `/motor/drive_speed` is in degrees per second; path length from pose
  overestimates about 10 percent.

## History

- cecb1ed1 2026-08-01: introduce `bag_recorder_node`, replacing an unconditional
  `ros2 bag record`; gate on RACING; drop `/camera/image_raw`; initial cap 20 runs
  / 4 GB.
- 70494ab0 2026-08-01: match the race-gate QoS to its publisher (`/robot_state`),
  which is what actually let the recorder and navigator see RACING.
- fac326f0 and abf3bc75 2026-08-02: source bag topics from `ros_topics.toml`; add
  real motor telemetry.
- e8303ae8 2026-08-11: per-run annotated video colocated in the run directory.
- 2df720f9 2026-08-30: move pulled artifacts to repo-root `data/`.
- 9f5ffe99 2026-09-09: retention 20/4 GB to 500 runs / 50 GB (407 GB free).
- 519b23c1 2026-09-09: 500 to 2000 runs; measured mean 30.0 MB, median 18.8 MB,
  p90 72.4 MB, so the 50 GB size cap is what binds.
- 29d31cf6 2026-09-13: stamp every run with the commit, because a 55-run corpus
  had to be attributed by hand and the first answer was wrong after a rebase.

## Cross-references

- 0066 owns the processes the recorder runs beside; 0004/0046 (repo layout) place
  `data/live` and `data/sim`.
