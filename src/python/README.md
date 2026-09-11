# src/python

Python/ROS2 code that runs on the robot itself: hardware drivers, navigation, the
competition state machine, vision, and a headless simulation harness for testing navigation
logic without hardware or Gazebo.

## Two source trees, one relationship

- **`src/`** (this directory's own `src/`, i.e. `src/python/src/`) - the actual
  implementation. Plain Python, importable and testable without ROS2 running
  (`tests/unit/` exercises this directly).
- **`ros2_ws/src/`** - ROS2 ament packages (`vtitan_bringup`, `vtitan_drivers`,
  `vtitan_navigation`, `vtitan_state_machine`, `vtitan_vision`). Node entry points here are
  thin wrappers: they import from `src/` and adapt it to ROS2 topics/params/lifecycle. This
  keeps the actual logic testable outside ROS2 and avoids duplicating it between a "plain"
  and a "ROS2" copy.
- **`shared/`** - a separate installable Python package (`shared.config`, `shared.domain`,
  `shared.io`), shared across this stack's own ROS2 nodes and simulation harness. Holds the
  pydantic-settings models and loaders that read the TOML under `src/config/` (repo root,
  outside `src/python/` entirely -- Go reads the same TOML tree), not the TOML itself.

Build the ROS2 workspace with `task robot:build-ws` (Linux only - colcon) before any
`run-`/`launch-` task.

## `src/` layout

| Directory | Contents |
|---|---|
| `hardware/` | Per-sensor/actuator drivers (camera, LIDAR passthrough, IMU variants, motors, button, display, Hailo NPU). Each has a `Config` (pydantic-settings, reads `src/config/hardware/*.toml` at the repo root) and a `Driver`. |
| `navigation/` | `CoreNavigator` and its controllers (pure pursuit, collision avoidance, stuck detection), maneuvers (parking, K-turn/slalom escapes), and planning (waypoint generation, sign routing/discovery, corridor estimation). Tunable via `NavigationTuning` (the `shared` package's `shared.config.navigation_tuning`), not hardcoded - the TOML itself lives at `src/config/navigation/*.toml` (repo root). |
| `state_machine/` | The 4-stage competition state machine (BOOT_CHECK → READY → RACING → FINISHED) and its data types. |
| `simulation/` | Headless closed-loop simulator: drives the real `CoreNavigator` against a simulated `HardwareGateway` (Ackermann kinematics + raycast LIDAR + collision), used for navigation regression testing without Gazebo. |
| `vision/` | Traffic-sign detector (Hailo NPU on hardware, Ultralytics/YOLO fallback in sim). |
| `ros2/` | The plain-Python side of ROS2 node logic that `ros2_ws/`'s node wrappers call into (parameter handling, topic callback logic) - kept here rather than in `ros2_ws/` for the same testability reason as everything else in `src/`. |
| `teleop/` | Bench-testing joystick teleop tool. |
| `config/`, `gen/`, `logger/` | Env/config loading (this package's own, distinct from the repo-root `src/config/` TOML tree), generated code, structured JSON logging setup. |

Hardware config lives under `src/config/hardware/*.toml` at the repo root (one file per
driver, read via pydantic-settings - see each driver's `Config`), alongside
`src/config/navigation/*.toml` (navigation tuning) and `src/config/robot.toml` (cross-language
physical constants). The `shared` package's `shared.config` is the Python loader for this
tree, not a copy of it -- Go reads the same TOML files directly.

## Testing

- `tests/unit/` - plain-Python tests against `src/` directly, no ROS2 required. Includes the
  closed-loop simulation regression suite (`test_obstacles_challenge_sim.py`,
  `test_deviation_recovery.py`, etc.).
- `tests/ros2/` - tests that exercise the ROS2 node wrappers.
- `tests/hardware/` - tests against real hardware drivers; most require the actual sensor
  attached and are skipped otherwise.

Run via `task robot:test` (`SCOPE=all|unit|hardware`, default `all` excludes on-device
hardware tests).

## Key Taskfile entry points

See `task --list` (or the repo root `Taskfile.yml`'s `ROBOT`/`ROBOT REMOTE` sections, defined in
`tasks/platform.yml`) for
the full list - `robot:install`, `robot:test`, `robot:lint`, `robot:run` (single node),
`robot:launch` (node sets: `rpi5`/`rpi-zero`/`state-machine`/`telemetry`/`simulator`/`lidar`),
`robot:deploy` (ship code to the Pi 5 over SSH), and `robot:drive`/`robot:reset-motors` for
runtime control-mode switching.
