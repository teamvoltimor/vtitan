# Joystick Teleop (bench testing)

Lets you drive the steering/drive motors directly with a Bluetooth gamepad (e.g. 8BitDo
Ultimate 2) instead of the autonomous navigation stack — for testing motors/steering by
hand, not competition use.

## What changed

- **`ros-kilted-joy`** added as a pixi dependency — the stock ROS2 `joy` package's `joy_node`
  reads the paired gamepad as a Linux joystick device and publishes `sensor_msgs/Joy` on
  `/joy`.
- **`src/teleop/config.py` + `mapping.py`** — pure, unit-tested logic mapping `/joy` axes/
  buttons to `(speed_mps, steering_deg)`. Steering always follows the stick; drive speed is
  zero unless a dead-man button is held *and* the joystick link is fresh (see
  `joy_timeout_s`). Axis/button indices, speed/steering limits, and invert flags are all
  `JOY_TELEOP_*` env vars (see the file for the full list and defaults).
- **`joy_teleop_node`** (`vtitan_drivers`) — subscribes to `/joy`, republishes
  `ackermann_msgs/AckermannDriveStamped` on `/ackermann_cmd` at 20 Hz, the same topic
  `ackermann_motor_node` and `scripts/hardware/test_motors.py` already use. No changes to
  `ackermann_motor_node.py` or any hardware driver were needed.
- **`joy_teleop_launch.py`** (`vtitan_bringup`) — launches `joy_node` + `joy_teleop_node`
  together.
- **`race.launch.py`** — unchanged, but is now one of two things you choose between (see
  Control modes below) rather than the only "brain" driving `/ackermann_cmd`.
- **`scripts/setup-joystick.sh`** — one-shot Bluetooth pairing for the controller.
- **`scripts/hardware/reset_motors.py`** — publishes a single zero-speed, zero-steering command and
  exits. `ackermann_motor_node`'s watchdog already stops the drive motor 1s after commands
  stop arriving, but it never recenters steering — this does that immediately.
- pixi tasks: `setup-joystick`, `drive-navigation`, `drive-controller`, `drive-mode`,
  `reset-motors`, `run-joy-teleop`, `launch-joy-teleop`.
- `task` (go-task) wrappers in `platform/Taskfile.yml`: `robot:joystick-setup`,
  `robot:drive ACTION=navigation|controller`, `robot:reset-motors`.

## Control modes

Exactly **one** of navigation or controller mode should run at a time — both publish to
`/ackermann_cmd` and nothing arbitrates between them if both run simultaneously.
`ackermann_motor_node` on the Pi Zero stays running throughout either way; switching modes
only means stopping one Pi-5-side process and starting the other.

| Mode | What runs | Command |
|---|---|---|
| Navigation | `race.launch.py` (`track_navigator_node`, optional rosbag) | `task robot:drive ACTION=navigation` |
| Controller | `joy_teleop_launch.py` (`joy_node` + `joy_teleop_node`) | `task robot:drive ACTION=controller` |

`ROBOT_CONTROL_MODE=joystick\|navigation` selects the same thing via the pixi-level
`drive-mode` task, if you'd rather not pass `ACTION=`.

## Setup order

Run on the **Raspberry Pi 5** (it has the Bluetooth radio; `ackermann_motor_node` keeps
running independently on the Pi Zero the whole time, same cross-board split
`scripts/hardware/test_motors.py` already uses):

1. **Pull in the dependency and build**: `pixi install -e dev` (picks up `ros-kilted-joy`),
   then `task robot:build-ws` (or `pixi run -e dev build-ws`) — needed once, to compile the
   new `joy_teleop_node` and register `joy_teleop_launch.py`.
2. **Pair the controller**: put it in Bluetooth pairing mode (8BitDo Ultimate 2: hold the
   pair button until the LED flashes rapidly), then `task robot:joystick-setup`. Idempotent —
   safe to re-run if the controller drops and needs re-pairing. Already know the MAC?
   `CONTROLLER_MAC=AA:BB:CC:DD:EE:FF task robot:joystick-setup` skips the scan.
3. **Verify axis/button mapping**: the defaults in `src/teleop/config.py` are an unverified
   best guess (typical Linux xpad-style mapping) — Bluetooth vs USB and input mode can shift
   indices. With the controller connected:
   ```
   pixi run launch-joy-teleop &
   ros2 topic echo /joy
   ```
   Wiggle each stick/button and note which `axes`/`buttons` index moves. Override any that
   don't match via `JOY_TELEOP_STEERING_AXIS_INDEX`, `JOY_TELEOP_THROTTLE_AXIS_INDEX`,
   `JOY_TELEOP_DEADMAN_BUTTON_INDEX`, etc. (env prefix `JOY_TELEOP_`, see `config.py`).
4. **Drive**: `task robot:drive ACTION=controller`. Steering follows the stick immediately;
   the drive motor only spins while the dead-man button is held.
5. **Switch back to navigation when done**: `Ctrl+C` the controller task, run
   `task robot:reset-motors` (recenters steering immediately instead of waiting on the
   watchdog), then `task robot:drive ACTION=navigation`.

## Troubleshooting

- **No `/dev/input/js*` after pairing**: `bluetoothctl info <MAC>` should show
  `Connected: yes`. Re-run `task robot:joystick-setup` with the same MAC — pairing/trust
  steps are skipped if already done, only reconnect is retried.
- **Steering/drive doesn't respond**: confirm `joy_teleop_node`'s log shows `Drive ARMED
  (dead-man held)` when you hold the button — if it never flips, the configured
  `deadman_button_index` doesn't match this controller (see step 3).
- **Permission denied on `/dev/input/js*`**: `setup-joystick.sh` adds the user to the `input`
  group — log out/in (or reboot) once after the first run for that to take effect.
