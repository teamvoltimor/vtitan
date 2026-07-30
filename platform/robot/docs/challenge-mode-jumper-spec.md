# Challenge Mode Jumper — Spec

**Status: implemented** (state-machine boot detection, diagnostics, OLED confirmation/fault
page, `.env.example`). `core_navigator`/`sign_router`/`parking` needed no changes: they
already gate obstacle-specific logic on the scenario metadata's `challenge_type`, which is
the source of truth for an actual competition run -- the jumper is a boot-time
confirmation/default-lap-count layer on `state_machine_node`, not a second source of truth
consumed by the navigator.

A 2-pin jumper across GPIO23 and an adjacent GND pin, read once at boot, tells the robot
whether it's about to run the Open Challenge or the Obstacle Challenge, so the right
navigation logic (`SignRouter`, `ParkController`, lap count) is selected automatically instead
of relying on someone passing the right launch file by hand.

## Hardware

- **Pin**: GPIO23 (physical pin 16 on the 40-pin header), Pi 5.
- **Wiring**: 2-pin jumper cap between GPIO23 and an adjacent GND pin (e.g. physical pin 14
  or 20). No external resistor — GPIO23 is configured as input with the **internal pull-up**
  enabled (same mechanism already used for the start button on GPIO4, see
  `src/hardware/button/gpio/driver.py`).
- **No conflicts**: confirmed against `.env.example` — existing GPIO usage is button=4,
  servo=12, motor PWM=13, motor IN3/IN4=5/6, encoders=16/20. LIDAR, camera, and IMU are
  USB/CSI/I2C, not GPIO. GPIO23 is free.

## Polarity

| Jumper state | GPIO23 reads | Mode |
|---|---|---|
| Inserted (shorted to GND) | LOW | **Obstacle Challenge** |
| Absent (pulled up internally) | HIGH | **Open Challenge** |

Both states are fully determinate by construction — the internal pull-up means "no jumper"
is a legitimate, stable HIGH reading, not a float. The only real failure mode is a flaky or
intermittent physical connection at the moment of the read.

## Boot-time detection

Add a `ChallengeModeStatus` to `state_machine_node.py`'s `_check_system_status()`, alongside
the existing `SensorStatus` checks for IMU/LiDAR/Hailo:

- Sample GPIO23 several times over ~200–300 ms during `BOOT_CHECK`.
- All samples agree → proceed with that mode, `is_ready=True`.
- Samples disagree (bounce/intermittent contact) → treat like a not-ready sensor: log a
  warning, keep `all_ready=False`, and stay in `BOOT_CHECK` indefinitely until the reading
  stabilizes. This is a deliberate fail-closed choice — an ambiguous reading must not let the
  robot silently pick a mode and start.

Config follows the existing per-driver `BaseSettings` pattern:

```
CHALLENGE_MODE_GPIO_PIN=23
```

## Propagating the mode

- New `ChallengeMode` enum (`OPEN` / `OBSTACLES`).
- Published on `/system_status` diagnostics (and/or a small dedicated topic/param) so
  `CoreNavigator`, `SignRouter`, and `ParkController` can enable or disable obstacle-specific
  logic at runtime, and `target_laps` picks the correct `CompetitionSpecs` constant
  automatically instead of depending on the launch-time `target_laps` parameter being set
  correctly by hand.

## OLED confirmation

`oled_display_node.py` already renders `/system_status` diagnostics during boot. Add:

- A "MODE: OBSTACLES" / "MODE: OPEN" line once detection succeeds, so the team has a visual
  pre-race check.
- A distinct "CHECK JUMPER" fault page if the reading is unstable.

## Out of scope

- The simulator does not need this. `scenario_catalog.py` already encodes obstacles-vs-open
  per scenario independently of real hardware — no sim changes required.

## Touch points (for implementation)

- `ros2_ws/src/vtitan_state_machine/vtitan_state_machine/state_machine_node.py` —
  boot-time detection, `SystemStatus` gating.
- `ros2_ws/src/vtitan_state_machine/vtitan_state_machine/state_machine.py` —
  `ChallengeMode` enum, `ChallengeModeStatus`/`SystemStatus` fields.
- New pin-read helper mirroring `src/hardware/button/gpio/driver.py`, minus debounce/long-press
  (the jumper is static, not a momentary control).
- `ros2_ws/src/vtitan_drivers/vtitan_drivers/oled_display_node.py` — mode/fault
  display.
- `.env.example` — document `CHALLENGE_MODE_GPIO_PIN`.
- `src/navigation/core_navigator.py`, `src/navigation/planning/sign_router.py`,
  `src/navigation/maneuvers/parking.py` — consume `ChallengeMode` to enable/disable
  obstacle-specific logic.
