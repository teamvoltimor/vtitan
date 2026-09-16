# 0079. The IMU runs 6-axis and its reference is zeroed at the start button

- Status: accepted
- Date: 2026-09-15

## Context

The heading reference has to survive being carried to the track after boot, and
the yaw must be bounded over a round. Three motors, a metal chassis and power
electronics make a magnetometer heading unreliable. The BNO085 in UART-RVC mode is
output-only, so the datasheet's own mitigation, disabling gyro auto-calibration,
is unavailable.

## Options considered

- (a) Use the 9-axis fusion with the magnetometer; zero the heading at node boot.
- (b) Use the 6-axis UART-RVC fusion, reject the magnetometer, and zero the
      heading reference at the start button.

## Decision

(b). The BNO085 runs UART-RVC at 100 Hz with internal 6-axis fusion (gyro plus
accelerometer, no magnetometer), accepting the datasheet drift of about
0.5 deg/min (0.0083 deg/s). The IMU feeds the pose, not a heading PID. Yaw is
relative to power-on, so the heading reference is zeroed on the not-racing to
racing transition (the start button), not at node boot, because the robot is
carried to the track after boot. Drift during the round is bounded by a
complementary filter against the Manhattan walls (see 0054).

## Consequences

- The heading-reference fix cannot be validated in simulation by construction,
  because there is no carry phase there.
- The placement error is free up to about 20 cm and never recovers past about
  50 cm, because the localizer is a local search.
- The gyro scale error is the tightest axis and has no datasheet figure; the bench
  measurement is still pending.
- 5 degrees of heading error already costs 8 of 28 fixtures, so the heading
  reference is the binding budget.

## History

- 49a85e4a 2026-04-02: BNO085 HAL with UART-RVC.
- edace671 and 0f7ad072 2026-07-26: blind-navigation evaluation; record the
  6-axis, no-magnetometer choice and the results (27/28 at spec, 23/28 at about
  twice the spec).
- 49170098 2026-07-26: zero the heading reference at the start button, not at
  boot.
- 1a49b076 2026-07-26: bound heading against the Manhattan walls.
- f6d53ac9 2026-07-30: single-source the IMU mount z-offset.
- 84974f88 and 67041694 2026-08-04: re-seed position on reset, and clear
  `_yaw_correction` on the heading reset.
- 2026-08-04: a CW race right after a CCW one started near 0 deg instead of near
  180 deg on real hardware, because a prior race's direction correction survived.
- 2026-08-03: `gyro_yaw` was hardcoded to 0.0 in `_imu_callback` and
  `current_velocity`/`current_steering` were only written by
  `_publish_stop_command`, so `/race_metrics` reported zero for the whole race.
- 2026-08-15: a reuse audit settled the shared I2C/RVC node surface; the RVC has
  no live gyro stream.
- f6b46da9 2026-08-28: Go UART-RVC driver.
- 2026-08-31: the second race of a button-restart pair published zero
  `/nav_debug` messages over 30 s and 4 s of RACING against 261 and 132 in the
  first (`run_20260831_224647`, `run_20260831_225308`); the fault was an
  exception escaping the state subscription reset, not IMU yaw itself.
- c426ca1e 2026-09-04: port the IMU error model and start-pose error to sim.
- ac2b6637 2026-09-14: run the corpus on the measured sensor error budget.

## Cross-references

- 0054 owns the absolute heading correction the filter applies.
- 0012 (yaw gain measured) is a separate model scale, carried in the navigation
  batch.
- 0053 owns the direction inference and start pose.
