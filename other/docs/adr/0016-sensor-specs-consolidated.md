# 0016. Sensor specs are consolidated into robot.toml

- Status: accepted
- Date: 2026-07-30
- Commit: f6d53ac9

## Context

Several sensor specs were hand-maintained in more than one language or file.
`lidar.min_range` sat in a hand-maintained Python constant that stated 0.05
(measured ~45 mm); `lidar.max_range` was a hand-maintained Python constant
(`LIDAR_MAX_RANGE`). `imu.mount_z_offset` (0.01 m) was duplicated as
`simconfig.RobotImuZOffset` (Go, correct, since removed: `simconfig.Robot` now
carries it loaded from `robot.toml`), a hardcoded 0.01 in
`src/python/ros2_ws/src/vtitan_bringup/launch/static_tfs.launch.py` (correct),
and the URDF xacro's `imu_link` joint origin
(WRONG, hardcoded to 0, i.e. the IMU was not actually placed there). The LIDAR
z-offset was similarly triplicated (see ADR 0014).

## Options considered

- (a) Leave each consumer's constant in place and keep them aligned by hand.
- (b) Consolidate the hardware description in `robot.toml`.

## Decision

(b). A sensor spec belongs with the rest of the hardware description, not beside
the simulation parameters or duplicated per consumer. `lidar.min_range = 0.045`
(measured ~45 mm), `lidar.max_range = 12.0`, and `imu.mount_z_offset = 0.01` live
here, consolidated 2026-07-30.

## Consequences

- The xacro's wrong `imu_link` origin (0) is fixed by reading the one value.
- The distinction that matters for `min_range` is that anything nearer is
  unmeasurable rather than clear, which matters when the chassis works within a
  few centimetres of a surface (parking, wall contact, escape).
