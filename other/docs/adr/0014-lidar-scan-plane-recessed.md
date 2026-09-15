# 0014. The LIDAR scan plane is recessed, 0.08 m off the floor

- Status: superseded by 0080
- Superseded by: 0080
- Date: 2026-09-07
- Commit: 63640a1e

## Context

`lidar.mount_z_offset` was claimed to be +0.02, added to `chassis.height`. The
unit is recessed, not stacked on the chassis top: the beam MEASURES ~0.08 m off
the floor, so it sits 20 mm BELOW `chassis.height` (0.10), not 20 mm above it.

The old +0.02 put the scan plane at 0.12 m. Every object on the field is 0.10 m
tall (wall height, the pillars and the blocks all), so the repo's own geometry
said the beam clears the entire track and the robot is blind. It plainly is not:
over `run_20260907_031019` the +/-90 deg rays read p10 0.20 / median 0.58 / p90
1.79 m in a 1.0 m corridor, exactly the walls. At the real 0.08 m the beam crosses
a 0.10 m sign 2 cm below its top, on the body.

The value was hand-duplicated as `simconfig.RobotLidarZOffset` (Go, since
removed: `simconfig.Robot` now carries it loaded from `robot.toml`) and a
hardcoded 0.12 (= chassis.height + this) in
`src/python/ros2_ws/src/vtitan_bringup/launch/static_tfs.launch.py` and the URDF
xacro; the duplication was consolidated 2026-07-30 so all three consumers derive
from one value instead of three that could drift.

## Options considered

- (a) Keep +0.02 and derive a 0.12 m scan plane.
- (b) Use the measured -0.02 so the scan plane is 0.08 m.

## Decision

(b). `mount_z_offset = -0.02` m relative to `chassis.height`, so the scan plane is
0.08 m off the floor.

## Consequences

- This is load-bearing for reasoning, not just for TF: from the wrong 0.12 it
  follows that the LIDAR cannot see traffic signs at all, which is false and would
  rule out using it to propose obstacle candidates ahead of the camera.
- Re-measure after any remount rather than assuming.
