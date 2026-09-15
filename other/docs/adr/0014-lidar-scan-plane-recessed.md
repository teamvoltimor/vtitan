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

## Superseded by 0080

This decision was replaced by [0080](0080-lidar-mount-and-scan-plane.md). Its content is reproduced below so this file stays self-contained; edit only the successor.

### Context

The simulated LIDAR was modelled at the chassis centre and with an inflated wall,
so every corridor behaved 8 cm narrower than spec and parking was arithmetically
impossible. The physical mount is flush with the front edge, the unit is upside-
down, and the beam plane sits below the 0.10 m object height so it crosses signs.

### Options considered

- (a) Keep the sensor at the chassis centre; inflate the wall to compensate; trust
      a rotation for the inverted mount.
- (b) Model the LIDAR at its real mount, derive the angle, and place the scan plane
      where it physically is.

### Decision

(b). `lidar.mount_x_offset = 0.1222` is derived, not declared: `chassis.length/2 -
mesh radius = 0.15 - 0.0278`. The simulated raycast, the localizer prediction and
the synthetic test fixtures all move to that offset together; separating them once
caused 26 failures and worse results. Collision thickness is 0.10 m, equal to the
visual wall, with no inflation.

`lidar.mount_z_offset = -0.02` relative to `chassis.height = 0.10`, so the scan
plane MEASURES about 0.08 m off the floor, 20 mm below the chassis height. It must
stay under the 0.10 m object height so the beam crosses signs (a 0.10 m sign is
crossed 2 cm below its top). Every consumer must derive from
`height + mount_z_offset`; no validator may confine the offset to positive.

`lidar.inverted = true` is the single source of truth for the upside-down mount,
and every angle consumer derives from it so the driver flag and the angle
correction cannot drift apart. `mount_yaw_offset_deg` stays a residual (0.0),
never a replacement. The mount is re-measured after any remount or cable work.

The RPLIDAR C1 runs Express Scan Dense Mode, not classic SCAN; classic-mode range
decode was never validated on this hardware and read 2 to 4 times too large.

### Consequences

- Forward ranges are no longer about 12 cm long, and clearance zones are honest.
- Pass rates before and after the mount fix are not comparable.
- The 0.08 m beam crosses the 0.10 m signs.
- Open documentation inconsistency: later hardware evidence shows the inverted
  correction is a MIRROR (negation), not a 180 degree rotation; the ADR's rotation
  model and `robot-physical-constants.md` still say rotation, while the README
  reports the mirror finding. Reconcile to the mirror.

### History

- 62b59bf5 2026-07-11: sync the physical mount architecture.
- bbc7c388 2026-07-26: scan at the C1's real rate and emit no-return rays.
- f6d53ac9 2026-07-30: promote the mount z-offsets into `robot.toml` (the wrong
  sign until 0014).
- 41815bcc 2026-08-02: couple the inverted flag to its yaw correction.
- 8d3ccd16 and 9206dbdf 2026-08-21: stop inflating the wall (0.18 to 0.10); model
  the LIDAR where it is (x 0.1222).
- 14f3bd40, 794a685c, 2ebe7d7d, ca5632cd, 1ee957e3, b85b7d77, 2d0f7f65 2026-08-31:
  find the scan-mode mismatch, add the Go dense driver, and refute the 180 degree
  rotation (a mirror/negation is what fits the bearing test).
- 63640a1e 2026-09-07: the scan plane is 0.08 m, not 0.12; evidence
  run_20260907_031019 (p10 0.20 / median 0.58 / p90 1.79 m in a 1.0 m corridor).
- 95bc9ab8 2026-09-14: put the scan plane back to 0.08 m in the URDF snapshot.

### Refuted

- Collision inflation (0.18 m / 40 mm); the +0.02 sign that gave a 0.12 m plane;
  the claim the LIDAR cannot see 0.10 m signs; the 180 degree rotation for the
  inverted mount; classic SCAN range decode; the hardcoded 84-byte dense packet.

### Cross-references

- 0002, 0014 and 0015 are superseded; their decisions are carried above.
- 0062 owns the wall collision thickness and parking model; 0078 owns the camera
  that sits on this mount.

