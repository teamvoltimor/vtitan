# Robot Physical Constants — Canonical Values and Where They're Duplicated

There is **no single source of truth** for the robot's physical dimensions across this
monorepo. The same numbers are hand-duplicated in Python, Go, and XML/xacro, and they have
already drifted out of sync once (chassis `LENGTH`/`WIDTH` were corrected from 0.28×0.15 to
0.30×0.20 in Python but never updated in the Go Gazebo generator or the xacro — both silently
kept using the old numbers for months). This doc exists so the next correction doesn't repeat
that.

**If you change any measurement below, update every file listed for it.** There's no
build-time check that catches drift between these — if one is missed, it fails silently (wrong
collision box, wrong sensor placement, wrong steering geometry) rather than erroring.

## Canonical values (measured 2026-07-11)

| Constant | Value | Notes |
|---|---|---|
| Chassis length | 0.30 m | |
| Chassis width | 0.20 m | |
| Chassis height | 0.10 m | unchanged |
| Wheelbase (front axle ↔ rear axle) | 0.19 m | |
| Track width (left wheel ↔ right wheel) | 0.1675 m | |
| Wheel diameter | 0.07 m (radius 0.035 m) | |
| Wheel width | 0.025 m | |
| LIDAR mount x-offset (front of chassis, centered) | 0.1222 m | derived: `chassis_length/2 − lidar mesh radius (0.0278)` — the C1 mounted flush with the front edge. Mounted upside-down (180° yaw), inverted left/right (see `docs/sensor-verification.md` Phase 2). |
| Camera mount x-offset | 0.1222 m (same as LIDAR — mounted directly over it) | **estimate**, not measured |
| Camera mount z-offset | 0.16 m | **estimate**, not measured |
| Camera mount pitch | ~30° down | **estimate** ("like 30 degrees"), not precision-measured. Sign convention (positive = down) follows precedent already in `wro_robot.urdf.xacro`'s old `rpy="0 0.2 0"` camera pitch — **not independently re-derived or verified in RViz**. If the camera renders tilted up instead of down, flip the sign at the one place each file applies it (see file list below). |

## Where each constant lives (update all, or drift happens again)

### Canonical Python source
`platform/shared/src/shared/config/constants.py`, `RobotSpecs` class — `LENGTH`, `WIDTH`,
`HEIGHT`, `WHEELBASE`, `TRACK_WIDTH`, `WHEEL_RADIUS`, `WHEEL_WIDTH`, `LIDAR_DIAMETER`,
`LIDAR_HEIGHT`, `LIDAR_MOUNT_X_OFFSET`, `CAMERA_MOUNT_X_OFFSET`, `CAMERA_MOUNT_Z_OFFSET`,
`CAMERA_MOUNT_PITCH_DEG`. Every Python consumer (navigation, simulation, ROS2 nodes) should
import from here — nowhere else in Python should hardcode these numbers.

Known Python consumer that *used to* duplicate instead of importing:
`src/hardware/motors/dc_encoder/driver.py`'s `_DEFAULT_WHEEL_DIAMETER_M` — now derived as
`RobotSpecs.WHEEL_RADIUS * 2` instead of its own literal.

### Real-robot TF (ROS2)
`platform/robot/ros2_ws/src/voldemorbot_bringup/launch/static_tfs.launch.py` — imports
`RobotSpecs` directly (the one place in this list that already reads from the canonical
Python source rather than duplicating). `camera_link`/`lidar_link` poses.

### URDF/xacro (RViz + Gazebo mesh/joints — hand-duplicated, does NOT import RobotSpecs)
- `platform/gazebo/runtime/robot_description/wro_robot.urdf.xacro` — the `xacro:property`
  block near the top (`chassis_length`, `chassis_width`, `wheelbase`, `track_width`,
  `wheel_radius`, `wheel_width`, `lidar_mount_x`, `camera_mount_x`, `camera_mount_z`,
  `camera_mount_pitch`). Drives wheel joint offsets, the Ackermann Gazebo plugin's
  `wheel_base`/`wheel_separation`/`wheel_radius`, and the `lidar_joint`/`camera_joint`
  origins.
- `platform/gazebo/runtime/robot_description/wro_robot.urdf` — a **separate, simplified,
  hand-maintained snapshot** (RViz/TF visualization only — no wheels/joints/steering), not
  generated from the xacro above. Its own `<box>`/`<cylinder>` sizes and joint `<origin>`s
  must be updated by hand in parallel. Its own `lidar_link` mesh uses a *different* radius
  (0.035) than the xacro's (0.0278) — a pre-existing inconsistency between the two files,
  not reconciled as part of this pass.

### Go Gazebo generator (hand-duplicated, does NOT import RobotSpecs)
`platform/gazebo/generator/internal/simconfig/constants.go` — the `Robot*` const block
(`RobotLength`, `RobotWidth`, `RobotHeight`, `RobotWheelbase`, `RobotTrackWidth`,
`RobotWheelRadius`, `RobotWheelWidth`, `RobotLidarMountXOffset`, `RobotCameraMountXOffset`,
`RobotCameraMountZOffset`, `RobotCameraPitchRad`). Consumed in
`platform/gazebo/generator/internal/sdf/robot.go` (chassis box/inertia, wheel joints, the
Ackermann plugin params, `buildCameraLink`/`buildLidarLink` poses) and transitively by
`internal/validate/validate.go` (spawn-clearance checks) and `internal/preview/svg.go`
(top-down preview rendering) — those two read the constants, not hardcode them, so they pick
up corrections automatically.

### Tests
`platform/robot/tests/test_constants.py`'s `ROBOT_CHASSIS_WIDTH`/`ROBOT_FOOTPRINT_RADIUS` —
another independent duplicate, not imported from `RobotSpecs`.

## Known follow-up (not fixed by this pass)

`src/navigation/planning/sign_router.py`'s `_detection_to_world()` — the pinhole projection
that turns a camera bounding box into a world position — implicitly assumes a **level**
camera (pitch = 0). With the camera's mount pitch now confirmed non-zero (~30° down), the
bbox-height → distance estimate has an unmodeled foreshortening bias for real camera
detections: a sign at a given real height projects to a shorter bbox than the level-camera
model expects, so the pinhole formula's distance estimate is systematically off by an amount
that grows with pitch and with how far the sign's true height differs from a
straight-ahead-projection assumption. Not fixed here — would require reworking
`_detection_to_world()`'s projection math, `src/simulation/vision_emulator.py`'s inverse
(the synthetic-detection emulator used in closed-loop sign-routing tests), and their tests.
Worth its own change once the camera's real x/z/pitch are confirmed rather than estimated.

## The actual structural fix (not done here, flagged for later)

The right long-term fix is generating the Go and xacro values from the Python `RobotSpecs`
source (or a shared JSON/YAML config all three read) instead of three independently
hand-maintained copies. This doc is the interim mitigation, not a replacement for that.
