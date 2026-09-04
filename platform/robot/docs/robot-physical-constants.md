# Robot Physical Constants — Canonical Values

**Single source of truth (2026-07-11 fix):** `platform/shared/config/robot.toml`. It used to
be hand-duplicated across Python, Go, and XML/xacro, and had already drifted out of sync once
(chassis `LENGTH`/`WIDTH` were corrected from 0.28×0.15 to 0.30×0.20 in Python but never updated
in the Go Gazebo generator or the xacro — both silently kept using the old numbers for months).

**To change a measurement:** edit `platform/shared/config/robot.toml`, then run
`task gen:robot-constants` (wraps `simgen generate-robot-constants`, see
`platform/robot-go/internal/simgen/robotconfig`). This regenerates the two Go/xacro consumers
below — do not hand-edit either, they're marked `DO NOT EDIT` and will be silently overwritten:

- `platform/robot-go/internal/simgen/simconfig/robot_constants.gen.go` (Go)
- `platform/gazebo/runtime/robot_description/robot_properties.gen.xacro` (`xacro:include`d from
  `wro_robot.urdf.xacro`)

Python has no generated file to regenerate: `RobotSpecs` in
`platform/shared/src/shared/config/constants.py` sources its values from
`shared.config.robot_constants.RobotConstants`, which reads `robot.toml` directly at runtime.

The rest of this doc records the canonical values and remaining known gaps (a second
hand-maintained `wro_robot.urdf` snapshot, and the sign-router pitch follow-up) — read on for
those, but the "update every file by hand" duplication problem itself is fixed.

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
| LIDAR mount x-offset (front of chassis, centered) | 0.1222 m | derived: `chassis_length/2 − lidar mesh radius (0.0278)` — the C1 mounted flush with the front edge. |
| LIDAR inverted (upside-down mount) | `true` | `robot.toml`'s `lidar.inverted` — the single source of truth for the upside-down mount. Drives BOTH the `sllidar_ros2` driver's own `inverted` launch parameter (its left-right mirror, read via `pixi.toml`'s `run-lidar` and `lidar_launch.py`) AND a mandatory 180° yaw rotation applied wherever raw `/scan` angles are consumed (`ros2_hardware_gateway.py`, `static_tfs.launch.py`). Confirmed 2026-08-02 against the physical mount fact itself (it is upside-down). An earlier same-day reading of `false` tested the wrong data path (raw `/scan` directly, which the navigator never reads). Letting the driver flag and the 180° rotation drift out of sync (each re-verified independently, at different times) was a real deployed bug, fixed 2026-08-02. |
| LIDAR mount yaw offset (residual) | 0° | `robot.toml`'s `lidar.mount_yaw_offset_deg` — any additional yaw miscalibration NOT explained by `inverted`'s 180°, added on top of it. Re-verify against a known object at chassis front/back after any remount or cable work. |
| Camera mount x-offset | 0.1222 m (same as LIDAR — mounted directly over it) | **estimate**, not measured |
| Camera mount z-offset | 0.16 m | **estimate**, not measured |
| Camera mount pitch | ~30° down | Magnitude is still an **estimate** ("like 30 degrees"), not precision-measured. Sign convention (positive = down) is confirmed correct — both by a rotation-matrix derivation (`R = Rz(yaw)·Ry(pitch)·Rx(roll)`, standard REP-103/tf2 convention) and visually, via the yellow direction-arrow marker added to `src/simulation/live_visualizer.py` and checked live in RViz 2026-07-11. |

## Camera sensor

Read off the device with `rpicam-hello --list-cameras` (2026-07-26), rather than
from the datasheet, so it reflects what is actually fitted:

| Property | Value |
|---|---|
| Sensor | `imx708_wide` — Camera Module 3 **Wide** |
| Full resolution | 4608×2592, 10-bit RGGB |
| Mode used by the detector | 1536×864 @ 120.13 fps (crop `(768,432)/3072×1728`) |
| Other modes | 2304×1296 @ 56.03 fps · 4608×2592 @ 14.35 fps |
| Horizontal FOV | 102° (Module 3 Wide spec; not measured on this build) |

The detector's input is 640×640 letterboxed from whatever frame it is given, so
capture resolution trades field detail against frame rate rather than changing
what the model sees.

## Where each constant lives

### Source of truth
`platform/shared/config/robot.toml` — chassis, Ackermann geometry, wheel, LIDAR mount offset,
camera mount offset/pitch. Edit this, then run `task gen:robot-constants`.

### Generated (do not hand-edit)
- `platform/robot-go/internal/simgen/simconfig/robot_constants.gen.go` — the `Robot*` const
  block, consumed by `platform/robot-go/internal/simgen/sdf/robot.go` (chassis box/inertia,
  wheel joints, Ackermann plugin params, `buildCameraLink`/`buildLidarLink` poses) and
  transitively by `internal/validate/validate.go` and `internal/preview/svg.go`.
- `platform/gazebo/runtime/robot_description/robot_properties.gen.xacro` — `xacro:include`d
  from `wro_robot.urdf.xacro`, which still hand-defines wheel joints, links, and sensors below
  it using the included properties (`chassis_length`, `wheelbase`, `lidar_mount_x`, etc.).

Python has no generated file here: `shared.config.robot_constants.RobotConstants` reads
`robot.toml` directly at runtime (no codegen step), and `RobotSpecs` in
`platform/shared/src/shared/config/constants.py` sources its fields (`LENGTH`, `WIDTH`,
`HEIGHT`, `WHEELBASE`, `TRACK_WIDTH`, `WHEEL_RADIUS`, `WHEEL_WIDTH`, `LIDAR_MOUNT_X_OFFSET`,
`LIDAR_INVERTED`, `LIDAR_MOUNT_YAW_OFFSET_DEG`, `CAMERA_MOUNT_X_OFFSET`, `CAMERA_MOUNT_Z_OFFSET`,
`CAMERA_MOUNT_PITCH_DEG`) from it. Every other Python consumer (navigation, simulation, ROS2
nodes — e.g. `static_tfs.launch.py`, `live_visualizer.py`) still imports `RobotSpecs`, unchanged.

### Not yet generated (pre-existing gap, unrelated to the drift this doc used to describe)
- `platform/gazebo/runtime/robot_description/wro_robot.urdf` — a **separate, simplified,
  hand-maintained snapshot** (RViz/TF visualization only — no wheels/joints/steering), not
  generated from the xacro above. Its own `<box>`/`<cylinder>` sizes and joint `<origin>`s are
  still updated by hand, so any future measurement change must be applied here manually too.
  Its `lidar_link` mesh radius/length (previously a stale 0.035/0.040, vs. the xacro's
  0.0278/0.0413) was corrected by hand to match on 2026-07-12 — re-check it against `robot.toml`
  after any future LIDAR mount change.
- `LIDAR_DIAMETER`/`LIDAR_HEIGHT` in `RobotSpecs` are not in `robot.toml` — they're only used
  to derive `LIDAR_MOUNT_X_OFFSET` in a comment, not independently duplicated elsewhere.

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

Done (2026-07-11): `platform/shared/config/robot.toml` is now the shared source all three read,
via `task gen:robot-constants` (see "Where each constant lives" above).
