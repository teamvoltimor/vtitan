# Robot Physical Constants - Canonical Values

Source of truth: `src/config/robot.toml` (schema `src/model/robot.schema.json`), read
at runtime by both the Python and Go stacks. Units are metres, kilograms and radians
unless a field is named `_deg`.

Rationale lives in the ADRs: `adr:0069-config-governance` (runtime reads and the
hand-synced xacro), `adr:0076-drivetrain-and-steering-hardware`
(drivetrain), `adr:0080-lidar-mount-and-scan-plane` (LIDAR mount and scan plane),
`adr:0078-camera-mount-and-focus` (camera mount and focus).

## Canonical values

| Constant | Value | Notes |
|---|---|---|
| Chassis length | 0.30 m | |
| Chassis width | 0.194 m | |
| Chassis height | 0.10 m | |
| Wheelbase (front axle to rear axle) | 0.19 m | |
| Track width (left wheel to right wheel) | 0.1675 m | |
| Wheel diameter | 0.07 m (radius 0.035 m) | |
| Wheel width | 0.025 m | |
| LIDAR mount x-offset | 0.1222 m | derived: `chassis.length/2 - mesh radius (0.0278)`; flush with the front edge |
| LIDAR mount z-offset | -0.02 m | relative to chassis height; scan plane at about 0.08 m |
| LIDAR inverted (upside-down mount) | `true` | `lidar.inverted`; every angle consumer derives from it |
| LIDAR mount yaw offset (residual) | 0 deg | `lidar.mount_yaw_offset_deg`; re-verify after any remount or cable work |
| Camera mount x-offset | 0.1222 m | same as the LIDAR, mounted directly above it |
| Camera mount z-offset | 0.20 m | |
| Camera mount pitch | 10 deg down | `camera.mount_pitch = 0.1745 rad` |

## Camera sensor

Read off the device with `rpicam-hello --list-cameras` (2026-07-26), rather than from
the datasheet, so it reflects what is actually fitted:

| Property | Value |
|---|---|
| Sensor | `imx708_wide` - Camera Module 3 Wide |
| Full resolution | 4608x2592, 10-bit RGGB |
| Mode used by the detector | 1536x864 @ 120.13 fps (crop `(768,432)/3072x1728`) |
| Other modes | 2304x1296 @ 56.03 fps, 4608x2592 @ 14.35 fps |
| Horizontal FOV | 102 deg (Module 3 Wide spec; not measured on this build) |

The detector's input is 640x640 letterboxed from whatever frame it is given, so
capture resolution trades field detail against frame rate rather than changing what the
model sees. The world projection still assumes a level camera, so the confirmed ~10 deg
mount pitch leaves an unmodeled foreshortening bias in the pinhole bbox-height range
(owned by `adr:0078-camera-mount-and-focus`).
