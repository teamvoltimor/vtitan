# 0015. The LIDAR is mounted upside-down

- Status: superseded by 0080
- Superseded by: 0080
- Date: 2026-08-02
- Commit: 41815bcc

## Context

The chassis is physically mounted upside-down, which is why an "inverted"
correction exists at all. Confirmed 2026-08-02: the mount IS upside-down, so
`inverted = true`. An earlier same-day measurement had found `false` matching
behaviour, but that was testing the wrong data path (raw `/scan` directly, which
the navigator never reads; see `ros2_hardware_gateway.py`); re-confirmed against
the physical mount fact itself, it is true.

## Options considered

- (a) Treat the driver's `inverted` parameter and the 180 deg yaw correction as
      two independent settings.
- (b) Derive both from the single `inverted` flag.

## Decision

(b). Every consumer that needs the LIDAR's angle correction derives it from
`inverted` alone: the `sllidar_ros2` driver's own `inverted` launch parameter (its
left-right mirror) AND a mandatory 180 deg yaw rotation applied wherever raw
`/scan` angles are consumed, because front/back come attached to the same
upside-down-mount fact as left/right, not a separate one. A driver-level
`inverted:=true` with no matching 180 deg yaw application (or vice versa) is
exactly the bug found and fixed: the two drifted out of sync when only one was
re-verified.

`mount_yaw_offset_deg` is independent of `inverted`: any additional yaw
miscalibration NOT explained by the upside-down mount (for example the unit sits
rotated a few degrees off dead-ahead within its bracket). It adds to the 180 deg
from `inverted`, it does not replace it, and it is zero until something measures
otherwise.

## Consequences

- One flag cannot drift from the other.
- Re-measure after any remount or cable work rather than assuming.

## Superseded by 0080

Replaced by [0080](0080-lidar-mount-and-scan-plane.md): The LIDAR is modelled at its
real mount, inverted, with a recessed scan plane.

The successor carries the current decision and its rationale; this file keeps
the original decision above so the supersede chain stays readable.
