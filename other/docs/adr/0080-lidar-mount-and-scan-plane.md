# 0080. The LIDAR is modelled at its real mount, inverted, with a recessed scan plane

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0002, 0014, 0015

## Context

The simulated LIDAR was modelled at the chassis centre and with an inflated wall,
so every corridor behaved 8 cm narrower than spec and parking was arithmetically
impossible. The physical mount is flush with the front edge, the unit is upside-
down, and the beam plane sits below the 0.10 m object height so it crosses signs.

## Options considered

- (a) Keep the sensor at the chassis centre; inflate the wall to compensate; trust
      a rotation for the inverted mount.
- (b) Model the LIDAR at its real mount, derive the angle, and place the scan plane
      where it physically is.

## Decision

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

Driver protocol (hardware 2026-08-31): a RESET reboots the C1 core, so the scan
request must wait about 1 s and under 2 ms it is silently ignored. Express Scan is
ignored until the motor actually spins, needing an about 800 ms spin-up; the HQ
motor command 0xA8 at 600 RPM is what spins it, while the 0xF0 PWM command left it
unresponsive. The Dense stream emits one scan then pauses about 2.1 s, so the 4 s
read timeout is set above that gap. The stream sets S=true exactly once, so scan end
is detected from the per-packet start-angle wrap rather than a second S flag; a scan
closed on the first wrap drop returns only 18-20 of about 300 points near the sweep
end. One corrupt Dense byte fails its checksum and misaligns every later fixed-size
read, so `readDensePacket` resyncs on the 0xA? 0x5? nibble pair. Classic mode's
angle decode saw only a coarse front-placement check and its range decode was never
per-bearing verified on this C1, which is why Dense is preferred.

## Consequences

- Forward ranges are no longer about 12 cm long, and clearance zones are honest.
- Pass rates before and after the mount fix are not comparable.
- The 0.08 m beam crosses the 0.10 m signs.
- Open documentation inconsistency: later hardware evidence shows the inverted
  correction is a MIRROR (negation), not a 180 degree rotation; the ADR's rotation
  model and `robot-physical-constants.md` still say rotation, while the README
  reports the mirror finding. Reconcile to the mirror.

## History

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

## Refuted

- Collision inflation (0.18 m / 40 mm); the +0.02 sign that gave a 0.12 m plane;
  the claim the LIDAR cannot see 0.10 m signs; the 180 degree rotation for the
  inverted mount; classic SCAN range decode; the hardcoded 84-byte dense packet.

## Cross-references

- 0002, 0014 and 0015 are superseded; their decisions are carried above.
- 0062 owns the wall collision thickness and parking model; 0078 owns the camera
  that sits on this mount.

## Evidence

- `robot.toml`, `static_tfs`, the URDF and the Go `simconfig` all said the beam
  was 12 cm and all four were wrong: four independent copies of one wrong number.
  The config-completeness test now exists to prevent that.
- The mount fix (`6c727c87`) moving the sensor 12.2 cm silently redefined every
  threshold, took the front gate from unreachable to live, and pushed the rear
  gate further out of reach.
- A threshold compared against a raw LIDAR range must first be converted to a
  bumper-referenced gap from `LIDAR_MOUNT_X_OFFSET` and the chassis half-length;
  do NOT retune thresholds to absorb a mount offset.
- `nav_debug.min_lidar_range_m` bottoms near 0.006 m on every run because the
  gateway leaves invalid near-zero returns in the scan; read `/scan` directly with
  the gateway's own mount correction.
- The localizer's candidate search must predict from the sensor mount, not the
  chassis centre: the C1 sits `lidar.mount_x_offset` forward of centre, flush with
  the bumper, so a scan taken there cannot be reproduced by casting from the
  centre. Until 2026-08-21 it cast from the centre, biasing every forward ray by
  the offset and pulling the fit along the corridor axis; the simulator raycast
  from the centre too, so the two agreed and the error was invisible in sim while
  present on hardware.
- The Go gateway repeated the Python mount bug: it cast rays from the body centre
  until 2026-09-06, biasing every return by the 12.2 cm mount offset, before moving
  to the 0.1222 m mount.
