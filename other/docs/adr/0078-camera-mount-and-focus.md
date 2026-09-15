# 0078. The camera is mounted above the LIDAR and its lens is parked on the decision range

- Status: accepted
- Date: 2026-09-15
- Supersedes: 0044

## Context

The camera must read signs on a 3 m track without a metric projection, and the
mount plus the lens focus decide what it can see. Autofocus hunts continuously
while the robot is driving. The camera is also physically mounted upside-down.

## Options considered

- (a) Calibrate the intrinsics and use the camera for distance; leave autofocus on.
- (b) No intrinsic calibration, LIDAR owns distance; manual focus parked on the
      decision range; correct the inverted orientation.

## Decision

(b). The camera is mounted directly above the LIDAR at the same forward offset
(x = 0.1222 m), 0.20 m above the floor, pitched about 10 degrees down. No factory
intrinsic calibration is done: the detector needs bearing (depth-independent) and
colour, and the LIDAR owns distance. The pinhole bbox-height range is a fallback
with about 3.6 cm error at 1.5 m and 14 cm at 3 m. Vision is not the collision
safety net (see 0072).

`camera_inverted = true`, because the camera is mounted upside-down; leaving it
false silently mirrored left and right, which corrupts the pass-side decision
inputs.

Autofocus is manual (`camera_af_mode = "manual"`), parked at `camera_lens_position
= 1.25` dioptres. Over 7149 boxes on four hardware rounds the signs were p10 0.19
/ p50 0.40 / p90 0.68 m, so the old 0.8 D (focus 1.25 m, in-focus from 0.63 m)
focused past nearly everything: 83.6 percent of detections were inside its near
limit. 1.25 D focuses 0.80 m and spans 0.485 to 2.29 m, chosen for the decision
range (first-usable detection 0.824 m down to router commitment 0.469 m).

## Consequences

- The camera reads bearing and colour; a missed detection does not remove
  collision safety.
- A measured confidence drop in the near band (before 0.83 to 0.90, after 0.767 to
  0.771) is attributed to lighting and the lens was deliberately not reverted.
- Known open limitation: the world projection still assumes a level camera while
  the mount is pitched 10 degrees down, an unmodeled foreshortening bias.
- 1.43 D centres the window better and is the value to try only after the dioptre
  scale is bench-verified, which it is not.

## History

- 62b59bf5 2026-07-11: first camera mount constants (z 0.16, pitch 30 deg).
- f6d53ac9 2026-07-30: consolidate mount offsets into `robot.toml`.
- e4de2f27 2026-08-28: enable `camera_inverted` for the upside-down mount.
- 5a97061a 2026-09-06: pin the camera focus, manual at 0.8 D (autofocus was
  hunting).
- 6d3e3328 2026-09-06: fix the mirrored bearing (see 0072).
- 6460e66e 2026-09-07: correct the mount to the measured pose (z 0.16 to 0.20,
  pitch 30 to 10 deg).
- b97a387c 2026-09-11: focus where the signs actually are (0.8 to 1.25 D).
- 52547a48 2026-09-11: record why the lens stays at 1.25 D despite the confidence
  drop.

## Refuted

- Continuous autofocus; the 0.8 D and 2.0 D lens positions; the 30 deg / 0.16 m
  mount estimates; `camera_inverted = false`; factory intrinsic calibration.

## Cross-references

- 0044 is superseded; its autofocus decision is carried above.
- 0072 owns the vision data path; 0080 owns the LIDAR mount the camera shares.
