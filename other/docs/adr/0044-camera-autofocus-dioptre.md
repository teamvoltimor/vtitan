# 0044. The camera autofocus is set to 1.25 dioptres

- Status: superseded by 0078
- Superseded by: 0078
- Date: 2026-09-11

## Context

The lens shipped at 0.8 D (focus 1.25 m, in-focus from 0.63 m). MEASURED over
7149 red/green boxes on four hardware rounds, range from the 0.10 m sign height
and a 621.9 px focal: p10 0.19 / p50 0.40 / p90 0.68 m, with 83.6 percent INSIDE
the old near limit and 99.9 percent inside 1.0 m. The lens was focused past
nearly everything it actually sees.

Chosen for the DECISION range, not the median box: the median is dominated by
frames of a sign already being passed, whose verdict was settled earlier. What
matters is first-usable-detection 0.824 m down to router commitment 0.469 m, and
0.485 m sits right on that lower edge. 1.25 D focuses 0.80 m and spans
0.485-2.29 m. 1.43 D centres the window better and is the value to try once the
dioptre scale is bench-verified, which it is still not on either driver.

Measured after the change and NOT reverted, deliberately: on the two rounds that
followed, confidence in the 0.0-0.5 m band fell with clean run-level separation
(before 0.831/0.842/0.862/0.869/0.902, after 0.767/0.771, no overlap, and the
drop survives inside every yaw bin). But the rounds differ in time and the
LIGHTING CHANGED, and one round that evening was discarded outright for poor
lighting. Operator's call 2026-09-11: the drop is attributable to light, keep
1.25 D. The optics agree: at 1.25 D the near in-focus limit moves 0.63 to
0.485 m, so this band should improve or hold, never worsen.

## Options considered

- (a) Keep 0.8 D.
- (b) 1.25 D.
- (c) 1.43 D.

## Decision

(b). What would settle the confidence question is a matched-lighting A/B, or the
bench check of the dioptre scale this file has always said is unverified.

## Consequences

- The near limit moves inside the decision range instead of starting past it.
- The confidence dip measured after the change is attributable to lighting on
  the operator's call, not to the optics.

## Superseded by 0078

This decision was replaced by [0078](0078-camera-mount-and-focus.md). Its content is reproduced below so this file stays self-contained; edit only the successor.

### Context

The camera must read signs on a 3 m track without a metric projection, and the
mount plus the lens focus decide what it can see. Autofocus hunts continuously
while the robot is driving. The camera is also physically mounted upside-down.

### Options considered

- (a) Calibrate the intrinsics and use the camera for distance; leave autofocus on.
- (b) No intrinsic calibration, LIDAR owns distance; manual focus parked on the
      decision range; correct the inverted orientation.

### Decision

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

### Consequences

- The camera reads bearing and colour; a missed detection does not remove
  collision safety.
- A measured confidence drop in the near band (before 0.83 to 0.90, after 0.767 to
  0.771) is attributed to lighting and the lens was deliberately not reverted.
- Known open limitation: the world projection still assumes a level camera while
  the mount is pitched 10 degrees down, an unmodeled foreshortening bias.
- 1.43 D centres the window better and is the value to try only after the dioptre
  scale is bench-verified, which it is not.

### History

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

### Refuted

- Continuous autofocus; the 0.8 D and 2.0 D lens positions; the 30 deg / 0.16 m
  mount estimates; `camera_inverted = false`; factory intrinsic calibration.

### Cross-references

- 0044 is superseded; its autofocus decision is carried above.
- 0072 owns the vision data path; 0080 owns the LIDAR mount the camera shares.

