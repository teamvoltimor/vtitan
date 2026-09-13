# Architecture Decision Records

One Markdown file per decision. These hold the *why*: the measurements, the
run ids, the options considered, and the premises that were later refuted.
The config values live in `src/config/`, their short descriptions live in the
JSON Schemas (`src/model/`), and each schema key points here through
`x-journal`.

## Why separate files

A single growing engineering log does not scale for per-value justification:
the log is a curated, regulation-shaped deliverable, and it would balloon.
Small per-decision files stay reviewable, blame-able, and conflict-free.

## Reference from a schema

```json
"collision_thickness": {
  "x-journal": "adr:0001-wall-collision-thickness"
}
```

- `adr:<stem>` resolves to `other/docs/adr/<stem>.md`.
- A plain string is a repo-root-relative path, optionally with `#anchor`.
- A list is allowed when genuinely independent decisions touched one value.
- One evolving story should be a supersede chain (see below), not a list.

`task config:check` fails if any reference does not resolve.

## Chain, do not pile up

When a later decision replaces an earlier one, the new ADR supersedes the old
and carries `Supersedes: 0001` in its header. The schema keeps only the current
ADR; the history is the chain.

## Naming and status

`NNNN-slug.md`, zero-padded, assigned in order. Status is `proposed`,
`accepted`, `superseded by NNNN`, or `deprecated`.

## Index

| ADR | Title | Status |
|---|---|---|
| [0001](0001-wall-collision-thickness.md) | Wall collision thickness matches the visual wall | accepted |
| [0002](0002-lidar-mount-forward-shift.md) | LIDAR collision geometry sits at the real mount | accepted |
| [0003](0003-track-constants-single-source.md) | Track constants live in one source | accepted |
| [0004](0004-corridor-division-lines-single-definition.md) | Corridor division lines are defined once | accepted |
| [0005](0005-pillar-displacement-tolerance.md) | Pillars are displaced, not scored as first contact | accepted |
| [0006](0006-parking-bay-scales-by-robot-length.md) | Parking bay length scales by the robot's length | accepted |
| [0007](0007-starting-zone-spawn-alignment.md) | Spawn hugs one band edge, never centred | accepted |
| [0008](0008-robot-constants-single-source.md) | Robot constants live in one source and are read at runtime | accepted |
| [0009](0009-chassis-mass-battery-middle-value.md) | Chassis mass is the middle of the battery configurations | accepted |
| [0010](0010-wheel-angle-derived-from-steering.md) | Road-wheel angle is derived from the steering hardware | accepted |
| [0011](0011-hardware-profile-fields-required.md) | Swappable servo and motor limits live in the hardware profile | accepted |
| [0012](0012-yaw-gain-measured.md) | Yaw gain is a measured fraction of the kinematic model | accepted |
| [0013](0013-turn-radius-speed-curve.md) | The turn-radius floor is speed-dependent | accepted |
| [0014](0014-lidar-scan-plane-recessed.md) | The LIDAR scan plane is recessed, 0.08 m off the floor | accepted |
| [0015](0015-lidar-inverted-mount.md) | The LIDAR is mounted upside-down | accepted |
| [0016](0016-sensor-specs-consolidated.md) | Sensor specs are consolidated into robot.toml | accepted |
| [0017](0017-ros-topics-centralized.md) | ROS2 topic names are centralized | accepted |
| [0018](0018-competition-specs-single-source.md) | Competition round rules live in config, not Python literals | accepted |
| [0019](0019-simulation-robot-model-topic-split.md) | Wheel geometry is split off the simulated track topic | accepted |
| [0020](0020-config-loaded-at-runtime.md) | Config is read at runtime from TOML, not generated into consumers | accepted |
| [0021](0021-hardware-profiles-stackable-overlays.md) | Hardware profiles are stackable partial overlays | accepted |
| [0022](0022-steering-physics-vs-policy.md) | Steering physics and steering policy are separate fields | accepted |
| [0023](0023-escape-durations-in-seconds.md) | Escape durations are stored in seconds, not frames | accepted |
| [0024](0024-localization-jump-guard-tracks-drivetrain.md) | The localization jump guard tracks the drivetrain ceiling | accepted |
| [0025](0025-global-relocalization.md) | A lost pose is recovered by global relocalization | accepted |
| [0026](0026-corridor-follower-no-centering.md) | The corridor follower does not centre | accepted |
| [0027](0027-corner-steer-cap-commit-distance.md) | The corner-turn steering cap scales with commit distance | accepted |
| [0028](0028-waypoint-center-bias-split.md) | Waypoint centre bias is split by corridor class | accepted |
| [0029](0029-open-challenge-lookahead.md) | The Open Challenge uses its own lookahead | accepted |
| [0030](0030-servo-slew-rate-split.md) | The servo slew rate is split from the steering-rate policy | accepted |
| [0031](0031-clearance-thresholds-distance-not-time.md) | Clearance thresholds are distance, not time | proposed |
| [0032](0032-heading-single-crawl-threshold.md) | Heading correction is a single crawl threshold | accepted |
| [0033](0033-per-challenge-speed-ladders.md) | The per-challenge speed ladders are a deliberate split | accepted |
| [0034](0034-wrong-side-pass-ground-truth.md) | Wrong-side passes are scored from ground truth | accepted |
| [0035](0035-obstacles-pass-side-tracking.md) | The Obstacles pass-side deficit is tracking, not routing | accepted |
| [0036](0036-no-parking-after-final-lap.md) | Parking is not pursued after the final lap | accepted |
| [0037](0037-parking-lot-from-in-bay-start.md) | The parking lot is derived from the in-bay start | accepted |
| [0038](0038-bay-exit-mirror-reverse-steer.md) | The bay exit mirrors its reverse-leg steering | accepted |
| [0039](0039-obstacles-contact-zone.md) | The Obstacles contact zone is per-challenge | accepted |
| [0040](0040-rear-self-detection-chassis-geometry.md) | The rear self-detection filter follows chassis geometry | accepted |
| [0041](0041-lidar-valid-range-floor.md) | The LIDAR valid-range floor sits below the rated minimum | accepted |
| [0042](0042-sign-range-fusion-gated.md) | Sign range fusion is gated and paired with cluster detection | accepted |
| [0043](0043-obstacles-inner-wall-scoring.md) | Obstacles inner-wall contact scoring is configurable | accepted |
| [0044](0044-camera-autofocus-dioptre.md) | The camera autofocus is set to 1.25 dioptres | accepted |
| [0045](0045-sign-router-commit-hysteresis.md) | The sign router holds its committed sign | accepted |
| [0046](0046-repo-layout-src-apps.md) | The repository layout moves code into src/ and apps/ | accepted |
| [0047](0047-contact-reverse-ships-disabled.md) | The contact reverse ships disabled | accepted |
| [0048](0048-risk-ray-window-noise-statistics.md) | The forward lane corroborates a short return across adjacent rays | accepted |
