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
and carries `Supersedes: 0001` in its header, while the old ADR's status becomes
`superseded by NNNN`. The schema keeps only the current ADR; the history is the
chain. When one variable was touched many times, the current ADR tells the whole
story in its `## History` section rather than spawning a micro-ADR per attempt.

## Naming and status

`NNNN-slug.md`, zero-padded, assigned in order. Status is `proposed`,
`accepted`, `superseded by NNNN`, or `deprecated`.

## Index

| ADR | Title | Status |
|---|---|---|
| [0001](0001-wall-collision-thickness.md) | Wall collision thickness matches the visual wall | superseded by 0062 |
| [0002](0002-lidar-mount-forward-shift.md) | LIDAR collision geometry sits at the real mount | superseded by 0080 |
| [0003](0003-track-constants-single-source.md) | Track constants live in one source | superseded by 0069 |
| [0004](0004-corridor-division-lines-single-definition.md) | Corridor division lines are defined once | superseded by 0069 |
| [0005](0005-pillar-displacement-tolerance.md) | Pillars are displaced, not scored as first contact | superseded by 0062 |
| [0006](0006-parking-bay-scales-by-robot-length.md) | Parking bay length scales by the robot's length | superseded by 0062 |
| [0007](0007-starting-zone-spawn-alignment.md) | Spawn hugs one band edge, never centred | accepted |
| [0008](0008-robot-constants-single-source.md) | Robot constants live in one source and are read at runtime | superseded by 0069 |
| [0009](0009-chassis-mass-battery-middle-value.md) | Chassis mass is the middle of the battery configurations | superseded by 0077 |
| [0010](0010-wheel-angle-derived-from-steering.md) | Road-wheel angle is derived from the steering hardware | superseded by 0076 |
| [0011](0011-hardware-profile-fields-required.md) | Swappable servo and motor limits live in the hardware profile | superseded by 0070 |
| [0012](0012-yaw-gain-measured.md) | Yaw gain is a measured fraction of the kinematic model | superseded by 0086 |
| [0013](0013-turn-radius-speed-curve.md) | The turn-radius floor is speed-dependent | superseded by 0086 |
| [0014](0014-lidar-scan-plane-recessed.md) | The LIDAR scan plane is recessed, 0.08 m off the floor | superseded by 0080 |
| [0015](0015-lidar-inverted-mount.md) | The LIDAR is mounted upside-down | superseded by 0080 |
| [0016](0016-sensor-specs-consolidated.md) | Sensor specs are consolidated into robot.toml | superseded by 0069 |
| [0017](0017-ros-topics-centralized.md) | ROS2 topic names are centralized | superseded by 0069 |
| [0018](0018-competition-specs-single-source.md) | Competition round rules live in config, not Python literals | superseded by 0069 |
| [0019](0019-simulation-robot-model-topic-split.md) | Wheel geometry is split off the simulated track topic | accepted |
| [0020](0020-config-loaded-at-runtime.md) | Config is read at runtime from TOML, not generated into consumers | superseded by 0069 |
| [0021](0021-hardware-profiles-stackable-overlays.md) | Hardware profiles are stackable partial overlays | superseded by 0070 |
| [0022](0022-steering-physics-vs-policy.md) | Steering physics and steering policy are separate fields | superseded by 0050 |
| [0023](0023-escape-durations-in-seconds.md) | Escape durations are stored in seconds, not frames | superseded by 0055 |
| [0024](0024-localization-jump-guard-tracks-drivetrain.md) | The localization jump guard tracks the drivetrain ceiling | superseded by 0084 |
| [0025](0025-global-relocalization.md) | A lost pose is recovered by global relocalization | superseded by 0084 |
| [0026](0026-corridor-follower-no-centering.md) | The corridor follower does not centre | superseded by 0057 |
| [0027](0027-corner-steer-cap-commit-distance.md) | The corner-turn steering cap scales with commit distance | superseded by 0049 |
| [0028](0028-waypoint-center-bias-split.md) | Waypoint centre bias is split by corridor class | superseded by 0057 |
| [0029](0029-open-challenge-lookahead.md) | The Open Challenge uses its own lookahead | superseded by 0052 |
| [0030](0030-servo-slew-rate-split.md) | The servo slew rate is split from the steering-rate policy | superseded by 0076 |
| [0031](0031-clearance-thresholds-distance-not-time.md) | Clearance thresholds are distance, not time | superseded by 0085 |
| [0032](0032-heading-single-crawl-threshold.md) | Heading correction is a single crawl threshold | superseded by 0085 |
| [0033](0033-per-challenge-speed-ladders.md) | The per-challenge speed ladders are a deliberate split | superseded by 0085 |
| [0034](0034-wrong-side-pass-ground-truth.md) | Wrong-side passes are scored from ground truth | superseded by 0059 |
| [0035](0035-obstacles-pass-side-tracking.md) | The Obstacles pass-side deficit is tracking, not routing | superseded by 0059 |
| [0036](0036-no-parking-after-final-lap.md) | Parking is not pursued after the final lap | superseded by 0062 |
| [0037](0037-parking-lot-from-in-bay-start.md) | The parking lot is derived from the in-bay start | superseded by 0062 |
| [0038](0038-bay-exit-mirror-reverse-steer.md) | The bay exit mirrors its reverse-leg steering | superseded by 0060 |
| [0039](0039-obstacles-contact-zone.md) | The Obstacles contact zone is per-challenge | superseded by 0061 |
| [0040](0040-rear-self-detection-chassis-geometry.md) | The rear self-detection filter follows chassis geometry | superseded by 0056 |
| [0041](0041-lidar-valid-range-floor.md) | The LIDAR valid-range floor sits below the rated minimum | superseded by 0056 |
| [0042](0042-sign-range-fusion-gated.md) | Sign range fusion is gated and paired with cluster detection | superseded by 0058 |
| [0043](0043-obstacles-inner-wall-scoring.md) | Obstacles inner-wall contact scoring is configurable | superseded by 0059 |
| [0044](0044-camera-autofocus-dioptre.md) | The camera autofocus is set to 1.25 dioptres | superseded by 0078 |
| [0045](0045-sign-router-commit-hysteresis.md) | The sign router holds its committed sign | superseded by 0051 |
| [0046](0046-repo-layout-src-apps.md) | The repository layout moves code into src/ and apps/ | accepted |
| [0047](0047-contact-reverse-ships-disabled.md) | The contact reverse ships disabled | superseded by 0088 |
| [0048](0048-risk-ray-window-noise-statistics.md) | The forward lane corroborates a short return across adjacent rays | superseded by 0056 |
| [0049](0049-corner-arcs-per-corridor-and-commit-distance.md) | Corner arcs are sized per corridor and the steering cap from the commit distance | accepted |
| [0050](0050-escape-steering-degrees-and-committed-side.md) | Escape steering is physical road-wheel degrees and follows the committed pass side | accepted |
| [0051](0051-sign-lane-planner.md) | Sign avoidance rewrites the path into a lane | accepted |
| [0052](0052-pursuit-target-selection.md) | Pursuit selects its target by arc length and path sense, and arms the lookahead on the corner ahead | accepted |
| [0053](0053-direction-inference-and-start-pose.md) | Direction is inferred from the corridor span and the start pose from the scan | accepted |
| [0054](0054-absolute-heading-from-walls.md) | Absolute heading is read off the Manhattan walls, mod 90 | accepted |
| [0055](0055-escape-maneuver-selection.md) | The escape manoeuvre fails closed on an unread rear, pivots when wedged, and re-anchors at its end | accepted |
| [0056](0056-raw-and-masked-scan.md) | The scan is read twice, raw for speed and masked for the escape | accepted |
| [0057](0057-blind-corridor-follower-and-width.md) | The blind corridor follower does not centre, and width is classified not measured | accepted |
| [0058](0058-sign-discovery-range-and-barrier-belief.md) | Sign discovery takes the closest observation and bounds ingest by range | accepted |
| [0059](0059-pass-side-travel-relative-and-scorer-independence.md) | The pass-side rule is travel-relative, and a scorer must not share the scored system's convention | accepted |
| [0060](0060-bay-exit-clearance-guard.md) | The bay exit is bounded by predicted fin clearance, not by contact | accepted |
| [0061](0061-contact-zone-per-challenge.md) | The contact zone is per challenge | accepted |
| [0062](0062-sim-contact-model-and-parking.md) | The simulator scores the rulebook contact model, and parking is a RACING phase | accepted |
| [0063](0063-corridor-flip-and-sense-guards.md) | The corridor flip and the sense guards are temporal, not geometric | accepted |
| [0064](0064-corridor-by-depth-and-clearance-budget.md) | A corner sign's corridor is decided by depth, and the clearance budget is thin | accepted |
| [0065](0065-fastdds-udp-only.md) | FastDDS shared memory is disabled and transport is UDP only | accepted |
| [0066](0066-two-board-compute-split.md) | Compute is split Pi 5 perception and Pi Zero real-time control | accepted |
| [0067](0067-usb-gadget-link-and-provisioning.md) | The board link is an NM-only USB-gadget subnet and the Zero receives built tarballs | accepted |
| [0068](0068-go-parallel-track-single-cutover.md) | The Go stack is a parallel track with a single cutover | accepted |
| [0069](0069-config-governance.md) | Config values live in TOML, descriptions in schemas, rationale in ADRs | accepted |
| [0070](0070-hardware-profiles-and-challenge-overlays.md) | Hardware profiles are per-component overlays and a wrong name fails loud | accepted |
| [0071](0071-round-recording-mcap.md) | Every round is recorded as an MCAP bag and analyzed offline | accepted |
| [0072](0072-vision-data-path.md) | The camera feeds the NPU in-process and vision owns identity, not distance | accepted |
| [0073](0073-challenge-mode-jumper-and-runtime.md) | The challenge is read from a boot jumper and resolved at runtime | accepted |
| [0074](0074-control-loop-rate-single-source.md) | The control loop rate has a single source | accepted |
| [0075](0075-counter-phase-four-wheel-steering.md) | Four-wheel counter-phase steering replaces Ackermann | deprecated (merged into 0076) |
| [0076](0076-drivetrain-and-steering-hardware.md) | The drivetrain is sized on measured current, and calibration is never inherited | accepted |
| [0077](0077-power-rails-and-chassis-mass.md) | Power rails are sized to measured load and the chassis mass is the battery midpoint | accepted |
| [0078](0078-camera-mount-and-focus.md) | The camera is mounted above the LIDAR and its lens is parked on the decision range | accepted |
| [0079](0079-imu-6axis-and-yaw-reference.md) | The IMU runs 6-axis and its reference is zeroed at the start button | accepted |
| [0080](0080-lidar-mount-and-scan-plane.md) | The LIDAR is modelled at its real mount, inverted, with a recessed scan plane | accepted |
| [0081](0081-step-stl-parts-and-naming.md) | Every 3D part is published as STEP and STL with kebab-case names | accepted |
| [0082](0082-wiring-harness-as-code.md) | The wiring harness is defined as code | accepted |
| [0083](0083-oled-backend-and-button-thresholds.md) | The OLED has a raw-I2C fallback and the button thresholds escalate | accepted |
| [0084](0084-localizer-divergence-and-relocalization.md) | Pose divergence is detected off-track and recovered by armed global relocalization | accepted |
| [0085](0085-speed-envelope.md) | The speed envelope is absolute m/s with the drivetrain as a clamp | accepted |
| [0086](0086-simulator-realism.md) | The simulator runs on a measured error budget, not a perfect world | accepted |
| [0087](0087-test-methodology.md) | Testing uses a four-level ladder and a fixed A/B protocol | accepted |
| [0088](0088-refuted-config-knobs.md) | Refuted config knobs ship off and stay documented | accepted |
| [0089](0089-robot-constants-runtime-and-xacro.md) | Robot constants are read at runtime and the xacro is the one hand-synced copy | deprecated (merged into 0069) |
| [0090](0090-commit-and-constant-conventions.md) | Commit messages and constant access follow fixed conventions | accepted |
| [0091](0091-dataset-frame-capture.md) | Dataset frames are captured periodically beside the run artifacts | accepted |
| [0092](0092-escape-does-not-retire-committed-sign.md) | An escape does not retire the committed sign, and ships off | accepted |
