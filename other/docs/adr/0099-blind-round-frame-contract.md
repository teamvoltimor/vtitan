# 0099. A blind round runs in one believed frame, built by one component on the robot and in the sim

- Status: proposed
- Date: 2026-09-25

## Context

Platform plan item 2.9 found that blind Obstacles runs in the Go sim count 0
laps in 256 of 256 scenarios while driving and discovering signs. Two
defects were confirmed (plan section 14), and the obvious fix was refuted:

1. `harness.SimHardwareGateway.CorrectHeadingForDirectionChange` rotates the
   simulated chassis (`state.Yaw += delta`). The real gateway (`natsgw`) adds
   the same delta to its heading offset. A North start is physically spun
   180 degrees when the robot settles its direction.
2. `Navigator.adoptDirection` measures the start and builds the
   `racetracker.LapDetector` in the canonical South frame
   (`trackmodel.South` hardcoded, ADR 0053), while the sim reports the pose
   in the true frame. The detector's section gate and finish line rarely
   line up with where the robot actually crosses.
3. Refuted: making the sim rotate the pose it reports instead of the body.
   Blind Open successes went 41 -> 26, Obstacles wrong-side passes 8 -> 67,
   still 0 Obstacles laps. Other blind consumers depend on today's mixed
   frames.

Traced on 2026-09-25, the three stacks do not share a frame contract at all:

| At | Python sim and ROS node | Go on the robot (`natsgw`) | Go sim (`harness`) |
|---|---|---|---|
| t = 0 | Whole belief in the canonical South-start frame: heading `imu + start_yaw`, estimator seeded at the assumed start, believed walls, plan, lap line, park controller | Heading in the IMU power-on frame (`ResetHeadingReference` has no production caller, against ADR 0079); position scan-matched from (0, 0) against the South-frame prior walls | True world frame: true state, or a localizer on the TRUE walls seeded at the TRUE start; `SetBelievedWalls` is a no-op |
| Direction settles | ROS node: heading corrected by +-pi, only on a flip; position re-seeded at the measured start; lap origin at the ASSUMED start; always replans | Heading corrected by the full `measured - belief` angle; position never re-seeded; lap origin at the MEASURED start; no replan | Chassis rotated by the full angle; pose stays in the true frame |
| Parking controller | Built from the believed start | Not wired (`run.go` passes nil) | Built from TRUE metadata, compared with the believed pose |
| Camera detections | Projected through the believed pose | Projected through `GetCurrentPose` | Projected through the TRUE state |
| Pass-side and 9.21 scoring | True frame (ADR 0059) | None | True frame |

What is fixed by the rules and the mat: the direction is drawn on the day,
and the mat is four-fold symmetric, so no inference at t = 0 can tell North
from South (ADR 0053: "declaring a corridor south only rotates the robot's
private frame, but the direction may not"). ADR 0053's evidence also holds:
discovered signs were off by a median 0.200 m against world truth and
0.010 m once mapped through the believed frame, and a run scored 0 laps
against the measured origin where a replay against the assumed origin
counted 4.

## Options considered

- (a) **One believed frame, built by one component.** Every blind consumer
  (pose, path, lap detector, sign discovery, sign router, parking
  controller) works in the canonical South-start frame from the start
  button on, exactly as Python's simulator does. The frame is built in ONE
  place, a component that turns raw sensors into the believed pose:
  - heading reference zeroed at the start button, offset to the canonical
    start yaw;
  - localizer seeded at the canonical assumed start, against the
    South-frame believed walls;
  - wall-heading correction into the same offset;
  - at settle, a +-pi correction only on a direction flip, then a position
    re-seed at the measured start.

  `natsgw` and the sim harness both use that component. The sim hands it
  what the robot's sensors hand `natsgw`: raw LIDAR rays (frame-free), an
  IMU yaw in an arbitrary power-on frame, wheel odometry. It never
  transforms a frame itself. Only the sim's world process, which owns
  scoring, uses the true frame: pass side, rule 9.21, contacts, park
  points.
- (b) **The true frame everywhere.** Not observable: the mat's symmetry
  means the robot cannot know its absolute section, so a stack built on it
  works only in the sim.
- (c) **Keep today's mixed frames and patch the lap detector.** For example,
  gate on a section derived in the pose's own frame. This is the smallest
  change, but the sim and the robot keep diverging, and the refuted fix
  showed that other blind consumers depend on the mixture silently.
- (d) **Rotate the reported frame in the sim only.** Refuted (41 -> 26
  successes, 8 -> 67 wrong-side passes).

## Decision

Proposed: (a).

Only (a) makes the sim's blind numbers mean what the robot does: the frame
logic that runs on the car is the frame logic the sim tests, and there is
no second copy to drift. (b) cannot run on the mat. (c) fixes a symptom and
leaves two frame contracts. (d) was measured and lost.

Three sub-decisions go with (a), each settled by the evidence already on
file:

1. **The lap origin is the ASSUMED start, not the measured one.** Measured
   starts land in the neighbouring corridor and make the section gate
   unsatisfiable (ADR 0053 evidence, run 180154). The Python ROS node
   already does this.
2. **The settle correction is +-pi, applied only on a direction flip.** The
   full `measured - belief` angle Go applies today also carries the
   start-pose measurement's own heading error into the frame. Python
   corrects by the travel-normal difference only.
3. **Position is re-seeded at settle.** Every fix computed during the creep
   was made under the wrong direction assumption (Python ROS node,
   `track_navigator_node.py` settle path).

Still open for the owner:

- Whether the parking controller is wired on the Go robot as part of this,
  or left nil as today.
- Whether this lands before or together with the 2.12 sensor-model
  re-baseline. Blind numbers move either way.

## Consequences

- Blind Open and Obstacles are expected to count laps in the sim. The
  acceptance test is plan item 2.10's: laps counted without regressing
  collisions or wrong-side passes against today's blind baseline.
- Every blind number in plan section 13 becomes non-comparable. Sighted
  numbers are unaffected: a sighted round is given its true start, so its
  believed frame is the true frame.
- The robot changes behaviour in three places, none verified on the car
  yet: the start-button heading zero, the position re-seed at settle, and
  the lap origin. Each needs a bench or mat check before it is trusted.
- `harness.SimHardwareGateway` loses its heading-correction exemption from
  the teleport invariant (plan 2.11), since nothing rotates the body any
  more.
- The sim camera projects through the believed pose in every mode, as the
  vision sensor model (2.12) already does.

## Cross-references

- 0053 (direction inference and start pose): its canonical-section rule is
  kept; the Go `trackmodel.South` hardcode it never recorded becomes this
  ADR's contract.
- 0054 (absolute heading from walls, mod 90) and 0079 (IMU zeroed at the
  start button): the component in (a) implements both; 0079 is currently
  not honoured by the Go node.
- 0059 (pass side is scored in the true frame, independent of the router):
  unchanged, and extended to rule 9.21.
- 0084 and 0086 describe the rigid believed-frame offset from the Python
  side.
- 0068 (Go parallel track): this is a parity decision, and the Python
  simulator is the reference for (a).
