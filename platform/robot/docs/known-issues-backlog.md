# Known Issues Backlog

Deferred items surfaced during debugging sessions but not yet worked. None of these block
the LIDAR `inverted`/yaw-offset coupling fix (2026-08-02, see `robot-physical-constants.md`)
-- they're unrelated, separately-tracked gaps.

## ROS2 graph discovery: jumper subscription count is 0

`/challenge_mode/jumper_inserted`'s Subscription count shows 0 in `ros2 topic info`, even
when a subscriber should be active. Not root-caused yet. See `challenge-mode-jumper-spec.md`
for the topic's intended behavior.

## Boot time: ~15s `_CHALLENGE_MODE_TIMEOUT_SEC` delay

A ~15 second timeout constant suspected of padding boot time unnecessarily. A mitigation was
proposed but not implemented or verified against real boot timing.

## LIDAR ghost readings

Alternating range readings (1.39m / 0.65m) observed on a static scene with no known cause.
Not related to the 2026-08-02 `inverted`/yaw-offset fixes -- present before and orthogonal to
that work. Needs its own investigation (raw driver output vs. downstream processing).

## Pre-existing test failures (unit suite, `platform/robot`)

15 failures observed in `tests/unit`, confirmed pre-existing (present before the 2026-08-02
LIDAR coupling change, unrelated to LIDAR code):

- `test_deviation_recovery.py::TestRecoversFromLateralKick::test_wide_corridor` (4 direction
  variants) and `TestRecoversFromHeadingKick::test_wide_corridor` (4 direction variants) --
  wide-corridor recovery budget gap.
- `test_obstacles_challenge_sim.py` -- 5 failures across
  `TestObstaclesDemoScenariosRun` (3) and `TestVisionConfirmedSignRouting` (2).
- `test_open_challenge_sim.py::TestThreeLapSolvability::test_symmetric_narrow_all_starts[0]`.
- `test_sensor_errors.py::TestStartPlacement::test_placement_error_is_absent_by_default` --
  traced to `simulated_hardware_gateway.py`'s `position_error_m`, modified in separate
  pre-existing uncommitted branch work predating this session.

## CCW-only sustained full-lock steering (turns correct direction, but into a U-turn)

Confirmed on real hardware 2026-08-02, reproduced twice (`run_20260802_205407`,
`run_20260802_205613`): driving CCW, the robot correctly starts turning toward the open
side, but holds full-lock steering (~70 deg) continuously for ~7 seconds before ever
releasing -- long enough to spin around into the wrong heading rather than complete a
normal corner turn. A CW run in the same session (`run_20260802_205031`, 142s) never held
full lock for more than ~3s anywhere.

Root cause hypothesis (not yet confirmed against live data): `corridor_follower.py`'s
`follow_corridor()` turn branch has no release condition of its own -- it commands full
lock every tick for as long as forward clearance stays below `TURN_CLEARANCE_M`, relying
entirely on `DirectionEstimator` settling to hand off control (see that module's own
docstring warning about this exact deadlock: "Measured at gain 2.0, that cost 12 of 28
fixtures their direction and put 9 into a wall"). `DirectionEstimator.infer_direction`
only accepts a reading inside a narrow alignment window (~8 scans) and rejects dropouts
(`_MAX_IN_TRACK_RANGE_M`) and insufficiently asymmetric readings (`_MIN_ASYMMETRY_M`). No
code branches explicitly on CW vs CCW -- the asymmetry most likely comes from CCW corner
geometry failing to produce 5 agreeing votes inside that window.

Diagnostic logging was added 2026-08-02 (`track_navigator_node.py`'s
`_direction_gate_verdict`, throttled `logger.info` in `_resolve_direction`) to capture
which gate (dropout/align-fail/span-fail/asym-fail) is refusing readings on the next live
CCW run -- read-only, no behavior change. Next step: run one more CCW test with this
logging deployed and read the log to confirm which gate is binding, then fix from there.

Separately, regardless of root cause: `follow_corridor()`'s full-lock branch having no
maximum-duration safety net is worth its own fix -- bound how long it can hold full lock
before falling back to something safer (stop or a slower creep), independent of whatever
turns out to be causing the CCW-specific failure.
