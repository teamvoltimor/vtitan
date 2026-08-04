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

## Sustained full-lock steering / U-turn oscillation -- ROOT-CAUSED 2026-08-03

Originally logged 2026-08-02 as "CCW-only", with a hypothesis pointing at
`corridor_follower.py`'s blind-phase turn branch / `DirectionEstimator` settling (see git
history of this file for that superseded writeup). The 2026-08-03 `replace_path`
heading-aware fix (`55e5a6d`) targeted that hypothesis and did not resolve the symptom on
real hardware -- three more on-track runs the same day (2 CCW, 1 CW) all reproduced full-lock
oscillation after direction had already settled correctly.

Full root cause, with file:line citations and real-hardware numbers, is now written up in
`docs/internal/audits/2026-08-03-realtrack-control-instability-findings.md`: the pursuit
controller (`WaypointController.compute_steering`) is an undamped proportional loop tuned
~3x too aggressive for the real chassis's turn geometry, feeding a steering actuator an
order of magnitude slower than the yaw rate it produces at commanded speed -- a rate-limited
limit cycle, independent of which waypoint is targeted. Compounded by speed never being
reduced for the size of the required heading correction, and a non-wrapping waypoint slice
that can hand the lookahead an out-of-lap point at the exact moment direction commits. Not
CW/CCW-specific -- the CW run that day failed the same way, just faster.

Same investigation also root-caused a related failure: the stuck-escape and K-turn
maneuvers both gate on rear clearance and have no fallback when it's blocked, so a wedged
robot just falls back to the same unstable forward-driving law that wedged it -- see the
audit doc for details.

Fix not yet implemented; see the audit doc's "Suggested next steps" for the concrete work
items (controller re-tune/redamp, speed-to-heading-error coupling, waypoint-slice wrap fix,
stuck-escape fallback maneuver).
