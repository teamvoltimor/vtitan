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

## Full unit suite runtime looks like a hang -- DIAGNOSED 2026-08-04, mitigated not fixed

Running plain `pytest tests/unit` (no marker filter) took 10+ minutes with zero output and
was indistinguishable from a genuine hang -- confirmed via `cProfile` it is not: the cost is
real work in `TrackWalls.raycast()` (`src/navigation/track_geometry.py:161`), called ~100
times per LIDAR scan refresh by `LidarLocalizer.estimate_position`'s coarse-to-fine grid
search (`src/navigation/localization.py`). Several files each run dozens of parametrized full
closed-loop sims through that path -- `test_open_challenge_sim.py` (many
section/direction/width combinations), `test_obstacles_challenge_sim.py` (7 full 3-lap+park
scenarios), `test_deviation_recovery.py` (24 parametrized cases), plus one especially
expensive case in `test_sensor_errors.py::test_drift_stops_accumulating` (two 600-tick sims,
~32s alone). Summed, the full suite plausibly needs 10-15+ minutes, and `pytest -q` piped
through `tail` buffers all output until the run ends, so there is no visible progress to
distinguish "slow" from "stuck" while it runs.

Mitigated by marking the expensive files/tests `@pytest.mark.slow` (registered in
`pytest.ini`) and adding `task robot:test SCOPE=fast` (`platform/Taskfile.yml`), which
excludes them -- verified at 55s for 523 tests. Not fixed: `raycast()`'s per-call cost itself
(~0.46ms) is unexamined -- worth profiling whether the coarse-to-fine grid search
(4 passes x 5x5 = 100 raycasts per scan) is doing more work than it needs to, independent of
whether it's fast enough for these tests' purposes.

## `telemetry_bridge_node.py`'s `_LIDAR_YAW_OFFSET_RAD` is missing the mount-inversion term

Pre-existing, confirmed present at `5378e42` (before the 2026-08-04 collision-avoidance
work) -- not caused by that session. `telemetry_bridge_node.py:194` computes
`_LIDAR_YAW_OFFSET_RAD = math.radians(RobotSpecs.LIDAR_MOUNT_YAW_OFFSET_DEG)`, but
`ros2_hardware_gateway.py`'s equivalent constant additionally includes
`180.0 if RobotSpecs.LIDAR_INVERTED else 0.0` (currently `LIDAR_INVERTED=True`,
`LIDAR_MOUNT_YAW_OFFSET_DEG=0.0`, so the real offset is 180 deg and this file's copy is
computing 0). The module's own docstring claims it "matches the correction
ros2/navigation/node.py applies" -- it doesn't. Surfaced by 4 failing tests in
`tests/ros2/test_telemetry_bridge_node.py::TestLidarClearancesCm` (front/left/right sector
means come back reading the 10m background fill instead of the injected near-range window,
because the sector center ends up 180 deg away from where the test places it). Likely means
the real OLED front/left/right clearance display is rotated 180 deg from reality on hardware,
the same class of bug the docstring says it was written to prevent.

## Vision detection payload import path -- FIXED 2026-08-04

`src/ros2/vision/node.py` imported `from src.vision.detection_payload_keys import (...)`
and `ros2_hardware_gateway.py::_vision_callback` imported `from ros2.vision.detection_payload_keys
import (...)` -- both wrong; the module actually lives at
`src/ros2/vision/detection_payload_keys.py`. Neither path was ever correct: both were
introduced in the same commit that added the file (`4f8dbdf`, 2026-08-02). No test exercised
either import path, which is how both went unnoticed for two days.

Real-hardware impact, confirmed live on the Pi 5 (2026-08-04): `vision_node`'s copy is a
top-level `ModuleNotFoundError` at import time, so the node crash-looped continuously
(`journalctl` showed it dying and respawning every few seconds). The gateway's copy is inside
a `try: ... except (json.JSONDecodeError, TypeError):` that does not catch
`ModuleNotFoundError`, so every real `/vision/detections` message raised uncaught out of the
ROS2 callback and `_latest_detections` never updated -- sign/vision detections silently never
reached the navigator, no crash, no error visible short of reading the node's own logs.

Fixed by correcting both import paths to `from src.ros2.vision.detection_payload_keys import
(...)`. Regression test added: `tests/ros2/test_navigation_node_blind.py::TestVisionCallbackParsesDetections`
(verified it fails against the broken path, passes against the fix -- no prior test covered
`_vision_callback` at all). `vision_node`'s own crash-loop has no unit-test path to pin
directly (`tests/ros2/test_vision_node.py` cannot even collect in this dev environment, see
the cv2 recursion entry elsewhere in this doc) -- confirmed only by redeploying and reading
`journalctl -u vtitan-pi5.service` on the Pi 5.

## Position estimate carried hundreds of metres of drift across race boundaries -- FIXED 2026-08-04

Confirmed on real hardware across two consecutive races the same session
(`run_20260804_143942`, `run_20260804_144019`): `pose_x`/`pose_y` in `nav_debug` reached
the hundreds (e.g. `-195, -197`) on a track no larger than 3m square, and the second race's
*first logged tick* continued directly from where the first race's *last* tick left off
(`-98.4, -100.1` -> `-125.4, -127.1`) rather than resetting -- confirmed not a parsing
artifact by replaying the real recorded `/scan` through `LidarLocalizer.estimate_position`
directly with the canonical seed `(1.5, 0.25)`: it returns a sane, in-track result (`1.22,
0.09`), so the localizer's own math is fine given a sane prior. The real running node simply
wasn't giving it one on a new race.

Root cause: `TrackNavigator.reset()` (runs at every FINISHED -> RACING transition -- the
state machine cycles this purely from the button, with no process restart) already re-zeroed
the heading reference but never re-seeded position. So while heading correctly started fresh
every race, position silently carried over from wherever the *previous* race's LIDAR
localizer estimate last drifted to. The localizer's own coarse-to-fine grid search is
provably bounded to `search_radius_m` (0.15m) per call, so it cannot itself explain a
sudden 100+m jump -- what compounds is a *persistent, unresetting* small per-tick disagreement
accumulating across many minutes and multiple races in one process lifetime, not one dramatic
event. The exact per-tick mechanism generating that disagreement in the first place -- most
likely the local search losing track after a fast true displacement it can't see across (e.g.
an escape/K-turn maneuver moving the robot faster than the search radius can follow) -- is
still open; this fix only stops the drift from surviving a race boundary, so future
investigation of the within-race drift itself now gets a clean starting point each race
instead of a contaminated one.

Fixed: `StateEstimator.reset_position(x, y)`, exposed via
`ROS2HardwareGateway.reset_position`, called from `TrackNavigator.reset()` alongside the
existing `reset_heading_reference()`. Regression tests:
`tests/ros2/test_navigation_node.py::TestReset::test_resetting_actually_clears_position_drift_on_the_estimator`
(goes through the real gateway/estimator, not a mock, seeds a hundreds-of-metres value and
confirms `reset()` clears it) and the existing mock-based `TestReset` test extended to assert
`reset_position` is called with the race's actual starting position.

Separately confirmed and unrelated: the same testing session also surfaced that restarting
only the Pi 5's systemd services (not the Pi Zero's) after a code deploy can leave the
Pi5<->Zero comms link half-dead -- motor commands published and logged correctly
(`/ackermann_cmd` nonzero throughout) but `/motor/drive_speed` read exactly 0.0 for an entire
~6-minute session despite continuous creep commands. A full reboot of both boards resolved it;
this matches the already-known "USB gadget link is nondeterministic" entry above. Worth a
documented redeploy procedure (restart/reboot both boards together) rather than relying on
each ad-hoc session to remember it.

## `_yaw_correction` also carried across race boundaries -- FIXED 2026-08-04

A second half of the same class of bug as the position-carryover fix above, in a field the
first pass missed. Confirmed on real hardware: a CW race (`run_20260804_151809`) that
immediately followed a CCW one (`run_20260804_151732`, same process, button-reset in
between, ~37s apart -- too fast for a reboot) started at `pose_yaw` ~0 deg instead of ~180
deg, which is CCW's assumed starting convention, not CW's. Traced to `steer_target` sitting
~2.4m away at an ~88 deg bearing from t=2s onward -- confirmed via `calculate_waypoints`
that the *planned path* itself is fine (first waypoints proceed near 0 deg local bearing from
the canonical CW start, nothing like an 88 deg turn), so the corruption was in the heading
estimate, not the plan.

Root cause: `apply_yaw_correction` (added earlier the same day, see the "heading estimate
when direction inference overturns the assumption" fix above) writes a full, up-to-pi jump to
`StateEstimator._yaw_correction` whenever blind direction inference overturns the assumed
direction. `reset_heading_reference()` -- called by `TrackNavigator.reset()` at every race
start -- only cleared `_imu_yaw_offset`/`_relative_imu_yaw` (the IMU power-on reference), not
`_yaw_correction`. So the CCW race's -pi correction survived into the next race untouched;
that race's own reset correctly rebuilt the path for CW and zeroed the IMU reference, but
silently kept the previous race's leftover correction on top of it, netting the heading
estimate out to CCW's convention while the direction label and path were both genuinely CW.

Fixed: `reset_heading_reference()` now also zeroes `_yaw_correction`. Regression test:
`tests/unit/test_heading_reference.py::TestHeadingReference::test_reset_clears_a_leftover_direction_reassumption_correction`
(applies a pi correction, resets, confirms the estimate falls back to the fresh start_yaw
rather than carrying the old correction forward).

## CW always worked, CCW never did -- root-caused and fixed 2026-08-04

After the two fixes above, four consecutive real-hardware attempts (2 CW, 2 CCW) still split
cleanly along direction: both CW attempts finished all 3 laps within the 180s WRO limit
(`run_20260804_161412` 139.4s, `run_20260804_161956` 157.7s); both CCW attempts failed
without completing a lap (`run_20260804_161650` froze for 27s and gave up,
`run_20260804_161931` wedged in a corner). Neither failure was the `_yaw_correction`
carryover bug -- both showed the *correct* ~0 deg heading for CCW once direction resolved.

Root cause: the LIDAR localizer takes yaw as given (see `LidarLocalizer`'s own docstring) --
it only solves for position, matching the scan against the walls at whatever yaw the
estimator currently reports. Blind races always start assuming the launch default (CW,
`--direction`'s default), so a CW race's `blind_creep` phase always fits position with the
*correct* yaw from the first tick. A CCW race's `blind_creep` always fits position with the
*wrong* (CW-assumed) yaw until direction inference resolves and `_commit_direction` corrects
it -- but correcting yaw at that moment does nothing to repair the position estimate the
wrong yaw already corrupted, and `LidarLocalizer.estimate_position`'s coarse-to-fine search
reseeds from whatever the previous (already wrong) fix was, so the error compounds across the
whole creep instead of self-correcting. Confirmed directly: both real CCW captures showed
physically impossible implied speeds in `pose_x`/`pose_y` (2.8-6.4 m/s against a ~0.156 m/s
real maximum) throughout `blind_creep` and immediately after `_commit_direction`'s yaw
correction landed. CW never hits this, because its creep's yaw assumption was never wrong in
the first place -- which is exactly the observed asymmetry.

Fixed: `_commit_direction()` now also calls `reset_position()` (the same method the
cross-race fix above added) whenever direction actually changes, discarding whatever position
the wrong-yaw creep produced and re-seeding from the known starting position. Safe because
this fires within ~1-2s of race start (both real captures committed by t=1.2s) against a
capped creep speed, so the true displacement discarded is at most ~0.2m -- far smaller than
the corruption it replaces. Regression test:
`tests/ros2/test_navigation_node_blind.py::TestBlindImpliesDirectionInference::test_overturning_the_assumed_direction_also_discards_position_drift_from_the_creep`.

Not yet re-verified on real hardware -- next CCW attempt is the real test.
