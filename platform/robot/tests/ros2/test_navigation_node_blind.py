"""Blind operation and the heading re-zero, on the real node rather than the sim.

``CorridorWidthEstimator`` was built and measured inside ``ScenarioSimulator``
and for a while existed *only* there, so the deployed node still read corridor
widths straight out of a metadata file. A sim pass rate said nothing about the
robot until this wiring existed. These pin the wiring itself -- that blind mode
starts from the safe prior, that a corrected belief reaches both the planner and
the localizer, and that the heading reference is re-zeroed at the start button.
"""

from __future__ import annotations

import json
import math
from unittest import mock

import numpy as np
import pytest
import rclpy
from shared.config.constants import CorridorDimensions
from shared.config.enums import Direction, Section
from shared.domain.models import IMUReading, Pose

from src.navigation.ports import DriveCommand, LidarScan
from src.ros2.navigation.node import ROS2HardwareGateway
from tests.ros2.test_navigation_node import _WIDTHS, _make_host_node, ros_context

_NARROW = CorridorDimensions.NARROW
_WIDE = CorridorDimensions.WIDE

# A scan whose side rays give a plausible, axis-aligned 1.0 m corridor reading
# (left=right=0.5 m at yaw=0.0) -- what measure_corridor_width and
# DirectionEstimator both need to accept a reading rather than discard it.
_STRAIGHT_SCAN = LidarScan(
    ranges_m=(0.5, 2.0, 0.5, 2.0),
    angles_rad=(-math.pi / 2, 0.0, math.pi / 2, math.pi),
)


def _write_open_metadata(tmp_path) -> str:
    """A minimal, valid Open Challenge metadata file -- for the not-blind branches."""
    path = tmp_path / "metadata.json"
    path.write_text(
        json.dumps(
            {
                "scenario_id": 0,
                "challenge_type": "open",
                "corridor_widths": {s: {"type": "wide", "width_mm": 1000} for s in ("north", "south", "east", "west")},
                "starting_conditions": {
                    "direction": "clockwise",
                    "section": "South",
                    "position": {"x": 1.5, "y": 0.3},
                    "yaw": 3.14,
                },
                "num_signs": 0,
                "sign_positions": [],
                "parking_lot": None,
            },
        ),
        encoding="utf-8",
    )
    return str(path)


class TestGatewayBeliefUpdate:
    """The gateway must be able to change which layout it localizes against."""

    def test_set_believed_walls_replaces_the_localizer(self, ros_context) -> None:  # noqa: F811 - pytest fixture
        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)
        before = gateway._localizer

        from src.navigation.track_geometry import TrackWalls

        gateway.set_believed_walls(TrackWalls(dict.fromkeys(Section, _NARROW)))

        assert gateway._localizer is not before
        node.destroy_node()

    def test_the_believed_layout_changes_the_position_fix(self) -> None:
        """The point of re-pointing: a wrong belief must produce a wrong fix.

        If matching a scan against the believed walls gave the same answer
        whatever those walls were, blind operation could never converge -- and
        a re-point that silently kept the old geometry would look identical.
        """
        from src.navigation.localization import LidarLocalizer
        from src.navigation.track_geometry import TrackWalls
        from src.simulation.track_model import TrackModel

        true_widths = dict.fromkeys(Section, _WIDE)
        truth = (1.5, 0.5)
        # raycast_scan takes a numpy array; estimate_position accepts either.
        angles_arr = np.linspace(-math.pi, math.pi, 360)
        angles = angles_arr.tolist()
        ranges = TrackModel(true_widths).raycast_scan(truth[0], truth[1], 0.0, angles_arr).tolist()

        right = LidarLocalizer(TrackWalls(true_widths)).estimate_position(truth, 0.0, ranges, angles)
        wrong = LidarLocalizer(TrackWalls(dict.fromkeys(Section, _NARROW))).estimate_position(
            truth,
            0.0,
            ranges,
            angles,
        )

        err_right = math.hypot(right[0] - truth[0], right[1] - truth[1])
        err_wrong = math.hypot(wrong[0] - truth[0], wrong[1] - truth[1])
        assert err_right < err_wrong


class TestVisionCallbackParsesDetections:
    """2026-08-04: _vision_callback's deferred import of detection_payload_keys
    named the wrong module path (``ros2.vision...``, missing the ``src.``
    prefix) since the commit that introduced it (2026-08-02, 4f8dbdf) --
    every real /vision/detections message raised ModuleNotFoundError, which
    the surrounding except (JSONDecodeError, TypeError) does not catch, so it
    propagated out of the callback and left _latest_detections permanently
    empty. No test exercised this method at all, which is how it went
    unnoticed. node.py's own top-level import of the same module had the same
    class of bug (missing ``ros2.``, not just ``src.``), which crash-looped
    vision_node outright rather than failing silently -- see
    docs/known-issues-backlog.md.
    """

    def test_a_real_detections_message_populates_latest_detections(self, ros_context) -> None:  # noqa: F811
        import json

        from std_msgs.msg import String

        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)
        try:
            msg = String()
            msg.data = json.dumps(
                [
                    {
                        "class_name": "red_sign",
                        "confidence": 0.9,
                        "bbox": [10.0, 10.0, 20.0, 20.0],
                        "x": 15.0,
                        "y": 15.0,
                        "width": 20.0,
                        "height": 20.0,
                        "area": 400.0,
                    },
                ],
            )

            gateway._vision_callback(msg)

            assert len(gateway._latest_detections) == 1
            assert gateway._latest_detections[0].class_name == "red_sign"
            assert gateway._latest_detections[0].confidence == pytest.approx(0.9)
        finally:
            node.destroy_node()


class TestHeadingResetReachesTheEstimator:
    """The race-start re-zero must actually reach the state estimator.

    The estimator lives on the gateway, not on the node, so calling it on the
    node raises AttributeError -- and only at the not-racing to racing
    transition, i.e. the one moment it matters and the one moment no test
    covered.
    """

    def test_gateway_exposes_the_reset(self, ros_context) -> None:  # noqa: F811
        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)
        assert callable(gateway.reset_heading_reference)
        node.destroy_node()

    def test_reset_rezeroes_the_heading(self, ros_context) -> None:  # noqa: F811
        node = _make_host_node()
        start_yaw = math.pi / 2
        gateway = ROS2HardwareGateway(node, 1.5, 0.45, start_yaw, _WIDTHS)

        # Node start while the robot is held, then carried to the track and set
        # down 90 degrees away from where it was when the first reading landed.
        gateway._estimator.update_imu(IMUReading(yaw=0.0, pitch=0.0, roll=0.0))
        gateway._estimator.update_imu(IMUReading(yaw=math.pi / 2, pitch=0.0, roll=0.0))
        assert gateway.get_current_pose().yaw == pytest.approx(start_yaw + math.pi / 2)

        gateway.reset_heading_reference()
        gateway._estimator.update_imu(IMUReading(yaw=math.pi / 2, pitch=0.0, roll=0.0))

        assert gateway.get_current_pose().yaw == pytest.approx(start_yaw)
        node.destroy_node()


class TestBlindPrior:
    """Blind mode must start from the safe prior, not from the metadata."""

    def test_estimator_starts_every_corridor_narrow(self) -> None:
        """Narrow is safe; wide is not.

        Planning a 1.0 m corridor as 0.6 m puts the path nearer the outer wall,
        still inside it. The converse puts it 0.15 m from the inner block face,
        inside the chassis half-diagonal, and clips it mid-turn.
        """
        from src.navigation.corridor_estimator import CorridorWidthEstimator

        widths = CorridorWidthEstimator().widths
        assert set(widths) == set(Section)
        assert all(w == _NARROW for w in widths.values())

    def test_nothing_is_marked_observed_before_driving(self) -> None:
        from src.navigation.corridor_estimator import CorridorWidthEstimator

        estimator = CorridorWidthEstimator()
        assert estimator.observed_sections == set()
        assert not estimator.is_complete


class TestRunsWithNoScenarioFile:
    """Competition has no metadata file, so the node must start without one."""

    def test_node_constructs_with_no_metadata(self, ros_context) -> None:  # noqa: F811
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=3)
        try:
            assert navigator._blind is True, "no file must imply blind"
            assert navigator._width_estimator is not None
            assert navigator._core_navigator is not None
        finally:
            navigator.destroy_node()

    def test_no_metadata_plans_a_full_lap(self, ros_context) -> None:  # noqa: F811
        """A path built from the prior still has to be a drivable loop."""
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=3)
        try:
            waypoints = navigator._plan(dict.fromkeys(Section, _NARROW))
            assert len(waypoints) > 4
            # Closes back on itself: a lap, not an out-and-back.
            span_x = max(p[0] for p in waypoints) - min(p[0] for p in waypoints)
            span_y = max(p[1] for p in waypoints) - min(p[1] for p in waypoints)
            assert span_x > 1.0
            assert span_y > 1.0
        finally:
            navigator.destroy_node()


class TestBlindImpliesDirectionInference:
    """A blind robot cannot be handed the direction either.

    The round's travel direction is drawn at random on the day, so a "blind"
    run that is told it measures a robot with information no robot has. Blind
    therefore implies inferring it from LIDAR, on the deployed node and not
    only in the simulator -- for a while the estimator existed only in the
    harness, which made every measured pass rate a statement about the sim.
    """

    def test_blind_node_has_a_direction_estimator(self, ros_context) -> None:  # noqa: F811
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=3)
        try:
            assert navigator._direction_estimator is not None
            assert not navigator._direction_estimator.is_settled
        finally:
            navigator.destroy_node()

    def test_direction_is_provisional_until_it_settles(self, ros_context) -> None:  # noqa: F811
        """The constructor's direction is a placeholder, not an input."""
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=3, direction=Direction.CLOCKWISE)
        try:
            assert navigator._direction is Direction.CLOCKWISE
            # Nothing has been observed, so nothing has been committed.
            assert navigator._direction_estimator.direction is None
        finally:
            navigator.destroy_node()

    def test_committing_a_direction_replans(self, ros_context) -> None:  # noqa: F811
        from shared.domain.models import Pose

        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=3, direction=Direction.CLOCKWISE)
        try:
            before = list(navigator._core_navigator._waypoints)
            navigator._commit_direction(Direction.COUNTERCLOCKWISE, Pose(x=1.5, y=0.25, yaw=0.0))
            assert navigator._direction is Direction.COUNTERCLOCKWISE
            # The lap is now driven the other way round, so the path differs.
            assert list(navigator._core_navigator._waypoints) != before
        finally:
            navigator.destroy_node()

    def test_overturning_the_assumed_direction_corrects_the_heading_estimate(self, ros_context) -> None:  # noqa: F811
        """2026-08-04: the bug behind "CCW never resolves its heading".

        assumed_start_conditions pairs a starting yaw with whichever direction
        was assumed at construction -- CW and CCW differ by exactly pi for the
        same section. Overturning the assumption used to rebuild the path
        (test above) without correcting the heading estimate to match, leaving
        it anchored to the old, wrong half of the pair for the rest of the
        run -- a fixed, non-decaying bias, confirmed on real hardware
        (2026-08-04, see docs/internal/audits/2026-08-03-realtrack-control-instability-findings.md).
        """
        from shared.domain.models import Pose

        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=3, direction=Direction.CLOCKWISE)
        try:
            # Nothing has published an IMU reading yet, so this is exactly the
            # yaw assumed_start_conditions(CLOCKWISE) seeded at construction.
            before_yaw = navigator._gateway.get_current_pose().yaw
            assert before_yaw == pytest.approx(math.pi)

            navigator._commit_direction(Direction.COUNTERCLOCKWISE, Pose(x=1.5, y=0.25, yaw=before_yaw))

            after_yaw = navigator._gateway.get_current_pose().yaw
            assert after_yaw == pytest.approx(0.0, abs=1e-9)
        finally:
            navigator.destroy_node()

    def test_overturning_the_assumed_direction_also_discards_position_drift_from_the_creep(self, ros_context) -> None:  # noqa: F811
        """2026-08-04: the bug behind "CW always works, CCW never does".

        The LIDAR localizer takes yaw as given, so every position fix taken
        during the creep -- while yaw was still anchored to whichever
        direction was assumed at construction -- was matched against the
        walls at the wrong orientation if that assumption turns out wrong.
        Correcting yaw alone (test above) doesn't fix a position estimate the
        wrong yaw already corrupted. Confirmed on real hardware: two CCW
        races showed physically impossible implied speeds (2.8-6.4 m/s
        against a ~0.156 m/s real maximum) in pose_x/pose_y throughout the
        creep and right after the yaw correction landed -- CW races never hit
        this because their creep's yaw assumption was already correct from
        the first tick. See docs/known-issues-backlog.md.
        """
        from shared.domain.models import Pose

        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=3, direction=Direction.CLOCKWISE)
        try:
            navigator._gateway._estimator.update_position(0.9, 0.1)  # corrupted creep fix
            drifted_pose = navigator._gateway.get_current_pose()
            assert (drifted_pose.x, drifted_pose.y) == pytest.approx((0.9, 0.1))

            navigator._commit_direction(Direction.COUNTERCLOCKWISE, drifted_pose)

            pose = navigator._gateway.get_current_pose()
            assert (pose.x, pose.y) == pytest.approx(navigator._start_xy)
        finally:
            navigator.destroy_node()


class TestAssumedStartConditions:
    """Section can be assumed; direction cannot."""

    def test_direction_reverses_the_start_heading(self) -> None:
        from src.navigation.start_conditions import assumed_start_conditions

        cw = assumed_start_conditions(Direction.CLOCKWISE)
        ccw = assumed_start_conditions(Direction.COUNTERCLOCKWISE)
        delta = abs(cw["yaw"] - ccw["yaw"])
        assert delta == pytest.approx(math.pi)

    def test_direction_does_not_move_the_start_position(self) -> None:
        """Only the heading flips -- the robot still starts in the same corridor."""
        from src.navigation.start_conditions import assumed_start_conditions

        cw = assumed_start_conditions(Direction.CLOCKWISE)
        ccw = assumed_start_conditions(Direction.COUNTERCLOCKWISE)
        assert cw["position"] == ccw["position"]

    def test_defaults_to_the_narrow_prior(self) -> None:
        """The start pose must come from the safe prior, not a wide guess."""
        from src.navigation.start_conditions import assumed_start_conditions, start_pose

        assumed = assumed_start_conditions(Direction.CLOCKWISE)
        expected = start_pose(
            Section.SOUTH,
            Direction.CLOCKWISE,
            dict.fromkeys(("north", "south", "east", "west"), _NARROW),
        )
        assert assumed["position"]["x"] == pytest.approx(expected[0])
        assert assumed["position"]["y"] == pytest.approx(expected[1])

    def test_scenario_builder_still_exports_start_pose(self) -> None:
        """The geometry moved to navigation; sim callers must be unaffected."""
        from src.navigation.start_conditions import start_pose as moved
        from src.simulation.scenario_builder import start_pose as reexported

        assert reexported is moved


class TestResolveDirection:
    """The creep-until-direction-known state machine, driven directly."""

    def test_returns_false_when_not_blind(self, tmp_path, ros_context) -> None:  # noqa: F811
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=_write_open_metadata(tmp_path), num_laps=1)
        try:
            assert navigator._direction_estimator is None
            assert navigator._resolve_direction() is False
        finally:
            navigator.destroy_node()

    def test_returns_false_once_the_estimator_has_settled(self, ros_context) -> None:  # noqa: F811
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=1)
        try:
            with (
                mock.patch.object(
                    type(navigator._direction_estimator),
                    "is_settled",
                    new_callable=mock.PropertyMock,
                    return_value=True,
                ),
                mock.patch.object(navigator._direction_estimator, "observe") as observe_mock,
            ):
                assert navigator._resolve_direction() is False
            observe_mock.assert_not_called()
        finally:
            navigator.destroy_node()

    def test_holds_and_returns_true_when_scan_or_pose_is_missing(self, ros_context) -> None:  # noqa: F811
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=1)
        try:
            with (
                mock.patch.object(navigator._gateway, "get_lidar_scan", return_value=None),
                mock.patch.object(navigator._gateway, "publish_drive") as publish_mock,
            ):
                assert navigator._resolve_direction() is True
            publish_mock.assert_called_once_with(DriveCommand(speed_mps=0.0, steering_norm=0.0))
        finally:
            navigator.destroy_node()

    def test_not_yet_settled_buffers_a_width_reading_and_creeps(self, ros_context) -> None:  # noqa: F811
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=1)
        try:
            pose = Pose(x=1.5, y=0.25, yaw=0.0)
            with (
                mock.patch.object(navigator._gateway, "get_lidar_scan", return_value=_STRAIGHT_SCAN),
                mock.patch.object(navigator._gateway, "get_current_pose", return_value=pose),
                mock.patch.object(navigator._direction_estimator, "observe", return_value=False),
                mock.patch.object(navigator._gateway, "publish_drive") as publish_mock,
            ):
                result = navigator._resolve_direction()
            assert result is True
            publish_mock.assert_called_once()
            # The reading is real (a plausible, aligned 1.0 m corridor) so it
            # must have been buffered for replay once the direction commits.
            assert navigator._creep_widths == [(0.0, pytest.approx(1.0))]
        finally:
            navigator.destroy_node()

    def test_settling_commits_the_direction_and_reports_no_plan_step(self, ros_context) -> None:  # noqa: F811
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=1, direction=Direction.CLOCKWISE)
        try:
            pose = Pose(x=1.5, y=0.25, yaw=0.0)
            with (
                mock.patch.object(navigator._gateway, "get_lidar_scan", return_value=_STRAIGHT_SCAN),
                mock.patch.object(navigator._gateway, "get_current_pose", return_value=pose),
                mock.patch.object(navigator._direction_estimator, "observe", return_value=True),
                mock.patch.object(
                    type(navigator._direction_estimator),
                    "direction",
                    new_callable=mock.PropertyMock,
                    return_value=Direction.COUNTERCLOCKWISE,
                ),
                mock.patch.object(navigator, "_commit_direction") as commit_mock,
            ):
                result = navigator._resolve_direction()
            assert result is False
            commit_mock.assert_called_once_with(Direction.COUNTERCLOCKWISE, pose)
        finally:
            navigator.destroy_node()


class TestCommitDirectionFlushesBufferedWidths:
    """The creep buffer built up while direction was unknown must reach the estimator."""

    def test_buffered_readings_are_replayed_and_the_buffer_is_cleared(self, ros_context) -> None:  # noqa: F811
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=1, direction=Direction.CLOCKWISE)
        try:
            navigator._creep_widths = [(0.0, 1.0), (0.01, 0.62)]
            with mock.patch.object(navigator._width_estimator, "observe_measurement") as observe_mock:
                navigator._commit_direction(Direction.CLOCKWISE, Pose(x=1.5, y=0.25, yaw=0.0))
            assert observe_mock.call_count == 2
            assert navigator._creep_widths == []
        finally:
            navigator.destroy_node()


class TestUpdateLayoutBelief:
    """Folding LIDAR readings into the width estimate mid-run, and replanning off it."""

    def test_returns_false_when_not_blind(self, tmp_path, ros_context) -> None:  # noqa: F811
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=_write_open_metadata(tmp_path), num_laps=1)
        try:
            assert navigator._width_estimator is None
            assert navigator._update_layout_belief() is False
        finally:
            navigator.destroy_node()

    def test_returns_false_when_scan_or_pose_is_missing(self, ros_context) -> None:  # noqa: F811
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=1)
        try:
            with mock.patch.object(navigator._gateway, "get_lidar_scan", return_value=None):
                assert navigator._update_layout_belief() is False
        finally:
            navigator.destroy_node()

    def test_returns_false_when_the_estimate_does_not_change(self, ros_context) -> None:  # noqa: F811
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=1)
        try:
            pose = Pose(x=1.5, y=0.25, yaw=0.0)
            with (
                mock.patch.object(navigator._gateway, "get_lidar_scan", return_value=_STRAIGHT_SCAN),
                mock.patch.object(navigator._gateway, "get_current_pose", return_value=pose),
                mock.patch.object(navigator._width_estimator, "observe", return_value=False),
            ):
                assert navigator._update_layout_belief() is False
        finally:
            navigator.destroy_node()

    def test_a_settled_estimate_replans_and_repoints_the_localizer(self, ros_context) -> None:  # noqa: F811
        from src.ros2.navigation.node import TrackNavigator

        navigator = TrackNavigator(metadata_path=None, num_laps=1)
        try:
            pose = Pose(x=1.5, y=0.25, yaw=0.0)
            with (
                mock.patch.object(navigator._gateway, "get_lidar_scan", return_value=_STRAIGHT_SCAN),
                mock.patch.object(navigator._gateway, "get_current_pose", return_value=pose),
                mock.patch.object(navigator._width_estimator, "observe", return_value=True),
                mock.patch.object(navigator._gateway, "set_believed_walls") as set_walls_mock,
                mock.patch.object(navigator._core_navigator, "replace_path") as replace_path_mock,
            ):
                assert navigator._update_layout_belief() is True
            set_walls_mock.assert_called_once()
            replace_path_mock.assert_called_once()
        finally:
            navigator.destroy_node()


if __name__ == "__main__":
    rclpy.init()
