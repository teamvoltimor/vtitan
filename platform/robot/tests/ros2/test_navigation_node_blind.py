"""Blind operation and the heading re-zero, on the real node rather than the sim.

``CorridorWidthEstimator`` was built and measured inside ``ScenarioSimulator``
and for a while existed *only* there, so the deployed node still read corridor
widths straight out of a metadata file. A sim pass rate said nothing about the
robot until this wiring existed. These pin the wiring itself -- that blind mode
starts from the safe prior, that a corrected belief reaches both the planner and
the localizer, and that the heading reference is re-zeroed at the start button.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
import rclpy
from shared.config.constants import CorridorDimensions
from shared.config.enums import Section
from shared.domain.models import IMUReading

from src.ros2.navigation.node import ROS2HardwareGateway
from tests.ros2.test_navigation_node import _WIDTHS, _make_host_node, ros_context

_NARROW = CorridorDimensions.NARROW
_WIDE = CorridorDimensions.WIDE


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


if __name__ == "__main__":
    rclpy.init()
