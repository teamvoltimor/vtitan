"""The pursuit crosstrack threshold must follow the path's room to the outer wall.

Measured on hardware 2026-08-06: under the blind narrow prior a corridor
believed 0.60 puts the planned path ~0.25-0.30 m from the outer wall, while
``LOOKAHEAD_TRANSITION`` is a fixed 0.30 m. Subtract the chassis half-width and
only 0.15-0.20 m of crosstrack exists before contact, so the corrective short
lookahead was armed to fire only after the wall had been reached. Crosstrack ran
0.09 -> 0.15 through the corner, never crossed 0.30, and the robot ended up
0.10 m from the wall on both the clockwise and counterclockwise rounds.

The budget is taken from the mat's outer edges rather than the width belief on
purpose: WRO moves the inner walls between rounds, but the mat's own edges are
fixed, and it is the outer wall the robot hits -- precisely because an
under-estimated width biases the path toward it.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest
from shared.config.constants import RobotSpecs, TrackDimensions
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Detection, IMUReading, Pose

from src.navigation.core_navigator import CoreNavigator

if TYPE_CHECKING:
    from src.navigation.ports import DriveCommand, LidarScan

_MAT = TrackDimensions.MAX_COORD


class _FakeGateway:
    def __init__(self, pose: Pose) -> None:
        self._pose = pose
        self.commands: list[DriveCommand] = []

    def publish_drive(self, command: DriveCommand) -> None:
        self.commands.append(command)

    def get_current_pose(self) -> Pose | None:
        return self._pose

    def get_lidar_scan(self) -> LidarScan | None:
        return None

    def get_imu_reading(self) -> IMUReading | None:
        return IMUReading(yaw=self._pose.yaw, pitch=0.0, roll=0.0)

    def get_vision_detections(self) -> list[Detection]:
        return []


def _navigator(waypoints: list[tuple[float, float]], tuning: NavigationTuning | None = None) -> CoreNavigator:
    return CoreNavigator(
        gateway=_FakeGateway(Pose(x=waypoints[0][0], y=waypoints[0][1], yaw=0.0)),
        waypoints=waypoints,
        num_laps=1,
        tuning=tuning or NavigationTuning(),
    )


def _expected(offset_m: float, tuning: NavigationTuning) -> float:
    return offset_m - RobotSpecs.WIDTH / 2 - tuning.pursuit.WALL_MARGIN_SAFETY_M


def _straight_path_at(offset_m: float) -> list[tuple[float, float]]:
    """A path running the length of the mat at ``offset_m`` from the south edge."""
    return [(x / 10.0, offset_m) for x in range(5, 26)]


class TestBudgetFollowsThePath:
    def test_wall_hugging_path_tightens_the_threshold(self):
        """The narrow-belief geometry that lost both 2026-08-06 rounds."""
        tuning = NavigationTuning()
        nav = _navigator(_straight_path_at(0.25), tuning)

        expected = _expected(0.25, tuning)
        assert nav._waypoint_controller.effective_transition == pytest.approx(expected)
        assert expected < tuning.pursuit.LOOKAHEAD_TRANSITION, "must be tighter than the fixed value"

    def test_centred_path_keeps_the_configured_threshold(self):
        """A confirmed-wide corridor must behave exactly as it did before."""
        tuning = NavigationTuning()
        nav = _navigator(_straight_path_at(0.50), tuning)

        assert nav._waypoint_controller.effective_transition == pytest.approx(
            tuning.pursuit.LOOKAHEAD_TRANSITION,
        )

    def test_budget_never_falls_below_the_floor(self):
        """A path almost touching a wall must not pin the short lookahead on
        forever -- that trades the wall for a twitchy straight."""
        tuning = NavigationTuning()
        nav = _navigator(_straight_path_at(0.10), tuning)

        assert nav._waypoint_controller.effective_transition == pytest.approx(
            tuning.pursuit.MIN_LOOKAHEAD_TRANSITION_M,
        )

    def test_measures_the_nearest_edge_whichever_it_is(self):
        """The path is scored against all four mat edges, not just the south one."""
        tuning = NavigationTuning()
        near_north = [(x / 10.0, _MAT - 0.25) for x in range(5, 26)]
        nav = _navigator(near_north, tuning)

        assert nav._waypoint_controller.effective_transition == pytest.approx(_expected(0.25, tuning))

    def test_replanning_onto_a_wider_path_relaxes_the_threshold(self):
        """The belief widening mid-round has to reach the controller.

        This is the moment the hardware bags show the path jumping ~0.20 m
        sideways; the threshold governing recovery from that jump has to move
        with it rather than stay at the tighter value.
        """
        tuning = NavigationTuning()
        nav = _navigator(_straight_path_at(0.25), tuning)
        assert nav._waypoint_controller.effective_transition < tuning.pursuit.LOOKAHEAD_TRANSITION

        nav.replace_path(_straight_path_at(0.50), (1.0, 0.30))

        assert nav._waypoint_controller.effective_transition == pytest.approx(
            tuning.pursuit.LOOKAHEAD_TRANSITION,
        )
