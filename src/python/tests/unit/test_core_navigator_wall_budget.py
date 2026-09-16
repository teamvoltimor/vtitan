"""The pursuit crosstrack threshold must follow the path's room to the outer wall.

Measured on hardware: under the blind narrow prior the planned path sits close
to the outer wall, while ``LOOKAHEAD_TRANSITION`` is a fixed mid value. Subtract
the chassis half-width and only a thin band of crosstrack exists before contact,
so the corrective short lookahead was armed to fire only after the wall had been
reached; the robot ended up against the wall on both the clockwise and
counterclockwise rounds. See adr:0052-pursuit-target-selection.

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
from shared.domain.models import Detection, IMUReading, Pose, Waypoint

from tests.fixtures import FakeGateway, build_navigator

if TYPE_CHECKING:
    from src.navigation.core_navigator import CoreNavigator
    from src.navigation.ports import DriveCommand, LidarScan


def _navigator(waypoints: list[tuple[float, float]], tuning: NavigationTuning | None = None) -> CoreNavigator:
    return build_navigator(
        FakeGateway(Pose(x=waypoints[0][0], y=waypoints[0][1], yaw=0.0)),
        [Waypoint(*wp) for wp in waypoints],
        tuning or NavigationTuning.load_default(),
    )


def _expected(offset_m: float, tuning: NavigationTuning) -> float:
    return offset_m - RobotSpecs.WIDTH / 2 - tuning.pursuit.wall_margin_safety_m


def _straight_path_at(offset_m: float) -> list[tuple[float, float]]:
    """A path running the length of the mat at ``offset_m`` from the south edge."""
    return [(x / 10.0, offset_m) for x in range(5, 26)]


class TestBudgetFollowsThePath:
    def test_wall_hugging_path_tightens_the_threshold(self, tuning):
        """The narrow-belief geometry that lost the hardware rounds.
        See adr:0052-pursuit-target-selection."""
        nav = _navigator(_straight_path_at(0.25), tuning)

        expected = _expected(0.25, tuning)
        assert nav._waypoint_controller.effective_transition == pytest.approx(expected)
        assert expected < tuning.pursuit.lookahead_transition, "must be tighter than the fixed value"

    def test_centred_path_keeps_the_configured_threshold(self, tuning):
        """A confirmed-wide corridor must behave exactly as it did before."""
        nav = _navigator(_straight_path_at(0.50), tuning)

        assert nav._waypoint_controller.effective_transition == pytest.approx(
            tuning.pursuit.lookahead_transition,
        )

    def test_budget_never_falls_below_the_floor(self, tuning):
        """A path almost touching a wall must not pin the short lookahead on
        forever -- that trades the wall for a twitchy straight."""
        nav = _navigator(_straight_path_at(0.10), tuning)

        assert nav._waypoint_controller.effective_transition == pytest.approx(
            tuning.pursuit.min_lookahead_transition_m,
        )

    def test_measures_the_nearest_edge_whichever_it_is(self, tuning):
        """The path is scored against all four mat edges, not just the south one."""
        near_north = [(x / 10.0, TrackDimensions.MAX_COORD - 0.25) for x in range(5, 26)]
        nav = _navigator(near_north, tuning)

        assert nav._waypoint_controller.effective_transition == pytest.approx(_expected(0.25, tuning))

    def test_replanning_onto_a_wider_path_relaxes_the_threshold(self, tuning):
        """The belief widening mid-round has to reach the controller.

        This is the moment the hardware bags show the path jumping sideways;
        the threshold governing recovery from that jump has to move with it
        rather than stay at the tighter value. See
        adr:0052-pursuit-target-selection.
        """
        nav = _navigator(_straight_path_at(0.25), tuning)
        assert nav._waypoint_controller.effective_transition < tuning.pursuit.lookahead_transition

        nav.replace_path([Waypoint(*wp) for wp in _straight_path_at(0.50)], (1.0, 0.30))

        assert nav._waypoint_controller.effective_transition == pytest.approx(
            tuning.pursuit.lookahead_transition,
        )
