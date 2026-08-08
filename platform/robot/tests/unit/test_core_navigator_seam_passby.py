"""The last waypoint of the loop must get the same pass-by rescue as the rest.

Every waypoint but one can be advanced past by simply driving beyond it -- the
robot need not enter the MAIN_LOOP_REACHED_DISTANCE_M circle around it. The
final waypoint used to be the exception, because the pass-by scan stopped at
``index + 1 < len(waypoints)`` and there was no ``index + 1`` for it. Entering
the 0.20 m circle was then the only way past, and a robot running wider than
that never got past it, never wrapped the index, and so never completed a lap
however many times it drove the loop.

Measured on the 2026-08-06 counterclockwise round: crosstrack ran 0.26-0.51 m
against the 0.20 m radius, the waypoint index froze on the last waypoint at
t=90 s, and the robot circled the mat for a further seven minutes with the lap
count stuck at zero while the pose data shows four more laps completed.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import pytest
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Direction, Section
from shared.domain.models import Pose

from src.navigation.core_navigator import CoreNavigator
from src.navigation.race_tracker import LapDetector
from tests.fixtures import FakeGateway

if TYPE_CHECKING:
    from src.navigation.ports import DriveCommand, LidarScan


@pytest.fixture()
def tuning():
    return NavigationTuning()


def _square_loop(side: float = 2.0, per_side: int = 12, origin: float = 0.5) -> list[tuple[float, float]]:
    """A closed rectangular lap, corners included, walked counterclockwise."""
    corners = [
        (origin, origin),
        (origin + side, origin),
        (origin + side, origin + side),
        (origin, origin + side),
    ]
    path: list[tuple[float, float]] = []
    for i, (ax, ay) in enumerate(corners):
        bx, by = corners[(i + 1) % len(corners)]
        for step in range(per_side):
            t = step / per_side
            path.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return path


def _navigator(waypoints: list[tuple[float, float]], pose: Pose, tuning: NavigationTuning) -> tuple[CoreNavigator, FakeGateway]:
    gateway = FakeGateway(pose)
    nav = CoreNavigator(
        gateway=gateway,
        waypoints=waypoints,
        num_laps=3,
        tuning=tuning,
        lap_detector=LapDetector(
            start_pos=waypoints[0],
            start_section=Section.SOUTH,
            direction=Direction.COUNTERCLOCKWISE,
        ),
    )
    return nav, gateway


def _offset_toward_centre(point: tuple[float, float], centre: tuple[float, float], distance: float):
    """Move ``point`` ``distance`` metres toward ``centre`` -- i.e. off-path by
    more than the waypoint-reached radius, the way a wide-running robot is."""
    dx, dy = centre[0] - point[0], centre[1] - point[1]
    norm = math.hypot(dx, dy)
    return (point[0] + dx / norm * distance, point[1] + dy / norm * distance)


class TestFinalWaypointPassBy:
    def test_index_advances_past_the_last_waypoint_without_entering_its_radius(self, tuning):
        """The exact CCW failure: sitting past the final waypoint but 0.35 m
        off it -- well outside MAIN_LOOP_REACHED_DISTANCE_M -- must still
        advance rather than freeze."""
        path = _square_loop()
        nav, gateway = _navigator(path, Pose(x=path[0][0], y=path[0][1], yaw=0.0), tuning=tuning)

        last = len(path) - 1
        nav._waypoint_index = last
        threshold = nav._tuning.waypoints.MAIN_LOOP_REACHED_DISTANCE_M

        # Stand nearer to waypoint 0 than to the final waypoint, but far enough
        # from both that neither reached-radius test can fire -- which is the
        # only reason the index could move before this fix.
        pose_xy = (path[0][0], path[0][1] - 0.35)
        gateway.pose = Pose(x=pose_xy[0], y=pose_xy[1], yaw=0.0)
        assert math.dist(pose_xy, path[0]) > threshold
        assert math.dist(pose_xy, path[last]) > threshold

        nav.step()

        assert nav._waypoint_index > last, "final waypoint must not be a dead end"

    def test_the_seam_crossing_counts_a_lap(self, tuning):
        """Advancing past the last waypoint is only useful if the wrap that
        follows it is what the lap counter reads."""
        path = _square_loop()
        nav, gateway = _navigator(path, Pose(x=path[0][0], y=path[0][1], yaw=0.0), tuning=tuning)
        nav._lap_detector.notify_waypoint_wrapped()
        nav._lap_detector = None  # take the waypoint-only fallback, which counts directly

        nav._waypoint_index = len(path) - 1
        gateway.pose = Pose(x=path[0][0], y=path[0][1] - 0.35, yaw=0.0)

        nav.step()  # crosses the seam, leaving the index one past the end
        assert nav._waypoint_index >= len(path)

        nav.step()  # the wrap branch runs at the top of this tick
        assert nav._waypoint_index == 0
        assert nav.laps_completed == 1

    def test_a_robot_behind_the_last_waypoint_does_not_skip_the_seam(self, tuning):
        """The rescue must not fire early -- still short of the final waypoint,
        the index has to stay put rather than jump the lap."""
        path = _square_loop()
        nav, gateway = _navigator(path, Pose(x=path[0][0], y=path[0][1], yaw=0.0), tuning=tuning)

        last = len(path) - 1
        nav._waypoint_index = last
        # Sit beside the final waypoint, still farther from waypoint 0 than
        # from it, and outside the reached radius of both.
        gateway.pose = Pose(x=path[last][0] - 0.30, y=path[last][1], yaw=0.0)

        nav.step()

        assert nav._waypoint_index == last

    def test_pass_by_still_stops_once_the_nearest_waypoint_is_reached(self, tuning):
        """The wraparound must not let the scan walk the whole ring in one tick."""
        path = _square_loop()
        nav, gateway = _navigator(path, Pose(x=path[0][0], y=path[0][1], yaw=0.0), tuning=tuning)

        nav._waypoint_index = 0
        centre = (1.5, 1.5)
        gateway.pose = Pose(*_offset_toward_centre(path[5], centre, 0.25), yaw=0.0)

        nav.step()

        assert nav._waypoint_index < len(path), "must not run away around the loop"
