"""A replan must not hand back a waypoint the robot has already driven past.

``CoreNavigator.replace_path`` seeks the nearest waypoint across the whole
rebuilt path. A large FORWARD jump has been guarded since the start/finish-seam
fix; the backward direction never was, and on hardware it moves: ``19 -> 16`` in
``normal_drive``, corridor north, lap 0, in BOTH 3-lap runs of 2026-09-08, each
while the yaw was swinging through a corner.

It matters because ``WaypointController`` answers a target behind the chassis
with FULL LOCK rather than a curvature, so a backward re-seek is a U-turn
command waiting for the geometry to allow it.

``FORWARD_ONLY_RESEEK`` ships FALSE, so the first test here is the one that
protects everything else: the default must still reproduce the old behaviour
exactly.
"""

from __future__ import annotations

import pytest
from shared.domain.models import Pose, Waypoint

from src.config.tuning_helpers import tuning_with_overrides
from src.navigation.core_navigator import CoreNavigator
from src.navigation.ports import LidarScan
from tests.fixtures import FakeGateway, create_scan_with_sectors
from tests.test_constants import ANGLES_FULL_ROTATION

_PATH_LEN = 20
"""Long enough that half of it is a meaningful bound, short enough to read."""


def _straight_path(n: int = _PATH_LEN) -> list[Waypoint]:
    """A path along +x, one waypoint every 0.25 m."""
    return [Waypoint(0.25 * i, 0.0) for i in range(n)]


def _navigator(tuning, waypoints):
    """A navigator parked at the origin with an unobstructed scan."""
    ranges = create_scan_with_sectors(front=3.0, back=3.0)
    gateway = FakeGateway(
        Pose(x=0.0, y=0.0, yaw=0.0),
        LidarScan(ranges_m=tuple(ranges), angles_rad=tuple(ANGLES_FULL_ROTATION.tolist())),
    )
    return CoreNavigator(gateway=gateway, waypoints=list(waypoints), num_laps=1, tuning=tuning)


@pytest.fixture()
def path() -> list[Waypoint]:
    return _straight_path()


class TestShippedDefaultIsUnchanged:
    def test_the_flag_ships_false(self) -> None:
        assert tuning_with_overrides({}).waypoints.FORWARD_ONLY_RESEEK is False

    def test_default_still_seeks_backward(self, path) -> None:
        """Without the flag the re-seek takes the nearest point, behind or not.

        This is the behaviour measured on hardware. Asserting it keeps the
        default honest: if it ever changes, it changes here first.
        """
        tuning = tuning_with_overrides({})
        nav = _navigator(tuning, path)
        nav._waypoint_index = 19

        nav.replace_path(list(path), robot_xy=(0.25 * 16, 0.0), robot_yaw=0.0)

        assert nav._waypoint_index == 16


class TestForwardOnly:
    def test_a_backward_seek_is_refused(self, path) -> None:
        """The hardware case: index 19, nearest is 16, keep 19."""
        tuning = tuning_with_overrides({"FORWARD_ONLY_RESEEK": True}, group="waypoints")
        nav = _navigator(tuning, path)
        nav._waypoint_index = 19

        nav.replace_path(list(path), robot_xy=(0.25 * 16, 0.0), robot_yaw=0.0)

        assert nav._waypoint_index == 19

    def test_a_forward_seek_is_still_allowed(self, path) -> None:
        """Progress is progress; the guard is one-directional by design."""
        tuning = tuning_with_overrides({"FORWARD_ONLY_RESEEK": True}, group="waypoints")
        nav = _navigator(tuning, path)
        nav._waypoint_index = 4

        nav.replace_path(list(path), robot_xy=(0.25 * 8, 0.0), robot_yaw=0.0)

        assert nav._waypoint_index == 8

    def test_a_seam_sized_backward_jump_is_left_alone(self, path) -> None:
        """Beyond half the path it is the seam, which the other guard owns.

        Clamping it here would fight that guard rather than complement it.
        """
        tuning = tuning_with_overrides({"FORWARD_ONLY_RESEEK": True}, group="waypoints")
        nav = _navigator(tuning, path)
        nav._waypoint_index = _PATH_LEN - 1

        nav.replace_path(list(path), robot_xy=(0.0, 0.0), robot_yaw=0.0)

        assert nav._waypoint_index == 0

    def test_a_resized_path_is_left_alone(self, path) -> None:
        """Different waypoint counts index different positions.

        "Backward" compares two numbers that no longer describe the same points,
        so the guard must not fire on them.
        """
        tuning = tuning_with_overrides({"FORWARD_ONLY_RESEEK": True}, group="waypoints")
        nav = _navigator(tuning, path)
        nav._waypoint_index = 19

        shorter = _straight_path(_PATH_LEN - 4)
        nav.replace_path(shorter, robot_xy=(0.25 * 10, 0.0), robot_yaw=0.0)

        assert nav._waypoint_index == 10
