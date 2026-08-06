"""The start pose must be read off the track, not assumed from geometry."""

from __future__ import annotations

import math

import numpy as np
import pytest
from shared.config.constants import RobotSpecs, TrackDimensions
from shared.config.enums import Direction, Section

from src.navigation.start_measurement import CLOSING_TOLERANCE_M, measure_start_pose
from src.navigation.track_geometry import TrackWalls, corridor_geometry_from_widths

_MAT = TrackDimensions.MAX_COORD
_WIDE = dict.fromkeys(Section, 1.0)


def _scan(x: float, y: float, yaw: float, widths: dict[Section, float] | None = None) -> tuple[np.ndarray, np.ndarray]:
    """A noise-free scan taken at a known pose on a known layout."""
    walls = TrackWalls(corridor_geometry_from_widths(widths or _WIDE))
    angles = np.linspace(-math.pi, math.pi, RobotSpecs.LIDAR_SAMPLES, endpoint=False)
    return walls.raycast(x, y, yaw, angles), angles


class TestMeasuredPose:
    """Recovering position from the four cardinal rays."""

    @pytest.mark.parametrize(
        ("x", "y", "direction", "yaw"),
        [
            # The two legal along-corridor cell centres, middle band.
            (1.25, 0.497, Direction.COUNTERCLOCKWISE, 0.0),
            (1.75, 0.497, Direction.COUNTERCLOCKWISE, 0.0),
            (1.25, 0.497, Direction.CLOCKWISE, math.pi),
            (1.75, 0.497, Direction.CLOCKWISE, math.pi),
            # Outer and inner bands.
            (1.25, 0.303, Direction.COUNTERCLOCKWISE, 0.0),
            (1.75, 0.697, Direction.CLOCKWISE, math.pi),
            # Placed OUTSIDE the marked square, which is what actually happened
            # on 2026-08-05 -- the measurement must not care.
            (2.30, 0.500, Direction.COUNTERCLOCKWISE, 0.0),
            (0.70, 0.500, Direction.CLOCKWISE, math.pi),
        ],
    )
    def test_recovers_the_pose_it_was_taken_at(self, x: float, y: float, direction: Direction, yaw: float) -> None:
        ranges, angles = _scan(x, y, yaw)

        measured = measure_start_pose(ranges, angles, direction)

        assert measured is not None
        assert measured.x == pytest.approx(x, abs=0.02)
        assert measured.y == pytest.approx(y, abs=0.02)

    def test_reports_the_track_actually_left_ahead(self) -> None:
        """The number whose absence lost the 2026-08-05 rounds.

        Placed at 2.30 travelling counterclockwise there is 0.70 m to the wall,
        not the ~1.5 m a pose assumed at the middle of the side implies.
        """
        ranges, angles = _scan(2.30, 0.50, 0.0)

        measured = measure_start_pose(ranges, angles, Direction.COUNTERCLOCKWISE)

        assert measured is not None
        assert measured.distance_ahead_m == pytest.approx(0.70, abs=0.02)

    def test_measures_corridor_width_when_beside_the_inner_block(self) -> None:
        """Level with the block, the side rays span the corridor and nothing else."""
        ranges, angles = _scan(1.25, 0.497, 0.0, widths=_WIDE)

        measured = measure_start_pose(ranges, angles, Direction.COUNTERCLOCKWISE)

        assert measured is not None
        assert measured.corridor_width_m == pytest.approx(1.0, abs=0.03)

    def test_reports_no_width_when_level_with_a_corner(self) -> None:
        """Past the block both side rays reach outer walls, measuring the mat.

        Returning that as a corridor width would hand the width estimator a
        3 m corridor, so it has to be withheld rather than reported.
        """
        ranges, angles = _scan(2.60, 0.50, 0.0)

        measured = measure_start_pose(ranges, angles, Direction.COUNTERCLOCKWISE)

        assert measured is not None
        assert measured.corridor_width_m is None

    def test_section_is_a_free_relabelling(self) -> None:
        """The same scan read against another section gives that section's pose.

        With equal corridors the track is invariant under a quarter turn, so the
        robot's choice of starting section is a label, not a claim -- the
        measured pose must rotate with it and stay self-consistent.
        """
        ranges, angles = _scan(1.25, 0.497, 0.0)

        south = measure_start_pose(ranges, angles, Direction.COUNTERCLOCKWISE, section=Section.SOUTH)
        east = measure_start_pose(ranges, angles, Direction.COUNTERCLOCKWISE, section=Section.EAST)

        assert south is not None
        assert east is not None
        assert (east.x, east.y) == pytest.approx((_MAT - south.y, south.x), abs=1e-6)


class TestRefusesToGuess:
    """When the scan cannot support a measurement, saying so beats assuming."""

    def test_rejects_a_blocked_ray(self) -> None:
        """An operator still standing over the robot is the realistic case.

        The rays either side of the mat must span it; a hand at 0.3 m where the
        wall is at 2.3 m breaks that by two metres, far outside any tolerance
        an imperfect track needs.
        """
        ranges, angles = _scan(1.25, 0.497, 0.0)
        blocked = np.asarray(ranges, dtype=float).copy()
        blocked[np.abs(np.arctan2(np.sin(angles - math.pi), np.cos(angles - math.pi))) < math.radians(6)] = 0.3

        assert measure_start_pose(blocked, angles, Direction.COUNTERCLOCKWISE) is None

    def test_accepts_a_track_that_is_not_perfect(self) -> None:
        """Real mats are not nominal, and rejecting them would be useless.

        Two real rounds closed to 2.978 m and 2.971 m against a nominal 3.0
        before any noise, so a measurement must survive several centimetres of
        error in the mat itself.
        """
        ranges, angles = _scan(1.25, 0.497, 0.0)
        shrunk = np.asarray(ranges, dtype=float) * (1.0 - 0.9 * CLOSING_TOLERANCE_M / _MAT)

        assert measure_start_pose(shrunk, angles, Direction.COUNTERCLOCKWISE) is not None

    def test_rejects_when_no_ray_returns(self) -> None:
        """Off the track entirely -- a bench, a table -- measures nothing."""
        angles = np.linspace(-math.pi, math.pi, RobotSpecs.LIDAR_SAMPLES, endpoint=False)
        empty = np.full_like(angles, RobotSpecs.LIDAR_MAX_RANGE)

        assert measure_start_pose(empty, angles, Direction.COUNTERCLOCKWISE) is None
