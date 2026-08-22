"""What the pose trail can say about the space behind the chassis.

The mount lost its rear slot (2026-08-22), so every LIDAR-based reverse gate now
refuses. The trail is the one remaining statement about that space that needs no
rear vision: ground the chassis physically occupied. These pin what it may and
may not claim.
"""

from __future__ import annotations

import math

import pytest
from shared.config.constants import RobotSpecs

from src.navigation.utils import trail_clearance_behind


def _straight_trail(length_m: float, spacing_m: float = 0.01) -> list[tuple[float, float, float]]:
    """Breadcrumbs along +x, oldest first, ending at the origin."""
    count = int(length_m / spacing_m)
    return [(-(count - i) * spacing_m, 0.0, 0.0) for i in range(count + 1)]


class TestStraightTrail:
    def test_reports_the_ground_actually_covered(self) -> None:
        trail = _straight_trail(0.5)
        assert trail_clearance_behind(trail, 0.0, 0.0, 0.0) == pytest.approx(0.5, abs=0.01)

    def test_empty_trail_is_no_evidence_not_open_road(self) -> None:
        """The distinction the whole gate turns on. None must not read as clear."""
        assert trail_clearance_behind([], 0.0, 0.0, 0.0) is None

    def test_a_trail_entirely_ahead_offers_nothing(self) -> None:
        ahead = [(0.1 * i, 0.0, 0.0) for i in range(1, 6)]
        assert trail_clearance_behind(ahead, 0.0, 0.0, 0.0) is None


class TestTrailLeavingTheFootprint:
    def test_stops_where_the_trail_curves_out_of_the_chassis_corridor(self) -> None:
        """A trail that swings wide stops describing the path a reverse takes.

        Straight for 0.20 m, then stepping sideways past the half-width. Only
        the straight part is ground the footprint would re-cover going back.
        """
        half_width = RobotSpecs.WIDTH / 2.0
        trail = [(-0.40, half_width * 4, 0.0), (-0.30, half_width * 3, 0.0)]
        trail += [(-0.20 + 0.01 * i, 0.0, 0.0) for i in range(21)]
        assert trail_clearance_behind(trail, 0.0, 0.0, 0.0) == pytest.approx(0.20, abs=0.01)

    def test_lateral_drift_inside_the_footprint_still_counts(self) -> None:
        half_width = RobotSpecs.WIDTH / 2.0
        trail = [(-0.01 * i, half_width * 0.5, 0.0) for i in range(31)]
        assert trail_clearance_behind(trail, 0.0, 0.0, 0.0) == pytest.approx(0.30, abs=0.01)


class TestChassisFrame:
    def test_measured_along_the_heading_not_the_world_axis(self) -> None:
        """Rotating robot and trail together must not change the answer."""
        straight = _straight_trail(0.4)
        flat = trail_clearance_behind(straight, 0.0, 0.0, 0.0)

        yaw = math.radians(37.0)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        rotated = [(x * cos_yaw - y * sin_yaw, x * sin_yaw + y * cos_yaw, yaw) for x, y, _ in straight]
        assert trail_clearance_behind(rotated, 0.0, 0.0, yaw) == pytest.approx(flat, abs=1e-6)

    def test_a_trail_behind_in_the_world_but_beside_the_chassis_is_rejected(self) -> None:
        """Facing +y, so ground at -x is abeam, not behind. Reversing there
        would leave the footprint immediately."""
        beside = [(-0.01 * i, 0.0, 0.0) for i in range(31)]
        assert trail_clearance_behind(beside, 0.0, 0.0, math.pi / 2) is None
