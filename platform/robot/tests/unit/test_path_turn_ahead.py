"""``path_turn_ahead`` must see a corner before the robot has missed it.

This is the leading signal that crosstrack error cannot be: crosstrack only
rises once the turn has already been run wide of, which is why the corrective
short lookahead used to arrive a corner late (hardware, 2026-08-06).
"""

from __future__ import annotations

import math

import pytest

from src.navigation.track_geometry import path_turn_ahead


def _straight(n: int = 20, spacing: float = 0.05) -> list[tuple[float, float]]:
    return [(i * spacing, 0.0) for i in range(n)]


def _right_angle(per_leg: int = 20, spacing: float = 0.05) -> list[tuple[float, float]]:
    """An L: east along y=0, then north at the corner. Closed by the caller's
    own wraparound, which is what the ring walk relies on."""
    east = [(i * spacing, 0.0) for i in range(per_leg)]
    north = [((per_leg - 1) * spacing, i * spacing) for i in range(1, per_leg)]
    return east + north


class TestStraightsReadFlat:
    def test_a_straight_reads_zero(self):
        assert path_turn_ahead(_straight(), 0, 0.40) == pytest.approx(0.0, abs=1e-9)

    def test_preview_of_zero_reads_zero(self):
        assert path_turn_ahead(_right_angle(), 18, 0.0) == pytest.approx(0.0)

    def test_a_degenerate_path_reads_zero(self):
        assert path_turn_ahead([(0.0, 0.0), (1.0, 0.0)], 0, 0.40) == pytest.approx(0.0)


class TestCornersReadAhead:
    def test_a_right_angle_reads_the_full_turn(self):
        path = _right_angle()
        # Index 18 is one waypoint short of the corner at index 19.
        assert path_turn_ahead(path, 18, 0.40) == pytest.approx(math.pi / 2, abs=1e-6)

    def test_the_corner_is_seen_before_it_is_reached(self):
        """The whole point: the signal must be up while the robot is still on
        the straight, with the corner a preview-distance away."""
        path = _right_angle()
        spacing = 0.05
        corner_index = 19
        preview = 0.40
        approach = corner_index - int(preview / spacing) + 1

        assert path_turn_ahead(path, approach, preview) > 0.35

    def test_far_from_the_corner_reads_flat(self):
        path = _right_angle()
        assert path_turn_ahead(path, 0, 0.40) == pytest.approx(0.0, abs=1e-9)


class TestRingWalk:
    def test_reads_across_the_start_finish_seam(self):
        """Measured from the last waypoints of a lap, the path must not look
        straight just because the list ended -- the loop continues at index 0."""
        # A closed square: the final leg runs west, wrapping into the north-up
        # first leg, so a turn is present only if the walk wraps.
        square: list[tuple[float, float]] = []
        corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        for i, (ax, ay) in enumerate(corners):
            bx, by = corners[(i + 1) % 4]
            for step in range(20):
                t = step / 20
                square.append((ax + (bx - ax) * t, ay + (by - ay) * t))

        last = len(square) - 1
        assert path_turn_ahead(square, last, 0.30) == pytest.approx(math.pi / 2, abs=1e-6)

    def test_turn_is_unsigned(self):
        """Left and right corners must both raise the signal, not cancel."""
        right = _right_angle()
        left = [(x, -y) for x, y in right]
        assert path_turn_ahead(left, 18, 0.40) == pytest.approx(
            path_turn_ahead(right, 18, 0.40),
        )
