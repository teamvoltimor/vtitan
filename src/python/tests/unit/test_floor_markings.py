"""Unit tests for the floor-marking geometry helpers."""

from __future__ import annotations

import math

import pytest
from shared.domain.enums import Section

from src.simulation.floor_markings import (
    corner_lines,
    starting_square_band_divisions,
    starting_square_geometry,
)


@pytest.mark.parametrize(("width", "expected_divisions"), [(1.0, [0.4, 0.6]), (0.6, [0.4])])
def test_band_divisions_match_corridor_width(width: float, expected_divisions: list[float]) -> None:
    assert starting_square_band_divisions(width) == pytest.approx(expected_divisions)


def test_starting_square_corners_are_axis_aligned() -> None:
    square = starting_square_geometry(Section.SOUTH)
    corners = square.corners(1.0)
    assert len(corners) == 4
    assert corners[0] == pytest.approx((1.0, 0.0))
    assert corners[1] == pytest.approx((2.0, 0.0))
    assert corners[2] == pytest.approx((2.0, 1.0))
    assert corners[3] == pytest.approx((1.0, 1.0))


def test_north_starting_square_is_reflected_across_top() -> None:
    square = starting_square_geometry(Section.NORTH)
    corners = square.corners(1.0)
    assert corners[0] == pytest.approx((1.0, 2.0))
    assert corners[2] == pytest.approx((2.0, 3.0))


def test_east_starting_square_runs_along_right_wall() -> None:
    square = starting_square_geometry(Section.EAST)
    corners = square.corners(0.6)
    assert corners[0] == pytest.approx((2.4, 1.0))
    assert corners[2] == pytest.approx((3.0, 2.0))


def test_across_line_matches_outer_wall_distance() -> None:
    square = starting_square_geometry(Section.WEST)
    a, b = square.across_line(0.4)
    assert a == pytest.approx((0.4, 1.0))
    assert b == pytest.approx((0.4, 2.0))


def test_along_line_splits_cells_at_corridor_centre() -> None:
    square = starting_square_geometry(Section.SOUTH)
    a, b = square.along_line(1.0)
    assert a == pytest.approx((1.5, 0.0))
    assert b == pytest.approx((1.5, 1.0))


def test_corner_lines_start_at_inner_block_corners() -> None:
    lines = corner_lines()
    assert len(lines) == 8
    starts = {line.start for line in lines}
    assert starts == {(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)}


def test_corner_lines_are_at_30_degrees_to_a_wall() -> None:
    """Each corner line is 30° from one inner-block wall and 60° from the other."""
    lines = corner_lines()
    for line in lines:
        dx = abs(line.end[0] - line.start[0])
        dy = abs(line.end[1] - line.start[1])
        angle = math.degrees(math.atan2(dy, dx))
        acute = min(angle, 90.0 - angle)
        assert acute == pytest.approx(30.0, abs=0.1)


def test_corner_lines_end_on_outer_walls() -> None:
    """Each line reaches the track border, not an arbitrary short length."""
    for line in corner_lines():
        end_x, end_y = line.end
        on_vertical_wall = math.isclose(end_x, 0.0) or math.isclose(end_x, 3.0)
        on_horizontal_wall = math.isclose(end_y, 0.0) or math.isclose(end_y, 3.0)
        assert on_vertical_wall or on_horizontal_wall


def test_corner_lines_stay_inside_corner_regions() -> None:
    """No line drifts into a corridor; both endpoints are within the 1m corner square."""
    for line in corner_lines():
        for x, y in (line.start, line.end):
            assert 0.0 <= x <= 3.0
            assert 0.0 <= y <= 3.0
            # At least one coordinate is on the inner-block side of the corner.
            assert (x <= 2.0 and y <= 2.0) or (x >= 1.0 and y <= 2.0) or (x >= 1.0 and y >= 1.0) or (x <= 2.0 and y >= 1.0)
