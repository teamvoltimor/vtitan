"""Unit tests for waypoint generation."""

from __future__ import annotations

import math

import pytest

from shared.config.enums import Direction, Section
from src.navigation.waypoints import (
    _arc_with_endpoints,
    _build_corridor_order,
    _deduplicate_consecutive,
    _rotate_to_start,
    _straight_waypoints,
    calculate_waypoints,
)


class TestOrderSectionsForLaps:
    """Test section ordering for lap-based navigation."""

    def test_clockwise_order(self) -> None:
        order = _build_corridor_order(Direction.CLOCKWISE)
        assert order == [Section.EAST, Section.SOUTH, Section.WEST, Section.NORTH]

    def test_counter_clockwise_order(self) -> None:
        order = _build_corridor_order(Direction.COUNTERCLOCKWISE)
        assert order == [Section.EAST, Section.NORTH, Section.WEST, Section.SOUTH]

    def test_rotate_to_start(self) -> None:
        order = [Section.EAST, Section.SOUTH, Section.WEST, Section.NORTH]
        rotated = _rotate_to_start(order, Section.WEST)
        assert rotated == [Section.WEST, Section.NORTH, Section.EAST, Section.SOUTH]


class TestGenerateCorridorWaypoints:
    """Test corridor waypoint generation."""

    def test_straight_waypoints_x(self) -> None:
        pts = _straight_waypoints(1.5, is_x=True, start=0.0, end=1.0, count=3)
        assert len(pts) == 3
        assert pts == [(1.5, 0.0), (1.5, 0.5), (1.5, 1.0)]

    def test_straight_waypoints_y(self) -> None:
        pts = _straight_waypoints(0.5, is_x=False, start=1.0, end=2.0, count=2)
        assert len(pts) == 2
        assert pts == [(1.0, 0.5), (2.0, 0.5)]


class TestGenerateCornerArc:
    """Test corner arc waypoint generation."""

    def test_corner_arc_radius_appropriate(self) -> None:
        arc = _arc_with_endpoints(
            center=(1.0, 1.0),
            radius=0.45,
            theta_start=0.0,
            theta_end=math.pi / 2,
            num_intermediate=3,
        )

        assert len(arc) == 5
        for x, y in arc:
            dist = math.sqrt((x - 1.0) ** 2 + (y - 1.0) ** 2)
            assert dist == pytest.approx(0.45, abs=0.01)

    def test_corner_arc_endpoints(self) -> None:
        arc = _arc_with_endpoints(
            center=(1.5, 1.5),
            radius=0.45,
            theta_start=math.pi,
            theta_end=1.5 * math.pi,
            num_intermediate=1,
        )

        assert len(arc) == 3
        # Start should be around (1.05, 1.5)
        assert arc[0][0] == pytest.approx(1.05, abs=0.01)
        assert arc[0][1] == pytest.approx(1.5, abs=0.01)
        # End should be around (1.5, 1.05)
        assert arc[-1][0] == pytest.approx(1.5, abs=0.01)
        assert arc[-1][1] == pytest.approx(1.05, abs=0.01)


class TestGenerateAllWaypoints:
    """Test full waypoint generation pipeline."""

    def test_open_challenge_waypoints(self, sample_metadata_open) -> None:
        waypoints = calculate_waypoints(sample_metadata_open, num_laps=1)
        assert len(waypoints) > 20

    def test_obstacles_challenge_waypoints(self, sample_metadata_obstacles) -> None:
        waypoints = calculate_waypoints(sample_metadata_obstacles, num_laps=1)
        assert len(waypoints) > 20

    def test_waypoints_within_track_bounds(self, sample_metadata_open) -> None:
        waypoints = calculate_waypoints(sample_metadata_open, num_laps=1)

        for x, y in waypoints:
            assert -0.2 < x < 3.2
            assert -0.2 < y < 3.2

    def test_multi_lap_extends_waypoints(self, sample_metadata_open) -> None:
        waypoints_1_lap = calculate_waypoints(sample_metadata_open, num_laps=1)
        waypoints_2_laps = calculate_waypoints(sample_metadata_open, num_laps=2)

        assert len(waypoints_2_laps) > len(waypoints_1_lap)


class TestWaypointDeduplication:
    """Test waypoint deduplication logic."""

    def test_close_points_removed(self) -> None:
        waypoints = [
            (0.0, 0.0),
            (0.0001, 0.0),
            (0.0002, 0.0),
            (0.5, 0.0),
        ]

        deduped = _deduplicate_consecutive(waypoints)
        assert len(deduped) == 2


@pytest.fixture
def sample_metadata_open():
    """Sample metadata for open challenge."""
    return {
        "scenario_id": 0,
        "challenge_type": "open",
        "corridor_widths": {
            "north": {"type": "wide", "width_mm": 1000},
            "south": {"type": "narrow", "width_mm": 600},
            "east": {"type": "wide", "width_mm": 1000},
            "west": {"type": "wide", "width_mm": 1000},
        },
        "starting_conditions": {
            "direction": "clockwise",
            "section": "South",
            "position": {"x": 1.5, "y": 0.3},
            "yaw": 3.14,
        },
    }


@pytest.fixture
def sample_metadata_obstacles():
    """Sample metadata for obstacles challenge."""
    return {
        "scenario_id": 1,
        "challenge_type": "obstacles",
        "corridor_widths": {
            "north": {"type": "wide", "width_mm": 1000},
            "south": {"type": "wide", "width_mm": 1000},
            "east": {"type": "wide", "width_mm": 1000},
            "west": {"type": "wide", "width_mm": 1000},
        },
        "starting_conditions": {
            "direction": "clockwise",
            "section": "South",
            "position": {"x": 1.5, "y": 0.5},
            "yaw": 3.14,
        },
    }
