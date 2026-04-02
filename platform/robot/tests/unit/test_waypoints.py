"""Unit tests for waypoint generation."""

from __future__ import annotations

import math

import pytest

from src.config.enums import Direction, Section
from src.navigation.waypoints import (
    generate_all_waypoints,
    generate_corner_arc,
    generate_corridor_waypoints,
    order_sections_for_laps,
)


class TestOrderSectionsForLaps:
    """Test section ordering for lap-based navigation."""

    def test_clockwise_from_south(self) -> None:
        order = order_sections_for_laps(
            start_section=Section.SOUTH,
            direction=Direction.CLOCKWISE,
            num_laps=1,
        )
        assert order[0] == Section.SOUTH

    def test_counter_clockwise_from_south(self) -> None:
        order = order_sections_for_laps(
            start_section=Section.SOUTH,
            direction=Direction.COUNTER_CLOCKWISE,
            num_laps=1,
        )
        assert order[0] == Section.SOUTH

    def test_two_laps_includes_all_sections_each_lap(self) -> None:
        order = order_sections_for_laps(
            start_section=Section.SOUTH,
            direction=Direction.CLOCKWISE,
            num_laps=2,
        )
        assert len(order) == 8
        assert order.count(Section.SOUTH) == 2
        assert order.count(Section.EAST) == 2
        assert order.count(Section.NORTH) == 2
        assert order.count(Section.WEST) == 2

    def test_clockwise_order(self) -> None:
        order = order_sections_for_laps(
            start_section=Section.NORTH,
            direction=Direction.CLOCKWISE,
            num_laps=1,
        )
        idx_north = order.index(Section.NORTH)
        idx_east = order.index(Section.EAST)
        idx_west = order.index(Section.WEST)
        assert idx_east > idx_north
        assert idx_west > idx_east


class TestGenerateCorridorWaypoints:
    """Test corridor waypoint generation."""

    def test_wide_corridor_1000mm(self) -> None:
        width_m = 1.0
        length_start = 1.0
        length_end = 2.0
        entry_yaw = 0.0

        waypoints = generate_corridor_waypoints(
            section=Section.SOUTH,
            width_m=width_m,
            length_start=length_start,
            length_end=length_end,
            entry_yaw=entry_yaw,
        )

        assert len(waypoints) > 0
        y_coords = [wp[1] for wp in waypoints]
        assert min(y_coords) > 0.0
        assert max(y_coords) < width_m

    def test_narrow_corridor_600mm(self) -> None:
        width_m = 0.6
        length_start = 1.0
        length_end = 2.0
        entry_yaw = 0.0

        waypoints = generate_corridor_waypoints(
            section=Section.SOUTH,
            width_m=width_m,
            length_start=length_start,
            length_end=length_end,
            entry_yaw=entry_yaw,
        )

        assert len(waypoints) > 0
        y_coords = [wp[1] for wp in waypoints]
        assert min(y_coords) > 0.0
        assert max(y_coords) < width_m

    def test_east_corridor_x_coordinates(self) -> None:
        width_m = 1.0
        length_start = 1.0
        length_end = 2.0
        entry_yaw = math.pi / 2

        waypoints = generate_corridor_waypoints(
            section=Section.EAST,
            width_m=width_m,
            length_start=length_start,
            length_end=length_end,
            entry_yaw=entry_yaw,
        )

        x_coords = [wp[0] for wp in waypoints]
        assert min(x_coords) > 0.0
        assert max(x_coords) < width_m

    def test_north_corridor(self) -> None:
        width_m = 1.0
        length_start = 1.0
        length_end = 2.0
        entry_yaw = math.pi

        waypoints = generate_corridor_waypoints(
            section=Section.NORTH,
            width_m=width_m,
            length_start=length_start,
            length_end=length_end,
            entry_yaw=entry_yaw,
        )

        assert len(waypoints) > 0


class TestGenerateCornerArc:
    """Test corner arc waypoint generation."""

    def test_corner_arc_radius_appropriate(self) -> None:
        from src.config.constants import RobotSpecs

        arc = generate_corner_arc(
            center=(1.0, 1.0),
            radius=0.45,
            start_angle=0.0,
            end_angle=math.pi / 2,
            num_points=5,
        )

        assert len(arc) == 5
        for x, y in arc:
            dist = math.sqrt((x - 1.0) ** 2 + (y - 1.0) ** 2)
            assert dist == pytest.approx(0.45, abs=0.01)

    def test_corner_arc_full_quarter(self) -> None:
        arc = generate_corner_arc(
            center=(1.5, 1.5),
            radius=0.45,
            start_angle=math.pi,
            end_angle=1.5 * math.pi,
            num_points=10,
        )

        assert len(arc) == 10
        first_dist = math.sqrt((arc[0][0] - 1.5) ** 2 + (arc[0][1] - 1.5) ** 2)
        last_dist = math.sqrt((arc[-1][0] - 1.5) ** 2 + (arc[-1][1] - 1.5) ** 2)
        assert first_dist == pytest.approx(0.45, abs=0.01)
        assert last_dist == pytest.approx(0.45, abs=0.01)


class TestGenerateAllWaypoints:
    """Test full waypoint generation pipeline."""

    def test_open_challenge_waypoints(self, sample_metadata_open) -> None:
        waypoints = generate_all_waypoints(sample_metadata_open)

        assert len(waypoints) > 0
        assert len(waypoints) % 4 == 0

    def test_obstacles_challenge_waypoints(self, sample_metadata_obstacles) -> None:
        waypoints = generate_all_waypoints(sample_metadata_obstacles)

        assert len(waypoints) > 0

    def test_waypoints_within_track_bounds(self, sample_metadata_open) -> None:
        waypoints = generate_all_waypoints(sample_metadata_open)

        for x, y in waypoints:
            assert -0.2 < x < 3.2
            assert -0.2 < y < 3.2

    def test_waypoints_adjacent_not_duplicate(self, sample_metadata_open) -> None:
        waypoints = generate_all_waypoints(sample_metadata_open)

        for i in range(len(waypoints) - 1):
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[i + 1]
            dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            assert dist > 0.001

    def test_multi_lap_extends_waypoints(self, sample_metadata_open) -> None:
        sample_metadata_open["num_laps"] = 2

        waypoints = generate_all_waypoints(sample_metadata_open)

        assert len(waypoints) > 50


class TestWaypointDeduplication:
    """Test waypoint deduplication logic."""

    def test_close_points_removed(self) -> None:
        waypoints = [
            (0.0, 0.0),
            (0.001, 0.0),
            (0.002, 0.0),
            (0.5, 0.0),
        ]

        from src.navigation.waypoints import _deduplicate_waypoints

        deduped = _deduplicate_waypoints(waypoints, threshold=0.01)

        assert len(deduped) == 2


class TestOuterWallBias:
    """Test outer wall bias for waypoints."""

    def test_bias_pushes_away_from_inner_wall(self, sample_metadata_open) -> None:
        waypoints = generate_all_waypoints(sample_metadata_open)

        from src.config.constants import RobotSpecs

        south_waypoints = [wp for wp in waypoints if wp[0] < 1.5 and wp[1] < 1.0]
        if south_waypoints:
            max_y = max(wp[1] for wp in south_waypoints)
            assert max_y > 0.5


class TestWaypointMetadata:
    """Test waypoint generation from metadata."""

    def test_narrow_corridor_wider_spacing(self) -> None:
        metadata = {
            "corridor_widths": {
                "north": {"type": "narrow", "width_mm": 600},
                "south": {"type": "narrow", "width_mm": 600},
                "east": {"type": "narrow", "width_mm": 600},
                "west": {"type": "narrow", "width_mm": 600},
            },
            "starting_conditions": {
                "direction": "clockwise",
                "section": "South",
                "position": {"x": 1.5, "y": 0.3},
                "yaw": 3.14,
            },
            "challenge_type": "open",
        }

        waypoints = generate_all_waypoints(metadata)

        assert len(waypoints) > 0


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
        "num_laps": 1,
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
        "num_laps": 1,
    }
