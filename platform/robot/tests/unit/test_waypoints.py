"""Unit tests for waypoint generation."""

from __future__ import annotations

import math

import pytest
from shared.config.enums import Direction, Section
from shared.config.navigation_tuning import NavigationTuning

from src.navigation.planning.waypoints import (
    _arc_with_endpoints,
    _build_corridor_order,
    _corner_arc_radius,
    _deduplicate_consecutive,
    _rotate_to_start,
    _straight_waypoints,
    calculate_waypoints,
    corridor_for_position,
    validate_path_feasibility,
)


@pytest.fixture
def tuning():
    return NavigationTuning.load_default()


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


class TestCornerArcRadius:
    """The corner radius is sized by the two corridors the corner joins.

    A single global radius was correct for every corner type except
    narrow-to-narrow, where it cost more than half the available clearance --
    the arc bulged past the centreline and toward the inner block while the
    straights sat comfortably clear.
    """

    _CAP = 0.45
    _BIAS = 0.05

    def test_only_narrow_to_narrow_tightens(self) -> None:
        """Three of the four corner types keep the configured radius."""
        assert _corner_arc_radius(0.6, 0.6, self._BIAS, self._CAP) == pytest.approx(0.25)
        assert _corner_arc_radius(0.6, 1.0, self._BIAS, self._CAP) == pytest.approx(0.45)
        assert _corner_arc_radius(1.0, 0.6, self._BIAS, self._CAP) == pytest.approx(0.45)
        assert _corner_arc_radius(1.0, 1.0, self._BIAS, self._CAP) == pytest.approx(0.45)

    def test_symmetric_in_entry_and_exit(self) -> None:
        """The same physical corner plans the same arc whichever way it is driven.

        Direction-asymmetric geometry is a recurring source of bugs here (the
        lap line anchored at the measured start, the router's reversed CCW
        rows), so this holds by construction rather than by coincidence.
        """
        for entry, exit_ in ((0.6, 1.0), (1.0, 0.6), (0.6, 0.6)):
            assert _corner_arc_radius(entry, exit_, self._BIAS, self._CAP) == pytest.approx(
                _corner_arc_radius(exit_, entry, self._BIAS, self._CAP)
            )

    def test_never_exceeds_the_configured_cap(self) -> None:
        assert _corner_arc_radius(4.0, 4.0, self._BIAS, self._CAP) == pytest.approx(self._CAP)

    def test_outward_bias_widens_the_arc(self) -> None:
        """An outward bias leaves more room at the corner, so the arc may open up."""
        inward = _corner_arc_radius(0.6, 0.6, 0.05, self._CAP)
        outward = _corner_arc_radius(0.6, 0.6, -0.05, self._CAP)
        assert outward > inward


class TestPathFeasibility:
    """Feasibility is about fitting the chassis, not about the corner arcs.

    The arcs are capped at the clearance the straights already have, so a corner
    can never be the tightest point -- which is what the old
    ``WIDTH/2 + arc_radius`` form tried and failed to express, since it summed a
    path curvature with a lateral half-extent and ignored the bias entirely.
    """

    def test_bias_consumes_margin(self) -> None:
        centred = validate_path_feasibility(0.6, 0.0)
        biased = validate_path_feasibility(0.6, 0.05)
        assert centred.is_feasible
        assert biased.is_feasible
        # Biasing 0.05 off centre spends 0.05 at each wall.
        assert centred.margin_m - biased.margin_m == pytest.approx(0.10)

    def test_bias_direction_does_not_matter(self) -> None:
        assert validate_path_feasibility(0.6, 0.05).margin_m == pytest.approx(
            validate_path_feasibility(0.6, -0.05).margin_m
        )

    def test_rejects_a_corridor_the_chassis_cannot_fit(self) -> None:
        verdict = validate_path_feasibility(0.15, 0.0)
        assert not verdict.is_feasible
        assert verdict.reason


class TestGenerateAllWaypoints:
    """Test full waypoint generation pipeline."""

    def test_open_challenge_waypoints(self, sample_metadata_open, tuning) -> None:
        waypoints = calculate_waypoints(sample_metadata_open, num_laps=1, tuning=tuning)
        assert len(waypoints) > 20

    def test_obstacles_challenge_waypoints(self, sample_metadata_obstacles, tuning) -> None:
        waypoints = calculate_waypoints(sample_metadata_obstacles, num_laps=1, tuning=tuning)
        assert len(waypoints) > 20

    def test_waypoints_within_track_bounds(self, sample_metadata_open, tuning) -> None:
        waypoints = calculate_waypoints(sample_metadata_open, num_laps=1, tuning=tuning)

        for x, y in waypoints:
            assert -0.2 < x < 3.2
            assert -0.2 < y < 3.2

    def test_multi_lap_extends_waypoints(self, sample_metadata_open, tuning) -> None:
        waypoints_1_lap = calculate_waypoints(sample_metadata_open, num_laps=1, tuning=tuning)
        waypoints_2_laps = calculate_waypoints(sample_metadata_open, num_laps=2, tuning=tuning)

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


@pytest.fixture()
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


@pytest.fixture()
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


class TestCorridorForPosition:
    """Tests for corridor_for_position()."""

    def test_south_corridor(self):
        assert corridor_for_position(1.5, 0.5) == Section.SOUTH

    def test_north_corridor(self):
        assert corridor_for_position(1.5, 2.5) == Section.NORTH

    def test_east_corridor(self):
        assert corridor_for_position(2.5, 1.5) == Section.EAST

    def test_west_corridor(self):
        assert corridor_for_position(0.5, 1.5) == Section.WEST

    def test_south_boundary(self):
        assert corridor_for_position(1.5, 0.99) == Section.SOUTH

    def test_north_boundary(self):
        assert corridor_for_position(1.5, 2.01) == Section.NORTH

    def test_east_boundary(self):
        assert corridor_for_position(2.01, 1.5) == Section.EAST

    def test_west_boundary(self):
        assert corridor_for_position(0.99, 1.5) == Section.WEST

    def test_sw_corner_classifies_to_nearest(self):
        # Point (0.5, 0.5): dist_s=0.5, dist_w=0.5 → tie goes to south (checked first)
        result = corridor_for_position(0.5, 0.5)
        assert result in (Section.SOUTH, Section.WEST)

    def test_ne_corner_classifies_to_nearest(self):
        result = corridor_for_position(2.5, 2.5)
        assert result in (Section.NORTH, Section.EAST)

    def test_all_four_sections_reachable(self):
        results = {
            corridor_for_position(1.5, 0.3),
            corridor_for_position(1.5, 2.7),
            corridor_for_position(2.7, 1.5),
            corridor_for_position(0.3, 1.5),
        }
        assert results == {Section.SOUTH, Section.NORTH, Section.EAST, Section.WEST}
