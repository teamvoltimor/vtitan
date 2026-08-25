"""Unit tests for waypoint generation."""

from __future__ import annotations

import math

import pytest
from shared.config.constants import CorridorDimensions, RobotSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Direction, Section
from shared.domain.models import CorridorWidthEntry, CorridorWidths, Waypoint

from src.navigation.planning.waypoints import (
    arc_with_endpoints,
    corner_arc_radius,
    deduplicate_consecutive,
    straight_waypoints,
    calculate_waypoints,
    corridor_for_position,
    validate_path_feasibility,
)


@pytest.fixture()
def tuning():
    return NavigationTuning.load_default()


class TestOrderSectionsForLaps:
    """Test section ordering for lap-based navigation."""

    def test_clockwise_order_from_south(self) -> None:
        order = Section.loop_order(Section.SOUTH, Direction.CLOCKWISE)
        assert order == [Section.SOUTH, Section.WEST, Section.NORTH, Section.EAST]

    def test_counter_clockwise_order_from_south(self) -> None:
        order = Section.loop_order(Section.SOUTH, Direction.COUNTERCLOCKWISE)
        assert order == [Section.SOUTH, Section.EAST, Section.NORTH, Section.WEST]

    def test_loop_order_rotates_to_start(self) -> None:
        order = Section.loop_order(Section.WEST, Direction.CLOCKWISE)
        assert order == [Section.WEST, Section.NORTH, Section.EAST, Section.SOUTH]


class TestGenerateCorridorWaypoints:
    """Test corridor waypoint generation."""

    def test_straight_waypoints_x(self) -> None:
        pts = straight_waypoints(1.5, is_x=True, start=0.0, end=1.0, count=3)
        assert len(pts) == 3
        assert pts == [Waypoint(1.5, 0.0), Waypoint(1.5, 0.5), Waypoint(1.5, 1.0)]

    def test_straight_waypoints_y(self) -> None:
        pts = straight_waypoints(0.5, is_x=False, start=1.0, end=2.0, count=2)
        assert len(pts) == 2
        assert pts == [Waypoint(1.0, 0.5), Waypoint(2.0, 0.5)]


class TestGenerateCornerArc:
    """Test corner arc waypoint generation."""

    def test_corner_arc_radius_appropriate(self) -> None:
        arc = arc_with_endpoints(
            center=Waypoint(1.0, 1.0),
            radius=0.45,
            theta_start=0.0,
            theta_end=math.pi / 2,
            num_intermediate=3,
        )

        assert len(arc) == 5
        for wp in arc:
            dist = math.sqrt((wp.x - 1.0) ** 2 + (wp.y - 1.0) ** 2)
            assert dist == pytest.approx(0.45, abs=0.01)

    def test_corner_arc_endpoints(self) -> None:
        arc = arc_with_endpoints(
            center=Waypoint(1.5, 1.5),
            radius=0.45,
            theta_start=math.pi,
            theta_end=1.5 * math.pi,
            num_intermediate=1,
        )

        assert len(arc) == 3
        # Start should be around (1.05, 1.5)
        assert arc[0].x == pytest.approx(1.05, abs=0.01)
        assert arc[0].y == pytest.approx(1.5, abs=0.01)
        # End should be around (1.5, 1.05)
        assert arc[-1].x == pytest.approx(1.5, abs=0.01)
        assert arc[-1].y == pytest.approx(1.05, abs=0.01)


class TestCornerArcRadius:
    """The corner radius is sized by the two corridors the corner joins.

    A single global radius was correct for every corner type except
    narrow-to-narrow, where it cost more than half the available clearance --
    the arc bulged past the centreline and toward the inner block while the
    straights sat comfortably clear.
    """

    _CAP = 0.45
    _BIAS = 0.05
    _NARROW = CorridorDimensions.NARROW
    _WIDE = CorridorDimensions.WIDE

    def test_only_narrow_to_narrow_tightens(self) -> None:
        """Three of the four corner types keep the configured radius.

        Expectations are derived from the same widths the rule reads rather than
        restated as 0.25/0.45, so re-measuring the mat moves the test with the
        geometry instead of turning it red.
        """
        narrow_corner = self._NARROW / 2 - self._BIAS
        assert corner_arc_radius(self._NARROW, self._NARROW, self._BIAS, self._CAP) == pytest.approx(narrow_corner)
        for entry, exit_ in ((self._NARROW, self._WIDE), (self._WIDE, self._NARROW), (self._WIDE, self._WIDE)):
            assert corner_arc_radius(entry, exit_, self._BIAS, self._CAP) == pytest.approx(
                self._WIDE / 2 - self._BIAS
            )

    def test_symmetric_in_entry_and_exit(self) -> None:
        """The same physical corner plans the same arc whichever way it is driven.

        Direction-asymmetric geometry is a recurring source of bugs here (the
        lap line anchored at the measured start, the router's reversed CCW
        rows), so this holds by construction rather than by coincidence.
        """
        for entry, exit_ in ((self._NARROW, self._WIDE), (self._WIDE, self._NARROW), (self._NARROW, self._NARROW)):
            assert corner_arc_radius(entry, exit_, self._BIAS, self._CAP) == pytest.approx(
                corner_arc_radius(exit_, entry, self._BIAS, self._CAP)
            )

    def test_never_exceeds_the_configured_cap(self) -> None:
        """A corridor wider than this track can present still respects the cap."""
        oversized = self._WIDE * 4
        assert corner_arc_radius(oversized, oversized, self._BIAS, self._CAP) == pytest.approx(self._CAP)

    def test_outward_bias_widens_the_arc(self) -> None:
        """An outward bias leaves more room at the corner, so the arc may open up."""
        inward = corner_arc_radius(self._NARROW, self._NARROW, self._BIAS, self._CAP)
        outward = corner_arc_radius(self._NARROW, self._NARROW, -self._BIAS, self._CAP)
        assert outward > inward


class TestPathFeasibility:
    """Feasibility is about fitting the chassis, not about the corner arcs.

    The arcs are capped at the clearance the straights already have, so a corner
    can never be the tightest point -- which is what the old
    ``WIDTH/2 + arc_radius`` form tried and failed to express, since it summed a
    path curvature with a lateral half-extent and ignored the bias entirely.
    """

    def test_bias_consumes_margin(self) -> None:
        centred = validate_path_feasibility(CorridorDimensions.NARROW, 0.0)
        biased = validate_path_feasibility(CorridorDimensions.NARROW, 0.05)
        assert centred.is_feasible
        assert biased.is_feasible
        # Biasing 0.05 off centre spends 0.05 at each wall.
        assert centred.margin_m - biased.margin_m == pytest.approx(0.10)

    def test_bias_direction_does_not_matter(self) -> None:
        assert validate_path_feasibility(CorridorDimensions.NARROW, 0.05).margin_m == pytest.approx(
            validate_path_feasibility(CorridorDimensions.NARROW, -0.05).margin_m
        )

    def test_rejects_a_corridor_the_chassis_cannot_fit(self) -> None:
        """Narrower than the chassis itself, so no bias could rescue it."""
        verdict = validate_path_feasibility(RobotSpecs.WIDTH * 0.75, 0.0)
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

        for wp in waypoints:
            assert -0.2 < wp.x < 3.2
            assert -0.2 < wp.y < 3.2

    def test_multi_lap_extends_waypoints(self, sample_metadata_open, tuning) -> None:
        waypoints_1_lap = calculate_waypoints(sample_metadata_open, num_laps=1, tuning=tuning)
        waypoints_2_laps = calculate_waypoints(sample_metadata_open, num_laps=2, tuning=tuning)

        assert len(waypoints_2_laps) > len(waypoints_1_lap)


class TestWaypointDeduplication:
    """Test waypoint deduplication logic."""

    def test_close_points_removed(self) -> None:
        waypoints = [
            Waypoint(0.0, 0.0),
            Waypoint(0.0001, 0.0),
            Waypoint(0.0002, 0.0),
            Waypoint(0.5, 0.0),
        ]

        deduped = deduplicate_consecutive(waypoints)
        assert len(deduped) == 2


@pytest.fixture()
def sample_metadata_open():
    """Sample metadata for open challenge."""
    return {
        "scenario_id": 0,
        "challenge_type": "open",
        "corridor_widths": CorridorWidths(
            north=CorridorWidthEntry(type="wide", width_mm=1000),
            south=CorridorWidthEntry(type="narrow", width_mm=600),
            east=CorridorWidthEntry(type="wide", width_mm=1000),
            west=CorridorWidthEntry(type="wide", width_mm=1000),
        ).model_dump(),
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
        "corridor_widths": CorridorWidths().model_dump(),
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


class TestCenterBiasOverride:
    """``center_bias_m`` must be inert unless a caller asks for it.

    The Obstacles Challenge plans on its own centreline bias
    (``OBSTACLES_CENTER_BIAS_M``), threaded through as an explicit override.
    The Open Challenge passes ``None`` and must therefore be affected in no
    way at all -- this pins that as a property rather than leaving it to the
    call sites to keep getting right.
    """

    def test_none_is_identical_to_the_tuning_default(self, sample_metadata_open, tuning) -> None:
        implicit = calculate_waypoints(sample_metadata_open, num_laps=1, tuning=tuning)
        explicit_none = calculate_waypoints(
            sample_metadata_open, num_laps=1, tuning=tuning, center_bias_m=None
        )
        assert explicit_none == implicit

    def test_passing_the_tuning_value_reproduces_the_default(self, sample_metadata_open, tuning) -> None:
        """The override path and the default path must agree on the same number.

        Guards the derivation itself: if the override were applied with the
        wrong sign or skipped ``CENTER_BIAS_SIDE``, this is where it shows,
        rather than as a silently shifted path in one challenge only.
        """
        implicit = calculate_waypoints(sample_metadata_open, num_laps=1, tuning=tuning)
        explicit = calculate_waypoints(
            sample_metadata_open,
            num_laps=1,
            tuning=tuning,
            center_bias_m=tuning.waypoints.CENTER_BIAS_M,
        )
        assert explicit == implicit

    def test_a_different_value_actually_moves_the_path(self, sample_metadata_open, tuning) -> None:
        """Regression guard for the override: without this the tests above pass on a no-op."""
        implicit = calculate_waypoints(sample_metadata_open, num_laps=1, tuning=tuning)
        shifted = calculate_waypoints(
            sample_metadata_open,
            num_laps=1,
            tuning=tuning,
            center_bias_m=tuning.waypoints.CENTER_BIAS_M + 0.05,
        )
        assert shifted != implicit
