"""Unit tests for waypoint generation."""

from __future__ import annotations

import math
from dataclasses import replace

import pytest
from shared.config.constants import CorridorDimensions, RobotSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import CorridorSide, Direction, Section
from shared.domain.models import CorridorWidthEntry, CorridorWidths, Waypoint

from src.navigation.planning.waypoints import (
    arc_with_endpoints,
    calculate_waypoints,
    center_bias_for_corridor,
    corner_arc_radius,
    corridor_for_position,
    deduplicate_consecutive,
    straight_waypoints,
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
        wrong sign or skipped ``WIDE_CENTER_BIAS_SIDE``, this is where it shows,
        rather than as a silently shifted path in one challenge only.

        The override is UNIFORM while the default is per-corridor-width, so the
        two are only comparable when both magnitudes are the same -- that is
        what ``flat`` sets up. Comparing against the shipped tuning instead
        would fail for a legitimate reason (narrow corridors take 0.0) and
        stop testing the sign/side derivation this exists for.
        """
        flat = replace(
            tuning,
            waypoints=tuning.waypoints.model_copy(
                update={"NARROW_CENTER_BIAS_M": tuning.waypoints.WIDE_CENTER_BIAS_M}
            ),
        )
        implicit = calculate_waypoints(sample_metadata_open, num_laps=1, tuning=flat)
        explicit = calculate_waypoints(
            sample_metadata_open,
            num_laps=1,
            tuning=flat,
            center_bias_m=flat.waypoints.WIDE_CENTER_BIAS_M,
        )
        assert explicit == implicit

    def test_narrow_corridors_take_the_narrow_bias(self, sample_metadata_open, tuning) -> None:
        """The narrow/wide split must actually reach the planned path.

        Guards the wiring rather than the geometry: if ``calculate_waypoints``
        regressed to applying one magnitude to all four corridors, moving only
        the NARROW value would stop moving the path and this fails.
        """
        shifted_narrow = replace(
            tuning,
            waypoints=tuning.waypoints.model_copy(
                update={"NARROW_CENTER_BIAS_M": tuning.waypoints.NARROW_CENTER_BIAS_M + 0.05}
            ),
        )
        assert calculate_waypoints(
            sample_metadata_open, num_laps=1, tuning=shifted_narrow
        ) != calculate_waypoints(sample_metadata_open, num_laps=1, tuning=tuning)

    def test_each_width_class_takes_its_own_bias_side(self, tuning) -> None:
        """Narrow and wide must read their OWN side, not a shared one.

        Flipping only the narrow side has to move a narrow corridor's shift and
        leave a wide one's alone. A helper that resolved the magnitude per class
        but the side globally would pass ``test_narrow_corridors_take_the_narrow_bias``
        and still steer a wide corridor the wrong way -- and with the shipped
        narrow magnitude at 0.0 the error would be invisible in the planned path
        until someone tuned it off zero, so the magnitudes are forced non-zero
        here rather than relying on the shipped values.
        """
        narrow_w, wide_w = CorridorDimensions.NARROW, CorridorDimensions.WIDE
        base = tuning.waypoints.model_copy(
            update={"NARROW_CENTER_BIAS_M": 0.05, "WIDE_CENTER_BIAS_M": 0.05}
        )
        flipped = base.model_copy(update={"NARROW_CENTER_BIAS_SIDE": CorridorSide.OUTER})
        base_tuning = replace(tuning, waypoints=base)
        flipped_tuning = replace(tuning, waypoints=flipped)

        assert center_bias_for_corridor(narrow_w, flipped_tuning) == pytest.approx(
            -center_bias_for_corridor(narrow_w, base_tuning)
        )
        assert center_bias_for_corridor(wide_w, flipped_tuning) == pytest.approx(
            center_bias_for_corridor(wide_w, base_tuning)
        )

    def test_a_different_value_actually_moves_the_path(self, sample_metadata_open, tuning) -> None:
        """Regression guard for the override: without this the tests above pass on a no-op."""
        implicit = calculate_waypoints(sample_metadata_open, num_laps=1, tuning=tuning)
        shifted = calculate_waypoints(
            sample_metadata_open,
            num_laps=1,
            tuning=tuning,
            center_bias_m=tuning.waypoints.WIDE_CENTER_BIAS_M + 0.05,
        )
        assert shifted != implicit


class TestUnconfirmedWidthInnerBias:
    """``UNCONFIRMED_WIDTH_INNER_BIAS_M`` pre-positions a still-guessed corridor.

    See the tuning field: a blind round believes every corridor NARROW, both
    hypotheses share the outer wall, and confirming WIDE therefore steps the
    planned centreline 0.30 m inward in one tick. Biasing the unconfirmed line
    inward shortens that step.
    """

    @staticmethod
    def _armed(tuning, magnitude: float):
        return replace(
            tuning,
            waypoints=tuning.waypoints.model_copy(update={"UNCONFIRMED_WIDTH_INNER_BIAS_M": magnitude}),
        )

    def test_zero_is_inert(self, sample_metadata_open, tuning) -> None:
        """At 0.0 an unconfirmed corridor must plan exactly where it always did.

        0.0 is the documented "restores the previous behaviour exactly" escape
        hatch, so declaring every section unconfirmed has to be a no-op there.
        Pinned against the tuning explicitly zeroed rather than against the
        shipped profile, which now ships the field ARMED -- see
        ``test_shipped_default_is_armed``.
        """
        off = self._armed(tuning, 0.0)
        assert calculate_waypoints(
            sample_metadata_open, num_laps=1, tuning=off, unconfirmed_sections=frozenset(Section)
        ) == calculate_waypoints(sample_metadata_open, num_laps=1, tuning=off)

    def test_shipped_default_is_armed(self, sample_metadata_open, tuning) -> None:
        """The shipped value must actually reach the planned path.

        Guards the wiring end-to-end rather than the magnitude: the 640-case
        result this field ships on is worth nothing if a refactor silently
        stops ``unconfirmed_sections`` reaching ``center_bias_for_corridor``,
        and every other test here would still pass on that no-op because they
        construct their own armed tuning.
        """
        assert tuning.waypoints.UNCONFIRMED_WIDTH_INNER_BIAS_M > 0.0
        assert calculate_waypoints(
            sample_metadata_open, num_laps=1, tuning=tuning, unconfirmed_sections=frozenset(Section)
        ) != calculate_waypoints(sample_metadata_open, num_laps=1, tuning=tuning)

    def test_the_arc_is_invariant_to_this_field(self, tuning) -> None:
        """The corner arc must keep reading the CONFIRMED bias at any magnitude.

        ``calculate_waypoints`` sizes each corner with a bias computed without
        ``confirmed=``, so the arc sees the confirmed value and this field can
        only move the straight. That separation is load-bearing: the arc is
        sized by geometry (it has to fit inside the commit clearance) while
        this field is sized by a belief, and coupling them would make a
        pre-positioning tweak silently retune corner entry.

        Pinned across magnitudes well past anything shippable, because the
        version of this test it replaces asserted the property of a
        hand-constructed ``corner_arc_radius`` call rather than of the code
        path, and so would have passed even if the two were coupled.
        """
        for magnitude in (0.0, 0.05, 0.15, 0.30):
            armed = self._armed(tuning, magnitude)
            for width in (CorridorDimensions.NARROW, CorridorDimensions.WIDE):
                assert center_bias_for_corridor(width, armed, None) == pytest.approx(
                    center_bias_for_corridor(width, tuning, None)
                )

    def test_none_and_empty_agree(self, sample_metadata_open, tuning) -> None:
        """"Nothing to say" and "everything confirmed" are the same statement."""
        armed = self._armed(tuning, 0.15)
        assert calculate_waypoints(
            sample_metadata_open, num_laps=1, tuning=armed, unconfirmed_sections=frozenset()
        ) == calculate_waypoints(sample_metadata_open, num_laps=1, tuning=armed, unconfirmed_sections=None)

    def test_unconfirmed_narrow_takes_the_new_bias(self, tuning) -> None:
        armed = self._armed(tuning, 0.15)
        assert center_bias_for_corridor(CorridorDimensions.NARROW, armed, confirmed=False) == pytest.approx(0.15)
        assert center_bias_for_corridor(
            CorridorDimensions.NARROW, armed, confirmed=True
        ) == pytest.approx(armed.waypoints.NARROW_CENTER_BIAS_M)

    def test_wide_corridors_are_untouched(self, tuning) -> None:
        """The field describes the NARROW prior only.

        A corridor already believed wide is not the case this exists for -- it
        has nothing left to confirm into -- so the unconfirmed flag must not
        reach ``WIDE_CENTER_BIAS_M``.
        """
        armed = self._armed(tuning, 0.15)
        assert center_bias_for_corridor(CorridorDimensions.WIDE, armed, confirmed=False) == pytest.approx(
            center_bias_for_corridor(CorridorDimensions.WIDE, armed, confirmed=True)
        )

    def test_explicit_override_still_wins(self, tuning) -> None:
        """Obstacles' swept magnitude must not be reinterpreted by a belief.

        ``OBSTACLES_CENTER_BIAS_M`` was measured as one number; letting an
        unconfirmed width substitute a different one underneath it would make
        the same field mean two things depending on the round.
        """
        armed = self._armed(tuning, 0.15)
        assert center_bias_for_corridor(CorridorDimensions.NARROW, armed, 0.05, confirmed=False) == pytest.approx(
            center_bias_for_corridor(CorridorDimensions.NARROW, armed, 0.05, confirmed=True)
        )

    def test_it_shortens_the_confirm_wide_step(self, tuning) -> None:
        """The point of the field, as a property rather than a comment.

        Both corridors share the fixed OUTER wall, so the planned centreline
        sits ``width/2 + bias`` in from it and the step taken when a belief
        resolves is the difference of those two. Arming the field must shrink
        that distance and must not change where the confirmed-wide line ends
        up -- this changes the START of the move, never its destination.
        """
        narrow, wide = CorridorDimensions.NARROW, CorridorDimensions.WIDE
        off = self._armed(tuning, 0.0)
        armed = self._armed(tuning, 0.15)

        def offset_from_outer_wall(width: float, t, *, confirmed: bool) -> float:
            return width / 2 + center_bias_for_corridor(width, t, confirmed=confirmed)

        wide_line = offset_from_outer_wall(wide, off, confirmed=True)
        assert offset_from_outer_wall(wide, armed, confirmed=True) == pytest.approx(wide_line)

        # 0.0 rather than the shipped profile: the field now ships armed, so
        # reading the un-pre-positioned step off `tuning` would measure 0.25
        # and quietly stop pinning the 0.30 m the hardware actually recorded.
        unarmed_step = abs(wide_line - offset_from_outer_wall(narrow, off, confirmed=False))
        armed_step = abs(wide_line - offset_from_outer_wall(narrow, armed, confirmed=False))
        assert unarmed_step == pytest.approx(0.30)
        assert armed_step == pytest.approx(0.15)
        shipped_step = abs(wide_line - offset_from_outer_wall(narrow, tuning, confirmed=False))
        assert shipped_step == pytest.approx(0.25)

    def test_it_stays_inside_the_narrow_corridor(self, tuning) -> None:
        """Arming must never plan a line the chassis cannot fit on.

        The budget is ``width/2 - RobotSpecs.WIDTH/2``. A magnitude past it is
        a path into the inner wall of a corridor that really is narrow, which
        is the whole risk this field trades against -- so the feasibility check
        must reject it rather than plan it.
        """
        budget = CorridorDimensions.NARROW / 2 - RobotSpecs.WIDTH / 2
        assert center_bias_for_corridor(
            CorridorDimensions.NARROW, self._armed(tuning, budget - 0.01), confirmed=False
        ) < budget

    def test_infeasible_magnitude_is_rejected(self, sample_metadata_open, tuning) -> None:
        """Feasibility is scored at the LARGER of the two narrow biases.

        Scoring only the confirmed value would pass a path that spends more
        margin than was checked, and would do so exactly on the runs where the
        belief is still wrong.
        """
        with pytest.raises(ValueError, match=r"narrow|width|fit|feasib"):
            calculate_waypoints(
                sample_metadata_open,
                num_laps=1,
                tuning=self._armed(tuning, 0.35),
                unconfirmed_sections=frozenset(Section),
            )
