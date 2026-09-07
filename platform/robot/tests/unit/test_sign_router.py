"""Unit tests for traffic-sign routing.

Verifies:
1. deform_waypoint() shifts the waypoint to the correct side for every
   corridor + color combination (8 cases covering all 4 sections × 2 colors).
2. 36 WRO scenario routing decisions produce the correct pass-side for a
   canonical obstacles scenario in each corridor.
3. Camera-detection color override replaces metadata color when confident.
4. Signs outside activation distance are ignored.
5. Passed signs are not re-applied.
"""

from __future__ import annotations

import math
from dataclasses import replace

import pytest
from shared.config.constants import RobotSpecs, TrackDimensions, TrafficSignSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Direction, Section
from shared.domain.models import BBox, Detection, SignColor, TrafficSignObservation, Waypoint

from src.config.tuning_helpers import get_tuning, tuning_with_overrides
from src.navigation.planning.sign_discovery import (
    _CAMERA_FOCAL_PX,
    _detection_to_world,
)
from src.navigation.planning.sign_router import (
    SignRouter,
    SignRouterConfig,
    SignRouterContext,
    SignSpec,
    pass_side_lateral_axis,
)
from src.navigation.planning.sign_router.config import (
    CHASSIS_HALF_DIAGONAL,
)
from src.navigation.planning.sign_router.deformation import (
    apply_deformation,
    match_detection_to_sign,
)
from src.navigation.planning.sign_router.routing import ROUTING_TABLE, depth_consistent_corridor
from src.navigation.planning.waypoints import corridor_for_position
from src.navigation.race_tracker import TRAVEL_DIRS
from tests.test_constants import (
    CORRIDOR_DEPTH_MAX,
    CORRIDOR_DEPTH_MIDPOINT,
    CORRIDOR_WIDTH_QUARTER_NORTH,
    CORRIDOR_WIDTH_QUARTER_SOUTH,
    SIGN_ACTIVATION_DIST,
    SIGN_GRID_POSITIONS,
    SIGN_LATERAL_OFFSET,
    SIGN_PASSED_DIST,
    TRACK_CORNER_EAST,
    TRACK_CORNER_NORTH,
    TRACK_CORNER_SOUTH,
    TRACK_CORNER_WEST,
)

# These were module constants until tuning was threaded through; the values are
# now derived per-call from NavigationTuning. Bound once here so the assertions
# below keep reading as statements about the shipped configuration.
_TUNING = NavigationTuning.load_default()
_MIN_RELIABLE_BBOX_HEIGHT_PX = _TUNING.sign_discovery.MIN_RELIABLE_BBOX_HEIGHT_PX
_SIGN_CLEARANCE_MARGIN = _TUNING.sign_router.SIGN_CLEARANCE_MARGIN_M
_SIGN_LATERAL_OFFSET = SignRouterConfig.from_tuning(_TUNING.sign_router).lateral_offset
_WALL_CLEARANCE = CHASSIS_HALF_DIAGONAL + _TUNING.sign_router.WALL_CLEARANCE_MARGIN_M

# Robot-to-sign gaps expressed against the configured thresholds instead of as
# literals. They used to be hardcoded (0.2 to engage, 1.5 to pass) against an
# activation of 0.80 and a passed of 1.20; when the tuning moved to 1.40/1.60
# those numbers stopped meaning "just inside" and "well beyond" and started
# meaning the opposite, without a single test changing.
_GAP_ENGAGED = SIGN_ACTIVATION_DIST / 4
"""Comfortably inside the activation radius."""

_GAP_CLEAR = SIGN_ACTIVATION_DIST + 0.2
"""Comfortably outside it."""

_GAP_PASSED = SIGN_PASSED_DIST + 0.2
"""Comfortably beyond the distance at which a sign counts as passed."""


@pytest.fixture()
def router_config(tuning_constants):
    """SignRouterConfig fixture computed from tuning instead of frozen at module level."""
    return SignRouterConfig(
        lateral_offset=SIGN_LATERAL_OFFSET,
        activation_dist=tuning_constants.sign_activation_dist,
        passed_dist=tuning_constants.sign_passed_dist,
        # These tests exercise engage/pass logic directly, in isolation, over a
        # handful of calls — not the settle-window feature itself (see
        # TestSettleWindow below), so disable it here.
        settle_ticks=0,
    )


# Helper


def _router(
    signs: list[SignSpec],
    config: SignRouterConfig,
    direction: Direction = Direction.COUNTERCLOCKWISE,
) -> SignRouter:
    """Build a router for one test case.

    ``direction`` matters since 2026-09-03: the pass-side rule is
    travel-relative, so a router left on the default CCW judges a clockwise
    case against the wrong half of ``ROUTING_TABLE``. It was safe to omit only
    while the table's two directions were identical, which was the bug.
    """
    return SignRouter(signs, config=config, direction=direction)


def _sign_at(x: float, y: float, color: str) -> SignSpec:
    return SignSpec(x=x, y=y, color=color)


def _shipped_corridor(x: float, y: float) -> Section:
    """Corridor a sign is filed under by the SHIPPED assignment rule.

    Mirrors ``SignRouter._geometric_corridor``. Reads the knob rather than
    assuming its value, so a test that pins what the damping sees keeps
    describing the configuration that actually ships.
    """
    fallback = corridor_for_position(x, y)
    if not _TUNING.sign_router.SIGN_LANE_DEPTH_CONSISTENT_CORRIDOR:
        return fallback
    return depth_consistent_corridor(x, y, fallback)


# 1. Deformation direction per corridor x color x travel direction

# Per-section: (perpendicular axis, sign position, red multiplier). Red's
# multiplier is the SAME for CW and CCW — outward/inward is a fixed property
# of the corridor, not the travel direction.
_SECTION_GEOMETRY = {
    Section.SOUTH: ("y", (CORRIDOR_DEPTH_MIDPOINT, CORRIDOR_WIDTH_QUARTER_NORTH), -1),
    Section.NORTH: ("y", (CORRIDOR_DEPTH_MIDPOINT, TRACK_CORNER_NORTH), +1),
    Section.EAST: ("x", (TRACK_CORNER_EAST, CORRIDOR_DEPTH_MIDPOINT), +1),
    Section.WEST: ("x", (TRACK_CORNER_WEST, CORRIDOR_DEPTH_MIDPOINT), -1),
}


class TestDeformationDirections:
    """16 cases: 4 sections x {red, green} x {CCW, CW}.

    ``red_mult`` in ``_SECTION_GEOMETRY`` is the section's OUTWARD multiplier,
    so ``color_sign`` says whether this (direction, colour) pair should deform
    outward (+1) or inward (-1). The rule is travel-relative -- red passes on
    the vehicle's right -- and the vehicle's right is the outer wall driving
    COUNTERCLOCKWISE but the inner square driving CLOCKWISE, so **the two
    directions take opposite signs**.

    They were identical here until 2026-09-03, which is what let the absolute
    misreading survive: this table asserted red-outward for both directions,
    the routing table implemented it, and the pair agreed with each other while
    disagreeing with rules 9.19.
    """

    @pytest.mark.parametrize("section", list(_SECTION_GEOMETRY))
    @pytest.mark.parametrize(
        ("direction", "color", "color_sign"),
        [
            # CCW: the chassis's right hand points OUTWARD.
            (Direction.COUNTERCLOCKWISE, "red", +1),
            (Direction.COUNTERCLOCKWISE, "green", -1),
            # CW: it points INWARD, so the same rule inverts in track terms.
            (Direction.CLOCKWISE, "red", -1),
            (Direction.CLOCKWISE, "green", +1),
        ],
    )
    def test_offset_side(self, section, direction, color, color_sign, router_config):
        axis, (sx, sy), red_mult = _SECTION_GEOMETRY[section]
        sign = _sign_at(sx, sy, color)
        rx, ry = apply_deformation((sx, sy), sign, color, section, direction, SIGN_LATERAL_OFFSET)
        expected = red_mult * color_sign * SIGN_LATERAL_OFFSET
        # Inner/outer-lane signs cannot always take the full offset — the
        # router clamps clear of the inner square and the outer wall.
        low_side = section in (Section.SOUTH, Section.WEST)
        if axis == "y":
            assert ry == pytest.approx(_expected_lateral(sy + expected, low_side=low_side))
            assert rx == pytest.approx(sx)
        else:
            assert rx == pytest.approx(_expected_lateral(sx + expected, low_side=low_side))
            assert ry == pytest.approx(sy)


class TestPassSideLateralAxis:
    """Direction-KEYED lookup of the travel-relative pass-side rule.

    Was ``TestOutwardLateralAxis``, asserting the lookup gave the same answer
    for both directions. That held only while ``ROUTING_TABLE``'s CW and CCW
    rows were identical, which was the 2026-07-05 bug: the real rule is
    travel-relative (9.19), so the rows are negations and the lookup MUST be
    keyed on direction. The old test could not have failed on the bug it was
    covering, because it asserted the bug.
    """

    @pytest.mark.parametrize("section", list(Section))
    @pytest.mark.parametrize("color", [SignColor.RED, SignColor.GREEN])
    @pytest.mark.parametrize("direction", [Direction.CLOCKWISE, Direction.COUNTERCLOCKWISE])
    def test_matches_routing_table_for_its_own_direction(self, section, color, direction):
        entry = ROUTING_TABLE[(section, direction)]
        expected_mult = entry.red_mult if color == SignColor.RED else entry.green_mult
        assert pass_side_lateral_axis(section, color, direction) == (entry.axis, expected_mult)

    @pytest.mark.parametrize("section", list(Section))
    @pytest.mark.parametrize("color", [SignColor.RED, SignColor.GREEN])
    def test_directions_are_opposite(self, section, color):
        """The whole point of the fix: CW and CCW must never agree."""
        cw = pass_side_lateral_axis(section, color, Direction.CLOCKWISE)
        ccw = pass_side_lateral_axis(section, color, Direction.COUNTERCLOCKWISE)
        assert cw is not None
        assert ccw is not None
        assert cw[0] == ccw[0], "same corridor, same lateral axis"
        assert cw[1] == -ccw[1], "opposite multiplier -- 'the vehicle's right' inverts with heading"

    @pytest.mark.parametrize("color", [SignColor.RED, SignColor.GREEN])
    def test_unknown_direction_declines_to_answer(self, color):
        """None, not a guess: a coin-flip here is a wrong-side pass half the
        time, and that ends the round under 9.24.5."""
        assert pass_side_lateral_axis(Section.SOUTH, color, None) is None


# 2. 36-scenario routing

# WRO official grid: 6 positions per corridor
# (depth, width) using SOUTH-corridor frame, depth ∈ {1.0, 1.5, 2.0}, width ∈ {0.4, 0.6}

# WRO 36 predefined scenarios: scenario ID → list of (color, depth, width) for SOUTH template
# Scenarios 1-12: single pillar
# Scenarios 13-36: double pillar
# We only exercise the core rule: red right, green left — no exhaustive enumeration needed
# Instead, verify all 6 grid positions × 2 colors × 4 sections = 48 routing decisions.


def _expected_lateral(value: float, *, low_side: bool) -> float:
    """Clamp an expected lateral coordinate into the corridor's legal band.

    The router refuses to deform a waypoint into the inner square or the outer
    wall, so for signs in the inner/outer lanes the full ``SIGN_LATERAL_OFFSET``
    is not always reachable — the last ~2cm is clipped. Mirroring that here
    keeps these cases pinning the pass *side* and offset magnitude, while
    ``TestClamping`` separately pins the clamp itself.
    """
    if low_side:
        return min(
            max(value, TrackDimensions.MIN_COORD + _WALL_CLEARANCE),
            TrackDimensions.CORNER_MIN - _WALL_CLEARANCE,
        )
    return max(
        min(value, TrackDimensions.MAX_COORD - _WALL_CLEARANCE),
        TrackDimensions.CORNER_MAX + _WALL_CLEARANCE,
    )


def _make_single_sign_scenario_cases():
    """Generate test cases: (corridor, sign_x, sign_y, color, expect_north_or_east).

    Red is avoided OUTWARD (away from the inner square), green INWARD — for
    every corridor, using the router's default direction (COUNTERCLOCKWISE):
    this rule is now identical for CW and CCW, so the direction doesn't matter.

    For SOUTH/NORTH corridors: "correct pass" means waypoint is on the expected y side.
    For EAST/WEST corridors: waypoint is on the expected x side.
    """
    cases = []
    for depth, width in SIGN_GRID_POSITIONS:
        for color in ("red", "green"):
            # SOUTH corridor: sign at (depth, width). Outward = south (lower y).
            sx, sy = depth, width
            if color == "red":
                expected_y = sy - SIGN_LATERAL_OFFSET
            else:
                expected_y = sy + SIGN_LATERAL_OFFSET
            cases.append(("south", sx, sy, color, _expected_lateral(expected_y, low_side=True), None))

            # NORTH corridor: sign at (depth, TRACK_MAX - width). Outward = north (higher y).
            sx, sy = depth, TrackDimensions.MAX_COORD - width
            if color == "red":
                expected_y = sy + SIGN_LATERAL_OFFSET
            else:
                expected_y = sy - SIGN_LATERAL_OFFSET
            cases.append(("north", sx, sy, color, _expected_lateral(expected_y, low_side=False), None))

            # EAST corridor: sign at (TRACK_MAX - width, depth). Outward = east (higher x).
            sx, sy = TrackDimensions.MAX_COORD - width, depth
            if color == "red":
                expected_x = sx + SIGN_LATERAL_OFFSET
            else:
                expected_x = sx - SIGN_LATERAL_OFFSET
            cases.append(("east", sx, sy, color, None, _expected_lateral(expected_x, low_side=False)))

            # WEST corridor: sign at (width, depth). Outward = west (lower x).
            sx, sy = width, depth
            if color == "red":
                expected_x = sx - SIGN_LATERAL_OFFSET
            else:
                expected_x = sx + SIGN_LATERAL_OFFSET
            cases.append(("west", sx, sy, color, None, _expected_lateral(expected_x, low_side=True)))

    return cases


_ROUTING_CASES = _make_single_sign_scenario_cases()


@pytest.mark.parametrize(
    "section_str,sx,sy,color,expected_y,expected_x",
    _ROUTING_CASES,
    ids=[f"{c[0]}_{c[3]}_d{c[1]:.1f}_w{c[2]:.1f}" for c in _ROUTING_CASES],
)
def test_routing_decision_all_grid_positions(
    section_str,
    sx,
    sy,
    color,
    expected_y,
    expected_x,
    router_config,
):
    """Routing produces correct pass-side for every grid position × color × section."""
    section = Section.from_string(section_str)
    sign = _sign_at(sx, sy, color)
    # Approach the sign from just outside activation distance
    if section in (Section.SOUTH, Section.NORTH):
        robot_pos = (sx - 0.3, sy)  # approach from west
    else:
        robot_pos = (sx, sy - 0.3)  # approach from south

    router = _router([sign], router_config)
    wp = (sx, sy)
    result_x, result_y = router.deform_waypoint(
        waypoint=wp,
        robot_pos=robot_pos,
        robot_yaw=0.0,
        corridor=section,
    )

    if expected_y is not None:
        assert result_y == pytest.approx(expected_y, abs=1e-6)
    if expected_x is not None:
        assert result_x == pytest.approx(expected_x, abs=1e-6)


# 3. Activation distance guard


class TestActivationDistance:
    def test_sign_outside_activation_not_deformed(self, router_config):
        sign = _sign_at(1.5, 0.4, "red")
        router = _router([sign], router_config)
        wp = (0.5, 0.4)
        # Robot far from sign (> activation_dist)
        result = router.deform_waypoint(
            waypoint=wp,
            robot_pos=(1.5 - _GAP_CLEAR, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert result == wp

    def test_sign_inside_activation_deformed(self, router_config):
        sign = _sign_at(1.5, 0.4, "red")
        router = _router([sign], router_config)
        wp = (1.5, 0.4)
        result = router.deform_waypoint(
            waypoint=wp,
            robot_pos=(1.5 - _GAP_ENGAGED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert result != wp


class TestLateralOffsetTracksChassis:
    """The production offset must follow the measured chassis, not a stale literal.

    ``robot.toml``'s chassis width changed from 0.200 to 0.194 mid-investigation
    and moved every clearance constant derived from it. Anything that hard-codes
    a number instead of deriving it silently stops matching the robot — and at
    this scale it matters: a 0.28 mm perturbation was enough to flip a corpus
    scenario.
    """

    def test_offset_is_half_diagonal_plus_sign_half_width_plus_margin(self, router_config):
        expected = (
            math.hypot(RobotSpecs.LENGTH / 2, RobotSpecs.WIDTH / 2)
            + TrafficSignSpecs.WIDTH / 2
            + _SIGN_CLEARANCE_MARGIN
        )
        assert pytest.approx(expected) == _SIGN_LATERAL_OFFSET

    def test_offset_uses_the_diagonal_not_the_width(self, router_config):
        """Half-width sizes a pass the robot can only make while already square.

        Two-thirds of legal WRO sign positions sit on a corner boundary, where
        the chassis is mid-turn and presents its corner. The half-width
        derivation gives an offset below what such a pass needs, which is the
        bug 47827ca fixed; this pins it from coming back.
        """
        half_width_derivation = RobotSpecs.WIDTH / 2 + TrafficSignSpecs.WIDTH / 2 + _SIGN_CLEARANCE_MARGIN
        assert half_width_derivation < _SIGN_LATERAL_OFFSET

    def test_wall_clearance_tracks_the_same_chassis(self, router_config):
        assert pytest.approx(math.hypot(RobotSpecs.LENGTH / 2, RobotSpecs.WIDTH / 2) + 0.04) == _WALL_CLEARANCE


class TestActivationPassedOrdering:
    """``activation_dist`` must stay below ``passed_dist``.

    ``_active_sign_candidates`` engages a sign nearer than ``activation_dist``
    and retires one further than ``passed_dist`` on the same tick, in that
    order. Invert them and every sign is engaged and marked passed in the same
    breath, from a metre away, then stays retired for the rest of the run:
    deformation never fires at the real pass. Measured on the 256-scenario
    corpus, ``activation_dist=1.30`` against the shipped ``passed_dist=1.20``
    took it from 209 collisions to 256/256 with zero laps completed, silently.
    """

    @pytest.mark.parametrize(
        ("activation", "passed"),
        [
            (1.30, 1.20),  # the measured cliff
            (1.20, 1.20),  # equal is just as broken: engage and retire coincide
            (2.00, 0.50),
        ],
    )
    def test_activation_at_or_above_passed_is_rejected(self, activation: float, passed: float, router_config):
        with pytest.raises(ValueError, match="must be < passed_dist"):
            SignRouterConfig(activation_dist=activation, passed_dist=passed)

    def test_valid_ordering_is_accepted(self, router_config):
        config = SignRouterConfig(activation_dist=1.60, passed_dist=1.80)
        assert config.activation_dist < config.passed_dist

    def test_shipped_defaults_satisfy_the_ordering(self, router_config):
        config = SignRouterConfig()
        assert config.activation_dist < config.passed_dist


# 4. Passed signs ignored


class TestRoutedSignPositions:
    """What the router publishes to the reactive collision layer as "mine".

    The collision layer withholds returns landing on these positions from its
    escape trigger, so this list is a safety-relevant contract: anything wrongly
    on it loses its guard, and anything wrongly off it triggers an escape the
    router did not want.
    """

    def test_lists_every_sign_it_still_intends_to_route_around(self, router_config):
        signs = [_sign_at(1.5, 0.4, "red"), _sign_at(2.5, 0.4, "green")]
        assert _router(signs, router_config).routed_sign_positions == [Waypoint(1.5, 0.4), Waypoint(2.5, 0.4)]

    def test_retired_sign_is_dropped_so_its_guard_comes_back(self, router_config):
        signs = [_sign_at(1.5, 0.4, "red"), _sign_at(2.5, 0.4, "green")]
        router = _router(signs, router_config)
        # Engage the first sign, then drive well past it, which retires it.
        router.deform_waypoint(
            waypoint=(1.5, 0.4),
            robot_pos=(1.5 - _GAP_ENGAGED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        router.deform_waypoint(
            waypoint=(0.5, 0.4),
            robot_pos=(1.5 + _GAP_PASSED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )

        assert router.routed_sign_positions == [Waypoint(2.5, 0.4)]

    def test_empty_when_there_are_no_signs(self, router_config):
        assert _router([], router_config).routed_sign_positions == []


class TestPassedSigns:
    def test_passed_sign_not_deformed(self, router_config):
        sign = _sign_at(1.5, 0.4, "red")
        router = _router([sign], router_config)
        # Engage the sign first (robot approaches within activation distance)...
        router.deform_waypoint(
            waypoint=(1.5, 0.4),
            robot_pos=(1.5 - _GAP_ENGAGED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        # ...then drive past it (> passed_dist), which retires it.
        router.deform_waypoint(
            waypoint=(0.5, 0.4),
            robot_pos=(1.5 + _GAP_PASSED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        wp = (1.5, 0.4)
        result = router.deform_waypoint(
            waypoint=wp,
            robot_pos=(1.5 - _GAP_ENGAGED, 0.4),  # back near sign — must stay retired
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert result == wp

    def test_active_sign_count_decrements(self, router_config):
        # The two signs are spaced against the passed threshold, not a metre
        # apart: the point that retires the first has to still be short of the
        # second, and with signs 1.0 m apart there is no such point once
        # passed_dist exceeds that spacing.
        first_x = 1.0
        second_x = first_x + _GAP_PASSED + 0.4
        signs = [_sign_at(first_x, 0.4, "red"), _sign_at(second_x, 0.4, "green")]
        router = _router(signs, router_config)
        assert router.active_sign_count == 2
        # Engage the first sign (within activation distance).
        router.deform_waypoint(
            waypoint=(first_x, 0.4),
            robot_pos=(first_x - _GAP_ENGAGED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        # Then drive past it (> passed_dist) but not yet past the second.
        router.deform_waypoint(
            waypoint=(second_x, 0.4),
            robot_pos=(first_x + _GAP_PASSED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert router.active_sign_count == 1

    def test_reset_for_new_lap_re_arms_passed_signs(self, router_config):
        """Every sign must route again each lap — the Obstacles Challenge runs 3."""
        sign = _sign_at(1.5, 0.4, "red")
        router = _router([sign], router_config)
        router.deform_waypoint(
            waypoint=(1.5, 0.4),
            robot_pos=(1.5 - _GAP_ENGAGED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        router.deform_waypoint(
            waypoint=(0.5, 0.4),
            robot_pos=(1.5 + _GAP_PASSED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert router.active_sign_count == 0

        router.reset_for_new_lap()
        assert router.active_sign_count == 1

        wp = (1.5, 0.4)
        result = router.deform_waypoint(
            waypoint=wp,
            robot_pos=(1.5 - _GAP_ENGAGED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert result != wp


class TestSettleWindow:
    """Engage/pass bookkeeping is deferred for the first ``settle_ticks`` of a lap.

    Regression guard for a real bug: right after spawn (or a lap boundary),
    the robot can briefly swing toward a corridor it hasn't actually reached
    yet while settling onto its planned route. If that swing grazes a
    not-yet-really-encountered sign's activation radius, engage+pass bookkeeping
    with no settle window retires the sign before its genuine pass ever happens.
    """

    def test_engage_and_pass_suppressed_within_settle_window(self, router_config):
        cfg = SignRouterConfig(
            lateral_offset=SIGN_LATERAL_OFFSET,
            activation_dist=SIGN_ACTIVATION_DIST,
            passed_dist=SIGN_PASSED_DIST,
            settle_ticks=3,
        )
        sign = _sign_at(1.5, 0.4, "red")
        router = SignRouter([sign], config=cfg)

        # Within the settle window (ticks 1-2, settle_ticks=3): engage-then-leave
        # must NOT retire the sign, even though the same sequence would retire
        # it once settled (see the follow-up calls below).
        router.deform_waypoint(
            waypoint=(1.5, 0.4),
            robot_pos=(1.5 - _GAP_ENGAGED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        router.deform_waypoint(
            waypoint=(0.5, 0.4),
            robot_pos=(1.5 + _GAP_PASSED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert router.active_sign_count == 1

        # Burn the remaining unsettled tick (tick 3) so both calls below (ticks
        # 4-5) land past the settle window.
        router.deform_waypoint(
            waypoint=(1.5, 0.4),
            robot_pos=(10.0, 10.0),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )

        # Past the settle window, the same engage-then-leave sequence retires it.
        router.deform_waypoint(
            waypoint=(1.5, 0.4),
            robot_pos=(1.5 - _GAP_ENGAGED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        router.deform_waypoint(
            waypoint=(0.5, 0.4),
            robot_pos=(1.5 + _GAP_PASSED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert router.active_sign_count == 0

    def test_candidate_selection_not_suppressed_within_settle_window(self, router_config):
        """A sign in the robot's real corridor still deforms during the settle window."""
        cfg = SignRouterConfig(
            lateral_offset=SIGN_LATERAL_OFFSET,
            activation_dist=SIGN_ACTIVATION_DIST,
            passed_dist=SIGN_PASSED_DIST,
            settle_ticks=100,
        )
        sign = _sign_at(1.5, 0.4, "red")
        router = SignRouter([sign], config=cfg)
        wp = (1.5, 0.4)
        result = router.deform_waypoint(
            waypoint=wp,
            robot_pos=(1.5 - _GAP_ENGAGED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert result != wp

    def test_reset_for_new_lap_restarts_the_settle_window(self, router_config):
        cfg = SignRouterConfig(
            lateral_offset=SIGN_LATERAL_OFFSET,
            activation_dist=SIGN_ACTIVATION_DIST,
            passed_dist=SIGN_PASSED_DIST,
            settle_ticks=3,
        )
        sign = _sign_at(1.5, 0.4, "red")
        router = SignRouter([sign], config=cfg)
        for _ in range(4):
            router.deform_waypoint(
                waypoint=(1.5, 0.4),
                robot_pos=(1.5 - _GAP_ENGAGED, 0.4),
                robot_yaw=0.0,
                corridor=Section.SOUTH,
            )
        router.deform_waypoint(
            waypoint=(0.5, 0.4),
            robot_pos=(1.5 + _GAP_PASSED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert router.active_sign_count == 0  # settled already, so this retired it

        router.reset_for_new_lap()
        # Immediately after reset, back inside a fresh settle window.
        router.deform_waypoint(
            waypoint=(1.5, 0.4),
            robot_pos=(1.5 - _GAP_ENGAGED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        router.deform_waypoint(
            waypoint=(0.5, 0.4),
            robot_pos=(1.5 + _GAP_PASSED, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert router.active_sign_count == 1


class TestEngagementGating:
    """A sign is only retired once approached — never discarded from afar."""

    def test_distant_sign_at_spawn_not_prematurely_passed(self, router_config):
        # Sign is farther than passed_dist at spawn; the buggy behaviour marked
        # it passed on the first tick, silently disabling routing.
        sign = _sign_at(2.0, 0.4, "red")
        router = _router([sign], router_config)

        result = router.deform_waypoint(
            waypoint=(0.4, 0.4),
            robot_pos=(0.4, 0.4),  # d = 1.6 m > passed_dist
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert result == (0.4, 0.4)  # too far to deform yet
        assert router.active_sign_count == 1  # still active, not retired

        # Once the robot approaches, the sign deforms the waypoint as expected.
        approached = router.deform_waypoint(
            waypoint=(2.0, 0.4),
            robot_pos=(1.7, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert approached != (2.0, 0.4)


class TestDepthPinCornerGuard:
    """The depth pin must not fire once the robot itself has curved out of the
    straight-corridor assumption it depends on, even if the (receding) waypoint
    it's evaluated against still reads as squarely in the corridor -- this is
    the fix for the 11-collision regression (pin off: 0 wall hits, pin on: 11).
    """

    def test_pin_does_not_fire_once_the_robot_is_past_the_corner_buffer(self, router_config):
        buffer = _TUNING.sign_router.DEFORM_DEPTH_BUFFER_M
        depth_max = TrackDimensions.CORNER_MAX + buffer
        waypoint_depth = depth_max - 0.15
        sign_depth = depth_max - 0.05
        robot_depth = depth_max + 0.05  # past the buffered corner window
        lateral_y = TrackDimensions.CORNER_MIN - 0.05  # still reads as SOUTH laterally
        sign = _sign_at(sign_depth, lateral_y, "red")

        result_x, _ = apply_deformation(
            (waypoint_depth, lateral_y),
            sign,
            "red",
            Section.SOUTH,
            Direction.CLOCKWISE,
            SIGN_LATERAL_OFFSET,
            robot_pos=(robot_depth, lateral_y),
        )

        assert result_x == pytest.approx(waypoint_depth)

    def test_pin_still_fires_when_the_robot_is_squarely_in_corridor(self, router_config):
        """Regression guard for the fix itself: the guard must not also kill
        legitimate pinning when the robot genuinely is square to the corridor.
        """
        buffer = _TUNING.sign_router.DEFORM_DEPTH_BUFFER_M
        depth_max = TrackDimensions.CORNER_MAX + buffer
        robot_depth = depth_max - 0.20  # inside the buffered corner window
        sign_depth = depth_max - 0.10  # between robot_depth and waypoint_depth
        waypoint_depth = depth_max - 0.05
        lateral_y = TrackDimensions.CORNER_MIN - 0.05
        sign = _sign_at(sign_depth, lateral_y, "red")

        result_x, _ = apply_deformation(
            (waypoint_depth, lateral_y),
            sign,
            "red",
            Section.SOUTH,
            Direction.CLOCKWISE,
            SIGN_LATERAL_OFFSET,
            robot_pos=(robot_depth, lateral_y),
        )

        assert result_x == pytest.approx(sign_depth)

    def test_guard_off_restores_the_pin_that_cost_11_wall_collisions(self, router_config):
        """``PIN_CORNER_GUARD=False`` must actually reach ``pin_depth``.

        The pre-guard arm is what the 2026-08-01 attribution was measured
        against, so a sweep that toggles this knob is only worth reading if the
        knob moves the geometry. Same setup as the first case in this class,
        which the guard suppresses: with the guard off the pin fires again.
        """
        buffer = _TUNING.sign_router.DEFORM_DEPTH_BUFFER_M
        depth_max = TrackDimensions.CORNER_MAX + buffer
        waypoint_depth = depth_max - 0.15
        sign_depth = depth_max - 0.05
        robot_depth = depth_max + 0.05  # past the buffered corner window
        lateral_y = TrackDimensions.CORNER_MIN - 0.05
        sign = _sign_at(sign_depth, lateral_y, "red")
        tuning = replace(_TUNING, sign_router=_TUNING.sign_router.model_copy(update={"PIN_CORNER_GUARD": False}))

        result_x, _ = apply_deformation(
            (waypoint_depth, lateral_y),
            sign,
            "red",
            Section.SOUTH,
            Direction.CLOCKWISE,
            SIGN_LATERAL_OFFSET,
            robot_pos=(robot_depth, lateral_y),
            context=SignRouterContext(tuning),
        )

        assert result_x == pytest.approx(sign_depth)


class TestDepthPinHeadingGuard:
    """The depth pin must also release once the ROBOT's heading has drifted
    away from where it stood when the pin engaged, not just once its position
    leaves the corridor -- traced on go_obstacles_0049 (subset64, sighted):
    the pin held a commanded point frozen for 46 ticks while the robot's yaw
    rotated 67 deg mid-corner, because PIN_CORNER_GUARD's position-only check
    never tripped (the raw waypoint stayed squarely in its corridor the whole
    time even though the chassis had already curved into the turn).
    """

    def _tuning_with_heading_guard(self, degrees: float) -> NavigationTuning:
        return replace(
            _TUNING,
            sign_router=_TUNING.sign_router.model_copy(
                update={"PIN_HEADING_GUARD": True, "PIN_HEADING_GUARD_DEG": degrees}
            ),
        )

    def test_pin_releases_once_yaw_has_drifted_past_the_threshold(self, router_config):
        buffer = _TUNING.sign_router.DEFORM_DEPTH_BUFFER_M
        depth_max = TrackDimensions.CORNER_MAX + buffer
        robot_depth = depth_max - 0.20  # inside the buffered corner window
        sign_depth = depth_max - 0.10  # between robot_depth and waypoint_depth
        waypoint_depth = depth_max - 0.05
        lateral_y = TrackDimensions.CORNER_MIN - 0.05
        sign = _sign_at(sign_depth, lateral_y, "red")
        tuning = self._tuning_with_heading_guard(35.0)

        result_x, _ = apply_deformation(
            (waypoint_depth, lateral_y),
            sign,
            "red",
            Section.SOUTH,
            Direction.CLOCKWISE,
            SIGN_LATERAL_OFFSET,
            robot_pos=(robot_depth, lateral_y),
            context=SignRouterContext(tuning),
            yaw_drift=math.radians(40.0),
        )

        assert result_x == pytest.approx(waypoint_depth)

    def test_pin_still_fires_under_the_yaw_drift_threshold(self, router_config):
        """Regression guard for the fix itself: small heading drift must not
        also kill legitimate pinning."""
        buffer = _TUNING.sign_router.DEFORM_DEPTH_BUFFER_M
        depth_max = TrackDimensions.CORNER_MAX + buffer
        robot_depth = depth_max - 0.20
        sign_depth = depth_max - 0.10
        waypoint_depth = depth_max - 0.05
        lateral_y = TrackDimensions.CORNER_MIN - 0.05
        sign = _sign_at(sign_depth, lateral_y, "red")
        tuning = self._tuning_with_heading_guard(35.0)

        result_x, _ = apply_deformation(
            (waypoint_depth, lateral_y),
            sign,
            "red",
            Section.SOUTH,
            Direction.CLOCKWISE,
            SIGN_LATERAL_OFFSET,
            robot_pos=(robot_depth, lateral_y),
            context=SignRouterContext(tuning),
            yaw_drift=math.radians(10.0),
        )

        assert result_x == pytest.approx(sign_depth)

    def test_guard_off_ignores_yaw_drift(self, router_config):
        """``PIN_HEADING_GUARD=False`` must actually reach ``pin_depth`` --
        a large yaw_drift must not suppress the pin unless the guard is on.

        Explicitly disables the guard rather than relying on the module
        default: ``PIN_HEADING_GUARD`` ships ``True`` (measured over the full
        256-scenario corpus, see ``SignRouterParams``), so the default arm no
        longer exercises the off path.
        """
        buffer = _TUNING.sign_router.DEFORM_DEPTH_BUFFER_M
        depth_max = TrackDimensions.CORNER_MAX + buffer
        robot_depth = depth_max - 0.20
        sign_depth = depth_max - 0.10
        waypoint_depth = depth_max - 0.05
        lateral_y = TrackDimensions.CORNER_MIN - 0.05
        sign = _sign_at(sign_depth, lateral_y, "red")
        tuning = replace(_TUNING, sign_router=_TUNING.sign_router.model_copy(update={"PIN_HEADING_GUARD": False}))

        result_x, _ = apply_deformation(
            (waypoint_depth, lateral_y),
            sign,
            "red",
            Section.SOUTH,
            Direction.CLOCKWISE,
            SIGN_LATERAL_OFFSET,
            robot_pos=(robot_depth, lateral_y),
            context=SignRouterContext(tuning),
            yaw_drift=math.radians(90.0),
        )

        assert result_x == pytest.approx(sign_depth)


# Camera-detection confirmation (pinhole projection)

# All direct _detection_to_world / match_detection_to_sign cases below use a
# robot at the origin facing east (yaw=0) unless stated otherwise, so
# theta_h == bearing and world position == (distance*cos, distance*sin).


def _detection_at_distance_bearing(
    distance: float,
    theta_h: float,
    *,
    color: str = "red",
    confidence: float = 0.9,
) -> Detection:
    """Build a Detection whose bbox pinhole-decodes to the given distance/bearing.

    Inverts `_detection_to_world`, so it moves with it -- including the 2026-09-06
    bearing-sign correction (`theta_h` is now POSITIVE TO THE LEFT, so a positive
    bearing is a box LEFT of centre) and `RANGE_SCALE`. That is exactly why this
    helper cannot be the only cover for either: a test built by inverting the
    formula agrees with the formula whatever it says, which is how a mirrored
    bearing survived in-tree. See
    `test_a_box_left_of_centre_is_a_sign_on_the_robots_left`, which states the
    convention from geometry instead of inheriting it.
    """
    scale = get_tuning(None).sign_discovery.RANGE_SCALE
    pixel_height = (_CAMERA_FOCAL_PX * TrafficSignSpecs.HEIGHT * scale) / distance
    cx = (0.5 - theta_h / RobotSpecs.CAMERA_HFOV) * RobotSpecs.CAMERA_WIDTH
    cy = RobotSpecs.CAMERA_HEIGHT / 2
    half = pixel_height / 2
    box = BBox(cx - half, cy - half, cx + half, cy + half)
    return Detection(
        class_name=color,
        confidence=confidence,
        bbox=tuple(box),
        x=cx,
        y=cy,
        width=box.width,
        height=box.height,
        area=box.area,
    )


def _observation_at(
    distance: float,
    theta_h: float,
    *,
    color: SignColor = SignColor.RED,
    confidence: float = 0.9,
    robot_pos: tuple[float, float] = (0.0, 0.0),
    robot_yaw: float = 0.0,
) -> TrafficSignObservation:
    """Build a TrafficSignObservation at a given distance and bearing from robot."""
    bearing = robot_yaw + theta_h
    world_x = robot_pos[0] + distance * math.cos(bearing)
    world_y = robot_pos[1] + distance * math.sin(bearing)
    return TrafficSignObservation(
        world_x_m=world_x,
        world_y_m=world_y,
        color=color,
        confidence=confidence,
        detected_at_timestamp=0.0,
    )


def _expected_world(distance: float, theta_h: float, robot_pos=(0.0, 0.0), robot_yaw: float = 0.0):
    """Where a sign at ``distance``/``theta_h`` from the SENSOR really is.

    ``distance`` is measured by the camera or the C1, and both sit
    LIDAR_MOUNT_X_OFFSET (0.1222 m) forward of the chassis centre, so the ray
    starts there -- not at ``robot_pos``. These tests asserted
    ``robot_pos + distance * direction`` until 2026-08-22, which pinned the
    projection to the body origin and so encoded a systematic ~12 cm
    under-range into the expected value; sign estimates drew that far short of
    the real signs in RViz. ``localization.py`` took the same correction on
    2026-08-21 for its predicted rays.
    """
    sensor = (
        robot_pos[0] + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.cos(robot_yaw),
        robot_pos[1] + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.sin(robot_yaw),
    )
    bearing = robot_yaw + theta_h
    return (sensor[0] + distance * math.cos(bearing), sensor[1] + distance * math.sin(bearing))


class TestDetectionToWorld:
    """Pins the pinhole-projection math ``_detection_to_world`` uses to turn a
    bbox into a world position — previously untested (review 2026-07-11 §2.2).
    """

    def test_round_trip_recovers_distance_and_bearing(self, router_config):
        distance, theta_h = 0.6, 0.15
        det = _detection_at_distance_bearing(distance, theta_h)
        world = _detection_to_world(det, robot_pos=(0.0, 0.0), robot_yaw=0.0)
        expected = _expected_world(distance, theta_h)
        assert world == pytest.approx(expected, abs=1e-6)

    def test_round_trip_with_nonzero_robot_pose(self, router_config):
        robot_pos = (1.2, 0.4)
        robot_yaw = 0.3
        distance, theta_h = 0.5, -0.1
        det = _detection_at_distance_bearing(distance, theta_h)
        world = _detection_to_world(det, robot_pos=robot_pos, robot_yaw=robot_yaw)
        expected = _expected_world(distance, theta_h, robot_pos, robot_yaw)
        assert world == pytest.approx(expected, abs=1e-6)

    def test_bbox_shorter_than_minimum_returns_none(self, router_config):
        tiny_height = _MIN_RELIABLE_BBOX_HEIGHT_PX - 1
        box = BBox(100.0, 100.0, 101.0, 100.0 + tiny_height)
        det = Detection(
            class_name="red",
            confidence=0.9,
            bbox=tuple(box),
            x=box.center.x,
            y=box.center.y,
            width=box.width,
            height=box.height,
            area=box.area,
        )
        assert _detection_to_world(det, robot_pos=(0.0, 0.0), robot_yaw=0.0) is None


def _single_ray_scan(theta_h: float, range_m: float, filler_range_m: float = 3.0) -> tuple[list[float], list[float]]:
    """A synthetic LIDAR sweep with one ray at ``theta_h`` reading ``range_m``.

    The other rays are far enough off ``theta_h`` (at least 0.2 rad away)
    that ``_nearest_ray`` always resolves to the intended one, and hold a
    plausible in-track range so they can never be mistaken for it.
    """
    angles = [theta_h, theta_h + 0.5, theta_h - 0.5, theta_h + math.pi]
    ranges = [range_m, filler_range_m, filler_range_m, filler_range_m]
    return ranges, angles


class TestDetectionToWorldLidarFusion:
    """The camera alone gives bearing + colour; LIDAR range at that bearing
    was trusted over the pinhole (bbox-height) distance estimate whenever the
    ray was a plausible return.

    **Shipped ON until 2026-09-06 and now OFF by default**, because a single ray
    at the camera's bearing is not the pillar: on run_20260906_192424 the return
    there is wall-shaped 51% of the time and pillar-shaped 27%, and the override
    fired on 92.5% of detections while costing 28 cm of median position error.
    See ``SignDiscoveryParams.LIDAR_RANGE_FUSION``.

    The mechanism is kept, so these still pin it -- with the flag ON explicitly,
    rather than inherited, so they keep testing the code and not the default.
    """

    def test_the_shipped_default_does_not_override_the_pinhole(self, router_config):
        """The default must be measurable from the test, not assumed."""
        distance, theta_h = 0.9, 0.15
        det = _detection_at_distance_bearing(distance, theta_h)
        ranges, angles = _single_ray_scan(theta_h, 0.6)

        world = _detection_to_world(
            det,
            robot_pos=(0.0, 0.0),
            robot_yaw=0.0,
            lidar_ranges_m=ranges,
            lidar_angles_rad=angles,
        )

        assert world == pytest.approx(_expected_world(distance, theta_h), abs=1e-6)

    def test_lidar_range_preferred_over_wrong_pinhole_distance(self, router_config):
        true_distance, theta_h = 0.6, 0.15
        # Bbox built for a WRONG pinhole distance (0.9 m); if the pinhole
        # estimate alone were used, the recovered world position would be
        # off by the same 0.3 m the bbox lies about.
        det = _detection_at_distance_bearing(0.9, theta_h)
        ranges, angles = _single_ray_scan(theta_h, true_distance)

        world = _detection_to_world(
            det,
            robot_pos=(0.0, 0.0),
            robot_yaw=0.0,
            lidar_ranges_m=ranges,
            lidar_angles_rad=angles,
            tuning=tuning_with_overrides({"LIDAR_RANGE_FUSION": True}, group="sign_discovery"),
        )

        expected = _expected_world(true_distance, theta_h)
        assert world == pytest.approx(expected, abs=1e-6)

    def test_falls_back_to_pinhole_when_lidar_ray_is_a_dropout(self, router_config):
        distance, theta_h = 0.6, 0.15
        det = _detection_at_distance_bearing(distance, theta_h)
        ranges, angles = _single_ray_scan(theta_h, RobotSpecs.CAMERA_FAR_CLIP + 1.0)

        world = _detection_to_world(
            det,
            robot_pos=(0.0, 0.0),
            robot_yaw=0.0,
            lidar_ranges_m=ranges,
            lidar_angles_rad=angles,
        )

        expected = _expected_world(distance, theta_h)
        assert world == pytest.approx(expected, abs=1e-6)

    def test_falls_back_to_pinhole_when_lidar_ray_is_self_detection(self, router_config):
        distance, theta_h = 0.6, 0.15
        det = _detection_at_distance_bearing(distance, theta_h)
        tiny = NavigationTuning.load_default().lidar_sectors.MIN_VALID_RANGE_M / 2
        ranges, angles = _single_ray_scan(theta_h, tiny)

        world = _detection_to_world(
            det,
            robot_pos=(0.0, 0.0),
            robot_yaw=0.0,
            lidar_ranges_m=ranges,
            lidar_angles_rad=angles,
        )

        expected = _expected_world(distance, theta_h)
        assert world == pytest.approx(expected, abs=1e-6)

    def test_no_lidar_data_keeps_pinhole_only_behaviour(self, router_config):
        distance, theta_h = 0.6, 0.15
        det = _detection_at_distance_bearing(distance, theta_h)

        world = _detection_to_world(
            det, robot_pos=(0.0, 0.0), robot_yaw=0.0, lidar_ranges_m=None, lidar_angles_rad=None
        )

        expected = _expected_world(distance, theta_h)
        assert world == pytest.approx(expected, abs=1e-6)


class TestMatchDetectionToSign:
    """Pins the confidence/match-distance/class gating in ``match_detection_to_sign``."""

    def test_low_confidence_observation_rejected(self, router_config):
        obs = _observation_at(0.5, 0.0, color=SignColor.RED, confidence=0.1)
        result = match_detection_to_sign(
            [obs],
            expected_world_pos=(0.5, 0.0),
            config=router_config,
        )
        assert result is None

    def test_far_match_rejected(self, router_config):
        obs = _observation_at(2.0, 0.0, color=SignColor.RED, confidence=0.9)
        result = match_detection_to_sign(
            [obs],
            expected_world_pos=(0.0, 0.0),
            config=router_config,
        )
        assert result is None

    @pytest.mark.parametrize("order", [("near", "far"), ("far", "near")])
    def test_nearest_candidate_wins_regardless_of_order(self, order, router_config):
        expected = (0.5, 0.0)
        near = _observation_at(0.5, 0.0, color=SignColor.GREEN, confidence=0.9)  # dist 0.0
        far = _observation_at(0.65, 0.0, color=SignColor.RED, confidence=0.9)  # dist 0.15
        candidates = [near, far] if order[0] == "near" else [far, near]

        result = match_detection_to_sign(
            candidates,
            expected_world_pos=expected,
            config=router_config,
        )
        assert result is SignColor.GREEN


class TestCameraDetectionOverridesGroundTruth:
    """A confident camera detection can override scenario-metadata ground truth.

    This is the one part of the navigation stack where a live sensor reading
    beats known-good ground truth (review 2026-07-11 §2.2) — proves the
    override actually changes which side the robot passes on, not just that
    the private color-matching helpers return the right string in isolation.
    """

    def test_camera_color_flips_avoidance_side(self, router_config):
        _, (sx, sy), _ = _SECTION_GEOMETRY[Section.SOUTH]
        sign = _sign_at(sx, sy, "red")  # ground truth: red
        router = _router([sign], router_config)

        robot_pos = (sx - 0.3, sy)
        obs = _observation_at(0.3, 0.0, color=SignColor.GREEN, confidence=0.9, robot_pos=robot_pos, robot_yaw=0.0)

        result = router.deform_waypoint(
            waypoint=(sx, sy),
            robot_pos=robot_pos,
            robot_yaw=0.0,
            corridor=Section.SOUTH,
            observations=[obs],
        )

        expected_if_green = apply_deformation(
            (sx, sy),
            sign,
            "green",
            Section.SOUTH,
            Direction.COUNTERCLOCKWISE,
            SIGN_LATERAL_OFFSET,
        )
        expected_if_red = apply_deformation(
            (sx, sy),
            sign,
            "red",
            Section.SOUTH,
            Direction.COUNTERCLOCKWISE,
            SIGN_LATERAL_OFFSET,
        )
        assert result == pytest.approx(expected_if_green, abs=1e-6)
        assert result != pytest.approx(expected_if_red, abs=1e-6)


# 5. No-op with empty sign list


def test_empty_sign_list_returns_waypoint_unchanged(router_config):
    router = _router([], router_config)
    wp = (1.5, 0.4)
    result = router.deform_waypoint(
        waypoint=wp,
        robot_pos=(1.4, 0.4),
        robot_yaw=0.0,
        corridor=Section.SOUTH,
    )
    assert result == wp


# 6. WP-1: deformation stays clear of the inner square and outer wall


class TestDeformationClamping:
    """A sign near a corridor edge must never deform the waypoint into the
    restricted inner square or beyond the outer wall.
    """

    def test_sign_at_inner_edge_does_not_enter_inner_square(self, router_config):
        # South corridor, sign right at the inner-square boundary (y=1.0):
        # unclamped this deforms to y=1.15 — inside the restricted square.
        sign = _sign_at(1.5, 1.0, "red")
        wx, wy = apply_deformation(
            (1.5, 1.0),
            sign,
            "red",
            Section.SOUTH,
            Direction.COUNTERCLOCKWISE,
            SIGN_LATERAL_OFFSET,
        )
        assert wx == pytest.approx(1.5)
        assert wy < 1.0, "deformed waypoint must stay below the inner square"

    def test_sign_at_outer_edge_does_not_cross_wall(self, router_config):
        # South corridor, sign right at the outer wall (y=0.0): unclamped this
        # deforms to y=-0.15 — beyond the track boundary.
        sign = _sign_at(1.5, 0.0, "green")
        wx, wy = apply_deformation(
            (1.5, 0.0),
            sign,
            "green",
            Section.SOUTH,
            Direction.COUNTERCLOCKWISE,
            SIGN_LATERAL_OFFSET,
        )
        assert wx == pytest.approx(1.5)
        assert wy >= 0.0, "deformed waypoint must stay on the track"

    def test_sign_at_inner_edge_east_corridor(self, router_config):
        # East corridor deforms x; sign at the inner-square boundary (x=2.0).
        # EAST/CCW red_mult=-1: unclamped this deforms to x=1.85 — inside the
        # inner square.
        sign = _sign_at(2.0, 1.5, "red")
        wx, wy = apply_deformation(
            (2.0, 1.5),
            sign,
            "red",
            Section.EAST,
            Direction.COUNTERCLOCKWISE,
            SIGN_LATERAL_OFFSET,
        )
        assert wy == pytest.approx(1.5)
        assert wx > 2.0, "deformed waypoint must stay clear of the inner square"


# 6. Pass-side rule pinned in absolute (track-relative) terms

# Unit vector pointing away from the inner square, per corridor.
_OUTWARD_DIR = {
    Section.SOUTH: (0.0, -1.0),
    Section.NORTH: (0.0, 1.0),
    Section.EAST: (1.0, 0.0),
    Section.WEST: (-1.0, 0.0),
}


class TestPassSideRule:
    """Red is passed on the vehicle's RIGHT, green on its LEFT — travel-relative.

    Pins the official rule as rules 2026 9.19 states it: "the red pillar must be
    passed from the right; the green pillar must be passed from the left", in
    the direction the round is actually driven. Asserted against the chassis's
    own heading (``TRAVEL_DIRS``, the production table) rather than a
    track-frame outward vector, because the two agree counterclockwise and are
    OPPOSITE clockwise -- which is exactly the bug this replaces.

    Between 2026-07-05 and 2026-09-03 this test asserted the absolute form ("red
    must be avoided on the outward side") for BOTH directions, so it passed
    while every clockwise round routed backwards. A test written from the same
    misreading as the code cannot catch the misreading.
    """

    @pytest.mark.parametrize(("section", "direction"), list(ROUTING_TABLE))
    @pytest.mark.parametrize("color", ["red", "green"])
    def test_sign_kept_on_correct_side(self, section, direction, color, router_config):
        # A realistic in-corridor sign position (clear of the inner square, per
        # WP-1 clamping) rather than a section-agnostic point — (1.5, 1.5) sits
        # inside the restricted inner square itself, which no real sign ever does.
        _, (sx, sy), _ = _SECTION_GEOMETRY[section]
        sign = _sign_at(sx, sy, color)
        wx, wy = apply_deformation(
            (sign.x, sign.y),
            sign,
            color,
            section,
            direction,
            SIGN_LATERAL_OFFSET,
        )
        # The chassis's own right-hand direction for the heading it drives here:
        # rotating the travel vector by -90 degrees gives (hy, -hx).
        heading = TRAVEL_DIRS[(section, direction)]
        right_x, right_y = heading.ny, -heading.nx
        right_component = right_x * (wx - sign.x) + right_y * (wy - sign.y)
        if color == "red":
            assert right_component > 0, "red must be passed on the vehicle's right"
        else:
            assert right_component < 0, "green must be passed on the vehicle's left"


# 7. Minimum edge-to-edge clearance from the sign's own footprint

# The offset is applied from the sign's CENTER (see apply_deformation), so
# both the robot's own half-width and the sign's half-width eat into the
# nominal lateral_offset before any real gap is left. A flat/undersized
# lateral_offset can pass every TestPassSideRule case above (correct side)
# while still leaving the chassis grazing the sign in practice.
_MIN_SIGN_EDGE_CLEARANCE_M = 0.05


class TestWrongSidePassDetection:
    """A sign retired on the forbidden side must register as a violation.

    The simulator stops the run on ``wrong_side_violations`` exactly as it does
    on a forbidden wall contact, so this pins that the router reports the miss
    in the first place: red is passed on the vehicle's RIGHT and green on its
    LEFT, and the side is judged from the robot's lateral coordinate relative
    to the sign's at the instant it is passed. A pass on the permitted side is
    NOT a violation.

    The permitted side is read out of ``ROUTING_TABLE`` for the case's own
    direction, and the router is BUILT for that direction -- both halves matter
    now that the two directions are negations of each other.
    """

    @pytest.mark.parametrize(("section", "direction"), list(ROUTING_TABLE))
    @pytest.mark.parametrize("color", ["red", "green"])
    def test_correct_side_is_not_a_violation(self, section, direction, color, router_config):
        _, (sx, sy), _ = _SECTION_GEOMETRY[section]
        entry = ROUTING_TABLE[(section, direction)]
        permitted = entry.red_mult if color == "red" else entry.green_mult
        # Place the robot on the permitted side: the corridor's lateral axis
        # value offset by the permitted direction.
        if section in (Section.SOUTH, Section.NORTH):
            robot = (sx, sy + permitted * 0.3)
        else:
            robot = (sx + permitted * 0.3, sy)
        router = _router([_sign_at(sx, sy, color)], router_config, direction)
        router._sign_corridors = [corridor_for_position(sx, sy)]
        router._engaged = {0}
        router._record_pass_side(0, Waypoint(*robot))
        assert router.wrong_side_violations == set(), "permitted side must not violate"

    @pytest.mark.parametrize(("section", "direction"), list(ROUTING_TABLE))
    @pytest.mark.parametrize("color", ["red", "green"])
    def test_wrong_side_is_a_violation(self, section, direction, color, router_config):
        _, (sx, sy), _ = _SECTION_GEOMETRY[section]
        entry = ROUTING_TABLE[(section, direction)]
        permitted = entry.red_mult if color == "red" else entry.green_mult
        forbidden = -permitted
        if section in (Section.SOUTH, Section.NORTH):
            robot = (sx, sy + forbidden * 0.3)
        else:
            robot = (sx + forbidden * 0.3, sy)
        router = _router([_sign_at(sx, sy, color)], router_config, direction)
        router._sign_corridors = [corridor_for_position(sx, sy)]
        router._engaged = {0}
        router._record_pass_side(0, Waypoint(*robot))
        assert router.wrong_side_violations == {0}, "forbidden side must violate"

    def test_reset_for_new_lap_clears_violations(self, router_config):
        _, (sx, sy), _ = _SECTION_GEOMETRY[Section.SOUTH]
        # Red in SOUTH is permitted outward (-Y); pass it on the inner (+Y) side.
        router = _router([_sign_at(sx, sy, "red")], router_config)
        router._sign_corridors = [Section.SOUTH]
        router._engaged = {0}
        router._record_pass_side(0, Waypoint(sx, sy + 0.3))
        assert router.wrong_side_violations == {0}
        router.reset_for_new_lap()
        assert router.wrong_side_violations == set()


class TestSignCorridorHysteresis:
    """A discovered sign's corridor picks which world axis its deformation
    treats as lateral, and it is re-derived every tick from an estimate that
    keeps moving. On a corner boundary -- where two-thirds of legal WRO grid
    positions sit -- millimetres of jitter otherwise swing the label between two
    corridors whose lateral axes are ORTHOGONAL.

    The coordinates here are the ones traced on ``go_obstacles_0000``: an
    estimate wobbling either side of y=2.00 at x=2.40 flipped EAST/NORTH on
    every tick for the whole approach.
    """

    _EAST_SIDE = (2.40, 1.997)
    _NORTH_SIDE = (2.40, 2.003)
    """The traced pair. These NO LONGER straddle a corridor under the shipped
    rule -- see ``test_the_traced_flip_no_longer_happens_at_all``. x=2.40 is a
    lateral value and y~2.00 a depth value, so both points are EAST signs and
    the NORTH label the old nearest-face rule produced was the misfile that
    ``SIGN_LANE_DEPTH_CONSISTENT_CORRIDOR`` removes. Kept to pin that."""

    _DIAG_EAST = (2.010, 2.003)
    _DIAG_NORTH = (2.003, 2.010)
    """A TRUE corner diagonal, which is what the damping is actually for.

    Both coordinates sit outside the inner square, so neither axis carries a
    legal depth and the depth rule cannot break the tie -- whichever coordinate
    is fractionally less far out wins, and millimetres of jitter still swing the
    label between two orthogonal lateral axes. Unlike the traced pair, no
    corridor assignment can be called correct here, so the hysteresis is the
    only thing standing between jitter and an oscillating lateral axis."""

    _FLIP_TICKS = 5
    """Set explicitly: the shipped default is 1, which leaves the mechanism
    inert, so a fixture-default config would exercise none of this."""

    @pytest.fixture()
    def damped_config(self, tuning_constants):
        return SignRouterConfig(
            lateral_offset=SIGN_LATERAL_OFFSET,
            activation_dist=tuning_constants.sign_activation_dist,
            passed_dist=tuning_constants.sign_passed_dist,
            settle_ticks=0,
            corridor_flip_ticks=self._FLIP_TICKS,
        )

    def _router_with_sign(self, config, x, y):
        return _router([_sign_at(x, y, "red")], config)

    def test_boundary_jitter_never_moves_the_corridor(self, damped_config):
        router = self._router_with_sign(damped_config, *self._DIAG_EAST)
        assert router._sign_corridors[0] == Section.EAST

        # Twenty ticks of dither -- a full second at 20Hz, far longer than any
        # real approach spends abeam a sign.
        for tick in range(20):
            x, y = self._DIAG_NORTH if tick % 2 == 0 else self._DIAG_EAST
            settled = router._settled_corridor(0, _sign_at(x, y, "red"))
            assert settled == Section.EAST, f"corridor flipped on tick {tick}"

    def test_a_sustained_move_still_lands(self, damped_config):
        """The label must still follow an estimate that genuinely improves --
        holding it forever would be its own bug, just a quieter one.
        """
        router = self._router_with_sign(damped_config, *self._DIAG_EAST)

        moved = _sign_at(*self._DIAG_NORTH, "red")
        for _ in range(self._FLIP_TICKS - 1):
            assert router._settled_corridor(0, moved) == Section.EAST
        assert router._settled_corridor(0, moved) == Section.NORTH

    def test_agreement_must_be_consecutive(self, damped_config):
        """One dissenting tick restarts the count, so dither that happens to
        favour the new corridor overall still never accumulates a flip.
        """
        router = self._router_with_sign(damped_config, *self._DIAG_EAST)
        north = _sign_at(*self._DIAG_NORTH, "red")
        east = _sign_at(*self._DIAG_EAST, "red")

        for _ in range(self._FLIP_TICKS * 3):
            for _ in range(self._FLIP_TICKS - 1):
                assert router._settled_corridor(0, north) == Section.EAST
            assert router._settled_corridor(0, east) == Section.EAST

    def test_one_tick_restores_immediate_reassignment(self, router_config):
        """The SHIPPED default: 1 leaves the mechanism inert, and the sweep's
        baseline arm depends on that staying true.
        """
        assert router_config.corridor_flip_ticks == 1
        router = self._router_with_sign(router_config, *self._DIAG_EAST)
        assert router._settled_corridor(0, _sign_at(*self._DIAG_NORTH, "red")) == Section.NORTH

    def test_the_diagonal_positions_really_do_straddle_a_corridor_boundary(self):
        """Guard the premise, not just the fix.

        If the track geometry ever moves and these two points stop landing in
        different corridors, every assertion above would still pass while
        testing nothing at all. Asserted through the SHIPPED assignment rule,
        which is what the damping above actually sees.
        """
        assert _shipped_corridor(*self._DIAG_EAST) == Section.EAST
        assert _shipped_corridor(*self._DIAG_NORTH) == Section.NORTH

    def test_the_traced_flip_no_longer_happens_at_all(self):
        """The traced oscillation is gone at the source, not merely damped.

        ``_EAST_SIDE``/``_NORTH_SIDE`` are the positions that flipped EAST/NORTH
        on every tick of an approach in ``go_obstacles_0000``. Nearest-face put
        them in different corridors -- that is what the damping was built for --
        but only one of those answers was ever right: x=2.40 is a LATERAL value
        and y~2.00 a DEPTH value, so both points describe an EAST sign, and the
        NORTH label was a misfile onto the perpendicular face. Under
        ``SIGN_LANE_DEPTH_CONSISTENT_CORRIDOR`` the label does not move, so
        there is no flip left to damp.
        """
        assert corridor_for_position(*self._EAST_SIDE) == Section.EAST
        assert corridor_for_position(*self._NORTH_SIDE) == Section.NORTH, "premise: nearest-face disagreed"
        # The RULE, not the shipped default, so this still states what the rule
        # does if the knob is ever revisited.
        for point in (self._EAST_SIDE, self._NORTH_SIDE):
            assert depth_consistent_corridor(*point, corridor_for_position(*point)) == Section.EAST


class TestMinimumClearance:
    """Guards against lateral_offset regressing to a value too small to
    give real edge-to-edge clearance, even though direction/side tests
    would still pass."""

    def test_default_offset_clears_sign_footprint(self, router_config):
        edge_clearance = SIGN_LATERAL_OFFSET - RobotSpecs.WIDTH / 2 - TrafficSignSpecs.WIDTH / 2
        assert edge_clearance >= _MIN_SIGN_EDGE_CLEARANCE_M, (
            f"lateral_offset={SIGN_LATERAL_OFFSET} leaves only {edge_clearance:.3f}m "
            f"edge-to-edge clearance between chassis and sign — below the "
            f"{_MIN_SIGN_EDGE_CLEARANCE_M}m minimum"
        )


class TestDirectionAdoption:
    """A blind round builds the router on the CLOCKWISE provisional.

    `track_navigator_node` cannot know the travel direction until LIDAR
    inference settles seconds into the round, so it builds the router on a
    placeholder. `_commit_direction` rebuilt the path, the lap detector, the
    width estimator and the start measurement -- but not the router, so
    `SignRouter._direction` stayed at the placeholder for the whole race.

    That does not degrade the lane, it MIRRORS it: every clockwise row of
    ROUTING_TABLE is the negation of its counterclockwise partner, so red and
    green swap sides. Measured across four hardware bags -- on the two rounds
    that inferred counterclockwise the commanded lane matched the CLOCKWISE row
    on 24 of 28 sign passes, and 22 of the 28 illegal passes are that mirror,
    against 2 caused by phantom signs and 0 by colour errors. The one round that
    inferred clockwise, agreeing with the placeholder by luck, passed 19 of 26
    legally.
    """

    def test_adopting_a_direction_flips_the_pass_side(self) -> None:
        """The whole point: the rule is travel-relative and must re-key."""
        spec = SignSpec(x=1.5, y=0.5, color=SignColor.RED)
        router = SignRouter(signs=[spec], direction=Direction.CLOCKWISE)
        clockwise = pass_side_lateral_axis(router._sign_corridors[0], SignColor.RED, router.direction)
        router.adopt_direction(Direction.COUNTERCLOCKWISE)
        counterclockwise = pass_side_lateral_axis(router._sign_corridors[0], SignColor.RED, router.direction)
        assert clockwise is not None
        assert counterclockwise is not None
        # Same axis, opposite permitted side -- that is the mirror.
        assert clockwise[0] == counterclockwise[0]
        assert clockwise[1] == -counterclockwise[1]

    def test_adoption_keeps_the_discovered_map(self) -> None:
        """In place, NOT via replace_sign_router, which drops discovered state.

        The map is built during the blind creep -- before the direction is even
        known -- and sign POSITIONS do not depend on which way the robot ends up
        driving. Throwing them away would make the fix cost a re-discovery.
        """
        router = SignRouter(signs=[], direction=Direction.CLOCKWISE, discover=True)
        before = router._sign_map
        router.adopt_direction(Direction.COUNTERCLOCKWISE)
        assert router._sign_map is before

    def test_adopting_the_same_direction_is_a_no_op(self) -> None:
        """A sighted round already has the right direction; it must not be disturbed."""
        spec = SignSpec(x=1.5, y=0.5, color=SignColor.RED)
        router = SignRouter(signs=[spec], direction=Direction.COUNTERCLOCKWISE)
        router._passed.add(0)
        router.adopt_direction(Direction.COUNTERCLOCKWISE)
        assert router._passed == {0}
