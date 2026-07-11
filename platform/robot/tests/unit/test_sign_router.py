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

import pytest
from shared.config.constants import RobotSpecs, TrackDimensions, TrafficSignSpecs
from shared.config.enums import Direction, Section
from shared.domain.models import Detection

from src.navigation.planning.sign_router import (
    _CAMERA_FOCAL_PX,
    _MIN_RELIABLE_BBOX_HEIGHT_PX,
    _ROUTING_TABLE,
    SignRouter,
    SignRouterConfig,
    SignSpec,
    _apply_deformation,
    _detection_to_world,
    _match_detection_to_sign,
)
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

CFG = SignRouterConfig(
    lateral_offset=SIGN_LATERAL_OFFSET,
    activation_dist=SIGN_ACTIVATION_DIST,
    passed_dist=SIGN_PASSED_DIST,
)
LATERAL = SIGN_LATERAL_OFFSET


# Helper


def _router(signs: list[SignSpec]) -> SignRouter:
    return SignRouter(signs, config=CFG)


def _sign_at(x: float, y: float, color: str) -> SignSpec:
    return SignSpec(x=x, y=y, color=color)


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

    Red always moves the deformed waypoint OUTWARD (away from the inner
    square), green always INWARD — identically for CW and CCW, since this is
    an absolute property of the track, not the travel direction.
    """

    @pytest.mark.parametrize("section", list(_SECTION_GEOMETRY))
    @pytest.mark.parametrize(
        ("direction", "color", "color_sign"),
        [
            (Direction.COUNTERCLOCKWISE, "red", +1),
            (Direction.COUNTERCLOCKWISE, "green", -1),
            (Direction.CLOCKWISE, "red", +1),
            (Direction.CLOCKWISE, "green", -1),
        ],
    )
    def test_offset_side(self, section, direction, color, color_sign):
        axis, (sx, sy), red_mult = _SECTION_GEOMETRY[section]
        sign = _sign_at(sx, sy, color)
        rx, ry = _apply_deformation((sx, sy), sign, color, section, direction, SIGN_LATERAL_OFFSET)
        expected = red_mult * color_sign * SIGN_LATERAL_OFFSET
        if axis == "y":
            assert ry == pytest.approx(sy + expected)
            assert rx == pytest.approx(sx)
        else:
            assert rx == pytest.approx(sx + expected)
            assert ry == pytest.approx(sy)


# 2. 36-scenario routing

# WRO official grid: 6 positions per corridor
# (depth, width) using SOUTH-corridor frame, depth ∈ {1.0, 1.5, 2.0}, width ∈ {0.4, 0.6}
_GRID_POSITIONS = SIGN_GRID_POSITIONS

# WRO 36 predefined scenarios: scenario ID → list of (color, depth, width) for SOUTH template
# Scenarios 1-12: single pillar
# Scenarios 13-36: double pillar
# We only exercise the core rule: red right, green left — no exhaustive enumeration needed
# Instead, verify all 6 grid positions × 2 colors × 4 sections = 48 routing decisions.


def _make_single_sign_scenario_cases():
    """Generate test cases: (corridor, sign_x, sign_y, color, expect_north_or_east).

    Red is avoided OUTWARD (away from the inner square), green INWARD — for
    every corridor, using the router's default direction (COUNTERCLOCKWISE):
    this rule is now identical for CW and CCW, so the direction doesn't matter.

    For SOUTH/NORTH corridors: "correct pass" means waypoint is on the expected y side.
    For EAST/WEST corridors: waypoint is on the expected x side.
    """
    cases = []
    for depth, width in _GRID_POSITIONS:
        for color in ("red", "green"):
            # SOUTH corridor: sign at (depth, width). Outward = south (lower y).
            sx, sy = depth, width
            if color == "red":
                expected_y = sy - SIGN_LATERAL_OFFSET
            else:
                expected_y = sy + SIGN_LATERAL_OFFSET
            cases.append(("south", sx, sy, color, expected_y, None))

            # NORTH corridor: sign at (depth, TRACK_MAX - width). Outward = north (higher y).
            sx, sy = depth, TrackDimensions.MAX_COORD - width
            if color == "red":
                expected_y = sy + SIGN_LATERAL_OFFSET
            else:
                expected_y = sy - SIGN_LATERAL_OFFSET
            cases.append(("north", sx, sy, color, expected_y, None))

            # EAST corridor: sign at (TRACK_MAX - width, depth). Outward = east (higher x).
            sx, sy = TrackDimensions.MAX_COORD - width, depth
            if color == "red":
                expected_x = sx + SIGN_LATERAL_OFFSET
            else:
                expected_x = sx - SIGN_LATERAL_OFFSET
            cases.append(("east", sx, sy, color, None, expected_x))

            # WEST corridor: sign at (width, depth). Outward = west (lower x).
            sx, sy = width, depth
            if color == "red":
                expected_x = sx - SIGN_LATERAL_OFFSET
            else:
                expected_x = sx + SIGN_LATERAL_OFFSET
            cases.append(("west", sx, sy, color, None, expected_x))

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
):
    """Routing produces correct pass-side for every grid position × color × section."""
    section = Section.from_string(section_str)
    sign = _sign_at(sx, sy, color)
    # Approach the sign from just outside activation distance
    if section in (Section.SOUTH, Section.NORTH):
        robot_pos = (sx - 0.3, sy)  # approach from west
    else:
        robot_pos = (sx, sy - 0.3)  # approach from south

    router = _router([sign])
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


# ── 3. Activation distance guard ──────────────────────────────────────────────


class TestActivationDistance:
    def test_sign_outside_activation_not_deformed(self):
        sign = _sign_at(1.5, 0.4, "red")
        router = _router([sign])
        wp = (0.5, 0.4)
        # Robot far from sign (> activation_dist)
        result = router.deform_waypoint(
            waypoint=wp,
            robot_pos=(0.5, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert result == wp

    def test_sign_inside_activation_deformed(self):
        sign = _sign_at(1.5, 0.4, "red")
        router = _router([sign])
        wp = (1.5, 0.4)
        result = router.deform_waypoint(
            waypoint=wp,
            robot_pos=(1.5 - 0.3, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert result != wp


# ── 4. Passed signs ignored ───────────────────────────────────────────────────


class TestPassedSigns:
    def test_passed_sign_not_deformed(self):
        sign = _sign_at(1.5, 0.4, "red")
        router = _router([sign])
        # Engage the sign first (robot approaches within activation distance)...
        router.deform_waypoint(
            waypoint=(1.5, 0.4),
            robot_pos=(1.5 - 0.2, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        # ...then drive past it (> passed_dist), which retires it.
        router.deform_waypoint(
            waypoint=(0.5, 0.4),
            robot_pos=(1.5 + 1.5, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        wp = (1.5, 0.4)
        result = router.deform_waypoint(
            waypoint=wp,
            robot_pos=(1.5 - 0.2, 0.4),  # back near sign — must stay retired
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert result == wp

    def test_active_sign_count_decrements(self):
        signs = [_sign_at(1.0, 0.4, "red"), _sign_at(2.0, 0.4, "green")]
        router = _router(signs)
        assert router.active_sign_count == 2
        # Engage the first sign (within activation distance).
        router.deform_waypoint(
            waypoint=(1.0, 0.4),
            robot_pos=(0.8, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        # Then drive past it (> passed_dist).
        router.deform_waypoint(
            waypoint=(2.0, 0.4),
            robot_pos=(2.5, 0.4),
            robot_yaw=0.0,
            corridor=Section.SOUTH,
        )
        assert router.active_sign_count == 1

    def test_reset_for_new_lap_re_arms_passed_signs(self):
        """Every sign must route again each lap — the Obstacles Challenge runs 3."""
        sign = _sign_at(1.5, 0.4, "red")
        router = _router([sign])
        router.deform_waypoint(
            waypoint=(1.5, 0.4), robot_pos=(1.5 - 0.2, 0.4), robot_yaw=0.0, corridor=Section.SOUTH,
        )
        router.deform_waypoint(
            waypoint=(0.5, 0.4), robot_pos=(1.5 + 1.5, 0.4), robot_yaw=0.0, corridor=Section.SOUTH,
        )
        assert router.active_sign_count == 0

        router.reset_for_new_lap()
        assert router.active_sign_count == 1

        wp = (1.5, 0.4)
        result = router.deform_waypoint(
            waypoint=wp, robot_pos=(1.5 - 0.2, 0.4), robot_yaw=0.0, corridor=Section.SOUTH,
        )
        assert result != wp


class TestEngagementGating:
    """A sign is only retired once approached — never discarded from afar."""

    def test_distant_sign_at_spawn_not_prematurely_passed(self):
        # Sign is farther than passed_dist at spawn; the buggy behaviour marked
        # it passed on the first tick, silently disabling routing.
        sign = _sign_at(2.0, 0.4, "red")
        router = _router([sign])

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


# ── Camera-detection confirmation (pinhole projection) ───────────────────────

# All direct _detection_to_world / _match_detection_to_sign cases below use a
# robot at the origin facing east (yaw=0) unless stated otherwise, so
# theta_h == bearing and world position == (distance*cos, distance*sin).


def _detection_at_distance_bearing(
    distance: float, theta_h: float, *, color: str = "red", confidence: float = 0.9,
) -> Detection:
    """Build a Detection whose bbox pinhole-decodes to the given distance/bearing.

    Inverts exactly the formula ``_detection_to_world`` decodes: pixel height
    from distance, bbox center from horizontal angle.
    """
    pixel_height = (_CAMERA_FOCAL_PX * TrafficSignSpecs.HEIGHT) / distance
    cx = (theta_h / RobotSpecs.CAMERA_HFOV + 0.5) * RobotSpecs.CAMERA_WIDTH
    cy = RobotSpecs.CAMERA_HEIGHT / 2
    half = pixel_height / 2
    bbox = (cx - half, cy - half, cx + half, cy + half)
    return Detection(
        class_name=color,
        confidence=confidence,
        bbox=bbox,
        x=cx,
        y=cy,
        width=pixel_height,
        height=pixel_height,
        area=pixel_height * pixel_height,
    )


class TestDetectionToWorld:
    """Pins the pinhole-projection math ``_detection_to_world`` uses to turn a
    bbox into a world position — previously untested (review 2026-07-11 §2.2).
    """

    def test_round_trip_recovers_distance_and_bearing(self):
        distance, theta_h = 0.6, 0.15
        det = _detection_at_distance_bearing(distance, theta_h)
        world = _detection_to_world(det, robot_pos=(0.0, 0.0), robot_yaw=0.0)
        expected = (distance * math.cos(theta_h), distance * math.sin(theta_h))
        assert world == pytest.approx(expected, abs=1e-6)

    def test_round_trip_with_nonzero_robot_pose(self):
        robot_pos = (1.2, 0.4)
        robot_yaw = 0.3
        distance, theta_h = 0.5, -0.1
        det = _detection_at_distance_bearing(distance, theta_h)
        world = _detection_to_world(det, robot_pos=robot_pos, robot_yaw=robot_yaw)
        bearing = robot_yaw + theta_h
        expected = (
            robot_pos[0] + distance * math.cos(bearing),
            robot_pos[1] + distance * math.sin(bearing),
        )
        assert world == pytest.approx(expected, abs=1e-6)

    def test_bbox_shorter_than_minimum_returns_none(self):
        tiny_height = _MIN_RELIABLE_BBOX_HEIGHT_PX - 1
        bbox = (100.0, 100.0, 101.0, 100.0 + tiny_height)
        det = Detection(
            class_name="red",
            confidence=0.9,
            bbox=bbox,
            x=100.5,
            y=100.0 + tiny_height / 2,
            width=1.0,
            height=tiny_height,
            area=tiny_height,
        )
        assert _detection_to_world(det, robot_pos=(0.0, 0.0), robot_yaw=0.0) is None


class TestMatchDetectionToSign:
    """Pins the confidence/match-distance/class gating in ``_match_detection_to_sign``."""

    def test_low_confidence_detection_rejected(self):
        det = _detection_at_distance_bearing(0.5, 0.0, color="red", confidence=0.1)
        result = _match_detection_to_sign(
            [det], expected_world_pos=(0.5, 0.0), robot_pos=(0.0, 0.0), robot_yaw=0.0, config=CFG,
        )
        assert result is None

    def test_far_match_rejected(self):
        det = _detection_at_distance_bearing(2.0, 0.0, color="red", confidence=0.9)
        result = _match_detection_to_sign(
            [det], expected_world_pos=(0.0, 0.0), robot_pos=(0.0, 0.0), robot_yaw=0.0, config=CFG,
        )
        assert result is None

    def test_non_sign_class_ignored(self):
        det = _detection_at_distance_bearing(0.5, 0.0, color="blue", confidence=0.9)
        result = _match_detection_to_sign(
            [det], expected_world_pos=(0.5, 0.0), robot_pos=(0.0, 0.0), robot_yaw=0.0, config=CFG,
        )
        assert result is None

    @pytest.mark.parametrize("order", [("near", "far"), ("far", "near")])
    def test_nearest_candidate_wins_regardless_of_order(self, order):
        expected = (0.5, 0.0)
        near = _detection_at_distance_bearing(0.5, 0.0, color="green", confidence=0.9)  # dist 0.0
        far = _detection_at_distance_bearing(0.65, 0.0, color="red", confidence=0.9)  # dist 0.15
        candidates = [near, far] if order[0] == "near" else [far, near]

        result = _match_detection_to_sign(
            candidates, expected_world_pos=expected, robot_pos=(0.0, 0.0), robot_yaw=0.0, config=CFG,
        )
        assert result == "green"


class TestCameraDetectionOverridesGroundTruth:
    """A confident camera detection can override scenario-metadata ground truth.

    This is the one part of the navigation stack where a live sensor reading
    beats known-good ground truth (review 2026-07-11 §2.2) — proves the
    override actually changes which side the robot passes on, not just that
    the private color-matching helpers return the right string in isolation.
    """

    def test_camera_color_flips_avoidance_side(self):
        _, (sx, sy), _ = _SECTION_GEOMETRY[Section.SOUTH]
        sign = _sign_at(sx, sy, "red")  # ground truth: red
        router = _router([sign])

        robot_pos = (sx - 0.3, sy)
        det = _detection_at_distance_bearing(0.3, 0.0, color="green", confidence=0.9)

        result = router.deform_waypoint(
            waypoint=(sx, sy),
            robot_pos=robot_pos,
            robot_yaw=0.0,
            corridor=Section.SOUTH,
            detections=[det],
        )

        expected_if_green = _apply_deformation(
            (sx, sy), sign, "green", Section.SOUTH, Direction.COUNTERCLOCKWISE, LATERAL,
        )
        expected_if_red = _apply_deformation(
            (sx, sy), sign, "red", Section.SOUTH, Direction.COUNTERCLOCKWISE, LATERAL,
        )
        assert result == pytest.approx(expected_if_green, abs=1e-6)
        assert result != pytest.approx(expected_if_red, abs=1e-6)


# ── 5. No-op with empty sign list ─────────────────────────────────────────────


def test_empty_sign_list_returns_waypoint_unchanged():
    router = _router([])
    wp = (1.5, 0.4)
    result = router.deform_waypoint(
        waypoint=wp,
        robot_pos=(1.4, 0.4),
        robot_yaw=0.0,
        corridor=Section.SOUTH,
    )
    assert result == wp


# ── 6. WP-1: deformation stays clear of the inner square and outer wall ──────


class TestDeformationClamping:
    """A sign near a corridor edge must never deform the waypoint into the
    restricted inner square or beyond the outer wall.
    """

    def test_sign_at_inner_edge_does_not_enter_inner_square(self):
        # South corridor, sign right at the inner-square boundary (y=1.0):
        # unclamped this deforms to y=1.15 — inside the restricted square.
        sign = _sign_at(1.5, 1.0, "red")
        wx, wy = _apply_deformation(
            (1.5, 1.0), sign, "red", Section.SOUTH, Direction.COUNTERCLOCKWISE, LATERAL,
        )
        assert wx == pytest.approx(1.5)
        assert wy < 1.0, "deformed waypoint must stay below the inner square"

    def test_sign_at_outer_edge_does_not_cross_wall(self):
        # South corridor, sign right at the outer wall (y=0.0): unclamped this
        # deforms to y=-0.15 — beyond the track boundary.
        sign = _sign_at(1.5, 0.0, "green")
        wx, wy = _apply_deformation(
            (1.5, 0.0), sign, "green", Section.SOUTH, Direction.COUNTERCLOCKWISE, LATERAL,
        )
        assert wx == pytest.approx(1.5)
        assert wy >= 0.0, "deformed waypoint must stay on the track"

    def test_sign_at_inner_edge_east_corridor(self):
        # East corridor deforms x; sign at the inner-square boundary (x=2.0).
        # EAST/CCW red_mult=-1: unclamped this deforms to x=1.85 — inside the
        # inner square.
        sign = _sign_at(2.0, 1.5, "red")
        wx, wy = _apply_deformation(
            (2.0, 1.5), sign, "red", Section.EAST, Direction.COUNTERCLOCKWISE, LATERAL,
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
    """Red is avoided outward, green inward — for every corridor, in BOTH directions.

    This pins the official WRO pass-side rule as an ABSOLUTE, track-relative
    invariant: it must hold identically whether the round is driven clockwise
    or counterclockwise, so a future edit cannot silently make it
    direction-dependent again (an earlier version of this table pinned "red on
    the robot's right" instead — a travel-relative rule that flips outward and
    inward between CW and CCW, which is not the actual official rule).
    """

    @pytest.mark.parametrize(("section", "direction"), list(_ROUTING_TABLE))
    @pytest.mark.parametrize("color", ["red", "green"])
    def test_sign_kept_on_correct_side(self, section, direction, color):
        # A realistic in-corridor sign position (clear of the inner square, per
        # WP-1 clamping) rather than a section-agnostic point — (1.5, 1.5) sits
        # inside the restricted inner square itself, which no real sign ever does.
        _, (sx, sy), _ = _SECTION_GEOMETRY[section]
        sign = _sign_at(sx, sy, color)
        wx, wy = _apply_deformation(
            (sign.x, sign.y), sign, color, section, direction, LATERAL,
        )
        ox, oy = _OUTWARD_DIR[section]
        outward_component = ox * (wx - sign.x) + oy * (wy - sign.y)
        if color == "red":
            assert outward_component > 0, "red must be avoided on the outward side"
        else:
            assert outward_component < 0, "green must be avoided on the inward side"
