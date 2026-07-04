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

import pytest
from shared.config.enums import Direction, Section

from src.navigation.planning.sign_router import (
    _ROUTING_TABLE,
    SignRouter,
    SignRouterConfig,
    SignSpec,
    _apply_deformation,
)
from src.navigation.race_tracker import _TRAVEL_DIRS

LATERAL = 0.15
CFG = SignRouterConfig(lateral_offset=LATERAL, activation_dist=0.80, passed_dist=1.20)


# ── Helper ────────────────────────────────────────────────────────────────────


def _router(signs: list[SignSpec]) -> SignRouter:
    return SignRouter(signs, config=CFG)


def _sign_at(x: float, y: float, color: str) -> SignSpec:
    return SignSpec(x=x, y=y, color=color)


# 1. Deformation direction per corridor x color x travel direction

# Per-section: (perpendicular axis, sign position, CCW red multiplier).
# CCW values mirror the live routing table; CW is the world-frame negation.
_SECTION_GEOMETRY = {
    Section.SOUTH: ("y", (1.5, 0.4), +1),
    Section.NORTH: ("y", (1.5, 2.6), -1),
    Section.EAST: ("x", (2.6, 1.5), -1),
    Section.WEST: ("x", (0.4, 1.5), +1),
}


class TestDeformationDirections:
    """16 cases: 4 sections x {red, green} x {CCW, CW}.

    Red keeps the sign on the robot's right, green on its left. Because the
    same corridor is driven with opposite headings under CW vs CCW, the
    world-frame offset flips sign between the two directions.
    """

    @pytest.mark.parametrize("section", list(_SECTION_GEOMETRY))
    @pytest.mark.parametrize(
        ("direction", "color", "flip"),
        [
            (Direction.COUNTERCLOCKWISE, "red", +1),
            (Direction.COUNTERCLOCKWISE, "green", -1),
            (Direction.CLOCKWISE, "red", -1),
            (Direction.CLOCKWISE, "green", +1),
        ],
    )
    def test_offset_side(self, section, direction, color, flip):
        axis, (sx, sy), ccw_red = _SECTION_GEOMETRY[section]
        sign = _sign_at(sx, sy, color)
        rx, ry = _apply_deformation((sx, sy), sign, color, section, direction, LATERAL)
        expected = ccw_red * flip * LATERAL
        if axis == "y":
            assert ry == pytest.approx(sy + expected)
            assert rx == pytest.approx(sx)
        else:
            assert rx == pytest.approx(sx + expected)
            assert ry == pytest.approx(sy)


# ── 2. 36-scenario routing ────────────────────────────────────────────────────


# WRO official grid: 6 positions per corridor
# (depth, width) using SOUTH-corridor frame, depth ∈ {1.0, 1.5, 2.0}, width ∈ {0.4, 0.6}
_GRID_POSITIONS: list[tuple[float, float]] = [
    (1.0, 0.4),
    (1.0, 0.6),
    (1.5, 0.4),
    (1.5, 0.6),
    (2.0, 0.4),
    (2.0, 0.6),
]

# WRO 36 predefined scenarios: scenario ID → list of (color, depth, width) for SOUTH template
# Scenarios 1-12: single pillar
# Scenarios 13-36: double pillar
# We only exercise the core rule: red right, green left — no exhaustive enumeration needed
# Instead, verify all 6 grid positions × 2 colors × 4 sections = 48 routing decisions.


def _make_single_sign_scenario_cases():
    """Generate test cases: (corridor, sign_x, sign_y, color, expect_north_or_east).

    For SOUTH/NORTH corridors: "correct pass" means waypoint is on the expected y side.
    For EAST/WEST corridors: waypoint is on the expected x side.
    """
    cases = []
    for depth, width in _GRID_POSITIONS:
        for color in ("red", "green"):
            # SOUTH corridor: sign at (depth, width)
            sx, sy = depth, width
            if color == "red":
                expected_y = sy + LATERAL  # north of sign
            else:
                expected_y = sy - LATERAL  # south of sign
            cases.append(("south", sx, sy, color, expected_y, None))

            # NORTH corridor: sign at (depth, 3.0 - width)
            sx, sy = depth, 3.0 - width
            if color == "red":
                expected_y = sy - LATERAL
            else:
                expected_y = sy + LATERAL
            cases.append(("north", sx, sy, color, expected_y, None))

            # EAST corridor: sign at (3.0 - width, depth)
            sx, sy = 3.0 - width, depth
            if color == "red":
                expected_x = sx - LATERAL
            else:
                expected_x = sx + LATERAL
            cases.append(("east", sx, sy, color, None, expected_x))

            # WEST corridor: sign at (width, depth)
            sx, sy = width, depth
            if color == "red":
                expected_x = sx + LATERAL
            else:
                expected_x = sx - LATERAL
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


# 6. Pass-side rule pinned in the robot's travel frame


class TestPassSideRule:
    """Red stays on the robot's right, green on its left — for every corridor.

    This pins the routing table against the official WRO pass-side rule in the
    travel frame, independent of world-axis bookkeeping, so a future edit cannot
    silently invert red/green.
    """

    @pytest.mark.parametrize(("section", "direction"), list(_ROUTING_TABLE))
    @pytest.mark.parametrize("color", ["red", "green"])
    def test_sign_kept_on_correct_side(self, section, direction, color):
        heading_x, heading_y = _TRAVEL_DIRS[(section, direction)]
        sign = _sign_at(1.5, 1.5, color)
        wx, wy = _apply_deformation(
            (sign.x, sign.y), sign, color, section, direction, LATERAL,
        )
        # Signed lateral position of the deformed waypoint relative to the sign,
        # in the robot's travel frame: cross > 0 => waypoint on the robot's left
        # => the sign stays on the robot's right.
        cross = heading_x * (wy - sign.y) - heading_y * (wx - sign.x)
        if color == "red":
            assert cross > 0, "red sign must stay on the robot's right"
        else:
            assert cross < 0, "green sign must stay on the robot's left"
