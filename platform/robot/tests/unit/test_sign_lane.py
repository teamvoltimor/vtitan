"""Pins for the Obstacles-only sign lane planner.

The lane transform rewrites the planned path, so the properties worth pinning
are the ones every consumer of that path silently depends on: it must be 1:1
and order-preserving (``CoreNavigator._waypoint_index`` denotes the same point
before and after), it must leave corner arcs alone (their radius is sized
against the corridors they join), and it must be a literal no-op with no
signs (which is every Open Challenge run).
"""

from __future__ import annotations

from itertools import pairwise

import pytest
from shared.config.constants import TrackDimensions, TrafficSignSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Section
from shared.domain.models import SignColor, Waypoint

from src.navigation.planning.sign_lane import SignLaneParams, apply_sign_lanes
from src.navigation.planning.sign_router import SignRouterConfig, SignSpec

_OFFSET = 0.28

_SHIPPED_OFFSET = SignRouterConfig.from_tuning(NavigationTuning.load_default().sign_router).lateral_offset
"""The real offset, unlike the round ``_OFFSET`` fixture above.

Whether the clamp binds is a comparison between this and the corridor's own
geometry, so the gap-centring tests are only meaningful at the shipped value.
"""
_PARAMS = SignLaneParams(lateral_offset=_OFFSET, ramp_m=0.70, hold_m=0.25)

# SOUTH corridor: depth is x, lateral is y, and the inner square is above, so
# OUTWARD (red) is -y and INWARD (green) is +y.
_SOUTH_BASE_Y = 0.5


def _south_straight(count: int = 21) -> list[Waypoint]:
    """Evenly spaced centreline waypoints spanning the SOUTH corridor's straight."""
    lo, hi = TrackDimensions.CORNER_MIN, TrackDimensions.CORNER_MAX
    return [Waypoint(lo + (hi - lo) * i / (count - 1), _SOUTH_BASE_Y) for i in range(count)]


def _lateral_at(waypoints: list[Waypoint], depth: float) -> float:
    """The y of the waypoint nearest ``depth`` along x."""
    return min(waypoints, key=lambda wp: abs(wp.x - depth)).y


class TestNoSigns:
    """With nothing to route around the path must come back untouched."""

    def test_empty_sign_list_returns_input_path(self) -> None:
        path = _south_straight()
        assert apply_sign_lanes(path, [], _PARAMS) == path

    def test_empty_path_is_handled(self) -> None:
        assert apply_sign_lanes([], [(SignSpec(x=1.5, y=0.5, color=SignColor.RED), Section.SOUTH)], _PARAMS) == []


class TestPassSide:
    """The lane must land on the same side the pass-side rule already demands."""

    @pytest.mark.parametrize(
        ("color", "expected_sign"),
        [(SignColor.RED, -1), (SignColor.GREEN, +1)],
    )
    def test_lane_offsets_to_the_ruled_side(self, color: SignColor, expected_sign: int) -> None:
        sign = SignSpec(x=1.5, y=0.5, color=color)
        laned = apply_sign_lanes(_south_straight(), [(sign, Section.SOUTH)], _PARAMS)
        assert _lateral_at(laned, 1.5) == pytest.approx(sign.y + expected_sign * _OFFSET, abs=0.02)

    def test_lane_meets_the_corner_arc_on_the_centreline(self) -> None:
        """Both ends of the straight must still be centred, however the ramp falls.

        The corner arcs either side are deliberately never moved, so if the
        lane were simply truncated where its nominal ramp ran past
        CORNER_MIN/MAX the path would step sideways between the last arc point
        and the first straight one -- a kink arriving exactly where the chassis
        is finishing a turn. Here the sign sits mid-straight, which puts the
        nominal ramp start (1.5 - 0.25 - 0.70 = 0.55) well outside it, so this
        only passes if the ramp compresses rather than truncates.
        """
        sign = SignSpec(x=1.5, y=0.5, color=SignColor.RED)
        laned = apply_sign_lanes(_south_straight(), [(sign, Section.SOUTH)], _PARAMS)
        assert _lateral_at(laned, TrackDimensions.CORNER_MIN) == pytest.approx(_SOUTH_BASE_Y)
        assert _lateral_at(laned, TrackDimensions.CORNER_MAX) == pytest.approx(_SOUTH_BASE_Y)

    def test_transition_is_gradual_not_a_step(self) -> None:
        """The whole point of the lane: the offset arrives over metres, not one waypoint.

        A carrot-level deformation jumps to full offset the tick a sign enters
        activation range. If the lane did the same it would only have moved
        that step earlier in time rather than spreading it over distance, and
        the pure-pursuit shortfall this exists to close would come straight
        back.
        """
        sign = SignSpec(x=1.5, y=0.5, color=SignColor.RED)
        laned = apply_sign_lanes(_south_straight(), [(sign, Section.SOUTH)], _PARAMS)
        steps = [abs(b.y - a.y) for a, b in pairwise(laned)]
        assert max(steps) < _OFFSET / 2


class TestPathInvariants:
    """Properties CoreNavigator's index bookkeeping depends on."""

    def test_transform_is_one_to_one_and_ordered(self) -> None:
        path = _south_straight()
        laned = apply_sign_lanes(path, [(SignSpec(x=1.5, y=0.5, color=SignColor.RED), Section.SOUTH)], _PARAMS)
        assert len(laned) == len(path)
        # Depth (the coordinate that orders the path) is never touched.
        assert [wp.x for wp in laned] == [wp.x for wp in path]

    def test_corner_arc_waypoints_are_untouched(self) -> None:
        """Arc points sit outside [CORNER_MIN, CORNER_MAX] in depth and must not move."""
        arc = [Waypoint(0.8, 0.42), Waypoint(2.2, 0.42)]
        path = [*arc[:1], *_south_straight(), *arc[1:]]
        laned = apply_sign_lanes(path, [(SignSpec(x=1.1, y=0.5, color=SignColor.RED), Section.SOUTH)], _PARAMS)
        assert laned[0] == arc[0]
        assert laned[-1] == arc[1]

    def test_lane_stays_clear_of_the_inner_square(self) -> None:
        """A green sign hard against the inner square must not command a lane inside it."""
        sign = SignSpec(x=1.5, y=TrackDimensions.CORNER_MIN - 0.05, color=SignColor.GREEN)
        laned = apply_sign_lanes(_south_straight(), [(sign, Section.SOUTH)], _PARAMS)
        assert max(wp.y for wp in laned) < TrackDimensions.CORNER_MIN


class TestCornerEntry:
    """Borrowed corner runway, for the ~94% of signs sitting at a section boundary.

    Corpus fact these rest on: 1211 of 1282 signs sit at along-corridor depth
    1.00 or 2.00, and 0 of 1282 sit in a corner.
    """

    # A boundary sign, i.e. the overwhelmingly common case.
    _BOUNDARY = SignSpec(x=TrackDimensions.CORNER_MIN, y=0.5, color=SignColor.RED)

    def _with_arc(self) -> list[Waypoint]:
        """Straight preceded by arc points that have begun turning off-centre."""
        arc = [Waypoint(0.70, 0.86), Waypoint(0.80, 0.72), Waypoint(0.90, 0.60)]
        return [*arc, *_south_straight()]

    def test_without_runway_a_boundary_sign_steps_off_the_arc(self) -> None:
        """The defect the runway exists to fix, pinned so it stays fixed.

        Confined to the straight, the lane has nowhere to ramp on the near
        side of a boundary sign, so its first straight waypoint is already at
        full offset while the arc point immediately before it has not moved.
        """
        path = self._with_arc()
        laned = apply_sign_lanes(path, [(self._BOUNDARY, Section.SOUTH)], _PARAMS)
        step_at_arc_join = abs(laned[3].y - laned[2].y)
        assert step_at_arc_join > 0.15

    def test_borrowed_runway_shrinks_the_worst_step(self) -> None:
        """Relative to the no-runway case, not against an absolute threshold.

        A ramp necessarily changes waypoint spacing while it ramps -- that IS
        the transition -- so the honest claim is that spreading the same
        lateral travel over more path makes the worst single step smaller,
        which is what the chassis actually has to track.
        """
        path = self._with_arc()
        confined = apply_sign_lanes(path, [(self._BOUNDARY, Section.SOUTH)], _PARAMS)
        borrowed = apply_sign_lanes(
            path,
            [(self._BOUNDARY, Section.SOUTH)],
            SignLaneParams(lateral_offset=_OFFSET, ramp_m=0.70, hold_m=0.25, corner_entry_m=0.45),
        )
        worst_confined = max(abs(b.y - a.y) for a, b in pairwise(confined))
        worst_borrowed = max(abs(b.y - a.y) for a, b in pairwise(borrowed))
        assert worst_borrowed < worst_confined * 0.75

    def test_borrowing_translates_the_arc_rather_than_flattening_it(self) -> None:
        """The arc must stay a turn; only its position may move.

        Applying the profile as an absolute lateral instead of a shift would
        assign every borrowed arc point the same value -- snapping the turn
        flat onto one line. The signature of that failure is arc points
        collapsing to a common lateral, so pin that they stay distinct and
        keep sweeping the same way.
        """
        path = self._with_arc()
        params = SignLaneParams(lateral_offset=_OFFSET, ramp_m=0.70, hold_m=0.25, corner_entry_m=0.45)
        laned = apply_sign_lanes(path, [(self._BOUNDARY, Section.SOUTH)], params)
        arc_before = [wp.y for wp in path[:3]]
        arc_after = [wp.y for wp in laned[:3]]
        assert len(set(arc_after)) == len(set(arc_before)), "arc points must not collapse onto one line"
        # Still monotonically descending toward the corridor, as the arc was.
        assert all(b < a for a, b in pairwise(arc_after))

    def test_borrowing_never_moves_a_point_into_the_neighbouring_corridor(self) -> None:
        """The lateral test does not widen with the depth test, so the borrow self-limits."""
        path = [Waypoint(0.4, 1.4), *self._with_arc()]  # a WEST-corridor point
        params = SignLaneParams(lateral_offset=_OFFSET, ramp_m=0.70, hold_m=0.25, corner_entry_m=0.90)
        laned = apply_sign_lanes(path, [(self._BOUNDARY, Section.SOUTH)], params)
        assert laned[0] == path[0]


class TestTwoSigns:
    """Opposite-side signs in one corridor must produce an S-bend, not an average."""

    def test_opposing_signs_each_get_their_own_side(self) -> None:
        red = SignSpec(x=1.3, y=0.5, color=SignColor.RED)
        green = SignSpec(x=2.0, y=0.5, color=SignColor.GREEN)
        laned = apply_sign_lanes(
            _south_straight(41),
            [(red, Section.SOUTH), (green, Section.SOUTH)],
            _PARAMS,
        )
        assert _lateral_at(laned, 1.3) == pytest.approx(red.y - _OFFSET, abs=0.03)
        assert _lateral_at(laned, 2.0) == pytest.approx(green.y + _OFFSET, abs=0.03)


class TestGapCentre:
    """Squeezed plateaux placed at the gap midpoint rather than the clamp limit.

    See ``sign_router.pass_lateral`` for the geometry. These pin the two ways
    the change can silently degrade: the relaxed clamp failing to reach the
    centred lane (leaving only a different ramp shape, which is what the first
    implementation actually did), and the same relaxation letting a borrowed
    corner arc through the boundary it is there to guard.
    """

    # RED in SOUTH must pass OUTWARD, and at y=0.40 the sign is already on
    # that side -- the narrow gap, so the clamp binds. This is 646 of the
    # corpus's 1282 signs.
    _SQUEEZED = SignSpec(x=1.5, y=0.40, color=SignColor.RED)
    _ROOMY = SignSpec(x=1.5, y=0.60, color=SignColor.RED)

    def _plateau(self, sign: SignSpec, *, gap_centre_frac: float, path: list[Waypoint] | None = None) -> float:
        params = SignLaneParams(
            lateral_offset=_SHIPPED_OFFSET, ramp_m=0.70, hold_m=0.25, gap_centre_frac=gap_centre_frac
        )
        laned = apply_sign_lanes(path or _south_straight(), [(sign, Section.SOUTH)], params)
        return min(wp.y for wp in laned)

    def test_centred_plateau_survives_the_shift_clamp(self) -> None:
        """The plateau must actually REACH the gap midpoint on the applied path.

        ``_control_points`` choosing a centred target is not enough: the
        profile is applied as a shift and re-bounded afterwards, and bounding
        it with the very margin being rebalanced pulls the plateau back to the
        clamp limit -- reducing the whole change to a different ramp shape
        while every direct test of ``pass_lateral`` still passes.
        """
        expected = (TrackDimensions.MIN_COORD + self._SQUEEZED.y - TrafficSignSpecs.WIDTH / 2) / 2
        assert self._plateau(self._SQUEEZED, gap_centre_frac=1.0) == pytest.approx(expected)
        assert self._plateau(self._SQUEEZED, gap_centre_frac=0.0) > expected

    def test_unsqueezed_sign_is_byte_identical(self) -> None:
        """Where the offset already fits, the flag must change nothing at all."""
        params = {"lateral_offset": _SHIPPED_OFFSET, "ramp_m": 0.70, "hold_m": 0.25}
        path = _south_straight()
        off = apply_sign_lanes(path, [(self._ROOMY, Section.SOUTH)], SignLaneParams(**params, gap_centre_frac=0.0))
        on = apply_sign_lanes(path, [(self._ROOMY, Section.SOUTH)], SignLaneParams(**params, gap_centre_frac=1.0))
        assert on == off

    def test_borrowed_arc_is_still_bounded(self) -> None:
        """Relaxing the clamp must not let corner curvature through the wall.

        The relaxation is allowed to reach the profile's own value and no
        further, so an arc point whose translated position overshoots BEYOND
        the lane is still caught. Uses an arc bent hard toward the outer wall,
        which is the shape that overshoots.
        """
        arc = [Waypoint(0.70, 0.30), Waypoint(0.80, 0.22), Waypoint(0.90, 0.16)]
        path = [*arc, *_south_straight()]
        params = SignLaneParams(
            lateral_offset=_SHIPPED_OFFSET, ramp_m=0.70, hold_m=0.25, corner_entry_m=0.45, gap_centre_frac=1.0
        )
        laned = apply_sign_lanes(path, [(self._SQUEEZED, Section.SOUTH)], params)
        floor = (TrackDimensions.MIN_COORD + self._SQUEEZED.y - TrafficSignSpecs.WIDTH / 2) / 2
        assert all(wp.y >= floor - 1e-9 for wp in laned)
