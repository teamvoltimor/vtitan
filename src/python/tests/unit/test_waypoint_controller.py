"""Unit tests for WaypointController's pure-pursuit steering and rate limiting."""

from __future__ import annotations

import math
from itertools import pairwise

import pytest
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import Waypoint

from src.navigation.control.controllers.waypoint_controller import WaypointController


def _make_controller(**overrides) -> WaypointController:
    controller = WaypointController.from_tuning(NavigationTuning.load_default())
    for key, value in overrides.items():
        setattr(controller, key, value)
    return controller


def test_large_angle_error_is_rate_limited_across_ticks():
    # A target directly behind the robot is outside the curvature formula's
    # valid range (x_local <= 0), so it saturates to full lock -- same
    # end result as the old P-controller's clamp, but via the explicit
    # target-behind fallback rather than an oversized gain.
    controller = _make_controller(max_steering_rate=2.0)
    dt = 0.05
    max_delta_norm = (2.0 * dt) / controller.max_steering_angle

    first, _, _ = controller.compute_steering(
        current_pos=Waypoint(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=Waypoint(-1.0, 0.0),
        crosstrack_error=0.0,
        dt=dt,
    )
    assert first == pytest.approx(max_delta_norm, abs=1e-9)

    second, _, _ = controller.compute_steering(
        current_pos=Waypoint(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=Waypoint(-1.0, 0.0),
        crosstrack_error=0.0,
        dt=dt,
    )
    assert abs(second - first) == pytest.approx(max_delta_norm, abs=1e-9)


def test_small_angle_error_is_not_rate_limited():
    controller = _make_controller(max_steering_rate=2.0)
    steering, _, angle_error = controller.compute_steering(
        current_pos=Waypoint(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=Waypoint(1.0, 0.01),
        crosstrack_error=0.0,
        dt=0.05,
    )
    # Small error: the curvature-based output is well under the per-tick rate cap.
    assert 0.0 < steering < 0.05
    assert angle_error == pytest.approx(math.atan2(0.01, 1.0), abs=1e-9)


def test_reset_clears_rate_limit_memory():
    controller = _make_controller(max_steering_rate=2.0)
    dt = 0.05
    controller.compute_steering(
        current_pos=Waypoint(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=Waypoint(-1.0, 0.0),
        crosstrack_error=0.0,
        dt=dt,
    )
    controller.reset()

    steering, _, _ = controller.compute_steering(
        current_pos=Waypoint(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=Waypoint(-1.0, 0.0),
        crosstrack_error=0.0,
        dt=dt,
    )
    max_delta_norm = (2.0 * dt) / controller.max_steering_angle
    assert steering == pytest.approx(max_delta_norm, abs=1e-9)


def test_forward_target_uses_curvature_not_gain():
    # A target ahead and to the left should steer left (positive), with a
    # magnitude set by chassis geometry (wheelbase/2), not an arbitrary gain.
    controller = _make_controller(max_steering_rate=100.0)  # effectively unrated for this check
    steering, _, angle_error = controller.compute_steering(
        current_pos=Waypoint(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=Waypoint(0.40, 0.20),
        crosstrack_error=0.0,
        dt=0.05,
    )
    assert angle_error > 0
    assert steering > 0
    assert steering < 1.0


class TestSelectTargetPointWrapsAndStaysAhead:
    """2026-08-03: select_target_point used to be handed a pre-sliced
    remainder (waypoints[waypoint_index:]) and searched it by distance alone,
    ignoring heading. Near the end of a lap that slice could run dry, falling
    back to a single fixed final point instead of continuing around the loop
    -- measured on real hardware as steering pinned near zero for tens of
    seconds while heading drifted 85+ degrees. And picking by distance alone
    could return a point behind the chassis, which the curvature steering law
    is not valid for -- measured as a wrong-direction turn. Neither is
    reproducible in sim (see
    docs/internal/audits/2026-08-03-realtrack-control-instability-findings.md),
    so these construct the failure geometry directly instead of relying on a
    sim run to happen to hit it.
    """

    def test_wraps_past_the_end_of_the_lap_instead_of_going_dry(self):
        # A small closed loop. Starting the search at the last index, nothing
        # from there to the literal end of the array reaches the lookahead
        # distance -- only continuing around to index 1 does.
        waypoints = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        controller = _make_controller()

        target = controller.select_target_point(
            current_pos=(0.0, 1.0),
            current_yaw=0.0,  # facing +x
            waypoints=waypoints,
            waypoint_index=3,
            lookahead_distance=1.0,
        )

        assert target == (1.0, 0.0)

    def test_skips_a_behind_candidate_even_when_it_is_farther_by_distance(self):
        # index 1 is far enough by pure distance but behind the chassis;
        # index 2 is closer but ahead. The old distance-only, array-order
        # search would have returned index 1 first.
        waypoints = [(0.0, 0.0), (-5.0, 0.0), (3.0, 0.0)]
        controller = _make_controller()

        target = controller.select_target_point(
            current_pos=(0.0, 0.0),
            current_yaw=0.0,  # facing +x -- index 1 (-5, 0) is directly behind
            waypoints=waypoints,
            waypoint_index=0,
            lookahead_distance=1.0,
        )

        assert target == (3.0, 0.0)

    def test_falls_back_to_nearest_ahead_when_nothing_reaches_lookahead(self):
        # 2026-08-04: was "farthest ahead" -- see select_target_point's
        # docstring for why that starved the curvature formula on real
        # hardware and got reversed to nearest.
        waypoints = [(0.0, 0.0), (0.1, 0.0), (0.2, 0.0)]
        controller = _make_controller()

        target = controller.select_target_point(
            current_pos=(0.0, 0.0),
            current_yaw=0.0,
            waypoints=waypoints,
            waypoint_index=0,
            lookahead_distance=5.0,  # farther than anything on this tiny loop
        )

        assert target == (0.1, 0.0)  # nearest point that is still ahead


class TestCrosstrackBudgetFromWallDistance:
    """The crosstrack threshold must not exceed what the path can afford.

    ``lookahead_transition`` is a fixed 0.30 m, which assumes the path has that
    much room to drift into before a wall. Under the blind narrow prior it does
    not: a corridor believed 0.60 puts the path ~0.25-0.30 m from the outer
    wall and the chassis half-width takes 0.097 of that. Measured on hardware
    2026-08-06, crosstrack ran 0.09 -> 0.15 through a corner and never crossed
    0.30, so the short lookahead never engaged and the robot drove to within
    0.10 m of the wall.
    """

    def test_defaults_to_the_configured_transition(self):
        controller = _make_controller(lookahead_transition=0.30)
        assert controller.effective_transition == pytest.approx(0.30)

    def test_a_tight_budget_lowers_the_threshold(self):
        controller = _make_controller(lookahead_transition=0.30)
        controller.set_crosstrack_budget(0.123)
        assert controller.effective_transition == pytest.approx(0.123)

    def test_a_generous_budget_does_not_raise_it(self):
        """A wide corridor must behave exactly as before, not steer twitchier."""
        controller = _make_controller(lookahead_transition=0.30)
        controller.set_crosstrack_budget(0.373)
        assert controller.effective_transition == pytest.approx(0.30)

    def test_clearing_the_budget_restores_the_configured_value(self):
        controller = _make_controller(lookahead_transition=0.30)
        controller.set_crosstrack_budget(0.10)
        controller.set_crosstrack_budget(None)
        assert controller.effective_transition == pytest.approx(0.30)

    def test_short_lookahead_engages_within_the_budget(self):
        """The point of the whole thing: 0.15 m of crosstrack must arm the
        correction on a wall-hugging path, where it previously did not."""
        controller = _make_controller(
            lookahead_transition=0.30,
            lookahead_short=0.20,
            lookahead_long=0.40,
        )
        assert controller.select_lookahead(0.15) == pytest.approx(0.40)

        controller.set_crosstrack_budget(0.123)

        assert controller.select_lookahead(0.15) == pytest.approx(0.20)


class TestUpcomingTurnArmsTheShortLookahead:
    """Crosstrack error cannot rise until a corner has already been missed, so
    gating on it alone lands the sharp correction after the corner. Measured on
    hardware 2026-08-06: 0.9 rad of heading error held for three seconds at
    0.23 of full lock while crosstrack sat near 0.01, then steering jumped to
    0.52 the moment crosstrack reached 0.13 -- right magnitude, a corner late.
    """

    def _controller(self):
        return _make_controller(
            lookahead_transition=0.30,
            lookahead_short=0.20,
            lookahead_long=0.40,
            corner_turn_threshold_rad=0.35,
        )

    def test_straight_ahead_keeps_the_long_lookahead(self):
        """A straight must behave exactly as before -- a short lookahead there
        is twitchy, which is the failure this trades against."""
        assert self._controller().select_lookahead(0.01, turn_ahead_rad=0.0) == pytest.approx(0.40)

    def test_corner_ahead_shortens_it_while_still_on_path(self):
        # 0.40 m of preview on the default 0.45 m corner arc turns ~0.89 rad.
        assert self._controller().select_lookahead(0.01, turn_ahead_rad=0.89) == pytest.approx(0.20)

    def test_a_gentle_bend_is_not_treated_as_a_corner(self):
        assert self._controller().select_lookahead(0.01, turn_ahead_rad=0.20) == pytest.approx(0.40)

    def test_crosstrack_still_arms_it_with_no_turn_ahead(self):
        """The lagging signal stays wired -- the new one is added, not swapped."""
        assert self._controller().select_lookahead(0.35, turn_ahead_rad=0.0) == pytest.approx(0.20)

    def test_omitting_the_turn_preserves_the_old_behaviour(self):
        controller = self._controller()
        assert controller.select_lookahead(0.01) == pytest.approx(0.40)
        assert controller.select_lookahead(0.35) == pytest.approx(0.20)


class TestLookaheadRampsRatherThanSwitching:
    """The switch used to be a step, and both arming signals sit near their
    thresholds in normal driving -- so it flipped long/short on consecutive
    ticks (hardware run_20260829_104641: 0.320, 0.160, 0.320, 0.160 at ~2.5 Hz).
    Curvature is 2y/L**2, so each flip swung the command by 4x and the chassis
    drew a visible zigzag. These pin the ramp that removes the discontinuity.
    """

    def _controller(self, **overrides):
        settings = {
            "lookahead_transition": 0.30,
            "lookahead_short": 0.20,
            "lookahead_long": 0.40,
            "corner_turn_threshold_rad": 0.35,
            "lookahead_blend_start": 0.70,
        }
        settings.update(overrides)
        return _make_controller(**settings)

    def test_no_single_step_moves_the_lookahead_far(self):
        """The actual anti-zigzag property: a small change in crosstrack cannot
        produce a large change in lookahead. Under the old step this failed by
        construction -- one tick either side of the threshold spanned the whole
        0.20 m range.
        """
        controller = self._controller()
        xs = [i * 0.005 for i in range(81)]  # 0 -> 0.40 m in 5 mm steps
        looks = [controller.select_lookahead(x) for x in xs]
        biggest_jump = max(abs(b - a) for a, b in pairwise(looks))
        span = controller.lookahead_long - controller.lookahead_short
        assert biggest_jump <= span / 4

    def test_lookahead_never_increases_as_the_robot_strays(self):
        """Monotone: straying further may only tighten tracking, never loosen
        it. A non-monotone blend would let a worsening error relax the
        correction, which is the failure mode this whole path exists to avoid.
        """
        controller = self._controller()
        looks = [controller.select_lookahead(i * 0.01) for i in range(41)]
        assert all(b <= a + 1e-12 for a, b in pairwise(looks))

    def test_the_deadband_leaves_straights_untouched(self):
        """Below the blend start the long lookahead must be exactly unchanged,
        so this cannot make calm driving twitchier than it already was.
        """
        controller = self._controller()
        below = 0.70 * controller.effective_transition * 0.99
        assert controller.select_lookahead(below) == pytest.approx(controller.lookahead_long)

    def test_blend_start_of_one_restores_the_hard_switch(self):
        """The escape hatch: 1.0 must reproduce the original step exactly, so
        the ramp can be disabled from config without a code change.
        """
        controller = self._controller(lookahead_blend_start=1.0)
        t = controller.effective_transition
        assert controller.select_lookahead(t * 0.99) == pytest.approx(controller.lookahead_long)
        assert controller.select_lookahead(t * 1.01) == pytest.approx(controller.lookahead_short)


class TestTargetMustBeReachable:
    """``MIN_TARGET_RADIUS_M`` skips aim points the chassis cannot curve onto.

    ``select_target_point`` accepted any candidate with ``x_local > 0``, so a
    point barely ahead but far to the side qualified. Measured over the
    2026-09-08 finishers, 57-59% of ticks aimed at a point demanding a radius
    tighter than the chassis's 0.29 m, which is a bearing no steering command
    can reduce -- and the heading speed cut fires on it every tick.
    """

    # 0.10 m ahead, 0.27 m to the side: the geometry measured at p50 on
    # hardware. Its pure-pursuit circle is d / (2 sin a) ~ 0.16 m.
    SIDEWAYS = (0.10, 0.27)
    # Straight ahead down the same path, reachable by construction.
    AHEAD = (0.60, 0.02)

    def test_disabled_by_default_keeps_the_nearest_ahead_point(self):
        controller = _make_controller(min_target_radius_m=0.0)
        target = controller.select_target_point(
            current_pos=(0.0, 0.0),
            current_yaw=0.0,
            waypoints=[self.SIDEWAYS, self.AHEAD],
            waypoint_index=0,
            lookahead_distance=0.16,
        )
        assert target == self.SIDEWAYS
        assert controller.last_target_unreachable is False

    def test_armed_filter_skips_to_the_reachable_point(self):
        controller = _make_controller(min_target_radius_m=0.29)
        target = controller.select_target_point(
            current_pos=(0.0, 0.0),
            current_yaw=0.0,
            waypoints=[self.SIDEWAYS, self.AHEAD],
            waypoint_index=0,
            lookahead_distance=0.16,
        )
        assert target == self.AHEAD
        assert controller.last_target_unreachable is False

    def test_the_skipped_point_really_was_unreachable(self):
        """Guards the premise, not just the branch."""
        dx, dy = self.SIDEWAYS
        dist = math.hypot(dx, dy)
        demanded_radius = dist / (2.0 * (abs(dy) / dist))
        assert demanded_radius < 0.29

    def test_falls_back_rather_than_inventing_a_target(self):
        """Every candidate unreachable: keep one, and say so.

        The fallback tiers are themselves answers to measured hardware
        failures, so the filter must narrow the choice and never remove it.
        """
        controller = _make_controller(min_target_radius_m=0.29)
        target = controller.select_target_point(
            current_pos=(0.0, 0.0),
            current_yaw=0.0,
            waypoints=[self.SIDEWAYS, (0.08, -0.30)],
            waypoint_index=0,
            lookahead_distance=0.16,
        )
        assert target in {self.SIDEWAYS, (0.08, -0.30)}
        assert controller.last_target_unreachable is True
