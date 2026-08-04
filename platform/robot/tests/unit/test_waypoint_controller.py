"""Unit tests for WaypointController's pure-pursuit steering and rate limiting."""

from __future__ import annotations

import math

import pytest

from src.navigation.control.controllers.waypoint_controller import WaypointController


def _make_controller(**overrides) -> WaypointController:
    kwargs = {
        "max_steering_angle": 0.5236,
        "max_steering_rate": 2.0,
    }
    kwargs.update(overrides)
    return WaypointController(**kwargs)


def test_large_angle_error_is_rate_limited_across_ticks():
    # A target directly behind the robot is outside the curvature formula's
    # valid range (x_local <= 0), so it saturates to full lock -- same
    # end result as the old P-controller's clamp, but via the explicit
    # target-behind fallback rather than an oversized gain.
    controller = _make_controller(max_steering_rate=2.0)
    dt = 0.05
    max_delta_norm = (2.0 * dt) / controller.max_steering_angle

    first, _, _ = controller.compute_steering(
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=(-1.0, 0.0),
        crosstrack_error=0.0,
        dt=dt,
    )
    assert first == pytest.approx(max_delta_norm, abs=1e-9)

    second, _, _ = controller.compute_steering(
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=(-1.0, 0.0),
        crosstrack_error=0.0,
        dt=dt,
    )
    assert abs(second - first) == pytest.approx(max_delta_norm, abs=1e-9)


def test_small_angle_error_is_not_rate_limited():
    controller = _make_controller(max_steering_rate=2.0)
    steering, _, angle_error = controller.compute_steering(
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=(1.0, 0.01),
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
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=(-1.0, 0.0),
        crosstrack_error=0.0,
        dt=dt,
    )
    controller.reset()

    steering, _, _ = controller.compute_steering(
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=(-1.0, 0.0),
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
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=(0.40, 0.20),
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
