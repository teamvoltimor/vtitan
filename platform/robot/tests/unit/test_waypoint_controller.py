"""Unit tests for WaypointController's pure-pursuit steering and rate limiting."""

from __future__ import annotations

import pytest

from src.navigation.control.controllers.waypoint_controller import WaypointController


def _make_controller(**overrides) -> WaypointController:
    kwargs = {
        "max_steering_angle": 0.5236,
        "steer_kp": 1.5,
        "max_steering_rate": 2.0,
    }
    kwargs.update(overrides)
    return WaypointController(**kwargs)


def test_large_angle_error_is_rate_limited_across_ticks():
    # A target directly behind the robot demands a ~pi angle error, which the
    # P-controller would otherwise saturate to max_steering_angle in one tick.
    controller = _make_controller(max_steering_rate=2.0)
    dt = 0.05
    max_delta_norm = (2.0 * dt) / controller.max_steering_angle

    first, _ = controller.compute_steering(
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=(-1.0, 0.0),
        forward_clearance=1.0,
        dt=dt,
    )
    assert first == pytest.approx(max_delta_norm, abs=1e-9)

    second, _ = controller.compute_steering(
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=(-1.0, 0.0),
        forward_clearance=1.0,
        dt=dt,
    )
    assert abs(second - first) == pytest.approx(max_delta_norm, abs=1e-9)


def test_small_angle_error_is_not_rate_limited():
    controller = _make_controller(max_steering_rate=2.0)
    steering, _ = controller.compute_steering(
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=(1.0, 0.01),
        forward_clearance=1.0,
        dt=0.05,
    )
    # Small error: the P-controller output is well under the per-tick rate cap.
    assert 0.0 < steering < 0.05


def test_reset_clears_rate_limit_memory():
    controller = _make_controller(max_steering_rate=2.0)
    dt = 0.05
    controller.compute_steering(
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=(-1.0, 0.0),
        forward_clearance=1.0,
        dt=dt,
    )
    controller.reset()

    steering, _ = controller.compute_steering(
        current_pos=(0.0, 0.0),
        current_yaw=0.0,
        target_waypoint=(-1.0, 0.0),
        forward_clearance=1.0,
        dt=dt,
    )
    max_delta_norm = (2.0 * dt) / controller.max_steering_angle
    assert steering == pytest.approx(max_delta_norm, abs=1e-9)
