"""Pure joystick -> Ackermann command mapping (no rclpy dependency, easy to unit test)."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.teleop.config import Config


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def compute_command(
    axes: list[float],
    buttons: list[int],
    config: Config,
    *,
    joy_is_fresh: bool,
) -> tuple[float, float]:
    """Map a /joy message's axes/buttons to (speed_mps, steering_deg).

    Steering always tracks the steering axis, so calibration can be checked
    without arming the drive motor. Speed is zero unless the joystick link
    is fresh and the dead-man button is currently held.

    Args:
        axes: `sensor_msgs/Joy.axes`, each in [-1.0, 1.0].
        buttons: `sensor_msgs/Joy.buttons`, each 0 or 1.
        config: Teleop configuration (axis/button indices, limits, invert flags).
        joy_is_fresh: False if no /joy message has arrived within `config.joy_timeout_s`.

    Returns:
        (speed_mps, steering_deg), both clamped to the configured limits.
    """
    steering_raw = axes[config.steering_axis_index]
    if config.steering_invert:
        steering_raw = -steering_raw
    steering_deg = _clamp(steering_raw * config.max_steering_deg, config.max_steering_deg)

    deadman_held = joy_is_fresh and bool(buttons[config.deadman_button_index])
    if not deadman_held:
        return 0.0, steering_deg

    throttle_raw = axes[config.throttle_axis_index]
    if config.throttle_invert:
        throttle_raw = -throttle_raw
    speed_mps = _clamp(throttle_raw * config.max_speed_mps, config.max_speed_mps)

    return speed_mps, steering_deg
