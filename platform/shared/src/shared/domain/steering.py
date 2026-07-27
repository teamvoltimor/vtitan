"""Steering command contract shared between the controller, sim, and actuators.

The navigation stack publishes steering in ``DriveCommand.steering_norm`` as a
*normalised* command in ``[-1, 1]`` (``+1`` = full left / counter-clockwise), NOT an angle in
radians and NOT a yaw rate. ``WaypointController`` produces it as
``steering_rad / MAX_STEERING_ANGLE``; the simulator's ``AckermannKinematics``
consumes it as ``clamp(norm) * MAX_STEERING_ANGLE``. Any actuator adapter must
decode it the same way, or the physical robot steers by a different amount than
every test proved safe.

This module is the single definition of that mapping so the controller, the
headless simulator, and the Build HAT motor adapter cannot drift apart.
"""

from __future__ import annotations

from typing import Final

STEERING_NORM_MIN: Final[float] = -1.0
STEERING_NORM_MAX: Final[float] = 1.0


def steering_norm_to_angle_rad(steering_norm: float, max_steering_angle: float) -> float:
    """Decode a normalised steering command into a physical front-wheel angle.

    Args:
        steering_norm: Normalised steering command in ``[-1, 1]`` as carried in
            ``DriveCommand.steering_norm``. Values outside the range are clamped.
        max_steering_angle: Physical steering saturation limit (radians).

    Returns:
        Front-wheel steering angle in radians, ``+`` = left (counter-clockwise).
    """
    clamped = max(STEERING_NORM_MIN, min(STEERING_NORM_MAX, steering_norm))
    return clamped * max_steering_angle


def angle_rad_to_steering_norm(angle_rad: float, max_steering_angle: float) -> float:
    """Encode a physical front-wheel angle into a normalised steering command.

    Inverse of :func:`steering_norm_to_angle_rad`; the controller uses this shape
    to publish ``DriveCommand.steering_norm``.

    Args:
        angle_rad: Front-wheel steering angle (radians), ``+`` = left.
        max_steering_angle: Physical steering saturation limit (radians).

    Returns:
        Normalised steering command in ``[-1, 1]``.
    """
    if max_steering_angle <= 0.0:
        return 0.0
    return max(STEERING_NORM_MIN, min(STEERING_NORM_MAX, angle_rad / max_steering_angle))
