"""Synthetic camera-detection emulator for closed-loop sign-routing tests.

Inverts the exact pinhole projection ``sign_router._detection_to_world()``
decodes, so a ``SignRouter`` driven through this emulator exercises the real
camera-confirmation code path (``_match_detection_to_sign`` /
``_detection_to_world``) instead of always seeing ``detections=None``.

Deliberately simple — a fixed high confidence, no false positives, no wall
occlusion modelling (the LIDAR raycast model in ``track_model.py`` does model
occlusion; this does not). The goal is exercising the detection-to-router
wiring end-to-end, not building a camera sensor model.
"""

from __future__ import annotations

import math

from shared.config.constants import RobotSpecs
from shared.domain.models import SignColor, TrafficSignObservation

from src.navigation.planning.sign_discovery import SignSpec
from src.simulation.geometry import _wrap_angle

_DETECTION_CONFIDENCE: float = 0.9
"""Fixed confidence reported for every emulated detection."""


def emulate_sign_observations(
    signs: list[SignSpec],
    robot_pos: tuple[float, float],
    robot_yaw: float,
    max_range: float = RobotSpecs.CAMERA_FAR_CLIP,
) -> list[TrafficSignObservation]:
    """Return synthetic ``TrafficSignObservation``s for every in-frame sign.

    A sign is "seen" if it's within ``max_range`` and within
    ``RobotSpecs.CAMERA_HFOV`` of the robot's heading; otherwise it's omitted.

    Args:
        signs: Ground-truth sign specs (position + color) to project.
        robot_pos: Robot (x, y) position.
        robot_yaw: Robot heading (radians, 0 = east).
        max_range: Maximum detection range.

    Returns:
        Emulated observations, one per in-frame sign.
    """
    observations: list[TrafficSignObservation] = []
    for sign in signs:
        dx = sign.x - robot_pos[0]
        dy = sign.y - robot_pos[1]
        distance = math.hypot(dx, dy)
        if distance <= 0.0 or distance > max_range:
            continue

        bearing = math.atan2(dy, dx)
        theta_h = _wrap_angle(bearing - robot_yaw)
        if abs(theta_h) > RobotSpecs.CAMERA_HFOV / 2:
            continue

        observations.append(
            TrafficSignObservation(
                world_x_m=sign.x,
                world_y_m=sign.y,
                color=SignColor.RED if sign.color == "red" else SignColor.GREEN,
                confidence=_DETECTION_CONFIDENCE,
                detected_at_timestamp=0.0,
            ),
        )
    return observations
