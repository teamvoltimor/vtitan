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

from shared.config.constants import RobotSpecs, TrafficSignSpecs
from shared.domain.models import Detection

from src.navigation.planning.sign_router import _CAMERA_FOCAL_PX, SignSpec

_DETECTION_CONFIDENCE: float = 0.9
"""Fixed confidence reported for every emulated detection."""


def emulate_sign_detections(
    signs: list[SignSpec],
    robot_pos: tuple[float, float],
    robot_yaw: float,
    max_range: float = RobotSpecs.CAMERA_FAR_CLIP,
) -> list[Detection]:
    """Return synthetic ``Detection``s for every sign in the camera's frame.

    A sign is "seen" if it's within ``max_range`` and within
    ``RobotSpecs.CAMERA_HFOV`` of the robot's heading; otherwise it's omitted,
    the same way a real sign behind or far from the robot wouldn't appear in
    a frame.

    Args:
        signs: Ground-truth sign specs (position + color) to project.
        robot_pos: Robot (x, y) position (metres).
        robot_yaw: Robot heading (radians, 0 = east).
        max_range: Maximum detection range (metres).

    Returns:
        Emulated detections, one per in-frame sign.
    """
    detections: list[Detection] = []
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

        pixel_height = (_CAMERA_FOCAL_PX * TrafficSignSpecs.HEIGHT) / distance
        cx = (theta_h / RobotSpecs.CAMERA_HFOV + 0.5) * RobotSpecs.CAMERA_WIDTH

        half_h = pixel_height / 2
        half_w = pixel_height / 2  # sign bbox treated as roughly square
        cy = RobotSpecs.CAMERA_HEIGHT / 2
        x1, y1 = cx - half_w, cy - half_h
        x2, y2 = cx + half_w, cy + half_h

        detections.append(
            Detection(
                class_name=sign.color,
                confidence=_DETECTION_CONFIDENCE,
                bbox=(x1, y1, x2, y2),
                x=(x1 + x2) / 2,
                y=(y1 + y2) / 2,
                width=x2 - x1,
                height=y2 - y1,
                area=(x2 - x1) * (y2 - y1),
            ),
        )
    return detections


def _wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
