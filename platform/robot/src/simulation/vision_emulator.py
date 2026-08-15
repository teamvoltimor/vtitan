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
from typing import TYPE_CHECKING

from shared.config.constants import RobotSpecs
from shared.domain.models import SignColor, TrafficSignObservation

from src.config.tuning_helpers import get_tuning
from src.navigation.utils import wrap_angle as _wrap_angle

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning

    from src.navigation.planning.sign_discovery import SignSpec


def emulate_sign_observations(
    signs: list[SignSpec],
    robot_pos: tuple[float, float],
    robot_yaw: float,
    max_range: float = RobotSpecs.CAMERA_FAR_CLIP,
    tuning: NavigationTuning | None = None,
    believed_pos: tuple[float, float] | None = None,
    believed_yaw: float | None = None,
) -> list[TrafficSignObservation]:
    """Return synthetic ``TrafficSignObservation``s for every in-frame sign.

    A sign is "seen" if it's within ``max_range`` and within
    ``RobotSpecs.CAMERA_HFOV`` of the robot's heading; otherwise it's omitted.

    Args:
        signs: Ground-truth sign specs (position + color) to project.
        robot_pos: Robot's TRUE (x, y) position -- what the camera actually
            sees from, used only to decide whether a sign is in frame and how
            far away it genuinely is.
        robot_yaw: Robot's TRUE heading (radians, 0 = east), for the same
            visibility check.
        max_range: Maximum detection range.
        tuning: Source for the fixed detection confidence (must stay above the
            sign router's ``min_confidence`` or no emulated detection would ever
            be accepted). Defaults to the checked-in tuning.
        believed_pos: Robot's BELIEVED (x, y) position -- what
            ``_detection_to_world`` on real hardware would reproject a
            detection's bearing/range through, since a real perception
            pipeline only ever has the robot's own pose estimate to place a
            detection in world coordinates, never ground truth. ``None``
            (the default) reprojects through ``robot_pos`` itself, i.e. no
            belief error -- correct when the caller has no separate estimate
            (ground-truth pose mode).
        believed_yaw: Robot's BELIEVED heading, paired with ``believed_pos``.

    Returns:
        Emulated observations, one per in-frame sign, reported in whatever
        frame ``believed_pos``/``believed_yaw`` describes -- NOT necessarily
        the sign's true position. Reporting true coordinates unconditionally
        used to leak ground truth straight past any localizer error: a
        SignRouter fed a discovered sign this way and a CoreNavigator
        steering on a diverged pose estimate were comparing two different
        reference frames, which is incoherent regardless of which frame is
        "right." A real camera has the same blind spot -- it measures a
        correct bearing/range but has only the robot's own (possibly wrong)
        pose to convert that into a world position -- so reproducing that
        error here, rather than omitting it, is what makes a diverged
        localizer's cost show up in sim instead of being invisible.
    """
    tuning = get_tuning(tuning)
    detection_confidence = tuning.simulation.DETECTION_CONFIDENCE
    report_pos = robot_pos if believed_pos is None else believed_pos
    report_yaw = robot_yaw if believed_yaw is None else believed_yaw
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

        # Reproject the TRUE relative bearing/range through the BELIEVED pose
        # -- the same relative geometry a real camera measured, placed in
        # world coordinates using the only pose the robot actually has.
        report_bearing = report_yaw + theta_h
        observations.append(
            TrafficSignObservation(
                world_x_m=report_pos[0] + distance * math.cos(report_bearing),
                world_y_m=report_pos[1] + distance * math.sin(report_bearing),
                color=SignColor.RED if sign.color == "red" else SignColor.GREEN,
                confidence=detection_confidence,
                detected_at_timestamp=0.0,
            ),
        )
    return observations
