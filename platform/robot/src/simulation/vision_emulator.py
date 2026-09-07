"""Synthetic camera-detection emulator for closed-loop sign-routing tests.

Inverts the exact pinhole projection ``sign_router._detection_to_world()``
decodes, so a ``SignRouter`` driven through this emulator exercises the real
camera-confirmation code path (``match_detection_to_sign`` /
``_detection_to_world``) instead of always seeing ``detections=None``.

Deliberately simple — a fixed high confidence, no false positives, no wall
occlusion modelling (the LIDAR raycast model in ``track_model.py`` does model
occlusion; this does not). The goal is exercising the detection-to-router
wiring end-to-end, not building a camera sensor model.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from shared.config.constants import RobotSpecs, TrafficSignSpecs
from shared.domain.models import Detection, SignColor, TrafficSignObservation, Waypoint

from src.config.tuning_helpers import get_tuning
from src.navigation.utils import wrap_angle as _wrap_angle

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning

    from src.navigation.planning.sign_discovery import SignSpec


def emulate_sign_observations(
    signs: list[SignSpec],
    robot_pos: Waypoint,
    robot_yaw: float,
    max_range: float = RobotSpecs.CAMERA_FAR_CLIP,
    tuning: NavigationTuning | None = None,
    believed_pos: Waypoint | None = None,
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
        sign_pos = Waypoint(sign.x, sign.y)
        distance = robot_pos.distance_to(sign_pos)
        if distance <= 0.0 or distance > max_range:
            continue

        bearing = robot_pos.bearing_to(sign_pos)
        theta_h = _wrap_angle(bearing - robot_yaw)
        if abs(theta_h) > RobotSpecs.CAMERA_HFOV / 2:
            continue

        # Reproject the TRUE relative bearing/range through the BELIEVED pose
        # -- the same relative geometry a real camera measured, placed in
        # world coordinates using the only pose the robot actually has.
        report_bearing = report_yaw + theta_h
        observations.append(
            TrafficSignObservation(
                world_x_m=report_pos.x + distance * math.cos(report_bearing),
                world_y_m=report_pos.y + distance * math.sin(report_bearing),
                color=SignColor.RED if sign.color == "red" else SignColor.GREEN,
                confidence=detection_confidence,
                detected_at_timestamp=0.0,
            ),
        )
    return observations


def emulate_sign_detections(
    signs: list[SignSpec],
    robot_pos: Waypoint,
    robot_yaw: float,
    max_range: float = RobotSpecs.CAMERA_FAR_CLIP,
    tuning: NavigationTuning | None = None,
) -> list[Detection]:
    """Synthetic BOUNDING BOXES, so the sim runs the real perception maths.

    ``emulate_sign_observations`` hands back world coordinates built from the
    TRUE range and bearing, which means the simulator never executes
    ``_detection_to_world`` at all -- no pinhole, no bearing formula, no aspect
    gate, no frame-clipping test. Every one of those is live on the robot, and
    the corpus was structurally unable to see any of them.

    That is not hypothetical. The camera's bearing formula was MIRRORED --
    positive for a box on the RIGHT of the image against a robot frame where
    left is positive -- so every sign was reflected across the heading axis onto
    the far wall of a 1 m corridor. It survived in-tree because this emulator
    reproduced the true geometry directly and the router's unit tests built
    their bounding boxes by INVERTING the same formula. Both agreed with the
    error. Only a hardware bag disagreed.

    So this inverts the projection to a BOX and stops there, leaving the decode
    to the shipped code. Anything wrong in that decode now shows up in the
    corpus instead of waiting for a race.

    Still deliberately optimistic about everything else: no false positives, no
    wall confusion, no occlusion, no dropout, fixed high confidence. Those are
    separate fidelity gaps and each wants its own measurement.
    """
    tuning = get_tuning(tuning)
    confidence = tuning.simulation.DETECTION_CONFIDENCE
    focal_px = (RobotSpecs.CAMERA_WIDTH / 2) / math.tan(RobotSpecs.CAMERA_HFOV / 2)
    # Measured FROM THE SENSOR, which is 0.1222 m forward of the chassis centre.
    # `_detection_to_world` projects its ray from there, so a range taken at the
    # centre comes back 12.2 cm long -- verified by round-tripping a sign at
    # 1.000 m and getting 1.122 m. (`emulate_sign_observations` above still
    # measures from the centre; it also REPORTS from the centre, so it is
    # self-consistent, but it does disagree with the shipped decoder by that
    # same offset.)
    sensor_pos = Waypoint(
        robot_pos.x + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.cos(robot_yaw),
        robot_pos.y + RobotSpecs.LIDAR_MOUNT_X_OFFSET * math.sin(robot_yaw),
    )
    detections: list[Detection] = []
    for sign in signs:
        sign_pos = Waypoint(sign.x, sign.y)
        distance = sensor_pos.distance_to(sign_pos)
        if distance <= 0.0 or distance > max_range:
            continue
        theta_h = _wrap_angle(sensor_pos.bearing_to(sign_pos) - robot_yaw)
        if abs(theta_h) > RobotSpecs.CAMERA_HFOV / 2:
            continue

        # The inverse of `_detection_to_world`: bearing -> centre column, range
        # -> box height. Written against the shipped formulae rather than
        # restating them, so the two move together -- but NOT shared with the
        # decode, because a decoder tested only against its own inverse is what
        # let the mirrored bearing through.
        pixel_height = focal_px * TrafficSignSpecs.HEIGHT / distance
        cx = (0.5 - theta_h / RobotSpecs.CAMERA_HFOV) * RobotSpecs.CAMERA_WIDTH
        # A pillar is 0.05 m wide and 0.10 m tall, so the box is half as wide as
        # it is high -- which also keeps it the right side of MAX_PILLAR_ASPECT.
        pixel_width = pixel_height * TrafficSignSpecs.WIDTH / TrafficSignSpecs.HEIGHT
        cy = RobotSpecs.CAMERA_HEIGHT / 2.0
        x_min = cx - pixel_width / 2.0
        x_max = cx + pixel_width / 2.0
        y_min = cy - pixel_height / 2.0
        y_max = cy + pixel_height / 2.0
        detections.append(
            Detection(
                class_name=SignColor.RED if sign.color == "red" else SignColor.GREEN,
                confidence=confidence,
                bbox=(x_min, y_min, x_max, y_max),
                x=cx,
                y=cy,
                width=pixel_width,
                height=pixel_height,
                area=pixel_width * pixel_height,
            ),
        )
    return detections
