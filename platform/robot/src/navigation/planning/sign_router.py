"""WRO 2026 traffic-sign routing for obstacles challenge.

Computes lateral waypoint deformations so the robot passes red signs on the
right and green signs on the left, per the official WRO pass-side rule.

Pure Python — no ROS2 dependencies. Designed to be unit-tested independently.

Pass-side rule (from robot's forward-travel perspective):
    - Red sign  → robot passes to the LEFT of the sign (sign on right).
    - Green sign → robot passes to the RIGHT of the sign (sign on left).

The deformation direction depends on BOTH the corridor section AND the travel
direction (CW/CCW): the same corridor is driven with opposite headings depending
on direction, which reverses left/right in world coordinates. The routing table
is therefore keyed by (Section, Direction); the CW rows are the world-frame
negation of the CCW rows.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

from shared.config.constants import RobotSpecs, TrafficSignSpecs
from shared.config.enums import Direction, Section


@dataclass(frozen=True)
class Detection:
    """A camera detection with class name, confidence, and bounding box."""

    class_name: str
    confidence: float
    bbox: tuple[float, float, float, float]


logger = logging.getLogger(__name__)

# Camera focal length in pixels — derived from HFOV and image width.
_CAMERA_FOCAL_PX: float = (RobotSpecs.CAMERA_WIDTH / 2) / math.tan(RobotSpecs.CAMERA_HFOV / 2)

# Per-(corridor, direction) routing table: (axis, red_mult, green_mult).
# axis: "y" means deform the y-coordinate; "x" deforms x.
# red_mult / green_mult: +1 or -1 multiplier applied to the LATERAL offset,
# chosen so the robot keeps a red sign on its right and a green sign on its
# left for the heading it actually drives in that corridor. The CLOCKWISE rows
# are the world-frame negation of the COUNTERCLOCKWISE rows.
_ROUTING_TABLE: dict[tuple[Section, Direction], tuple[str, int, int]] = {
    (Section.SOUTH, Direction.COUNTERCLOCKWISE): ("y", +1, -1),
    (Section.NORTH, Direction.COUNTERCLOCKWISE): ("y", -1, +1),
    (Section.EAST, Direction.COUNTERCLOCKWISE): ("x", -1, +1),
    (Section.WEST, Direction.COUNTERCLOCKWISE): ("x", +1, -1),
    (Section.SOUTH, Direction.CLOCKWISE): ("y", -1, +1),
    (Section.NORTH, Direction.CLOCKWISE): ("y", +1, -1),
    (Section.EAST, Direction.CLOCKWISE): ("x", +1, -1),
    (Section.WEST, Direction.CLOCKWISE): ("x", -1, +1),
}


@dataclass(frozen=True)
class SignSpec:
    """Expected traffic sign location and color from scenario metadata."""

    x: float
    y: float
    color: str  # ColorNames.RED or ColorNames.GREEN


@dataclass(frozen=True)
class SignRouterConfig:
    """Tuning parameters for the sign router."""

    lateral_offset: float = 0.15
    """Metres of lateral deformation perpendicular to the corridor."""

    activation_dist: float = 0.80
    """Deformation activates when robot is within this distance of a sign (m)."""

    passed_dist: float = 1.20
    """Sign is marked as passed once robot moves further than this from it (m)."""

    detection_match_dist: float = 0.30
    """Max world-frame distance to associate a camera detection with an expected sign (m)."""

    min_confidence: float = 0.25
    """Minimum detection confidence to accept a camera-based color update."""


class SignRouter:
    """Routes the robot past traffic signs using lateral waypoint deformations.

    Usage in navigation tick::

        deformed_wp = router.deform_waypoint(
            waypoint=target_wp,
            robot_pos=(robot_x, robot_y),
            robot_yaw=robot_yaw,
            corridor=current_corridor,
            detections=latest_detections,
        )

    Args:
        signs: Expected sign list from scenario metadata (position + color).
        config: Tuning parameters.
    """

    def __init__(
        self,
        signs: list[SignSpec],
        config: SignRouterConfig | None = None,
        direction: Direction = Direction.COUNTERCLOCKWISE,
    ) -> None:
        self._signs = signs
        self._config = config or SignRouterConfig()
        self._direction = direction
        self._passed: set[int] = set()

    @property
    def active_sign_count(self) -> int:
        """Number of signs not yet marked as passed."""
        return len(self._signs) - len(self._passed)

    def deform_waypoint(
        self,
        waypoint: tuple[float, float],
        robot_pos: tuple[float, float],
        robot_yaw: float,
        corridor: Section,
        detections: list[Detection] | None = None,
    ) -> tuple[float, float]:
        """Return a (possibly laterally deformed) version of the target waypoint.

        Checks all uncleared signs. The NEAREST active sign within activation
        distance drives the deformation. Camera detections are used to confirm
        the sign color if available and within match distance.

        Args:
            waypoint: Current target waypoint (x, y).
            robot_pos: Current robot position (x, y).
            robot_yaw: Robot heading (radians, 0 = east).
            corridor: Current track section.
            detections: Latest camera detections (may be empty or None).

        Returns:
            Deformed waypoint (x, y). Unchanged if no active sign nearby.
        """
        nearest_dist = float("inf")
        nearest_idx = -1

        for i, sign in enumerate(self._signs):
            if i in self._passed:
                continue
            d = _dist2d(robot_pos, (sign.x, sign.y))
            if d > self._config.passed_dist:
                self._passed.add(i)
                logger.debug("Sign %d marked as passed (dist=%.2f m)", i, d)
                continue
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        if nearest_idx < 0 or nearest_dist > self._config.activation_dist:
            return waypoint

        sign = self._signs[nearest_idx]
        color = sign.color

        # Optionally override color with camera detection.
        if detections:
            camera_color = _match_detection_to_sign(
                detections,
                (sign.x, sign.y),
                robot_pos,
                robot_yaw,
                self._config,
            )
            if camera_color is not None:
                color = camera_color

        deformed = _apply_deformation(
            waypoint, sign, color, corridor, self._direction, self._config.lateral_offset,
        )

        if deformed != waypoint:
            logger.debug(
                "Sign %d (%s) deformation: wp (%.3f,%.3f) → (%.3f,%.3f) [dist=%.2f m]",
                nearest_idx,
                color,
                waypoint[0],
                waypoint[1],
                deformed[0],
                deformed[1],
                nearest_dist,
            )

        return deformed


def _apply_deformation(
    waypoint: tuple[float, float],
    sign: SignSpec,
    color: str,
    corridor: Section,
    direction: Direction,
    lateral_offset: float,
) -> tuple[float, float]:
    """Compute the laterally deformed waypoint for a given sign and corridor.

    Args:
        waypoint: Original target waypoint (x, y).
        sign: Traffic sign spec (position + color).
        color: Effective sign color (may be camera-confirmed).
        corridor: Current track section.
        direction: Travel direction (CW/CCW) — selects the pass-side mapping.
        lateral_offset: Lateral deformation magnitude (m).

    Returns:
        Deformed waypoint (x, y).
    """
    if (corridor, direction) not in _ROUTING_TABLE:
        return waypoint

    axis, red_mult, green_mult = _ROUTING_TABLE[(corridor, direction)]
    mult = red_mult if color == "red" else green_mult

    wx, wy = waypoint
    if axis == "y":
        return wx, sign.y + mult * lateral_offset
    return sign.x + mult * lateral_offset, wy


def _match_detection_to_sign(
    detections: list[Detection],
    expected_world_pos: tuple[float, float],
    robot_pos: tuple[float, float],
    robot_yaw: float,
    config: SignRouterConfig,
) -> str | None:
    """Try to confirm sign color using camera detection.

    Projects each detection from image space to approximate world coordinates
    using camera intrinsics and robot pose, then matches against the expected
    sign world position.

    Args:
        detections: Current frame detections.
        expected_world_pos: Expected (x, y) world position of the sign.
        robot_pos: Robot (x, y) position.
        robot_yaw: Robot heading (radians).
        config: Router config (confidence threshold, match distance).

    Returns:
        Confirmed color string ("red"/"green"), or None if no confident match.
    """
    best_match_dist = float("inf")
    best_color: str | None = None

    for det in detections:
        if det.confidence < config.min_confidence:
            continue
        if det.class_name not in ("red", "green"):
            continue

        world_pos = _detection_to_world(det, robot_pos, robot_yaw)
        if world_pos is None:
            continue

        d = _dist2d(world_pos, expected_world_pos)
        if d < config.detection_match_dist and d < best_match_dist:
            best_match_dist = d
            best_color = det.class_name

    return best_color


def _detection_to_world(
    det: Detection,
    robot_pos: tuple[float, float],
    robot_yaw: float,
) -> tuple[float, float] | None:
    """Project a bbox detection to an approximate world position.

    Uses known sign height (TrafficSignSpecs.HEIGHT) as the reference to
    estimate distance from the pixel-space bounding-box height.

    Args:
        det: Single camera detection with bbox (x1, y1, x2, y2).
        robot_pos: Robot (x, y) position (metres).
        robot_yaw: Robot heading (radians, 0 = east).

    Returns:
        Estimated world (x, y) of the sign, or None if bbox is too small.
    """
    x1, y1, x2, y2 = det.bbox
    pixel_height = abs(y2 - y1)
    if pixel_height < 5:  # degenerate bbox
        return None

    # Estimate distance using pinhole model: d = (f * real_h) / pixel_h
    distance = (_CAMERA_FOCAL_PX * TrafficSignSpecs.HEIGHT) / pixel_height

    # Horizontal angle from image centre.
    cx = (x1 + x2) / 2.0
    theta_h = (cx / RobotSpecs.CAMERA_WIDTH - 0.5) * RobotSpecs.CAMERA_HFOV

    bearing = robot_yaw + theta_h
    wx = robot_pos[0] + distance * math.cos(bearing)
    wy = robot_pos[1] + distance * math.sin(bearing)
    return wx, wy


def signs_from_metadata(metadata: dict) -> list[SignSpec]:
    """Extract sign specs from scenario metadata.

    Args:
        metadata: Scenario metadata dict (from ScenarioGenerator).

    Returns:
        List of SignSpec for all signs in the scenario.
    """
    sign_positions = metadata.get("sign_positions", [])
    return [SignSpec(x=entry["x"], y=entry["y"], color=entry["color"]) for entry in sign_positions]


def _dist2d(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
