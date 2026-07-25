"""WRO 2026 traffic-sign routing for obstacles challenge.

Computes lateral waypoint deformations so the robot avoids a red obstacle on
its OUTWARD side (toward the outer wall) and a green obstacle on its INWARD
side (toward the inner square) — an absolute rule tied to the track geometry,
not the travel direction: it holds identically whether the round is run
clockwise or counterclockwise.

Pure Python — no ROS2 dependencies. Designed to be unit-tested independently.

Pass-side rule:
    - Red obstacle   → robot passes on the OUTWARD side (away from centre).
    - Green obstacle → robot passes on the INWARD side (toward centre).

Pinned by ``TestPassSideRule`` in ``tests/unit/test_sign_router.py``: for
every (section, direction) the deformed waypoint moves outward for red and
inward for green.

The deformation is still keyed by (Section, Direction) because the AXIS and
WORLD-FRAME SIGN of "outward" both depend on which corridor is being driven,
but — unlike an earlier version of this table — the CLOCKWISE and
COUNTERCLOCKWISE rows for a given section are now IDENTICAL, not negations of
each other: outward/inward is a fixed property of the corridor, independent
of which way the robot is circling it.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from shared.config.constants import ColorNames, RobotSpecs, TrackDimensions, TrafficSignSpecs
from shared.config.enums import Direction, Section

from src.navigation.planning.waypoints import corridor_for_position

if TYPE_CHECKING:
    from shared.domain.models import Detection

logger = logging.getLogger(__name__)

# Camera focal length in pixels — derived from HFOV and image width.
_CAMERA_FOCAL_PX: float = (RobotSpecs.CAMERA_WIDTH / 2) / math.tan(RobotSpecs.CAMERA_HFOV / 2)

# Bounding boxes shorter than this (px) are too degenerate for a reliable
# pinhole distance estimate.
_MIN_RELIABLE_BBOX_HEIGHT_PX: int = 5

# Chassis half-width plus a small margin: how far a deformed waypoint must
# stay clear of the restricted inner square and the outer wall (WP-1). An
# unclamped deformation can otherwise place the waypoint inside the inner
# square or against a wall for a sign positioned near a corridor edge.
_WALL_CLEARANCE = RobotSpecs.WIDTH / 2 + 0.02

# Default lateral deformation magnitude, derived the same way as
# _WALL_CLEARANCE above: chassis half-width + the sign's own half-width (the
# offset is applied from the sign's CENTER, so its footprint eats into the
# gap too) + a safety margin. A flat 0.15m default here previously left only
# ~2.5cm of actual edge-to-edge clearance once those two half-widths were
# subtracted — the robot visibly grazed signs in RViz even though it wasn't
# technically colliding.
_SIGN_CLEARANCE_MARGIN = 0.075
_SIGN_LATERAL_OFFSET = RobotSpecs.WIDTH / 2 + TrafficSignSpecs.WIDTH / 2 + _SIGN_CLEARANCE_MARGIN

# How far past the inner square's own span [CORNER_MIN, CORNER_MAX] the depth
# axis may drift and still count as a valid straight-corridor deformation
# candidate — see _is_squarely_in_corridor.
_DEFORM_DEPTH_BUFFER = 0.3

# Per-(corridor, direction) routing table: (axis, red_mult, green_mult).
# axis: "y" means deform the y-coordinate; "x" deforms x.
# red_mult / green_mult: +1 or -1 multiplier applied to the LATERAL offset,
# chosen so red always moves the deformed waypoint OUTWARD (away from the
# inner square) and green always moves it INWARD — identically for CW and
# CCW, since outward/inward is a fixed property of the corridor, not the
# travel direction. (An earlier version of this table made the CW rows the
# world-frame negation of the CCW rows, which instead pinned "red on the
# robot's right" — a travel-RELATIVE rule that flips outward/inward between
# CW and CCW. That was wrong: the official rule is the absolute one above.)
_ROUTING_TABLE: dict[tuple[Section, Direction], tuple[str, int, int]] = {
    (Section.SOUTH, Direction.COUNTERCLOCKWISE): ("y", -1, +1),
    (Section.NORTH, Direction.COUNTERCLOCKWISE): ("y", +1, -1),
    (Section.EAST, Direction.COUNTERCLOCKWISE): ("x", +1, -1),
    (Section.WEST, Direction.COUNTERCLOCKWISE): ("x", -1, +1),
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

    lateral_offset: float = _SIGN_LATERAL_OFFSET
    """Metres of lateral deformation perpendicular to the corridor."""

    activation_dist: float = 0.80
    """Deformation activates when robot is within this distance of a sign (m)."""

    passed_dist: float = 1.20
    """Sign is marked as passed once robot moves further than this from it (m)."""

    detection_match_dist: float = 0.30
    """Max world-frame distance to associate a camera detection with an expected sign (m)."""

    min_confidence: float = 0.25
    """Minimum detection confidence to accept a camera-based color update."""

    settle_ticks: int = 150
    """Ticks since this lap started (~7.5s at the standard 20Hz control loop)
    before a sign may be engaged/passed at all. Right after spawn (or a lap
    boundary), the robot can briefly swing toward a corridor it hasn't
    actually reached yet while settling onto its planned route — if that
    swing grazes a not-yet-really-encountered sign's activation radius, it
    gets engaged and then marked passed as the robot continues on its real
    route away from it, retiring the sign before its genuine pass ever
    happens. Deferring bookkeeping (not candidate selection — a sign already
    in the robot's actual corridor still deforms normally) for this settle
    window prevents that incidental graze from ever registering."""


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
        self._engaged: set[int] = set()
        self._lap_tick = 0
        # Each sign's own corridor, precomputed once — deform_waypoint() must
        # never apply a sign's (x, y) through a different corridor's axis
        # convention (see _nearest_active_sign).
        self._sign_corridors = [corridor_for_position(s.x, s.y) for s in signs]

    @property
    def active_sign_count(self) -> int:
        """Number of signs not yet marked as passed (this lap)."""
        return len(self._signs) - len(self._passed)

    def reset_for_new_lap(self) -> None:
        """Re-arm every sign so it's routed again on the next lap.

        Without this, a sign marked ``_passed`` on lap 1 (once the robot moves
        beyond ``passed_dist``) stays passed for the rest of the run — the
        Obstacles Challenge requires clearing every sign on all 3 laps, not
        just the first time each one is encountered.
        """
        self._passed.clear()
        self._engaged.clear()
        self._lap_tick = 0

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
        nearest_idx, nearest_dist = self._nearest_active_sign(robot_pos, corridor)
        if nearest_idx < 0 or nearest_dist > self._config.activation_dist:
            return waypoint

        # Key the corner check and the deformation math off the CANDIDATE
        # SIGN's own corridor, not the robot's current corridor label. The two
        # can legitimately disagree right at a corner — see
        # _nearest_active_sign's same-corridor-OR-within-activation_dist
        # comment — and the sign's own corridor is what actually determines
        # which world axis is "lateral" for it; using the robot's (possibly
        # stale, pre-corner) label here would deform the wrong axis.
        sign_corridor = self._sign_corridors[nearest_idx]

        # The deformation model assumes a straight corridor segment (hold the
        # depth axis, override the lateral axis with a value derived from the
        # sign's fixed position). Once the *target* waypoint itself has curved
        # into a corner, that override is stale and increasingly wrong — skip
        # it rather than fight the path's own curve. Deliberately stricter than
        # corridor_for_position()'s corner tie-break (which exists to always
        # assign the ROBOT some corridor, even ambiguously): a corner waypoint
        # like (2.42, 2.42) ties NORTH vs EAST there and gets assigned NORTH by
        # insertion order, but it's still on the turning arc, not the straight
        # segment this deformation model assumes.
        if not _is_squarely_in_corridor(waypoint[0], waypoint[1], sign_corridor):
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

        # Taper the offset by *this waypoint's* own distance to the sign (not
        # the robot's — that only gates whether the sign is engaged at all).
        # A single target-point substitution never needed this: it was always
        # the point nearest the robot's own engagement. But biasing a whole
        # forward window the same fixed amount regardless of how far each
        # point already is from the sign snaps the offset from full magnitude
        # to zero in a single waypoint step at the corridor boundary — a kink
        # arriving at exactly the same place the car is also turning through.
        # Fading it out over `passed_dist` keeps every existing single-point
        # call (waypoint == sign position, taper == 1.0) byte-identical.
        waypoint_dist = _dist2d(waypoint, (sign.x, sign.y))
        taper = max(0.0, 1.0 - waypoint_dist / self._config.passed_dist)
        effective_offset = self._config.lateral_offset * taper

        deformed = _apply_deformation(
            waypoint,
            sign,
            color,
            sign_corridor,
            self._direction,
            effective_offset,
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

    def _nearest_active_sign(
        self,
        robot_pos: tuple[float, float],
        corridor: Section,
    ) -> tuple[int, float]:
        """Index and distance of the nearest not-yet-passed sign near ``corridor``.

        Also maintains engagement/passed bookkeeping: a sign is engaged once the
        robot comes within activation distance, and retired only after it has
        been engaged and then left beyond ``passed_dist`` — never discarded from
        afar (which would silently disable routing at spawn). Bookkeeping runs
        for every sign regardless of corridor.

        The returned *candidate* is restricted to signs that either belong to
        ``corridor`` or are within ``activation_dist`` of the robot — not
        strict same-corridor equality. A sign one corridor over can sit right
        at a corner (e.g. at that corridor's own "near" grid depth, exactly on
        CORNER_MIN/MAX); the robot's corridor label only flips once its
        cornering arc has already carried it past that point, which is too
        late for any deformation to matter. Requiring same-corridor OR
        within-activation_dist lets a genuinely close cross-corridor sign start
        bending the path before the label flips, while still keeping distant
        cross-corridor signs from being engaged prematurely. The caller
        (``deform_waypoint``) uses the CANDIDATE's own corridor — not this
        method's ``corridor`` argument — for the actual axis/clamp math, so a
        cross-corridor candidate is never run through the wrong convention.
        Engage/pass bookkeeping itself is suppressed for the first
        ``settle_ticks`` of a lap (see ``SignRouterConfig.settle_ticks``);
        candidate selection isn't, so a sign genuinely in the robot's current
        corridor still deforms normally even during that window.

        Returns:
            ``(index, distance)``; index is -1 when no active sign remains.
        """
        self._lap_tick += 1
        settled = self._lap_tick > self._config.settle_ticks
        nearest_dist = float("inf")
        nearest_idx = -1

        for i, sign in enumerate(self._signs):
            if i in self._passed:
                continue
            d = _dist2d(robot_pos, (sign.x, sign.y))
            if settled and d < self._config.activation_dist:
                self._engaged.add(i)
            if d > self._config.passed_dist:
                if settled and i in self._engaged:
                    self._passed.add(i)
                    logger.debug("Sign %d marked as passed (dist=%.2f m)", i, d)
                continue
            same_corridor = self._sign_corridors[i] == corridor
            # A sign in a DIFFERENT corridor than the robot's current label only
            # qualifies once the robot is within activation_dist of it — i.e.
            # close enough that the sign's own geometry is what actually
            # matters, not the robot's corridor bookkeeping. Without this, a
            # sign sitting right at a corner (e.g. at the corridor's own "near"
            # depth, exactly on CORNER_MIN/MAX) never becomes a deformation
            # candidate until the robot's corridor label flips — which happens
            # only once the robot's cornering arc has already carried it
            # straight past the sign, too late for any deformation to matter.
            # Requiring same-corridor OR within-activation_dist keeps distant
            # cross-corridor signs from being engaged prematurely while still
            # letting a genuinely close one start bending the path early.
            if not same_corridor and d > self._config.activation_dist:
                continue
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        return nearest_idx, nearest_dist


def _apply_deformation(
    waypoint: tuple[float, float],
    sign: SignSpec,
    color: str,
    corridor: Section,
    direction: Direction,
    lateral_offset: float,
) -> tuple[float, float]:
    """Compute the laterally deformed waypoint for a given sign and corridor.

    The result is clamped so it can't land inside the restricted inner square
    or beyond the outer wall (WP-1) — a sign positioned near a corridor edge
    would otherwise deform the waypoint straight into a hazard.

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
    mult = red_mult if color == ColorNames.RED else green_mult

    wx, wy = waypoint
    if axis == "y":
        return wx, _clamp_lateral(sign.y + mult * lateral_offset, corridor)
    return _clamp_lateral(sign.x + mult * lateral_offset, corridor), wy


def _clamp_lateral(value: float, corridor: Section) -> float:
    """Clamp a deformed lateral coordinate clear of the inner square and outer wall.

    SOUTH/WEST corridors border the inner square on their high side (the
    coordinate must stay below ``CORNER_MIN``); NORTH/EAST border it on their
    low side (must stay above ``CORNER_MAX``). Every corridor is also bounded
    on its outer side by the track wall.
    """
    low_side = corridor in (Section.SOUTH, Section.WEST)
    if low_side:
        value = min(value, TrackDimensions.CORNER_MIN - _WALL_CLEARANCE)
        value = max(value, TrackDimensions.MIN_COORD + _WALL_CLEARANCE)
    else:
        value = max(value, TrackDimensions.CORNER_MAX + _WALL_CLEARANCE)
        value = min(value, TrackDimensions.MAX_COORD - _WALL_CLEARANCE)
    return value


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
        if det.class_name not in (ColorNames.RED, ColorNames.GREEN):
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
    if pixel_height < _MIN_RELIABLE_BBOX_HEIGHT_PX:
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


def _is_squarely_in_corridor(x: float, y: float, corridor: Section) -> bool:
    """True if this waypoint is still a reasonable candidate for straight-corridor deformation.

    The deformation model holds the depth axis (whatever value the raw path
    already gives it) and overrides only the lateral axis with a value derived
    from the sign's position, then clamps that result into the corridor's own
    free-space band. Two independent checks:

    * Lateral axis (the one being overridden) must still read as this
      corridor, not already the opposite wall.
    * Depth axis (held, never touched) must stay within
      ``_DEFORM_DEPTH_BUFFER`` of the inner square's own span — not the exact
      ``[CORNER_MIN, CORNER_MAX]`` window ``corridor_for_position()`` uses for
      its own robot-position classification, which is far too strict here: the
      lookahead target runs 0.2-0.4m ahead of the robot, so it's often already
      past that window well before the robot itself is anywhere near a corner,
      and requiring it anyway silently killed deformation through most of a
      sign's real engagement. But with no depth check at all, deformation can
      keep firing long after the robot has geometrically left this corridor
      for the next one, building up an offset that snaps back hard once the
      sign finally disengages by corridor mismatch — this buffer catches that
      case without reintroducing the original over-strict cutoff.
    """
    depth_min = TrackDimensions.CORNER_MIN - _DEFORM_DEPTH_BUFFER
    depth_max = TrackDimensions.CORNER_MAX + _DEFORM_DEPTH_BUFFER
    if corridor is Section.SOUTH:
        return y < TrackDimensions.CORNER_MIN and depth_min <= x <= depth_max
    if corridor is Section.NORTH:
        return y > TrackDimensions.CORNER_MAX and depth_min <= x <= depth_max
    if corridor is Section.EAST:
        return x > TrackDimensions.CORNER_MAX and depth_min <= y <= depth_max
    if corridor is Section.WEST:
        return x < TrackDimensions.CORNER_MIN and depth_min <= y <= depth_max
    return False
