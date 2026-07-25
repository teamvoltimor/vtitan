"""Parallel-park controller for WRO 2026 obstacles challenge.

Drives the robot into the gap between the two magenta parking blocks after
completing 3 laps.  Uses a two-phase pure-pursuit strategy:

  Phase 1 — STAGE : drive to a staging position directly in front of the gap
                    opening (perpendicular approach from the track side).
  Phase 2 — ENTER : drive straight into the gap, correcting heading toward
                    the gap centre.

Stop condition: chassis centre inside the parking-zone bounding box AND
yaw within ±10° (0.1745 rad) of the expected parking angle.

Pure Python — no ROS2 dependencies. Unit testable.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from enum import StrEnum

from shared.config.constants import ParkingLotSpecs, RobotSpecs
from shared.config.enums import Section
from shared.config.navigation_tuning import NavigationTuning

logger = logging.getLogger(__name__)

_YAW_TOLERANCE = math.radians(10.0)  # ±10° stop condition
# Staging stand-off, perpendicular to the gap opening. Reuses ARC_RADIUS (not a fresh
# literal) because both share the same real constraint: must clear the chassis's Ackermann
# minimum turning radius. At the old 0.25m this was smaller than R_min, and for N/S
# sections nearly collinear with the corridor cruise line -- a robot cruising through could
# overshoot the staging point in x before y converged, flipping the bearing ~180° in a
# single tick and forcing `_pursuit_steer` into a non-convergent orbit. See
# docs/internal/2026-07-11-navigation-logic-review.md §2.3 for the full trace.
_APPROACH_CLEARANCE = NavigationTuning().waypoints.ARC_RADIUS
_POS_REACH_DIST = 0.04  # metres: "reached staging" threshold
_INSIDE_TOLERANCE = 0.02  # metres: zone wall clearance
_DEFAULT_MAX_FRAMES = 400  # 20s at 20Hz: bound on the parking maneuver's duration

_SATURATED_STEER_THRESHOLD = 0.999
"""abs(steering) at/above this counts as "at physical lock" for _SATURATION_STUCK_TICKS."""

_SATURATION_STUCK_TICKS = 20  # 1s at 20Hz
"""Consecutive ticks of saturated steering before assuming the target requires a tighter
turn than the chassis can make going forward -- a bearing-angle threshold alone isn't a
reliable signal (a curvature-correct pure-pursuit controller saturates at exactly the same
physical limit as a naive one once the *true* required curvature exceeds R_min, regardless
of how "large" the bearing error looks); sustained saturation is what a genuine
non-convergent orbit actually looks like in practice -- see _APPROACH_CLEARANCE's
module-level comment for the full failure trace this fixes."""

# Reuses the same reverse-maneuver tuning CoreNavigator's escape maneuvers already use
# (K-turn/stuck-reverse) rather than inventing fresh constants -- this is functionally the
# same kind of "back away and reorient" recovery, just triggered by parking geometry instead
# of a LIDAR collision risk. A single-tick reaction (re-evaluated every frame) isn't enough:
# it flickers in and out before actually creating separation -- latched for a fixed duration,
# like the CoreNavigator escape maneuvers, so it commits to a real repositioning motion.
_escape_tuning = NavigationTuning().escape
_REPOSITION_SPEED = _escape_tuning.REV_SPEED
_REPOSITION_STEER_MAG = _escape_tuning.REV_STEERING_SCALE
_REPOSITION_FRAMES = _escape_tuning.MAX_ESCAPE_FRAMES


@dataclass(frozen=True)
class ParkZone:
    """Bounding box and parking angle for the zone."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    target_yaw: float  # expected robot yaw when parked (radians)
    gap_cx: float  # lateral centre of gap (world x or y)
    gap_cy: float  # depth centre of gap


@dataclass
class ParkCommand:
    """Motor command from the park controller."""

    linear: float  # m/s
    steering: float  # normalised [-1, 1]
    done: bool = False
    phase: str = "stage"


class ParkPhase(StrEnum):
    """Parking maneuver phases."""

    STAGE = "stage"
    ENTER = "enter"
    DONE = "done"


class ParkController:
    """Two-phase park controller.

    Phase 1 — STAGE: pure-pursuit toward the staging position directly in
        front of the gap opening (on the track side).
    Phase 2 — ENTER: pure-pursuit toward the gap centre; switches to DONE
        when both position and yaw are within tolerance.

    Args:
        parking_config: Dict with 'block1_pos' and 'block2_pos' keys.
        start_section: Corridor that contains the parking lot.
        speed: Constant driving speed (m/s).
        max_frames: Hard bound on how many control ticks the maneuver may run
            before giving up and holding position. Without this, a robot that
            can never satisfy the position+yaw stop condition (e.g. wedged
            against a block) would chase the gap centre for the rest of the
            match instead of coming to a controlled stop.
    """

    def __init__(
        self,
        parking_config: dict,
        start_section: Section,
        speed: float = 0.12,
        max_frames: int = _DEFAULT_MAX_FRAMES,
    ) -> None:
        self._section = start_section
        self._speed = speed
        self._phase = ParkPhase.STAGE
        self._max_frames = max_frames
        self._frames_elapsed = 0
        self._timed_out = False
        self._reposition_frames_left = 0
        self._reposition_speed = 0.0
        self._reposition_steer = 0.0
        self._saturated_ticks = 0

        b1 = parking_config["block1_pos"]
        b2 = parking_config["block2_pos"]
        self._zone = _build_zone(b1, b2, start_section)
        self._staging = _staging_pos(self._zone, start_section)

        logger.info(
            "ParkController: zone=%s staging=%s section=%s",
            self._zone,
            self._staging,
            start_section,
        )

    @property
    def is_repositioning(self) -> bool:
        """Whether STAGE is mid reverse-and-reorient recovery (see _handle_stage).

        CoreNavigator's generic stuck-detector escape is blind to parking geometry (the
        inner keep-out block, the staging point) and can fight this maneuver -- e.g. its
        own low-net-displacement check can misfire during a deliberate reverse burst that
        genuinely IS making progress (rotating toward a reachable heading) but doesn't move
        far in a straight line. CoreNavigator should not override this with a generic
        escape while it's active.
        """
        return self._reposition_frames_left > 0

    @property
    def is_done(self) -> bool:
        """Whether the parking maneuver is complete (cleanly or via timeout)."""
        return self._phase is ParkPhase.DONE

    @property
    def is_timed_out(self) -> bool:
        """Whether the maneuver gave up on its frame budget rather than parking cleanly."""
        return self._timed_out

    @property
    def section(self) -> Section:
        """Corridor that contains the parking lot."""
        return self._section

    @property
    def staging(self) -> tuple[float, float]:
        """Staging position in front of the gap opening (world x, y)."""
        return self._staging

    def update(
        self,
        robot_pos: tuple[float, float],
        robot_yaw: float,
    ) -> ParkCommand:
        """Compute next motor command.

        Args:
            robot_pos: Current (x, y) world position of robot centre.
            robot_yaw: Current heading (radians, 0=east, π/2=north).

        Returns:
            ParkCommand with speed, normalised steering, and done flag.
        """
        if self._phase is ParkPhase.DONE:
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase="done")

        self._frames_elapsed += 1
        if self._frames_elapsed > self._max_frames:
            logger.warning(
                "ParkController: giving up after %d frames without parking cleanly",
                self._frames_elapsed,
            )
            self._timed_out = True
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase="done")

        # Early exit: if already inside the zone at any phase, we're done.
        pos_inside, yaw_ok = _inside_zone(robot_pos[0], robot_pos[1], robot_yaw, self._zone)
        if pos_inside and yaw_ok:
            logger.info("ParkController: DONE — already inside zone")
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase="done")

        if self._phase is ParkPhase.STAGE:
            return self._handle_stage(robot_pos, robot_yaw)
        return self._handle_enter(robot_pos, robot_yaw)

    # Phase handlers

    def _pursue_with_reposition(
        self,
        robot_pos: tuple[float, float],
        robot_yaw: float,
        target: tuple[float, float],
        phase_name: str,
    ) -> ParkCommand:
        """Curvature-based pure pursuit of ``target``, with reverse-and-reorient recovery.

        Shared by both STAGE (toward the staging point) and ENTER (toward the gap centre)
        — both are the same underlying problem (drive toward a fixed target point), and
        both can hit the same degenerate case: a target that requires a sharper turn than
        the chassis's minimum turning radius allows on a forward arc. Real pure pursuit
        (curvature from lookahead geometry, clamped only at the physical steering limit)
        replaces what used to be a plain bearing-error * kp controller — at this
        controller's old kp=2.5, *any* bearing error past ~23° already saturated to full
        lock, meaning it was never actually proportional in practice, just bang-bang. See
        _APPROACH_CLEARANCE's module-level comment for the full failure trace this fixes.
        """
        # An in-progress reposition runs for its full latched duration rather than being
        # re-evaluated (and potentially cancelled) every tick -- a single-tick reaction
        # flickers in and out without ever creating enough separation to actually escape
        # the degenerate geometry.
        if self._reposition_frames_left > 0:
            self._reposition_frames_left -= 1
            return ParkCommand(linear=self._reposition_speed, steering=self._reposition_steer, phase=phase_name)

        x_local, y_local = _local_frame(robot_pos, robot_yaw, target)

        if x_local < 0:
            # Target is behind the robot: pure pursuit's curvature formula is only valid
            # for a roughly-forward target -- for a rearward one it can produce a
            # plausible-looking (non-saturated) steering command that actually drives away
            # from the target instead of toward it. Reverse immediately rather than trust it.
            return self._start_reposition(robot_pos, robot_yaw, target, phase_name, "target behind")

        steer = _pure_pursuit_steer(x_local, y_local)

        if abs(steer) >= _SATURATED_STEER_THRESHOLD:
            self._saturated_ticks += 1
        else:
            self._saturated_ticks = 0

        if self._saturated_ticks >= _SATURATION_STUCK_TICKS:
            # Steering has been pinned at physical lock for a full second straight: the
            # required curvature genuinely exceeds what the chassis can do going forward
            # (see the module-level comment). Reverse to open room instead of continuing to
            # orbit.
            reason = f"{self._saturated_ticks} saturated ticks"
            return self._start_reposition(robot_pos, robot_yaw, target, phase_name, reason)

        return ParkCommand(linear=self._speed, steering=steer, phase=phase_name)

    def _start_reposition(
        self,
        robot_pos: tuple[float, float],
        robot_yaw: float,
        target: tuple[float, float],
        phase_name: str,
        reason: str,
    ) -> ParkCommand:
        """Latch a reverse-and-reorient recovery burst. See _pursue_with_reposition."""
        bearing_err = _bearing_error(robot_pos, robot_yaw, target)
        logger.debug(
            "ParkController: %s reposition (%s, bearing_err=%.1f deg)",
            phase_name,
            reason,
            math.degrees(bearing_err),
        )
        self._saturated_ticks = 0
        self._reposition_frames_left = _REPOSITION_FRAMES
        self._reposition_speed = _REPOSITION_SPEED
        # Sign-flipped for reverse Ackermann geometry (v<0 inverts the yaw-rate response
        # to a given steer sign), biased toward whichever side the target currently bears.
        self._reposition_steer = -_clamp(
            _REPOSITION_STEER_MAG * (1.0 if bearing_err > 0 else -1.0),
            -1.0,
            1.0,
        )
        self._reposition_frames_left -= 1
        return ParkCommand(linear=self._reposition_speed, steering=self._reposition_steer, phase=phase_name)

    def _handle_stage(
        self,
        robot_pos: tuple[float, float],
        robot_yaw: float,
    ) -> ParkCommand:
        tx, ty = self._staging
        rx, ry = robot_pos
        dist = math.sqrt((tx - rx) ** 2 + (ty - ry) ** 2)

        if self._reposition_frames_left <= 0 and dist < _POS_REACH_DIST:
            logger.debug("ParkController: STAGE → ENTER")
            self._phase = ParkPhase.ENTER
            return self._handle_enter(robot_pos, robot_yaw)

        return self._pursue_with_reposition(robot_pos, robot_yaw, (tx, ty), "stage")

    def _handle_enter(
        self,
        robot_pos: tuple[float, float],
        robot_yaw: float,
    ) -> ParkCommand:
        z = self._zone
        rx, ry = robot_pos

        pos_inside, yaw_ok = _inside_zone(rx, ry, robot_yaw, z)
        if pos_inside and yaw_ok:
            logger.info("ParkController: DONE — inside zone, yaw ok")
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase="done")

        return self._pursue_with_reposition(robot_pos, robot_yaw, (z.gap_cx, z.gap_cy), "enter")


# Pure helpers


def _build_zone(
    b1: tuple[float, float],
    b2: tuple[float, float],
    section: Section,
) -> ParkZone:
    """Compute parking bounding box from block positions."""
    hw = ParkingLotSpecs.WIDTH / 2  # half block width (10 mm)
    hl = ParkingLotSpecs.LENGTH / 2  # half block length (100 mm)

    if section in (Section.SOUTH, Section.NORTH):
        x1, x2 = sorted([b1[0], b2[0]])
        gap_cx = (b1[0] + b2[0]) / 2
        gap_cy = (b1[1] + b2[1]) / 2
        x_min = x1 + hw
        x_max = x2 - hw
        if section is Section.SOUTH:
            y_min = 0.0
            y_max = gap_cy + hl + _INSIDE_TOLERANCE
            target_yaw = -math.pi / 2
        else:
            y_min = gap_cy - hl - _INSIDE_TOLERANCE
            y_max = 3.0
            target_yaw = math.pi / 2
    else:
        y1, y2 = sorted([b1[1], b2[1]])
        gap_cx = (b1[0] + b2[0]) / 2
        gap_cy = (b1[1] + b2[1]) / 2
        y_min = y1 + hw
        y_max = y2 - hw
        if section is Section.EAST:
            x_min = gap_cx - hl - _INSIDE_TOLERANCE
            x_max = 3.0
            target_yaw = 0.0
        else:
            x_min = 0.0
            x_max = gap_cx + hl + _INSIDE_TOLERANCE
            target_yaw = math.pi

    return ParkZone(
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        target_yaw=target_yaw,
        gap_cx=gap_cx,
        gap_cy=gap_cy,
    )


def _staging_pos(zone: ParkZone, section: Section) -> tuple[float, float]:
    """Position directly in front of the gap opening, on the track side."""
    if section is Section.SOUTH:
        return zone.gap_cx, zone.y_max + _APPROACH_CLEARANCE
    if section is Section.NORTH:
        return zone.gap_cx, zone.y_min - _APPROACH_CLEARANCE
    if section is Section.EAST:
        return zone.x_min - _APPROACH_CLEARANCE, zone.gap_cy
    return zone.x_max + _APPROACH_CLEARANCE, zone.gap_cy  # WEST


def _bearing_error(
    robot_pos: tuple[float, float],
    robot_yaw: float,
    target: tuple[float, float],
) -> float:
    """Signed angle (radians) from the robot's heading to the bearing toward ``target``."""
    dx = target[0] - robot_pos[0]
    dy = target[1] - robot_pos[1]
    desired_yaw = math.atan2(dy, dx)
    return _normalise_angle(desired_yaw - robot_yaw)


_MIN_LOOKAHEAD_DIST = 0.02  # metres: floor to avoid a near-zero-distance curvature blow-up


def _local_frame(
    robot_pos: tuple[float, float],
    robot_yaw: float,
    target: tuple[float, float],
) -> tuple[float, float]:
    """``target`` expressed in the robot's local frame (x=forward, y=left)."""
    dx = target[0] - robot_pos[0]
    dy = target[1] - robot_pos[1]
    cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
    x_local = dx * cos_yaw + dy * sin_yaw
    y_local = -dx * sin_yaw + dy * cos_yaw
    return x_local, y_local


def _pure_pursuit_steer(x_local: float, y_local: float) -> float:
    """Curvature-based pure pursuit steering toward a local-frame target (normalised [-1, 1]).

    Standard formulation, treating the target itself as the lookahead point (unlike
    ``WaypointController``, which searches a path for a point at a fixed lookahead
    distance, ParkController always aims directly at a single fixed target):
    curvature = 2*y_local / L_d**2, steering angle = atan(curvature * wheelbase), clamped
    to the chassis's physical steering limit.

    Only valid for a target roughly ahead (``x_local > 0``) — the formula gives a
    plausible-looking but wrong result for a target behind the robot; callers must check
    that separately (see ``_pursue_with_reposition``'s target-behind check).
    """
    lookahead = max(math.hypot(x_local, y_local), _MIN_LOOKAHEAD_DIST)
    curvature = 2.0 * y_local / (lookahead**2)
    steer_angle = math.atan(curvature * RobotSpecs.WHEELBASE)
    steer_angle = _clamp(steer_angle, -RobotSpecs.MAX_STEERING_ANGLE, RobotSpecs.MAX_STEERING_ANGLE)
    return steer_angle / RobotSpecs.MAX_STEERING_ANGLE


def _inside_zone(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
) -> tuple[bool, bool]:
    """Return (position_inside, yaw_ok)."""
    pos_inside = zone.x_min <= rx <= zone.x_max and zone.y_min <= ry <= zone.y_max
    yaw_err = abs(_normalise_angle(robot_yaw - zone.target_yaw))
    return pos_inside, yaw_err <= _YAW_TOLERANCE


def _normalise_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def park_controller_from_metadata(
    metadata: dict,
    start_section: Section,
) -> ParkController | None:
    """Construct a ParkController from scenario metadata. None for open challenge."""
    parking = metadata.get("parking_lot")
    if parking is None:
        return None
    b1 = (parking["block1_position"]["x"], parking["block1_position"]["y"])
    b2 = (parking["block2_position"]["x"], parking["block2_position"]["y"])
    return ParkController(
        parking_config={"block1_pos": b1, "block2_pos": b2},
        start_section=start_section,
    )
