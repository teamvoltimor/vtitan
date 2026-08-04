"""Parallel-park controller for WRO 2026 obstacles challenge.

Drives the robot into the bay between the two magenta parking blocks after
completing 3 laps.  Uses a two-phase pure-pursuit strategy:

  Phase 1 — STAGE : drive to a staging position in front of the bay opening.
  Phase 2 — ENTER : drive toward the bay, correcting heading toward its centre.

Stop condition (WRO rule): the robot's whole projection on the mat must lie
inside the rectangle between the two markers, and the robot must be parallel to
the field wall — "parallel" meaning the two wheels on one side differ by no more
than 2 cm in their distance to that wall.

Bay geometry — the two markers are fins standing *perpendicular* to the outer
wall, each ``ParkingLotSpecs.LENGTH`` long and spanning the bay's full depth, so
the bay is a pocket closed on three sides and open only toward the corridor. The
final pose is therefore parallel to the outer wall, and the entry is lateral.

Pure Python — no ROS2 dependencies. Unit testable.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from enum import StrEnum

from shared.config.constants import DictKeys, ParkingLotSpecs, RobotSpecs, TrackDimensions
from shared.config.enums import Direction, Section
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.models import BlockPosition, ParkingLot

from src.navigation.utils import _local_frame
from src.navigation.utils import _pure_pursuit_steer as _shared_pure_pursuit_steer

logger = logging.getLogger(__name__)


# One load, reused by every module-level constant below. These feed module-level
# geometry functions rather than ParkController methods, so there is no instance
# to inject tuning into -- but that is not a reason to restate a configured
# number. Each of these previously carried its own literal plus a "same
# concept/value as NavigationTuning.parking.X" comment, which is two sources of
# truth: editing parking.toml changed nothing, and the comment was the only
# thing keeping them in step. The sign router shipped the same pattern and its
# DEFORM_DEPTH_BUFFER_M sat in the TOML with no reader at all.
_TUNING = NavigationTuning.load_default()
_parking_tuning = _TUNING.parking

_PARALLEL_TOLERANCE_M = _parking_tuning.PARALLEL_TOLERANCE_M
"""WRO rule: the two wheels on one side may differ by at most 2 cm in wall distance.

A competition rule, not a tuning choice -- and ``_YAW_TOLERANCE`` below is
derived from it, so a rules change has to reach both. Config is the one place
that should carry it."""

_YAW_TOLERANCE = math.atan2(_PARALLEL_TOLERANCE_M, RobotSpecs.WHEELBASE)
"""Heading tolerance implied by the rule above (~6.0 deg at WHEELBASE=0.19).

Derived from the rule rather than picked: the "two wheels on one side" are the front and
rear wheel of one flank, i.e. WHEELBASE apart along the chassis, so a heading error of
``theta`` puts ``WHEELBASE * sin(theta)`` between their wall distances. Note this is the
*looser* of the two stop criteria -- footprint containment (``_footprint_inside``) binds
first for any chassis whose width approaches the bay depth."""
# Staging stand-off, perpendicular to the gap opening. Reuses ARC_RADIUS (not a fresh
# literal) because both share the same real constraint: must clear the chassis's Ackermann
# minimum turning radius. At the old 0.25m this was smaller than R_min, and for N/S
# sections nearly collinear with the corridor cruise line -- a robot cruising through could
# overshoot the staging point in x before y converged, flipping the bearing ~180° in a
# single tick and forcing `_pursuit_steer` into a non-convergent orbit. See
# docs/internal/2026-07-11-navigation-logic-review.md §2.3 for the full trace.
_APPROACH_CLEARANCE = _TUNING.waypoints.ARC_RADIUS
_POS_REACH_DIST = _parking_tuning.POS_REACH_DIST_M  # metres: "reached staging" threshold
_DEFAULT_MAX_FRAMES = _parking_tuning.DEFAULT_MAX_FRAMES  # 20s at 20Hz at the configured default

_SATURATED_STEER_THRESHOLD = _parking_tuning.SATURATED_STEER_THRESHOLD
"""abs(steering) at/above this counts as "at physical lock" for _SATURATION_STUCK_TICKS.

Configured in parking.toml."""

_SATURATION_STUCK_TICKS = _parking_tuning.SATURATION_STUCK_TICKS  # 1s at 20Hz at the configured default
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
_escape_tuning = _TUNING.escape
_REPOSITION_SPEED = _escape_tuning.REV_SPEED
_REPOSITION_STEER_MAG = _escape_tuning.REV_STEERING_SCALE
_REPOSITION_FRAMES = _escape_tuning.MAX_ESCAPE_FRAMES


@dataclass(frozen=True)
class ParkZone:
    """The parking lot rectangle (WRO: "the rectangle between the two markers").

    This is the bay itself, not a tolerance box around it: bounded along the wall by the
    two fins' inner faces, and in depth by the outer wall and the fins' inner ends. The
    robot's whole projection has to fit inside it, so it is deliberately the *true* lot
    outline with no slack added — containment margin belongs in the stop check, not here.
    """

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    target_yaw: float  # expected robot yaw when parked (radians), parallel to the outer wall
    gap_cx: float  # centre of the bay (world x)
    gap_cy: float  # centre of the bay (world y)
    wall_is_x: bool  # whether the field wall backing this lot runs along x (E/W sections)
    wall_coord: float  # the wall's coordinate on the axis normal to it

    def project(self, x: float, y: float) -> tuple[float, float]:
        """Split a world point into (along-wall, depth) coordinates for this lot.

        Which world axis plays which role flips between the N/S and E/W corridors, and
        getting it backwards silently swaps the bay's 0.43 m mouth for its 0.20 m depth.
        Both guards below need the same split, so it is derived once here rather than
        re-spelled at each use.
        """
        return (y, x) if self.wall_is_x else (x, y)

    def bounds_along(self) -> tuple[float, float]:
        """The lot's extent along the wall — i.e. between the two fins' inner faces."""
        return (self.y_min, self.y_max) if self.wall_is_x else (self.x_min, self.x_max)

    def bounds_depth(self) -> tuple[float, float]:
        """The lot's extent out from the wall — i.e. the depth the fins span."""
        return (self.x_min, self.x_max) if self.wall_is_x else (self.y_min, self.y_max)


class ParkPhase(StrEnum):
    """Parking maneuver phases."""

    STAGE = "stage"
    ENTER = "enter"
    DONE = "done"


@dataclass
class ParkCommand:
    """Motor command from the park controller."""

    linear: float  # m/s
    steering: float  # normalised [-1, 1]
    done: bool = False
    phase: ParkPhase = ParkPhase.STAGE


class ParkController:
    """Two-phase park controller.

    Phase 1 — STAGE: pure-pursuit toward the staging position in front of the
        bay opening (on the track side).
    Phase 2 — ENTER: pure-pursuit toward the bay centre; switches to DONE only
        when the whole footprint is inside the lot AND the heading is parallel
        to the wall within tolerance.

    Note: ENTER still pure-pursues a single point, which steers for position
    without controlling the final heading. With the stop condition now requiring
    genuine containment, that is not sufficient to park this chassis — the
    controller will honestly time out rather than falsely report success. The
    entry maneuver itself is a separate piece of work; see
    ``platform/docs/internal/2026-07-25-parking-review.md``.

    Args:
        parking_config: Dict with 'block1_pos' and 'block2_pos' keys.
        start_section: Corridor that contains the parking lot.
        direction: Traversal direction, which selects between the two wall-parallel
            headings so the robot parks facing the way it was already travelling.
        speed: Constant driving speed (m/s).
        max_frames: Hard bound on how many control ticks the maneuver may run
            before giving up and holding position. Without this, a robot that
            can never satisfy the position+yaw stop condition (e.g. wedged
            against a block) would chase the gap centre for the rest of the
            match instead of coming to a controlled stop.
    """

    def __init__(
        self,
        parking_config: ParkingLot,
        start_section: Section,
        direction: Direction,
        speed: float = 0.12,
        max_frames: int = _DEFAULT_MAX_FRAMES,
    ) -> None:
        self._section = start_section
        self._direction = direction
        self._speed = speed
        self._phase = ParkPhase.STAGE
        self._max_frames = max_frames
        self._frames_elapsed = 0
        self._timed_out = False
        self._reposition_frames_left = 0
        self._reposition_speed = 0.0
        self._reposition_steer = 0.0
        self._saturated_ticks = 0

        self._zone = _build_zone(
            parking_config.block1_position,
            parking_config.block2_position,
            start_section,
            direction,
        )
        self._staging = _staging_pos(self._zone, start_section)

        logger.info(
            "ParkController: zone=%s staging=%s section=%s direction=%s",
            self._zone,
            self._staging,
            start_section,
            direction,
        )

    @classmethod
    def from_tuning(cls, parking_config: ParkingLot, start_section: Section, direction: Direction, tuning: NavigationTuning | None = None) -> ParkController:
        """Build controller from tuning parameters.

        Args:
            parking_config: Parking lot geometry configuration.
            start_section: Starting section of the parking lot.
            direction: Travel direction.
            tuning: NavigationTuning instance (defaults to load_default).

        Returns:
            ParkController with values from tuning.
        """
        tuning = tuning or NavigationTuning.load_default()
        return cls(
            parking_config=parking_config,
            start_section=start_section,
            direction=direction,
            speed=tuning.parking.SPEED,
            max_frames=tuning.parking.DEFAULT_MAX_FRAMES,
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
        """Whether the maneuver gave up rather than parking cleanly.

        Two ways to give up: exhausting the frame budget, or ENTER reaching the field wall
        without achieving containment (see ``_footprint_breaches_wall``). Both mean "stopped,
        not parked", which is the distinction callers actually act on.
        """
        return self._timed_out

    @property
    def section(self) -> Section:
        """Corridor that contains the parking lot."""
        return self._section

    @property
    def direction(self) -> Direction:
        """Traversal direction the parked heading was chosen to match."""
        return self._direction

    @property
    def zone(self) -> ParkZone:
        """The parking lot rectangle and target heading this controller is aiming for."""
        return self._zone

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
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase=ParkPhase.DONE)

        self._frames_elapsed += 1
        if self._frames_elapsed > self._max_frames:
            logger.warning(
                "ParkController: giving up after %d frames without parking cleanly",
                self._frames_elapsed,
            )
            self._timed_out = True
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase=ParkPhase.DONE)

        # Early exit: if already inside the zone at any phase, we're done.
        pos_inside, yaw_ok = _inside_zone(robot_pos[0], robot_pos[1], robot_yaw, self._zone)
        if pos_inside and yaw_ok:
            logger.info("ParkController: DONE — already inside zone")
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase=ParkPhase.DONE)

        if self._phase is ParkPhase.STAGE:
            return self._handle_stage(robot_pos, robot_yaw)
        return self._handle_enter(robot_pos, robot_yaw)

    # Phase handlers

    def _pursue_with_reposition(
        self,
        robot_pos: tuple[float, float],
        robot_yaw: float,
        target: tuple[float, float],
        phase_name: ParkPhase,
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
        phase_name: ParkPhase,
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

        return self._pursue_with_reposition(robot_pos, robot_yaw, (tx, ty), ParkPhase.STAGE)

    def _handle_enter(
        self,
        robot_pos: tuple[float, float],
        robot_yaw: float,
    ) -> ParkCommand:
        z = self._zone
        rx, ry = robot_pos

        pos_inside, yaw_ok = _inside_zone(rx, ry, robot_yaw, z)
        if pos_inside and yaw_ok:
            logger.info("ParkController: DONE — fully inside the lot, wall-parallel")
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase=ParkPhase.DONE)

        if _footprint_breaches_wall(rx, ry, robot_yaw, z):
            logger.warning("ParkController: giving up — footprint reached the field wall without parking")
            self._timed_out = True
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase=ParkPhase.DONE)

        if _footprint_breaches_markers(rx, ry, robot_yaw, z):
            logger.warning("ParkController: giving up — footprint reached a marker fin without parking")
            self._timed_out = True
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase=ParkPhase.DONE)

        return self._pursue_with_reposition(robot_pos, robot_yaw, (z.gap_cx, z.gap_cy), ParkPhase.ENTER)


# Pure helpers


def _build_zone(
    b1: BlockPosition,
    b2: BlockPosition,
    section: Section,
    direction: Direction,
) -> ParkZone:
    """Compute the parking lot rectangle and the wall-parallel target yaw.

    The markers are fins perpendicular to the outer wall: ``ParkingLotSpecs.WIDTH`` (20 mm)
    thick along the wall, ``ParkingLotSpecs.LENGTH`` (200 mm) deep out from it. So the lot
    spans, along the wall, between the fins' inner faces, and in depth from the wall out to
    the fins' inner ends.

    ``target_yaw`` is parallel to the outer wall — the lot is only as deep as the chassis is
    wide, so a nose-in pose cannot fit and is not what the rule asks for. Of the two parallel
    headings, the one matching ``direction`` of travel is chosen, so the robot never has to
    turn around inside a bay with no room to do it.
    """
    half_fin_thickness = ParkingLotSpecs.WIDTH / 2
    bay_depth = ParkingLotSpecs.LENGTH
    cw = direction is Direction.CLOCKWISE

    if section in (Section.SOUTH, Section.NORTH):
        x1, x2 = sorted([b1.x, b2.x])
        x_min = x1 + half_fin_thickness
        x_max = x2 - half_fin_thickness
        if section is Section.SOUTH:
            y_min, y_max = TrackDimensions.MIN_COORD, TrackDimensions.MIN_COORD + bay_depth
            target_yaw = math.pi if cw else 0.0
        else:
            y_min, y_max = TrackDimensions.MAX_COORD - bay_depth, TrackDimensions.MAX_COORD
            target_yaw = 0.0 if cw else math.pi
    else:
        y1, y2 = sorted([b1.y, b2.y])
        y_min = y1 + half_fin_thickness
        y_max = y2 - half_fin_thickness
        if section is Section.EAST:
            x_min, x_max = TrackDimensions.MAX_COORD - bay_depth, TrackDimensions.MAX_COORD
            target_yaw = math.pi / 2 if cw else -math.pi / 2
        else:
            x_min, x_max = TrackDimensions.MIN_COORD, TrackDimensions.MIN_COORD + bay_depth
            target_yaw = -math.pi / 2 if cw else math.pi / 2

    wall_is_x = section in (Section.EAST, Section.WEST)
    if wall_is_x:
        wall_coord = x_max if section is Section.EAST else x_min
    else:
        wall_coord = y_max if section is Section.NORTH else y_min

    return ParkZone(
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        target_yaw=target_yaw,
        gap_cx=(x_min + x_max) / 2,
        gap_cy=(y_min + y_max) / 2,
        wall_is_x=wall_is_x,
        wall_coord=wall_coord,
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


_MIN_LOOKAHEAD_DIST = _parking_tuning.MIN_LOOKAHEAD_DIST_M  # floor to avoid a near-zero-distance curvature blow-up


def _pure_pursuit_steer(x_local: float, y_local: float) -> float:
    """``ParkController``-bound wrapper: always aims directly at a single fixed
    target (unlike ``WaypointController``, which searches a path for a point at a
    fixed lookahead distance), so it supplies its own lookahead floor here rather
    than at each call site. See ``src.navigation.utils._pure_pursuit_steer`` for
    the shared formula and its physical reasoning.
    """
    return _shared_pure_pursuit_steer(x_local, y_local, _MIN_LOOKAHEAD_DIST)


def _chassis_corners(
    rx: float,
    ry: float,
    robot_yaw: float,
) -> list[tuple[float, float]]:
    """The four corners of the chassis footprint at this pose (world frame)."""
    half_l, half_w = RobotSpecs.LENGTH / 2, RobotSpecs.WIDTH / 2
    cos_yaw, sin_yaw = math.cos(robot_yaw), math.sin(robot_yaw)
    return [
        (rx + dx * cos_yaw - dy * sin_yaw, ry + dx * sin_yaw + dy * cos_yaw)
        for dx, dy in ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w))
    ]


_WALL_STANDOFF = _parking_tuning.WALL_STANDOFF_M
"""Closest the chassis footprint may come to the field wall backing the parking lot.

The lot's far edge *is* the wall, so "drive to the lot centre" and "don't touch the wall"
pull against each other for a chassis as wide as the lot is deep. Something has to give,
and it must be the maneuver, not the wall. Sized above the simulator's own wall collision
margin (0.04 m: the 0.18 m collision mesh vs the 0.10 m visual thickness), which is the
tightest the sim will let anything approach before registering a contact anyway."""


def _footprint_breaches_wall(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
) -> bool:
    """Whether any chassis corner has come within ``_WALL_STANDOFF`` of the field wall.

    ENTER pure-pursues the lot centre, which controls position but not heading, so a robot
    that arrives across the lot rather than along it drives its nose at the wall and keeps
    going. Previously this was masked: the old zone extended past the lot's real far edge
    and used a centre-in-box test, so the maneuver "succeeded" and stopped short of the wall
    by accident. With the stop condition corrected to the actual rule, nothing stops it any
    more -- so the maneuver gives up here instead of pushing into the wall. Not colliding
    takes priority over completing the park.
    """
    for cx, cy in _chassis_corners(rx, ry, robot_yaw):
        coord = cx if zone.wall_is_x else cy
        if abs(coord - zone.wall_coord) < _WALL_STANDOFF and _is_beyond_lot_centre(coord, zone):
            return True
    return False


_MARKER_STANDOFF = _parking_tuning.MARKER_STANDOFF_M
"""Closest the chassis footprint may come to either marker fin's inner face.

Much smaller than ``_WALL_STANDOFF`` because the budget is smaller: the mouth between the
fins is ``BLOCK_SPACING_FACTOR`` x chassis length, leaving only ~6.5 cm of longitudinal
clearance per end once a 0.30 m chassis is centred in it. A wall-sized 5 cm standoff would
consume nearly all of that and abort approaches that are in fact clean, so this is sized to
catch an actual graze rather than to reserve maneuvering room."""


def _footprint_breaches_markers(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
) -> bool:
    """Whether any chassis corner has come within ``_MARKER_STANDOFF`` of a marker fin.

    The wall guard alone used to be sufficient by accident: with the steering limit modelled
    at 30 deg the chassis could not turn tightly enough to swing a corner into a fin before
    the wall stopped it. At the real ~70 deg lock (R_min 0.034 m rather than 0.165 m) ENTER's
    pure pursuit of the lot centre turns hard enough to reach them, so the fins need the same
    explicit give-up the wall has. Same priority as there: not colliding beats parking.

    A fin flanks the lot along the wall and spans its full depth, so a corner is in fin
    territory when it lies within the lot's depth band and at or past a fin's inner face.
    """
    depth_min, depth_max = zone.bounds_depth()
    along_min, along_max = zone.bounds_along()
    for cx, cy in _chassis_corners(rx, ry, robot_yaw):
        along, depth = zone.project(cx, cy)
        if not (depth_min - _MARKER_STANDOFF <= depth <= depth_max + _MARKER_STANDOFF):
            continue  # out in the corridor, past the fins' ends — nothing to hit
        if along <= along_min + _MARKER_STANDOFF or along >= along_max - _MARKER_STANDOFF:
            return True
    return False


def _is_beyond_lot_centre(coord: float, zone: ParkZone) -> bool:
    """Whether ``coord`` lies on the wall side of the lot's midline."""
    centre = zone.gap_cx if zone.wall_is_x else zone.gap_cy
    return coord > centre if zone.wall_coord > centre else coord < centre


def _footprint_inside(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
) -> bool:
    """Whether the robot's whole projection on the mat lies inside the parking lot.

    This is the rule as written ("the projection of the robot on the mat is fully inside the
    rectangle between the two markers"), not the centre-of-chassis approximation it replaces.
    The difference is not cosmetic: a centre-in-box test reports a successful park for a robot
    sitting mostly in the corridor, or with its nose through the outer wall, because neither
    the footprint nor the heading enters into it.
    """
    return all(
        zone.x_min <= cx <= zone.x_max and zone.y_min <= cy <= zone.y_max
        for cx, cy in _chassis_corners(rx, ry, robot_yaw)
    )


def _inside_zone(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
) -> tuple[bool, bool]:
    """Return (fully_parked, parallel_ok) per the WRO parking rule."""
    yaw_err = abs(_normalise_angle(robot_yaw - zone.target_yaw))
    return _footprint_inside(rx, ry, robot_yaw, zone), yaw_err <= _YAW_TOLERANCE


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
    direction: Direction | None = None,
    tuning: NavigationTuning | None = None,
) -> ParkController | None:
    """Construct a ParkController from scenario metadata. None for open challenge.

    ``direction`` falls back to the scenario's own ``starting_conditions.direction`` when not
    passed explicitly, so callers that already hold it (the sim) and callers that only hold
    the metadata (the ROS2 node) both get the correct wall-parallel target heading.

    ``tuning`` defaults to ``NavigationTuning.load_default()`` (the checked-in
    config tree) rather than the class's own bare literal defaults, so a
    loaded/edited tuning profile actually takes effect here too.
    """
    parking = metadata.get(DictKeys.PARKING_LOT)
    if parking is None:
        return None
    if direction is None:
        direction = Direction.from_string(metadata[DictKeys.STARTING_CONDITIONS][DictKeys.DIRECTION])
    tuning = tuning or NavigationTuning.load_default()
    return ParkController(
        parking_config=ParkingLot(
            block1_position=BlockPosition(
                x=parking[DictKeys.BLOCK1_POSITION][DictKeys.X],
                y=parking[DictKeys.BLOCK1_POSITION][DictKeys.Y],
            ),
            block2_position=BlockPosition(
                x=parking[DictKeys.BLOCK2_POSITION][DictKeys.X],
                y=parking[DictKeys.BLOCK2_POSITION][DictKeys.Y],
            ),
        ),
        start_section=start_section,
        direction=direction,
        speed=tuning.parking.SPEED,
        max_frames=tuning.parking.DEFAULT_MAX_FRAMES,
    )
