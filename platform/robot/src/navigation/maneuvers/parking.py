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
from typing import TYPE_CHECKING

from shared.config.constants import DictKeys, ParkingLotSpecs, RobotSpecs, TrackDimensions
from shared.domain.enums import Direction, ParkPhase, Section
from shared.domain.models import BBox, BlockPosition, ParkingLot, Pose, Waypoint

from src.config.tuning_helpers import TuningContext, get_tuning
from src.navigation.utils import (
    _pure_pursuit_steer as _shared_pure_pursuit_steer,
    clamp as _clamp,
)

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning

logger = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class _ParkingConstants:
    """Tuning-derived parking constants, computed on-demand instead of frozen at module level."""

    parallel_tolerance_m: float
    yaw_tolerance: float
    approach_clearance: float
    pos_reach_dist_m: float
    default_max_frames: int
    saturated_steer_threshold: float
    saturation_stuck_ticks: int
    reposition_speed: float
    reposition_steer_mag: float
    min_lookahead_dist_m: float
    wall_standoff_m: float
    marker_standoff_m: float

    @classmethod
    def from_tuning(cls, tuning: NavigationTuning) -> _ParkingConstants:
        """Create from a NavigationTuning instance."""
        parking_tuning = tuning.parking
        escape_tuning = tuning.escape
        parallel_tolerance = parking_tuning.PARALLEL_TOLERANCE_M
        return cls(
            parallel_tolerance_m=parallel_tolerance,
            yaw_tolerance=math.atan2(parallel_tolerance, RobotSpecs.WHEELBASE),
            approach_clearance=tuning.waypoints.ARC_RADIUS,
            pos_reach_dist_m=parking_tuning.POS_REACH_DIST_M,
            default_max_frames=parking_tuning.DEFAULT_MAX_FRAMES,
            saturated_steer_threshold=parking_tuning.SATURATED_STEER_THRESHOLD,
            saturation_stuck_ticks=parking_tuning.SATURATION_STUCK_TICKS,
            reposition_speed=escape_tuning.REV_SPEED,
            reposition_steer_mag=escape_tuning.rev_steer_norm(),
            min_lookahead_dist_m=parking_tuning.MIN_LOOKAHEAD_DIST_M,
            wall_standoff_m=parking_tuning.WALL_STANDOFF_M,
            marker_standoff_m=parking_tuning.MARKER_STANDOFF_M,
        )


class ParkingContext(TuningContext[_ParkingConstants]):
    """Context holding tuning-derived parking constants, passed to helper functions.

    Eliminates module-level constants by holding them in an instance,
    which is passed to functions that need them. Enables test-time tuning injection.
    """

    _constants_cls = _ParkingConstants


_DEFAULT_PARKING_CONTEXT = ParkingContext()


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


@dataclass(slots=True)
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
        max_frames: int | None = None,
        tuning: NavigationTuning | None = None,
    ) -> None:
        self._tuning = get_tuning(tuning)
        self._context = ParkingContext(self._tuning)
        if max_frames is None:
            max_frames = self._context.constants.default_max_frames
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
        self._staging = _staging_pos(self._zone, start_section, self._context)

        logger.info(
            "ParkController: zone=%s staging=%s section=%s direction=%s",
            self._zone,
            self._staging,
            start_section,
            direction,
        )

    @classmethod
    def from_tuning(
        cls,
        parking_config: ParkingLot,
        start_section: Section,
        direction: Direction,
        tuning: NavigationTuning | None = None,
    ) -> ParkController:
        """Build controller from tuning parameters.

        Args:
            parking_config: Parking lot geometry configuration.
            start_section: Starting section of the parking lot.
            direction: Travel direction.
            tuning: NavigationTuning instance (defaults to load_default).

        Returns:
            ParkController with values from tuning.
        """
        tuning = get_tuning(tuning)
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
    def staging(self) -> Waypoint:
        """Staging position in front of the gap opening (world x, y)."""
        return self._staging

    def update(
        self,
        robot_pose: Pose,
    ) -> ParkCommand:
        """Compute next motor command.

        Args:
            robot_pose: Current world pose of robot centre (heading 0=east, π/2=north).

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
        pos_inside, yaw_ok = _inside_zone(robot_pose.x, robot_pose.y, robot_pose.yaw, self._zone, self._context)
        if pos_inside and yaw_ok:
            logger.info("ParkController: DONE — already inside zone")
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase=ParkPhase.DONE)

        if self._phase is ParkPhase.STAGE:
            return self._handle_stage(robot_pose)
        return self._handle_enter(robot_pose)

    # Phase handlers

    def _pursue_with_reposition(
        self,
        robot_pose: Pose,
        target: Waypoint,
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

        x_local, y_local = robot_pose.to_local_frame(target)

        if x_local < 0:
            # Target is behind the robot: pure pursuit's curvature formula is only valid
            # for a roughly-forward target -- for a rearward one it can produce a
            # plausible-looking (non-saturated) steering command that actually drives away
            # from the target instead of toward it. Reverse immediately rather than trust it.
            return self._start_reposition(robot_pose, target, phase_name, "target behind")

        steer = _pure_pursuit_steer(x_local, y_local, self._context)

        if abs(steer) >= self._context.constants.saturated_steer_threshold:
            self._saturated_ticks += 1
        else:
            self._saturated_ticks = 0

        if self._saturated_ticks >= self._context.constants.saturation_stuck_ticks:
            # Steering has been pinned at physical lock for a full second straight: the
            # required curvature genuinely exceeds what the chassis can do going forward
            # (see the module-level comment). Reverse to open room instead of continuing to
            # orbit.
            reason = f"{self._saturated_ticks} saturated ticks"
            return self._start_reposition(robot_pose, target, phase_name, reason)

        return ParkCommand(linear=self._speed, steering=steer, phase=phase_name)

    def _start_reposition(
        self,
        robot_pose: Pose,
        target: Waypoint,
        phase_name: ParkPhase,
        reason: str,
    ) -> ParkCommand:
        """Latch a reverse-and-reorient recovery burst. See _pursue_with_reposition."""
        bearing_err = _bearing_error(robot_pose, target)
        logger.debug(
            "ParkController: %s reposition (%s, bearing_err=%.1f deg)",
            phase_name,
            reason,
            math.degrees(bearing_err),
        )
        self._saturated_ticks = 0
        self._reposition_frames_left = self._context.constants.default_max_frames
        self._reposition_speed = self._context.constants.reposition_speed
        # Sign-flipped for reverse Ackermann geometry (v<0 inverts the yaw-rate response
        # to a given steer sign), biased toward whichever side the target currently bears.
        self._reposition_steer = -_clamp(
            self._context.constants.reposition_steer_mag * (1.0 if bearing_err > 0 else -1.0),
            -1.0,
            1.0,
        )
        self._reposition_frames_left -= 1
        return ParkCommand(linear=self._reposition_speed, steering=self._reposition_steer, phase=phase_name)

    def _handle_stage(
        self,
        robot_pose: Pose,
    ) -> ParkCommand:
        dist = math.sqrt((self._staging.x - robot_pose.x) ** 2 + (self._staging.y - robot_pose.y) ** 2)

        if self._reposition_frames_left <= 0 and dist < self._context.constants.pos_reach_dist_m:
            logger.debug("ParkController: STAGE → ENTER")
            self._phase = ParkPhase.ENTER
            return self._handle_enter(robot_pose)

        return self._pursue_with_reposition(robot_pose, self._staging, ParkPhase.STAGE)

    def _handle_enter(
        self,
        robot_pose: Pose,
    ) -> ParkCommand:
        z = self._zone
        rx, ry, robot_yaw = robot_pose.x, robot_pose.y, robot_pose.yaw

        pos_inside, yaw_ok = _inside_zone(rx, ry, robot_yaw, z, self._context)
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

        return self._pursue_with_reposition(robot_pose, Waypoint(z.gap_cx, z.gap_cy), ParkPhase.ENTER)


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
    cw = direction is Direction.CLOCKWISE

    if section in (Section.SOUTH, Section.NORTH):
        x1, x2 = sorted([b1.x, b2.x])
        x_min = x1 + half_fin_thickness
        x_max = x2 - half_fin_thickness
        if section is Section.SOUTH:
            y_min, y_max = TrackDimensions.MIN_COORD, TrackDimensions.MIN_COORD + ParkingLotSpecs.LENGTH
            target_yaw = math.pi if cw else 0.0
        else:
            y_min, y_max = TrackDimensions.MAX_COORD - ParkingLotSpecs.LENGTH, TrackDimensions.MAX_COORD
            target_yaw = 0.0 if cw else math.pi
    else:
        y1, y2 = sorted([b1.y, b2.y])
        y_min = y1 + half_fin_thickness
        y_max = y2 - half_fin_thickness
        if section is Section.EAST:
            x_min, x_max = TrackDimensions.MAX_COORD - ParkingLotSpecs.LENGTH, TrackDimensions.MAX_COORD
            target_yaw = math.pi / 2 if cw else -math.pi / 2
        else:
            x_min, x_max = TrackDimensions.MIN_COORD, TrackDimensions.MIN_COORD + ParkingLotSpecs.LENGTH
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
        gap_cx=BBox(x_min, y_min, x_max, y_max).center.x,
        gap_cy=BBox(x_min, y_min, x_max, y_max).center.y,
        wall_is_x=wall_is_x,
        wall_coord=wall_coord,
    )


def _staging_pos(zone: ParkZone, section: Section, context: ParkingContext) -> Waypoint:
    """Position directly in front of the gap opening, on the track side."""
    clearance = context.constants.approach_clearance
    if section is Section.SOUTH:
        return Waypoint(zone.gap_cx, zone.y_max + clearance)
    if section is Section.NORTH:
        return Waypoint(zone.gap_cx, zone.y_min - clearance)
    if section is Section.EAST:
        return Waypoint(zone.x_min - clearance, zone.gap_cy)
    return Waypoint(zone.x_max + clearance, zone.gap_cy)  # WEST


def _bearing_error(
    robot_pose: Pose,
    target: Waypoint,
) -> float:
    """Signed angle (radians) from the robot's heading to the bearing toward ``target``."""
    return _normalise_angle(robot_pose.bearing_to(target.to_pose()) - robot_pose.yaw)


# Already defined above via _DEFAULT_PARKING_CONSTANTS


def _pure_pursuit_steer(x_local: float, y_local: float, context: ParkingContext) -> float:
    """Aim directly at a single fixed target for ``ParkController``.

    Unlike ``WaypointController`` (which searches a path for a point at a fixed
    lookahead distance), this supplies its own lookahead floor here rather than
    at each call site. See ``src.navigation.utils._pure_pursuit_steer`` for the
    shared formula and its physical reasoning.
    """
    return _shared_pure_pursuit_steer(x_local, y_local, context.constants.min_lookahead_dist_m)


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


# Already defined above via _DEFAULT_PARKING_CONSTANTS
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
        if abs(coord - zone.wall_coord) < _DEFAULT_PARKING_CONTEXT.constants.wall_standoff_m and _is_beyond_lot_centre(
            coord, zone
        ):
            return True
    return False


# Already defined above via _DEFAULT_PARKING_CONSTANTS
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
    marker_standoff = _DEFAULT_PARKING_CONTEXT.constants.marker_standoff_m
    depth_min, depth_max = zone.bounds_depth()
    along_min, along_max = zone.bounds_along()
    for cx, cy in _chassis_corners(rx, ry, robot_yaw):
        along, depth = zone.project(cx, cy)
        if not (depth_min - marker_standoff <= depth <= depth_max + marker_standoff):
            continue  # out in the corridor, past the fins' ends — nothing to hit
        if along <= along_min + marker_standoff or along >= along_max - marker_standoff:
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
    lot = BBox(zone.x_min, zone.y_min, zone.x_max, zone.y_max)
    return all(lot.contains(Waypoint(cx, cy)) for cx, cy in _chassis_corners(rx, ry, robot_yaw))


def _inside_zone(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
    context: ParkingContext,
) -> tuple[bool, bool]:
    """Return (fully_parked, parallel_ok) per the WRO parking rule."""
    yaw_err = abs(_normalise_angle(robot_yaw - zone.target_yaw))
    return _footprint_inside(rx, ry, robot_yaw, zone), yaw_err <= context.constants.yaw_tolerance


def _normalise_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


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
        raw_direction = metadata[DictKeys.STARTING_CONDITIONS][DictKeys.DIRECTION]
        if raw_direction is None:
            msg = (
                "park_controller_from_metadata requires a resolved direction; pass it "
                "explicitly (every real caller already does) rather than relying on "
                "metadata.starting_conditions.direction, which may still be unresolved"
            )
            raise ValueError(msg)
        direction = Direction.from_string(raw_direction)
    tuning = get_tuning(tuning)
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
