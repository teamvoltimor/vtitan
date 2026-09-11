"""Two-phase parallel-park controller for the WRO 2026 obstacles challenge.

Drives the robot into the bay between the two magenta parking blocks after
completing 3 laps, using a two-phase pure-pursuit strategy:

  Phase 1 -- STAGE : drive to a staging position in front of the bay opening.
  Phase 2 -- ENTER : drive toward the bay, correcting heading toward its centre.

Stop condition (WRO rule): the robot's whole projection on the mat must lie
inside the rectangle between the two markers, and the robot must be parallel to
the field wall -- "parallel" meaning the two wheels on one side differ by no more
than 2 cm in their distance to that wall.

Pure Python -- no ROS2 dependencies. Unit testable.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from shared.config.constants import DictKeys
from shared.domain.enums import Direction, ParkPhase, Section
from shared.domain.models import BlockPosition, ParkingLot, Pose, Waypoint

from src.config.tuning_helpers import get_tuning
from src.navigation.maneuvers.parking.context import ParkingContext
from src.navigation.maneuvers.parking.footprint import (
    footprint_breaches_markers,
    footprint_breaches_wall,
    footprint_inside,
)
from src.navigation.maneuvers.parking.geometry import normalise_angle
from src.navigation.maneuvers.parking.zone import ParkZone, build_zone, staging_pos
from src.navigation.utils import (
    clamp as _clamp,
    pure_pursuit_steer as _shared_pure_pursuit_steer,
)

if TYPE_CHECKING:
    from shared.config.navigation_tuning import NavigationTuning

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class ParkCommand:
    """Motor command from the park controller."""

    linear: float  # m/s
    steering: float  # normalised [-1, 1]
    done: bool = False
    phase: ParkPhase = ParkPhase.STAGE


class ParkController:
    """Two-phase park controller.

    Phase 1 -- STAGE: pure-pursuit toward the staging position in front of the
        bay opening (on the track side).
    Phase 2 -- ENTER: pure-pursuit toward the bay centre; switches to DONE only
        when the whole footprint is inside the lot AND the heading is parallel
        to the wall within tolerance.

    Note: ENTER still pure-pursues a single point, which steers for position
    without controlling the final heading. With the stop condition now requiring
    genuine containment, that is not sufficient to park this chassis -- the
    controller will honestly time out rather than falsely report success. The
    entry maneuver itself is a separate piece of work; see
    ``platform/docs/internal/2026-07-25-parking-review.md``.

    Args:
        parking_config: ParkingLot geometry (block positions).
        start_section: Corridor that contains the parking lot.
        direction: Traversal direction, which selects between the two
            wall-parallel headings so the robot parks facing the way it was
            already travelling.
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
        speed: float | None = None,
        max_frames: int | None = None,
        tuning: NavigationTuning | None = None,
    ) -> None:
        self._tuning = get_tuning(tuning)
        self._context = ParkingContext(self._tuning)
        if speed is None:
            speed = self._tuning.parking.SPEED
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

        self._zone = build_zone(
            parking_config.block1_position,
            parking_config.block2_position,
            start_section,
            direction,
        )
        self._staging = staging_pos(self._zone, start_section, self._context)

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

        CoreNavigator's generic stuck-detector escape is blind to parking geometry
        (the inner keep-out block, the staging point) and can fight this maneuver --
        e.g. its own low-net-displacement check can misfire during a deliberate
        reverse burst that genuinely IS making progress (rotating toward a reachable
        heading) but doesn't move far in a straight line. CoreNavigator should not
        override this with a generic escape while it's active.
        """
        return self._reposition_frames_left > 0

    @property
    def is_done(self) -> bool:
        """Whether the parking maneuver is complete (cleanly or via timeout)."""
        return self._phase is ParkPhase.DONE

    @property
    def is_timed_out(self) -> bool:
        """Whether the maneuver gave up rather than parking cleanly.

        Two ways to give up: exhausting the frame budget, or ENTER reaching the
        field wall without achieving containment (see ``footprint_breaches_wall``).
        Both mean "stopped, not parked", which is the distinction callers actually
        act on.
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
            robot_pose: Current world pose of robot centre (heading 0=east, pi/2=north).

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
        pos_inside, yaw_ok = inside_zone(robot_pose.x, robot_pose.y, robot_pose.yaw, self._zone, self._context)
        if pos_inside and yaw_ok:
            logger.info("ParkController: DONE -- already inside zone")
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

        Shared by both STAGE (toward the staging point) and ENTER (toward the gap
        centre) -- both are the same underlying problem (drive toward a fixed target
        point), and both can hit the same degenerate case: a target that requires a
        sharper turn than the chassis's minimum turning radius allows on a forward
        arc. Real pure pursuit (curvature from lookahead geometry, clamped only at the
        physical steering limit) replaces what used to be a plain bearing-error * kp
        controller -- at this controller's old kp=2.5, *any* bearing error past ~23
        degrees already saturated to full lock, meaning it was never actually
        proportional in practice, just bang-bang. See ``_APPROACH_CLEARANCE``'s
        module-level comment for the full failure trace this fixes.
        """
        # An in-progress reposition runs for its full latched duration rather than
        # being re-evaluated (and potentially cancelled) every tick -- a single-tick
        # reaction flickers in and out without ever creating enough separation to
        # actually escape the degenerate geometry.
        if self._reposition_frames_left > 0:
            self._reposition_frames_left -= 1
            return ParkCommand(linear=self._reposition_speed, steering=self._reposition_steer, phase=phase_name)

        x_local, y_local = robot_pose.to_local_frame(target)

        if x_local < 0:
            # Target is behind the robot: pure pursuit's curvature formula is only
            # valid for a roughly-forward target -- for a rearward one it can produce
            # a plausible-looking (non-saturated) steering command that actually
            # drives away from the target instead of toward it. Reverse immediately
            # rather than trust it.
            return self._start_reposition(robot_pose, target, phase_name, "target behind")

        steer = pure_pursuit_steer(x_local, y_local, self._context)

        if abs(steer) >= self._context.constants.saturated_steer_threshold:
            self._saturated_ticks += 1
        else:
            self._saturated_ticks = 0

        if self._saturated_ticks >= self._context.constants.saturation_stuck_ticks:
            # Steering has been pinned at physical lock for a full second straight:
            # the required curvature genuinely exceeds what the chassis can do going
            # forward (see the module-level comment). Reverse to open room instead of
            # continuing to orbit.
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
        bearing_err = bearing_error(robot_pose, target)
        logger.debug(
            "ParkController: %s reposition (%s, bearing_err=%.1f deg)",
            phase_name,
            reason,
            math.degrees(bearing_err),
        )
        self._saturated_ticks = 0
        self._reposition_frames_left = self._context.constants.default_max_frames
        self._reposition_speed = self._context.constants.reposition_speed
        # Sign-flipped for reverse Ackermann geometry (v<0 inverts the yaw-rate
        # response to a given steer sign), biased toward whichever side the target
        # currently bears.
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
            logger.debug("ParkController: STAGE -> ENTER")
            self._phase = ParkPhase.ENTER
            return self._handle_enter(robot_pose)

        return self._pursue_with_reposition(robot_pose, self._staging, ParkPhase.STAGE)

    def _handle_enter(
        self,
        robot_pose: Pose,
    ) -> ParkCommand:
        z = self._zone
        rx, ry, robot_yaw = robot_pose.x, robot_pose.y, robot_pose.yaw

        pos_inside, yaw_ok = inside_zone(rx, ry, robot_yaw, z, self._context)
        if pos_inside and yaw_ok:
            logger.info("ParkController: DONE -- fully inside the lot, wall-parallel")
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase=ParkPhase.DONE)

        if footprint_breaches_wall(rx, ry, robot_yaw, z):
            logger.warning("ParkController: giving up -- footprint reached the field wall without parking")
            self._timed_out = True
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase=ParkPhase.DONE)

        if footprint_breaches_markers(rx, ry, robot_yaw, z):
            logger.warning("ParkController: giving up -- footprint reached a marker fin without parking")
            self._timed_out = True
            self._phase = ParkPhase.DONE
            return ParkCommand(linear=0.0, steering=0.0, done=True, phase=ParkPhase.DONE)

        return self._pursue_with_reposition(robot_pose, Waypoint(z.gap_cx, z.gap_cy), ParkPhase.ENTER)


def inside_zone(
    rx: float,
    ry: float,
    robot_yaw: float,
    zone: ParkZone,
    context: ParkingContext,
) -> tuple[bool, bool]:
    """Return (fully_parked, parallel_ok) per the WRO parking rule."""
    yaw_err = abs(normalise_angle(robot_yaw - zone.target_yaw))
    return footprint_inside(rx, ry, robot_yaw, zone), yaw_err <= context.constants.yaw_tolerance


def bearing_error(
    robot_pose: Pose,
    target: Waypoint,
) -> float:
    """Signed angle (radians) from the robot's heading to the bearing toward ``target``."""
    return normalise_angle(robot_pose.bearing_to(target.to_pose()) - robot_pose.yaw)


def pure_pursuit_steer(x_local: float, y_local: float, context: ParkingContext) -> float:
    """Aim directly at a single fixed target for ``ParkController``.

    Unlike ``WaypointController`` (which searches a path for a point at a fixed
    lookahead distance), this supplies its own lookahead floor here rather than at
    each call site. See ``src.navigation.utils.pure_pursuit_steer`` for the shared
    formula and its physical reasoning.
    """
    return _shared_pure_pursuit_steer(x_local, y_local, context.constants.min_lookahead_dist_m)


def park_controller_from_metadata(
    metadata: dict,
    start_section: Section,
    direction: Direction | None = None,
    tuning: NavigationTuning | None = None,
) -> ParkController | None:
    """Construct a ParkController from scenario metadata. None for open challenge.

    ``direction`` falls back to the scenario's own ``starting_conditions.direction``
    when not passed explicitly, so callers that already hold it (the sim) and callers
    that only hold the metadata (the ROS2 node) both get the correct wall-parallel
    target heading.

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
