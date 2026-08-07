"""Core Navigator decoupled from ROS2.

This class implements the navigation loop purely in Python using the
HardwareGateway interface. This achieves Dependency Inversion (SOLID),
making the navigation logic trivial to test and execute in a simulator.
"""

from __future__ import annotations

import logging
import math
from dataclasses import replace
from typing import TYPE_CHECKING

from shared.config.constants import CompetitionSpecs, RobotSpecs, TrackDimensions
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Direction, NavigatorPhase, RiskLevel
from shared.domain.models import NavigatorDebugSnapshot

from src.navigation.control.controllers import (
    CollisionAvoidanceController,
    EscapeManeuver,
    ManeuverType,
    StuckDetector,
    WaypointController,
    mask_mapped_obstacles,
)
from src.navigation.planning.waypoints import corridor_for_position
from src.navigation.ports import DriveCommand
from src.navigation.track_geometry import cross_track_error, path_turn_ahead
from src.navigation.utils import _wrap

if TYPE_CHECKING:
    from shared.config.enums import Section

    from src.navigation.maneuvers.parking import ParkController
    from src.navigation.planning.sign_router import SignRouter
    from src.navigation.ports import HardwareGateway, LidarScan
    from src.navigation.race_tracker import LapDetector

logger = logging.getLogger(__name__)


def _outgoing_bearing(waypoints: list[tuple[float, float]], index: int) -> float:
    """Direction the path points at ``index``, toward its next waypoint.

    Wraps to waypoint 0 past the end -- the planned path is one canonical lap
    of a closed loop (see ``replace_path``), not an open segment.
    """
    x0, y0 = waypoints[index]
    x1, y1 = waypoints[(index + 1) % len(waypoints)]
    return math.atan2(y1 - y0, x1 - x0)


class CoreNavigator:
    """Orchestrates navigation using a HardwareGateway interface."""

    def __init__(
        self,
        gateway: HardwareGateway,
        waypoints: list[tuple[float, float]],
        num_laps: int = CompetitionSpecs.OPEN_CHALLENGE_LAPS,
        tuning: NavigationTuning | None = None,
        sign_router: SignRouter | None = None,
        lap_detector: LapDetector | None = None,
        park_controller: ParkController | None = None,
        direction: Direction | None = None,
    ) -> None:
        self._gateway = gateway
        self._waypoints = waypoints
        self._num_laps = num_laps
        self._tuning = tuning or NavigationTuning.load_default()
        self._sign_router = sign_router
        self._lap_detector = lap_detector
        # Best current guess of travel direction, same source LapDetector and
        # the planned path already trust. ``None`` only in the sliver before
        # LIDAR inference settles; see ``set_travel_direction``.
        self._direction = direction

        self._waypoint_index = 0
        self._laps_completed = 0
        self._suppress_next_wrap = False
        self._waypoint_threshold = self._tuning.waypoints.MAIN_LOOP_REACHED_DISTANCE_M
        self._current_corridor: Section | None = None
        self._park_controller = park_controller
        self._parking_engaged = False
        # Must clear the chassis's minimum turning radius with real margin: engaging any
        # closer than that hands ParkController a staging target already inside its own
        # turning circle, which no forward-only steering law can reach (see
        # docs/internal/2026-07-11-navigation-logic-review.md §2.3). Reuses ARC_RADIUS, same
        # as ParkController's own staging stand-off, rather than a disconnected literal.
        #
        # That margin is now much larger than it needs to be. This was sized against a
        # ~0.329 m R_min, computed as WHEELBASE/tan(MAX_STEERING_ANGLE) with the steering
        # limit still modelled at 30 deg. Both inputs were wrong: the real lock is ~70 deg,
        # and counter-phase steering pivots about the chassis centre, so the reference
        # length is WHEELBASE/2. True R_min is ~0.034 m -- an order of magnitude smaller,
        # meaning ARC_RADIUS (0.45 m) is no longer near this constraint and the engage
        # distance could be tightened on its own merits rather than on this one.
        self._park_engage_dist = self._tuning.waypoints.ARC_RADIUS

        # Escape-maneuver latching: an escape runs for its full duration_frames
        # instead of a single 50 ms tick, and repeated escapes escalate (reverse
        # longer, switch side) rather than repeating an identical failed pulse.
        self._active_maneuver: EscapeManeuver | None = None
        self._maneuver_frames_left = 0
        self._escape_count = 0  # escapes begun since the last normal drive tick with real progress
        # Base side for escapes, not a running toggle: which side a given
        # attempt actually uses is derived in _escape_steer_sign_for_attempt.
        self._escape_steer_sign = 1.0
        self._escape_sequence_start_xy: tuple[float, float] | None = None

        # Controllers
        self._waypoint_controller = WaypointController(
            max_steering_angle=RobotSpecs.MAX_STEERING_ANGLE,
            lookahead_short=self._tuning.pursuit.LOOKAHEAD_SHORT,
            lookahead_long=self._tuning.pursuit.LOOKAHEAD_LONG,
            lookahead_transition=self._tuning.pursuit.LOOKAHEAD_TRANSITION,
            steer_kp=self._tuning.pursuit.STEER_KP,
            max_steering_rate=self._tuning.pursuit.MAX_STEERING_RATE,
            waypoint_reached_distance_m=self._tuning.waypoints.CONTROLLER_REACHED_DISTANCE_M,
        )
        self._apply_path_wall_budget()

        self._collision_controller = CollisionAvoidanceController(
            contact_dist=self._tuning.clearance.CONTACT_DIST,
            slow_dist=self._tuning.clearance.SLOW_DIST,
            fast_dist=self._tuning.clearance.FAST_DIST,
            escape_rev_speed=self._tuning.escape.REV_SPEED,
            escape_steer_scale=self._tuning.escape.REV_STEERING_SCALE,
            stuck_threshold=self._tuning.escape.STUCK_MOVE_THRESHOLD,
            path_margin=self._tuning.clearance.PATH_MARGIN,
            k_turn_min_frames=self._tuning.escape.K_TURN_MIN_FRAMES,
            k_turn_max_frames=self._tuning.escape.K_TURN_MAX_FRAMES,
            side_correction_steer=self._tuning.escape.SIDE_CORRECTION_STEER,
            side_correction_speed=self._tuning.escape.SIDE_CORRECTION_SPEED,
            side_correction_frames=self._tuning.escape.SIDE_CORRECTION_FRAMES,
            front_half_fov_deg=self._tuning.lidar_sectors.FRONT_HALF_FOV_DEG,
            threat_half_fov_deg=self._tuning.lidar_sectors.THREAT_HALF_FOV_DEG,
            self_detection_threshold_m=self._tuning.lidar_sectors.SELF_DETECTION_THRESHOLD_M,
            min_valid_range_m=self._tuning.lidar_sectors.MIN_VALID_RANGE_M,
            threat_no_detection_range_m=self._tuning.lidar_sectors.THREAT_NO_DETECTION_RANGE_M,
        )

        self._stuck_detector = StuckDetector(
            move_threshold=self._tuning.escape.STUCK_MOVE_THRESHOLD,
            timeout_frames=self._tuning.escape.STUCK_TIMEOUT_FRAMES,
            confirmation_checks=self._tuning.escape.STUCK_CONFIRMATION_CHECKS,
        )

        # Full internal state of the most recent step(), for telemetry -- see
        # NavigatorDebugSnapshot's own docstring for why this exists.
        self._debug = NavigatorDebugSnapshot()

    @property
    def debug_snapshot(self) -> NavigatorDebugSnapshot:
        """Full internal state of the most recent ``step()`` call."""
        return self._debug

    @property
    def current_corridor(self) -> Section | None:
        """Current track corridor derived from robot position. None before first step."""
        return self._current_corridor

    @property
    def sign_router(self) -> SignRouter | None:
        """The traffic-sign router, or None outside the Obstacles Challenge."""
        return self._sign_router

    def _apply_path_wall_budget(self) -> None:
        """Tell the pursuit controller how much drift this path can absorb.

        Measured off the mat's outer walls rather than the corridor-width
        belief, deliberately. WRO moves the inner walls between rounds but the
        mat's own edges are fixed, so the distance from a waypoint to the
        nearest edge is knowable without believing anything -- and it is the
        outer wall the robot keeps hitting, because an under-estimated corridor
        width biases the planned path toward it (narrow belief -> path at
        ~0.30 where the true centre is 0.50). Deriving the budget from the
        belief instead would make it wrong in exactly the rounds it matters.

        Subtracts the chassis half-width, since the budget is how far the
        *body* may stray, not its centreline.
        """
        if not self._waypoints:
            return
        mat = TrackDimensions.MAX_COORD
        clearance = min(min(x, mat - x, y, mat - y) for x, y in self._waypoints)
        budget = clearance - RobotSpecs.WIDTH / 2 - self._tuning.pursuit.WALL_MARGIN_SAFETY_M
        self._waypoint_controller.set_crosstrack_budget(
            max(budget, self._tuning.pursuit.MIN_LOOKAHEAD_TRANSITION_M),
        )

    def replace_path(
        self,
        waypoints: list[tuple[float, float]],
        robot_xy: tuple[float, float],
        robot_yaw: float | None = None,
    ) -> None:
        """Swap in a new planned path mid-run, resuming at the nearest point.

        Needed when the layout the path was planned against turns out to be
        wrong — the robot estimates corridor widths from LIDAR as it drives
        (see :mod:`src.navigation.corridor_estimator`), so the path has to be
        rebuilt when an estimate changes rather than being fixed at startup.

        The waypoint index cannot carry over: the new path has its own indexing
        and the old index would point somewhere arbitrary on it. Re-seeking to
        the nearest waypoint keeps progress instead of restarting the lap, and
        matters because the paths differ by centimetres, not corridors — the
        nearest point on the new path is essentially where the robot already
        was on the old one.

        Nearest-by-position alone can go wrong right after a blind round's
        direction inference commits: near a corner, several waypoints sit at
        almost the same distance while pointing in very different directions,
        and the robot's heading at that instant does not always match the
        path's local direction there yet (it has been steering, within the
        blind corridor-follower's own limits, during the creep leading up to
        the commit). Picking purely by position can then hand
        :class:`~src.navigation.control.controllers.WaypointController` a
        point past the turn, demanding a correction far larger than finishing
        the corner needs — measured on real hardware as a ~193 deg swing
        where ~90 deg would do. Passing ``robot_yaw`` re-ranks the
        near-tied-by-distance candidates (see
        ``NavigationTuning.waypoints.REPLAN_HEADING_TIE_MARGIN_M``) by
        heading agreement instead.

        Args:
            waypoints: The replacement path (single canonical lap).
            robot_xy: Current position, used to resume at the nearest waypoint.
            robot_yaw: Current heading (radians), if known. When given, breaks
                near-ties in the position search by heading agreement instead
                of taking the strict nearest. Omit where the robot has been
                tracking a path very similar to the new one (e.g. a small
                corridor-width belief update), where nearest-by-position alone
                is already safe.
        """
        previous_index = self._waypoint_index
        self._waypoints = waypoints
        self._apply_path_wall_budget()
        robot_x, robot_y = robot_xy
        distances = [math.hypot(wx - robot_x, wy - robot_y) for wx, wy in waypoints]
        nearest_index = min(range(len(waypoints)), key=lambda i: distances[i])

        if robot_yaw is not None:
            margin = distances[nearest_index] + self._tuning.waypoints.REPLAN_HEADING_TIE_MARGIN_M
            candidates = [i for i, d in enumerate(distances) if d <= margin]
            nearest_index = min(
                candidates,
                key=lambda i: abs(_wrap(_outgoing_bearing(waypoints, i) - robot_yaw)),
            )

        self._waypoint_index = nearest_index

        # A large forward jump is never earned progress — the robot cannot skip
        # most of a lap between two ticks. It means the re-seek landed on the
        # far side of the start/finish seam: the path was rebuilt running the
        # other way, leaving the robot just *behind* the new waypoint 0, which
        # on a closed loop is also the tail of the list. The tail it is about to
        # drive belongs to a lap it never ran, so the wrap at the end of it is
        # the entry into lap 1, not the completion of it.
        #
        # A correct direction inference never lands here: the robot creeps
        # forward along its corridor, so its heading already matches the true
        # travel direction and the rebuilt path runs the way it is pointing.
        # This is a guard against a *wrong* inference, which on the mat is the
        # case the robot cannot rule out for itself.
        if self._waypoint_index - previous_index > len(waypoints) // 2:
            self._suppress_next_wrap = True

    def replace_park_controller(self, park_controller: ParkController | None) -> None:
        """Swap in a fresh ParkController ahead of a new race.

        ParkController's phase only moves forward (STAGE -> ENTER -> DONE),
        so a finished or timed-out one from the previous race can't be
        rewound in place -- the caller builds a new one from the same
        section/direction/metadata it used the first time and hands it here.
        """
        self._park_controller = park_controller
        self._parking_engaged = False

    def reset(self) -> None:
        """Clear per-race state so a new race starts as if this were the first.

        Needed because the state machine can cycle FINISHED -> BOOT_CHECK ->
        READY -> RACING purely from the physical button (two long presses and
        a short one), with no process restart -- so nothing else re-creates
        this object between races. Without this, ``_laps_completed`` alone
        would stay at its previous value and the very first tick of the new
        race would immediately read as already finished.
        """
        self._waypoint_index = 0
        self._laps_completed = 0
        self._suppress_next_wrap = False
        self._parking_engaged = False
        self._active_maneuver = None
        self._maneuver_frames_left = 0
        self._escape_count = 0
        self._escape_steer_sign = 1.0
        self._escape_sequence_start_xy = None
        self._stuck_detector.reset()
        self._waypoint_controller.reset()
        if self._sign_router is not None:
            self._sign_router.reset_for_new_lap()

    def replace_lap_detector(self, lap_detector: LapDetector) -> None:
        """Swap in a lap detector built for a different travel direction.

        The finish line's normal is the travel direction, so a detector built
        for the wrong one counts crossings with the sign inverted. Blind rounds
        infer the direction from LIDAR after they have started driving (see
        :mod:`src.navigation.direction_estimator`), so the detector that was
        provisional at startup has to be replaced once the answer arrives.

        Only valid before any lap has been counted, which is guaranteed here:
        the direction settles inside the first metre of travel, long before the
        finish line is re-crossed.
        """
        self._lap_detector = lap_detector

    def set_travel_direction(self, direction: Direction) -> None:
        """Adopt the (re-)inferred travel direction.

        Called alongside ``replace_lap_detector`` whenever inference settles or
        revises its answer. Consumed by the collision-avoidance escape maneuver
        as the fallback side when a LIDAR-only clearance comparison cannot
        decide one (see ``CollisionAvoidanceController._k_turn_steer_sign``).
        """
        self._direction = direction

    @property
    def laps_completed(self) -> int:
        """Number of laps confirmed completed so far."""
        return self._laps_completed

    def _base_debug(self, robot_x: float | None, robot_y: float | None, robot_yaw: float | None) -> NavigatorDebugSnapshot:
        """Fields available on every phase once pose is known -- the common
        prefix every ``step()`` branch's snapshot builds on."""
        return NavigatorDebugSnapshot(
            pose_x=robot_x,
            pose_y=robot_y,
            pose_yaw=robot_yaw,
            direction=self._direction,
            current_corridor=self._current_corridor,
            waypoint_index=self._waypoint_index,
            laps_completed=self._laps_completed,
            num_laps=self._num_laps,
            parking_engaged=self._parking_engaged if self._park_controller is not None else None,
        )

    def step(self) -> None:
        """Execute one control step.

        Pulls current state from the gateway, calculates commands,
        and pushes them back to the gateway.
        """
        pose = self._gateway.get_current_pose()
        if not pose:
            # No localisation available (startup or sensor dropout): stop rather
            # than coast on the last published command.
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            self._debug = NavigatorDebugSnapshot(
                phase=NavigatorPhase.NO_POSE,
                commanded_speed_mps=0.0,
                commanded_steering_norm=0.0,
            )
            return

        robot_x, robot_y = pose.x, pose.y
        robot_yaw = pose.yaw

        self._current_corridor = corridor_for_position(robot_x, robot_y)

        # Continue an in-progress escape maneuver until its latched duration
        # elapses, so escapes are real motions rather than single-tick pulses that
        # never clear the wall.
        if self._active_maneuver is not None:
            self._drive_active_maneuver(robot_x, robot_y, robot_yaw, phase=NavigatorPhase.ACTIVE_MANEUVER)
            return

        # Update stuck detector — runs while actively driving OR maneuvering
        # into the parking gap, but not once the robot has reached its final
        # deliberate stop (open-challenge hold, or parking done): otherwise a
        # robot correctly holding position at zero velocity would eventually
        # read as "stuck" and reverse itself back out of a completed park.
        # Also suspended while ParkController is mid reverse-and-reorient recovery
        # (see ParkController.is_repositioning) — that maneuver is itself a deliberate,
        # low-net-displacement reverse burst, and the generic escape it would otherwise
        # trigger is blind to the inner keep-out block ParkController is navigating around.
        # Reset (not just skip) during repositioning: otherwise the history queue still
        # spans across the gap, and the first check afterward compares a newest position
        # against an oldest one from well before the reposition started — reintroducing
        # the same false "stuck" trigger one tick later instead of preventing it.
        pc = self._park_controller
        if pc is not None and pc.is_repositioning:
            self._stuck_detector.reset()
        elif not self._is_holding():
            self._stuck_detector.update((robot_x, robot_y))
            if self._stuck_detector.is_stuck:
                self._handle_stuck_escape(robot_x, robot_y, robot_yaw)
                return

        # Lap completion: defer the parking handoff until the robot is actually
        # in the parking corridor and within reach of the staging point. Until
        # then keep navigating so the handoff never fires mid-corridor.
        if self._laps_completed >= self._num_laps and self._handle_finish(
            robot_x,
            robot_y,
            robot_yaw,
        ):
            return

        # Waypoint-wrap detection: signal LapDetector and reset index.
        if self._waypoint_index >= len(self._waypoints):
            self._waypoint_index = 0
            self._stuck_detector.reset()
            if self._suppress_next_wrap:
                # Seeded past the seam by replace_path, not driven — see there.
                self._suppress_next_wrap = False
            elif self._lap_detector is not None:
                self._lap_detector.notify_waypoint_wrapped()
            else:
                # Fallback: no geometric guard — count directly.
                self._laps_completed += 1
                logger.info("Lap %d complete (waypoint-only fallback)", self._laps_completed)
                if self._sign_router is not None:
                    self._sign_router.reset_for_new_lap()
                self._debug = self._base_debug(robot_x, robot_y, robot_yaw)
                self._debug.phase = NavigatorPhase.WAYPOINT_WRAP_FALLBACK
                return

        # Geometric lap counting (requires LapDetector).
        if (
            self._lap_detector is not None
            and self._current_corridor is not None
            and self._lap_detector.update((robot_x, robot_y), self._current_corridor)
        ):
            self._laps_completed += 1
            logger.info("Lap %d complete (geometric + waypoint confirmed)", self._laps_completed)
            if self._sign_router is not None:
                self._sign_router.reset_for_new_lap()

        raw_wp = self._waypoints[self._waypoint_index]

        # Advance past any waypoint the robot has already gone by — not just
        # one it happens to pass within waypoint_threshold of. A robot that
        # starts somewhere other than exactly on the planned centerline (e.g.
        # anywhere in an official starting zone) can curve past a waypoint
        # without ever entering that radius; left un-advanced, the lookahead
        # search below keeps re-targeting that same, increasingly stale point
        # long after the robot has passed it. Once far enough away, that
        # stale point can itself satisfy the search's lookahead-distance test
        # and get selected as the steering target even though it's now behind
        # the robot, spiraling the heading around to chase a point it already
        # passed instead of the path ahead.
        #
        # The comparison wraps around the seam, so the *last* waypoint gets the
        # same pass-by rescue as every other one. It used to stop at
        # ``index + 1 < len``, which left entering a MAIN_LOOP_REACHED_DISTANCE_M
        # circle as the only way past the final point — and a robot running
        # wider than that radius never gets past it, never wraps, and so never
        # completes a lap no matter how many times it drives the loop. Measured
        # on the 2026-08-06 counterclockwise round: crosstrack ran 0.26-0.51 m
        # against a 0.20 m radius, the index froze on the last waypoint at
        # t=90s, and the robot circled the mat for a further 7 minutes with the
        # lap count stuck at zero. Advancing to ``len(waypoints)`` here is the
        # same state reaching the last waypoint produces, and the wrap branch
        # at the top of the next tick is what turns it into a counted lap.
        count = len(self._waypoints)
        for _ in range(count):
            next_index = self._waypoint_index + 1
            next_wp = self._waypoints[next_index % count]
            if math.hypot(next_wp[0] - robot_x, next_wp[1] - robot_y) >= math.hypot(
                raw_wp[0] - robot_x, raw_wp[1] - robot_y
            ):
                break
            self._waypoint_index = next_index
            if next_index >= count:
                # Seam crossed. Leave raw_wp on the final waypoint and let the
                # wrap branch count the lap next tick — walking on into the new
                # lap here would skip waypoints the wrap is about to rewind to.
                break
            raw_wp = next_wp

        # Get LIDAR scan from gateway
        scan = self._gateway.get_lidar_scan()
        if scan:
            forward_clearance = self._collision_controller.compute_forward_clearance(
                scan.ranges_m,
                scan.angles_rad,
            )
            risk = self._collision_controller.assess_risk(scan.ranges_m, scan.angles_rad)
        else:
            # No LIDAR: a degraded sensor is not open road. Drive cautiously
            # (slow zone + non-SAFE risk) instead of blasting forward blind.
            forward_clearance = self._tuning.clearance.SLOW_DIST
            risk = RiskLevel.OBSTACLE

        # Two risk readings, deliberately: the RAW scan governs how fast the
        # robot may go, the MAPPED-OBSTACLE-MASKED scan governs whether the
        # escape maneuver fires.
        #
        # A sign the router is routing around is passed at ``lateral_offset``
        # centre-to-centre by design, which is inside CONTACT_DIST — so on the
        # raw scan the escape maneuver fires at every sign pass and reverses the
        # robot out of the very gap the planner aimed for, deciding the run
        # before the router's aim can matter. Withholding those returns from the
        # escape trigger alone keeps the split honest: the robot still slows
        # down for a sign (raw ``risk`` caps speed below), it just no longer
        # panics at one the planner is already handling. Walls and unmapped
        # returns are untouched in both readings.
        escape_ranges = scan.ranges_m if scan else None
        escape_risk = risk
        if scan and self._sign_router is not None:
            escape_ranges = mask_mapped_obstacles(
                scan.ranges_m,
                scan.angles_rad,
                (robot_x, robot_y, robot_yaw),
                self._sign_router.routed_sign_positions,
                self._tuning.sign_router.ESCAPE_MASK_RADIUS_M,
            )
            escape_risk = self._collision_controller.assess_risk(escape_ranges, scan.angles_rad)

        # Check waypoint reached — against the *raw* planned point, not the
        # sign-deformed one: deformation only biases steering near a sign, it
        # must never stall path progression. A sign can pull the steering
        # target sideways by up to lateral_offset, so the robot's real
        # trajectory may never pass within waypoint_threshold of the deformed
        # point — checking that point would freeze waypoint_index indefinitely
        # while the sign stays engaged, corrupting every later tick's lookahead
        # search with a stale target.
        dist_to_wp = math.hypot(raw_wp[0] - robot_x, raw_wp[1] - robot_y)
        if dist_to_wp < self._waypoint_threshold:
            self._waypoint_index += 1
            self._debug = self._base_debug(robot_x, robot_y, robot_yaw)
            self._debug.phase = NavigatorPhase.WAYPOINT_REACHED
            self._debug.forward_clearance_m = forward_clearance
            self._debug.min_lidar_range_m = min(scan.ranges_m) if scan and scan.ranges_m else None
            self._debug.risk = risk.value
            self._debug.escape_risk = escape_risk.value
            return

        # Steer at a lookahead point, not directly at the (often much closer)
        # next waypoint — otherwise the lookahead distance is computed but
        # discarded, producing weave on straights and corner cutting (PP-1).
        #
        # Lookahead selection itself is gated on crosstrack error (how far off
        # the planned path the robot actually is), not forward LIDAR clearance
        # -- see WaypointController.select_lookahead's docstring for why that
        # mismatch produced slow, lazy cornering and weak centering on real
        # hardware once the steering law became curvature-based (2026-08-03).
        crosstrack = cross_track_error(self._waypoints, robot_x, robot_y)
        # Crosstrack alone arms the short lookahead only after a corner has
        # been missed; the path's own upcoming turn arms it on entry. See
        # select_lookahead's docstring for the hardware measurement behind it.
        turn_ahead = path_turn_ahead(
            self._waypoints,
            self._waypoint_index,
            self._tuning.pursuit.CORNER_PREVIEW_DISTANCE_M,
        )
        lookahead_distance = self._waypoint_controller.select_lookahead(crosstrack, turn_ahead)
        # Full waypoint list, not a slice from _waypoint_index -- select_target_point
        # wraps the search around the lap itself now (see its docstring); slicing here
        # would cut that wraparound off right back out again.
        steer_target = self._waypoint_controller.select_target_point(
            current_pos=(robot_x, robot_y),
            current_yaw=robot_yaw,
            waypoints=self._waypoints,
            waypoint_index=self._waypoint_index,
            lookahead_distance=lookahead_distance,
        )

        # Apply sign routing to whichever point steering will actually chase —
        # deforming a raw-path *candidate* before the lookahead search picked
        # from it meant the search itself, not the sign, decided whether the
        # nudge ever reached steering (it almost never did: waypoints are
        # spaced well under the 0.20-0.40m lookahead, so the search kept
        # skipping past a single deformed candidate to a further, undeformed
        # one). Deforming the search's own output guarantees the bias is
        # exactly what gets steered toward, at full tapered strength whenever
        # that point is close to the sign.
        sign_deform_magnitude: float | None = None
        active_sign_count: int | None = None
        if self._sign_router is not None and self._current_corridor is not None:
            observations = self._gateway.get_vision_detections()
            raw_target = steer_target
            steer_target = self._sign_router.deform_waypoint(
                waypoint=steer_target,
                robot_pos=(robot_x, robot_y),
                robot_yaw=robot_yaw,
                corridor=self._current_corridor,
                observations=observations,
            )
            sign_deform_magnitude = math.hypot(steer_target[0] - raw_target[0], steer_target[1] - raw_target[1])
            active_sign_count = self._sign_router.active_sign_count

        # Get steering from waypoint controller
        steering_normalized, _, angle_error = self._waypoint_controller.compute_steering(
            current_pos=(robot_x, robot_y),
            current_yaw=robot_yaw,
            target_waypoint=steer_target,
            crosstrack_error=crosstrack,
        )

        # Determine speed
        if forward_clearance < self._tuning.clearance.CONTACT_DIST:
            speed = self._tuning.speed.CREEP_SPEED
        elif forward_clearance < self._tuning.clearance.SLOW_DIST:
            speed = self._tuning.speed.SLOW_SPEED
        elif forward_clearance < self._tuning.clearance.MEDIUM_DIST:
            speed = self._tuning.speed.MEDIUM_SPEED
        else:
            speed = self._tuning.speed.FAST_SPEED

        # Never take a sharp turn at a speed the steering actuator can't keep
        # up with. The steering servo has a fixed slew rate (MAX_STEERING_RATE)
        # independent of how fast the chassis is moving, so a big required
        # heading correction taken at full speed demands a yaw rate the
        # actuator cannot track -- it saturates, overshoots, and oscillates
        # instead of settling (measured on real hardware 2026-08-03, see
        # docs/internal/audits/2026-08-03-realtrack-control-instability-findings.md).
        # Clearance alone never catches this: a corner can have 0.50m+ of open
        # space ahead while still demanding a 90-180 deg correction.
        abs_error = abs(angle_error)
        if abs_error >= self._tuning.heading.CRAWL:
            heading_speed = self._tuning.speed.CREEP_SPEED
        elif abs_error >= self._tuning.heading.SLOW:
            heading_speed = self._tuning.speed.SLOW_SPEED
        elif abs_error >= self._tuning.heading.MEDIUM:
            heading_speed = self._tuning.speed.MEDIUM_SPEED
        else:
            heading_speed = self._tuning.speed.FAST_SPEED
        speed = min(speed, heading_speed)

        # Bound the selected cruise speed by the configured envelope. MIN_SPEED
        # and MAX_SPEED read like hard limits on the robot and were enforced
        # nowhere: a profile setting FAST_SPEED above MAX_SPEED was simply
        # obeyed. A no-op at the shipped values (CREEP == MIN_SPEED, FAST ==
        # MAX_SPEED), which is the point -- it constrains mis-tuned profiles
        # without changing this one.
        #
        # Deliberately applied HERE, to a zone speed that is always positive,
        # and not to the final command: clamping that up to MIN_SPEED would turn
        # every legitimate stop (escape hand-off, park complete, blocked at both
        # ends) into a 0.05 m/s crawl the robot cannot be commanded out of.
        speed = min(max(speed, self._tuning.speed.MIN_SPEED), self._tuning.speed.MAX_SPEED)

        # Never blast past a non-forward obstacle (e.g. a sign alongside the
        # robot) just because the path ahead is clear.
        if risk != RiskLevel.SAFE:
            speed = min(speed, self._tuning.speed.SLOW_SPEED)

        # Snapshot everything decided so far -- both the escape-trigger branch
        # below and the normal publish at the end of this method share it, only
        # differing in phase and the final command actually sent.
        debug = self._base_debug(robot_x, robot_y, robot_yaw)
        debug.forward_clearance_m = forward_clearance
        debug.min_lidar_range_m = min(scan.ranges_m) if scan and scan.ranges_m else None
        debug.risk = risk.value
        debug.escape_risk = escape_risk.value
        debug.crosstrack_error_m = crosstrack
        debug.lookahead_distance_m = lookahead_distance
        debug.path_turn_ahead_rad = turn_ahead
        debug.steer_target_x = steer_target[0]
        debug.steer_target_y = steer_target[1]
        debug.angle_error_rad = angle_error
        debug.clearance_speed_mps = speed
        debug.heading_speed_mps = heading_speed
        debug.sign_deform_magnitude_m = sign_deform_magnitude
        debug.active_sign_count = active_sign_count

        # Escape maneuvers if critical — judged on the masked scan, so a mapped
        # sign cannot trigger one, and steered by the masked scan too: the
        # threat this escape is running from is by construction not the sign.
        if escape_risk == RiskLevel.CRITICAL and scan:
            threat_dir = self._collision_controller.detect_threat_direction(escape_ranges, scan.angles_rad)
            maneuver = self._collision_controller.compute_escape_maneuver(
                escape_risk,
                threat_dir,
                escape_ranges,
                scan.angles_rad,
                self._direction,
            )
            # Rear clearance is checked against the RAW scan: a sign behind the
            # robot is still something to not reverse into, whoever owns it.
            if maneuver and self._reversing_into_unseen_wall(maneuver, scan):
                # Blocked at both ends: fall through to the capped creep-speed
                # publish below rather than backing into an unseen wall. The
                # stuck detector is the backstop if the robot truly can't move.
                maneuver = None
            if maneuver:
                if self._escape_count == 0:
                    self._escape_sequence_start_xy = (robot_x, robot_y)
                self._escape_count += 1
                self._begin_maneuver(self._maybe_escalate(maneuver))
                self._debug = debug
                self._drive_active_maneuver(robot_x, robot_y, robot_yaw, phase=NavigatorPhase.ESCAPE_TRIGGERED)
                return

        # Normal publish — clear the escape escalation, but only once the
        # robot has actually moved since the sequence started. A single
        # normal_drive tick between escape attempts doesn't mean the escape
        # worked -- confirmed on real hardware 2026-08-04 (run_20260804_213147):
        # normal_drive -> escape_triggered alternated for 34+ seconds with the
        # robot pinned in place, and this unconditional reset zeroed
        # escape_count every single cycle, so it never reached
        # ESCALATE_AFTER_ATTEMPTS and _maybe_escalate never fired. Requiring
        # real displacement first means a genuinely stuck sequence keeps
        # accumulating toward escalation instead of resetting on every tick
        # that merely classifies as "not critical" for one frame.
        if self._escape_sequence_start_xy is None or math.hypot(
            robot_x - self._escape_sequence_start_xy[0],
            robot_y - self._escape_sequence_start_xy[1],
        ) >= self._tuning.escape.STUCK_MOVE_THRESHOLD:
            self._escape_count = 0
            self._escape_sequence_start_xy = None
        self._gateway.publish_drive(DriveCommand(speed_mps=speed, steering_norm=steering_normalized))
        debug.phase = NavigatorPhase.NORMAL_DRIVE
        debug.commanded_speed_mps = speed
        debug.commanded_steering_norm = steering_normalized
        debug.escape_count = self._escape_count
        self._debug = debug

    def _reversing_into_unseen_wall(self, maneuver: EscapeManeuver, scan: LidarScan) -> bool:
        """True if executing ``maneuver`` would back into a wall behind the robot."""
        if maneuver.speed >= 0:
            return False
        rear_clear = self._collision_controller.compute_rear_clearance(scan.ranges_m, scan.angles_rad)
        return rear_clear < self._tuning.clearance.CONTACT_DIST

    def _begin_maneuver(self, maneuver: EscapeManeuver) -> None:
        """Latch an escape maneuver so it executes for its full duration."""
        self._active_maneuver = maneuver
        self._maneuver_frames_left = max(1, maneuver.duration_frames)
        # An escape maneuver drives steering directly, bypassing pure pursuit.
        # Clear the rate-limit memory so pure pursuit doesn't rate-limit its
        # first post-maneuver command against a stale pre-maneuver angle.
        self._waypoint_controller.reset()

    def _drive_active_maneuver(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        phase: NavigatorPhase,
    ) -> None:
        """Publish the active escape command and count down its latched duration."""
        maneuver = self._active_maneuver
        if maneuver is None:
            return
        self._maneuver_frames_left -= 1
        if self._maneuver_frames_left <= 0:
            self._active_maneuver = None
        self._gateway.publish_drive(DriveCommand(speed_mps=maneuver.speed, steering_norm=maneuver.steering))
        debug = self._base_debug(robot_x, robot_y, robot_yaw)
        debug.phase = phase
        debug.active_maneuver_type = maneuver.maneuver_type.value
        debug.maneuver_steering = maneuver.steering
        debug.maneuver_speed_mps = maneuver.speed
        debug.maneuver_frames_left = self._maneuver_frames_left
        debug.escape_count = self._escape_count
        debug.commanded_speed_mps = maneuver.speed
        debug.commanded_steering_norm = maneuver.steering
        self._debug = debug

    def _escape_steer_sign_for_attempt(self, first_attempt: int = 1, start_sign: float | None = None) -> float:
        """Which side this escape attempt swings toward.

        Derived from ``_escape_count`` rather than flipped in place, so a side
        is held for ``ESCAPE_SIDE_COMMIT_ATTEMPTS`` consecutive attempts before
        the other is tried. Flipping on every attempt (which all three escape
        paths used to do independently) means consecutive attempts rotate the
        chassis in opposite directions and undo each other: measured on real
        hardware 2026-08-05 (run_20260805_200011) as four escalating escapes
        over 40 s that rocked the yaw between -0.4 and -0.8 rad and translated
        the robot exactly nowhere. Escaping a wedge needs several attempts
        pushing the *same* way to accumulate; alternating guarantees they
        cannot.

        ``_escape_steer_sign`` is the base side, not a running toggle -- the
        blocks alternate around it.

        Args:
            first_attempt: The ``_escape_count`` at which this caller's sequence
                begins, so its blocks line up with it. Anchoring every caller at
                1 instead leaves whichever attempt a caller actually starts on
                stranded mid-block, and a block of one is the alternating
                behaviour this exists to stop.
            start_sign: Side for the sequence's first block, defaulting to the
                base. Escalation passes the opposite, since switching sides is
                the point of escalating.
        """
        base = self._escape_steer_sign if start_sign is None else start_sign
        commit = max(1, self._tuning.escape.ESCAPE_SIDE_COMMIT_ATTEMPTS)
        block = max(0, self._escape_count - first_attempt) // commit
        return base if block % 2 == 0 else -base

    def _maybe_escalate(self, maneuver: EscapeManeuver) -> EscapeManeuver:
        """Escalate a repeated escape instead of repeating an identical pulse.

        After a few consecutive escapes that clearly aren't working, reverse for
        longer and swing toward the opposite side, so the robot stops slamming
        the same failing maneuver into the same wall.
        """
        if self._escape_count <= self._tuning.escape.ESCALATE_AFTER_ATTEMPTS:
            return maneuver
        side = self._escape_steer_sign_for_attempt(
            first_attempt=self._tuning.escape.ESCALATE_AFTER_ATTEMPTS + 1,
            start_sign=-self._escape_steer_sign,
        )
        steering = abs(maneuver.steering) * side if maneuver.steering else 0.0
        return replace(
            maneuver,
            steering=steering,
            duration_frames=min(maneuver.duration_frames * 2, self._tuning.escape.MAX_ESCAPE_FRAMES),
        )

    def _is_holding(self) -> bool:
        """True once the robot has reached a deliberate, terminal stop.

        Both terminal states are monotonic (never revert once reached), so it
        is safe to permanently stop running stuck detection once this is True.
        """
        if self._laps_completed < self._num_laps:
            return False
        pc = self._park_controller
        return pc is None or pc.is_done

    def _handle_finish(self, robot_x: float, robot_y: float, robot_yaw: float) -> bool:
        """Handle the post-final-lap phase.

        Returns True if a command was issued (caller should stop this tick);
        False if the robot should keep navigating toward the parking corridor.
        """
        pc = self._park_controller
        if pc is None:
            # Open challenge: no parking maneuver — hold position.
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            debug = self._base_debug(robot_x, robot_y, robot_yaw)
            debug.phase = NavigatorPhase.FINISHED_HOLD
            debug.commanded_speed_mps = 0.0
            debug.commanded_steering_norm = 0.0
            self._debug = debug
            return True

        if not self._parking_engaged and self._should_engage_parking(robot_x, robot_y):
            logger.info("Parking engaged in corridor %s", self._current_corridor)
            self._parking_engaged = True

        if not self._parking_engaged:
            return False  # Keep navigating until at the staging point.

        if pc.is_done:
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            debug = self._base_debug(robot_x, robot_y, robot_yaw)
            debug.phase = NavigatorPhase.FINISHED_HOLD
            debug.commanded_speed_mps = 0.0
            debug.commanded_steering_norm = 0.0
            self._debug = debug
            return True

        cmd = pc.update((robot_x, robot_y), robot_yaw)
        linear = cmd.linear
        scan = self._gateway.get_lidar_scan()
        if scan:
            # Forward-clearance gate so the staging vector never drives into a wall head-on.
            fwd = self._collision_controller.compute_forward_clearance(scan.ranges_m, scan.angles_rad)
            # Full-sweep gate (all 360°) so any maneuver that swings the chassis sideways or
            # threads a tight gap (ParkController's STAGE arc/reposition, or its ENTER
            # approach into the block gap) is stopped before ANY-direction clip that the
            # narrow forward cone alone would never see coming -- parking geometry can clip
            # a wall or block edge from the side or even slightly behind the direction of
            # travel, not just from in front.
            #
            # CONTACT_DIST alone isn't a safe threshold here: LIDAR range is measured from
            # roughly the chassis centre, but the chassis itself extends up to WIDTH/2
            # (0.10m) or LENGTH/2 (0.15m) from that centre depending on bearing -- a raw
            # centre-to-obstacle reading of CONTACT_DIST can already mean the footprint
            # edge, not just the sensor, has reached the obstacle. Pad by the chassis
            # half-width so the gate reacts while there's still real clearance left.
            #
            # CONTACT_DIST alone isn't a safe threshold here: LIDAR range is measured from
            # roughly the chassis centre, but the chassis itself extends up to WIDTH/2
            # (0.10m) or LENGTH/2 (0.15m) from that centre depending on bearing -- a raw
            # centre-to-obstacle reading of CONTACT_DIST can already mean the footprint
            # edge, not just the sensor, has reached the obstacle. Pad by the chassis
            # half-width so the gate reacts while there's still real clearance left.
            #
            # This applies in both phases, including ENTER: a smaller pad there still let
            # the chassis clip a block edge in testing (the WRO-regulation gap is only
            # ~4cm wider than the chassis per side, tighter than this controller's approach
            # precision can reliably guarantee). Full padding means ENTER can stall short of
            # a clean park (see ParkController's own max_frames give-up) rather than thread
            # the gap in every case -- a known, documented limitation, not a silent one.
            # Not colliding takes priority over completing the maneuver.
            side = self._collision_controller.compute_min_clearance(
                scan.ranges_m,
                scan.angles_rad,
                half_fov_rad=math.pi,
            )
            side_margin = self._tuning.clearance.CONTACT_DIST + RobotSpecs.WIDTH / 2
            if fwd < self._tuning.clearance.CONTACT_DIST or side < side_margin:
                linear = 0.0
        self._gateway.publish_drive(DriveCommand(speed_mps=linear, steering_norm=cmd.steering))
        debug = self._base_debug(robot_x, robot_y, robot_yaw)
        debug.phase = NavigatorPhase.PARKING
        debug.park_phase = str(cmd.phase)
        debug.commanded_speed_mps = linear
        debug.commanded_steering_norm = cmd.steering
        self._debug = debug
        return True

    def _should_engage_parking(self, robot_x: float, robot_y: float) -> bool:
        """Engage parking only once in the parking corridor and near staging."""
        pc = self._park_controller
        if pc is None:
            return False
        if self._current_corridor is not None and self._current_corridor != pc.section:
            return False
        sx, sy = pc.staging
        return math.hypot(sx - robot_x, sy - robot_y) < self._park_engage_dist

    def _handle_stuck_escape(self, robot_x: float, robot_y: float, robot_yaw: float) -> None:
        """Reverse out of a stuck state, but never back into an unseen wall.

        The reverse is latched for several frames (escalating with repeated
        attempts) and switches steering side only after committing to one for
        several attempts (see ``_escape_steer_sign_for_attempt``), so a
        wall-pinned robot actually backs away instead of twitching one
        centimetre every few seconds forever.

        When reverse itself is blocked (wedged both front and rear -- a real
        corner, or a moderate turn normal_drive's own curvature-based
        steering isn't decisive enough to complete at creep speed), this used
        to just hold and reset the stuck detector, over and over, forever:
        confirmed on real hardware 2026-08-04 as a robot frozen at the same
        position for 27s straight, is_stuck firing repeatedly and each time
        just re-arming the same forward command that had already failed for
        the previous window (see docs/known-issues-backlog.md). Holding is
        only actually the safe choice when forward is *also* blocked; when
        it isn't, a forward creep at full steering lock (same side-commit and
        escalation pattern as the reverse case) gives the
        robot a real chance to walk itself clear using more decisive
        steering than normal_drive's own pure-pursuit curvature was willing
        to command for this same geometry.
        """
        logger.warning("Robot stuck - triggering escape")
        stuck_diag = self._stuck_detector.get_diagnostics()
        rear_clear = 10.0
        forward_clear = 10.0
        scan = self._gateway.get_lidar_scan()
        if scan:
            rear_clear = self._collision_controller.compute_rear_clearance(
                scan.ranges_m,
                scan.angles_rad,
            )
            forward_clear = self._collision_controller.compute_forward_clearance(
                scan.ranges_m,
                scan.angles_rad,
            )
        if rear_clear < self._tuning.clearance.CONTACT_DIST:
            if forward_clear >= self._tuning.clearance.CONTACT_DIST:
                logger.warning(
                    "Stuck escape: rear blocked (%.2f m), forward clear (%.2f m) - forcing forward escape",
                    rear_clear,
                    forward_clear,
                )
                if self._escape_count == 0:
                    self._escape_sequence_start_xy = (robot_x, robot_y)
                self._escape_count += 1
                frames = min(
                    self._tuning.escape.K_TURN_MIN_FRAMES
                    + self._tuning.escape.STUCK_ESCALATION_FRAMES_PER_ATTEMPT * (self._escape_count - 1),
                    self._tuning.escape.MAX_ESCAPE_FRAMES,
                )
                steering = self._tuning.escape.REV_STEERING_SCALE * self._escape_steer_sign_for_attempt()
                self._begin_maneuver(
                    EscapeManeuver(
                        maneuver_type=ManeuverType.STUCK_FORWARD,
                        steering=steering,
                        speed=self._tuning.speed.CREEP_SPEED,
                        duration_frames=frames,
                    ),
                )
                self._stuck_detector.reset()
                self._drive_active_maneuver(robot_x, robot_y, robot_yaw, phase=NavigatorPhase.STUCK_ESCAPE_MANEUVER)
                self._debug.is_stuck = bool(stuck_diag["is_stuck"])
                self._debug.stuck_count = int(stuck_diag["stuck_count"])
                self._debug.recent_movement_m = float(stuck_diag["recent_movement"])
                self._debug.rear_clearance_m = rear_clear
                self._debug.forward_clearance_m = forward_clear
                return
            logger.warning(
                "Stuck escape blocked: rear clearance %.2f m, forward clearance %.2f m - holding",
                rear_clear,
                forward_clear,
            )
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            self._stuck_detector.reset()
            debug = self._base_debug(robot_x, robot_y, robot_yaw)
            debug.phase = NavigatorPhase.STUCK_ESCAPE_HOLDING
            debug.is_stuck = bool(stuck_diag["is_stuck"])
            debug.stuck_count = int(stuck_diag["stuck_count"])
            debug.recent_movement_m = float(stuck_diag["recent_movement"])
            debug.rear_clearance_m = rear_clear
            debug.forward_clearance_m = forward_clear
            debug.commanded_speed_mps = 0.0
            debug.commanded_steering_norm = 0.0
            self._debug = debug
            return

        if self._escape_count == 0:
            self._escape_sequence_start_xy = (robot_x, robot_y)
        self._escape_count += 1
        frames = min(
            self._tuning.escape.K_TURN_MIN_FRAMES
            + self._tuning.escape.STUCK_ESCALATION_FRAMES_PER_ATTEMPT * (self._escape_count - 1),
            self._tuning.escape.MAX_ESCAPE_FRAMES,
        )
        steering = self._tuning.escape.REV_STEERING_SCALE * self._escape_steer_sign_for_attempt()
        self._begin_maneuver(
            EscapeManeuver(
                maneuver_type=ManeuverType.STUCK_REVERSE,
                steering=steering,
                speed=self._tuning.escape.REV_SPEED,
                duration_frames=frames,
            ),
        )
        self._stuck_detector.reset()
        self._drive_active_maneuver(robot_x, robot_y, robot_yaw, phase=NavigatorPhase.STUCK_ESCAPE_MANEUVER)
        self._debug.is_stuck = bool(stuck_diag["is_stuck"])
        self._debug.stuck_count = int(stuck_diag["stuck_count"])
        self._debug.recent_movement_m = float(stuck_diag["recent_movement"])
        self._debug.rear_clearance_m = rear_clear
