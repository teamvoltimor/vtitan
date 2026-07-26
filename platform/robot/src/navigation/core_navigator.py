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

from shared.config.constants import RobotSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import RiskLevel

from src.navigation.control.controllers import (
    CollisionAvoidanceController,
    EscapeManeuver,
    ManeuverType,
    StuckDetector,
    WaypointController,
)
from src.navigation.planning.waypoints import corridor_for_position
from src.navigation.ports import DriveCommand

if TYPE_CHECKING:
    from shared.config.enums import Section

    from src.navigation.maneuvers.parking import ParkController
    from src.navigation.planning.sign_router import SignRouter
    from src.navigation.ports import HardwareGateway, LidarScan
    from src.navigation.race_tracker import LapDetector

logger = logging.getLogger(__name__)


class CoreNavigator:
    """Orchestrates navigation using a HardwareGateway interface."""

    def __init__(
        self,
        gateway: HardwareGateway,
        waypoints: list[tuple[float, float]],
        num_laps: int = 3,
        tuning: NavigationTuning | None = None,
        sign_router: SignRouter | None = None,
        lap_detector: LapDetector | None = None,
        park_controller: ParkController | None = None,
    ) -> None:
        self._gateway = gateway
        self._waypoints = waypoints
        self._num_laps = num_laps
        self._tuning = tuning or NavigationTuning()
        self._sign_router = sign_router
        self._lap_detector = lap_detector

        self._waypoint_index = 0
        self._laps_completed = 0
        self._waypoint_threshold = 0.20
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
        # longer, alternate side) rather than repeating an identical failed pulse.
        self._active_maneuver: EscapeManeuver | None = None
        self._maneuver_frames_left = 0
        self._escape_count = 0  # escapes begun since the last normal drive tick
        self._escape_steer_sign = 1.0

        # Controllers
        self._waypoint_controller = WaypointController(
            max_steering_angle=RobotSpecs.MAX_STEERING_ANGLE,
            lookahead_short=self._tuning.pursuit.LOOKAHEAD_SHORT,
            lookahead_long=self._tuning.pursuit.LOOKAHEAD_LONG,
            lookahead_transition=self._tuning.pursuit.LOOKAHEAD_TRANSITION,
            steer_kp=self._tuning.pursuit.STEER_KP,
            max_steering_rate=self._tuning.pursuit.MAX_STEERING_RATE,
        )

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
        )

        self._stuck_detector = StuckDetector(
            move_threshold=self._tuning.escape.STUCK_MOVE_THRESHOLD,
            timeout_frames=self._tuning.escape.STUCK_TIMEOUT_FRAMES,
        )

    @property
    def current_corridor(self) -> Section | None:
        """Current track corridor derived from robot position. None before first step."""
        return self._current_corridor

    def replace_path(self, waypoints: list[tuple[float, float]], robot_xy: tuple[float, float]) -> None:
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

        Args:
            waypoints: The replacement path (single canonical lap).
            robot_xy: Current position, used to resume at the nearest waypoint.
        """
        self._waypoints = waypoints
        robot_x, robot_y = robot_xy
        self._waypoint_index = min(
            range(len(waypoints)),
            key=lambda i: math.hypot(waypoints[i][0] - robot_x, waypoints[i][1] - robot_y),
        )

    @property
    def laps_completed(self) -> int:
        """Number of laps confirmed completed so far."""
        return self._laps_completed

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
            return

        robot_x, robot_y = pose.x, pose.y
        robot_yaw = pose.yaw

        self._current_corridor = corridor_for_position(robot_x, robot_y)

        # Continue an in-progress escape maneuver until its latched duration
        # elapses, so escapes are real motions rather than single-tick pulses that
        # never clear the wall.
        if self._active_maneuver is not None:
            self._drive_active_maneuver()
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
                self._handle_stuck_escape()
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
            if self._lap_detector is not None:
                self._lap_detector.notify_waypoint_wrapped()
            else:
                # Fallback: no geometric guard — count directly.
                self._laps_completed += 1
                logger.info("Lap %d complete (waypoint-only fallback)", self._laps_completed)
                if self._sign_router is not None:
                    self._sign_router.reset_for_new_lap()
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
        while self._waypoint_index + 1 < len(self._waypoints) and math.hypot(
            self._waypoints[self._waypoint_index + 1][0] - robot_x,
            self._waypoints[self._waypoint_index + 1][1] - robot_y,
        ) < math.hypot(raw_wp[0] - robot_x, raw_wp[1] - robot_y):
            self._waypoint_index += 1
            raw_wp = self._waypoints[self._waypoint_index]

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
            return

        # Steer at a lookahead point, not directly at the (often much closer)
        # next waypoint — otherwise the lookahead distance is computed but
        # discarded, producing weave on straights and corner cutting (PP-1).
        lookahead_distance = self._waypoint_controller.select_lookahead(forward_clearance)
        steer_target = self._waypoint_controller.select_target_point(
            current_pos=(robot_x, robot_y),
            waypoints=self._waypoints[self._waypoint_index :],
            waypoint_index=0,
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
        if self._sign_router is not None and self._current_corridor is not None:
            detections = self._gateway.get_vision_detections()
            steer_target = self._sign_router.deform_waypoint(
                waypoint=steer_target,
                robot_pos=(robot_x, robot_y),
                robot_yaw=robot_yaw,
                corridor=self._current_corridor,
                detections=detections,
            )

        # Get steering from waypoint controller
        steering_normalized, _ = self._waypoint_controller.compute_steering(
            current_pos=(robot_x, robot_y),
            current_yaw=robot_yaw,
            target_waypoint=steer_target,
            forward_clearance=forward_clearance,
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

        # Never blast past a non-forward obstacle (e.g. a sign alongside the
        # robot) just because the path ahead is clear.
        if risk != RiskLevel.SAFE:
            speed = min(speed, self._tuning.speed.SLOW_SPEED)

        # Escape maneuvers if critical
        if risk == RiskLevel.CRITICAL and scan:
            threat_dir = self._collision_controller.detect_threat_direction(scan.ranges_m, scan.angles_rad)
            maneuver = self._collision_controller.compute_escape_maneuver(
                risk,
                threat_dir,
                scan.ranges_m,
                scan.angles_rad,
            )
            if maneuver and self._reversing_into_unseen_wall(maneuver, scan):
                # Blocked at both ends: fall through to the capped creep-speed
                # publish below rather than backing into an unseen wall. The
                # stuck detector is the backstop if the robot truly can't move.
                maneuver = None
            if maneuver:
                self._escape_count += 1
                self._begin_maneuver(self._maybe_escalate(maneuver))
                self._drive_active_maneuver()
                return

        # Normal publish — the robot is driving, so clear the escape escalation.
        self._escape_count = 0
        self._gateway.publish_drive(DriveCommand(speed_mps=speed, steering_norm=steering_normalized))

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

    def _drive_active_maneuver(self) -> None:
        """Publish the active escape command and count down its latched duration."""
        maneuver = self._active_maneuver
        if maneuver is None:
            return
        self._maneuver_frames_left -= 1
        if self._maneuver_frames_left <= 0:
            self._active_maneuver = None
        self._gateway.publish_drive(DriveCommand(speed_mps=maneuver.speed, steering_norm=maneuver.steering))

    def _maybe_escalate(self, maneuver: EscapeManeuver) -> EscapeManeuver:
        """Escalate a repeated escape instead of repeating an identical pulse.

        After a few consecutive escapes that clearly aren't working, reverse for
        longer and swing toward the opposite side, so the robot stops slamming
        the same failing maneuver into the same wall.
        """
        if self._escape_count <= self._tuning.escape.ESCALATE_AFTER_ATTEMPTS:
            return maneuver
        self._escape_steer_sign = -self._escape_steer_sign
        steering = abs(maneuver.steering) * self._escape_steer_sign if maneuver.steering else 0.0
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
            return True

        if not self._parking_engaged and self._should_engage_parking(robot_x, robot_y):
            logger.info("Parking engaged in corridor %s", self._current_corridor)
            self._parking_engaged = True

        if not self._parking_engaged:
            return False  # Keep navigating until at the staging point.

        if pc.is_done:
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
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

    def _handle_stuck_escape(self) -> None:
        """Reverse out of a stuck state, but never back into an unseen wall.

        The reverse is latched for several frames (escalating with repeated
        attempts) and alternates steering side each attempt, so a wall-pinned
        robot actually backs away instead of twitching one centimetre every few
        seconds forever.
        """
        logger.warning("Robot stuck - triggering escape")
        rear_clear = 10.0
        scan = self._gateway.get_lidar_scan()
        if scan:
            rear_clear = self._collision_controller.compute_rear_clearance(
                scan.ranges_m,
                scan.angles_rad,
            )
        if rear_clear < self._tuning.clearance.CONTACT_DIST:
            logger.warning("Stuck escape blocked: rear clearance %.2f m - holding", rear_clear)
            self._gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            self._stuck_detector.reset()
            return

        self._escape_count += 1
        frames = min(
            self._tuning.escape.K_TURN_MIN_FRAMES + 2 * (self._escape_count - 1),
            self._tuning.escape.MAX_ESCAPE_FRAMES,
        )
        steering = self._tuning.escape.REV_STEERING_SCALE * self._escape_steer_sign
        self._escape_steer_sign = -self._escape_steer_sign  # alternate side each attempt
        self._begin_maneuver(
            EscapeManeuver(
                maneuver_type=ManeuverType.STUCK_REVERSE,
                steering=steering,
                speed=self._tuning.escape.REV_SPEED,
                duration_frames=frames,
            ),
        )
        self._stuck_detector.reset()
        self._drive_active_maneuver()
