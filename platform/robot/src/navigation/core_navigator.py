"""Core Navigator decoupled from ROS2.

This class implements the navigation loop purely in Python using the
HardwareGateway interface. This achieves Dependency Inversion (SOLID),
making the navigation logic trivial to test and execute in a simulator.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

from shared.config.constants import RobotSpecs
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import RiskLevel
from shared.domain.models import Velocity

from src.navigation.control.controllers import (
    CollisionAvoidanceController,
    StuckDetector,
    WaypointController,
)
from src.navigation.planning.waypoints import corridor_for_position

if TYPE_CHECKING:
    from shared.config.enums import Section

    from src.hardware.gateway import HardwareGateway
    from src.navigation.maneuvers.parking import ParkController
    from src.navigation.planning.sign_router import SignRouter
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
        self._park_engage_dist = 0.30  # Engage parking within 30 cm of staging

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
        )

        self._stuck_detector = StuckDetector(
            move_threshold=self._tuning.escape.STUCK_MOVE_THRESHOLD,
            timeout_frames=self._tuning.escape.STUCK_TIMEOUT_FRAMES,
        )

    @property
    def current_corridor(self) -> Section | None:
        """Current track corridor derived from robot position. None before first step."""
        return self._current_corridor

    def step(self) -> None:
        """Execute one control step.

        Pulls current state from the gateway, calculates commands,
        and pushes them back to the gateway.
        """
        pose = self._gateway.get_current_pose()
        if not pose:
            return  # No pose available yet

        robot_x, robot_y = pose.x, pose.y
        robot_yaw = pose.yaw

        self._current_corridor = corridor_for_position(robot_x, robot_y)

        # Lap completion: defer the parking handoff until the robot is actually
        # in the parking corridor and within reach of the staging point. Until
        # then keep navigating so the handoff never fires mid-corridor.
        if self._laps_completed >= self._num_laps and self._handle_finish(
            robot_x, robot_y, robot_yaw,
        ):
            return

        # Update stuck detector
        self._stuck_detector.update((robot_x, robot_y))
        if self._stuck_detector.is_stuck:
            self._handle_stuck_escape()
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
                return

        # Geometric lap counting (requires LapDetector).
        if (
            self._lap_detector is not None
            and self._current_corridor is not None
            and self._lap_detector.update((robot_x, robot_y), self._current_corridor)
        ):
            self._laps_completed += 1
            logger.info("Lap %d complete (geometric + waypoint confirmed)", self._laps_completed)

        target_wp = self._waypoints[self._waypoint_index]

        # Apply sign routing deformation if in obstacles challenge.
        if self._sign_router is not None and self._current_corridor is not None:
            detections = self._gateway.get_vision_detections()
            target_wp = self._sign_router.deform_waypoint(
                waypoint=target_wp,
                robot_pos=(robot_x, robot_y),
                robot_yaw=robot_yaw,
                corridor=self._current_corridor,
                detections=detections,
            )

        # Get LIDAR ranges from gateway
        lidar_data = self._gateway.get_lidar_scan()
        if lidar_data:
            ranges, angles = lidar_data
            forward_clearance = self._collision_controller.compute_forward_clearance(ranges, angles)
            risk = self._collision_controller.assess_risk(ranges)
        else:
            forward_clearance = 5.0
            risk = RiskLevel.SAFE

        # Check waypoint reached
        dist_to_wp = math.sqrt((target_wp[0] - robot_x) ** 2 + (target_wp[1] - robot_y) ** 2)
        if dist_to_wp < self._waypoint_threshold:
            self._waypoint_index += 1
            return

        # Get steering from waypoint controller
        steering_normalized, _ = self._waypoint_controller.compute_steering(
            current_pos=(robot_x, robot_y),
            current_yaw=robot_yaw,
            target_waypoint=target_wp,
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
        if risk == RiskLevel.CRITICAL and lidar_data:
            threat_dir = self._collision_controller.detect_threat_direction(lidar_data[0], lidar_data[1])
            maneuver = self._collision_controller.compute_escape_maneuver(risk, threat_dir)
            if maneuver:
                self._gateway.publish_velocity(Velocity(linear=maneuver.speed, angular=maneuver.steering))
                return

        # Normal publish
        self._gateway.publish_velocity(Velocity(linear=speed, angular=steering_normalized))

    def _handle_finish(self, robot_x: float, robot_y: float, robot_yaw: float) -> bool:
        """Handle the post-final-lap phase.

        Returns True if a command was issued (caller should stop this tick);
        False if the robot should keep navigating toward the parking corridor.
        """
        pc = self._park_controller
        if pc is None:
            # Open challenge: no parking maneuver — hold position.
            self._gateway.publish_velocity(Velocity(linear=0.0, angular=0.0))
            return True

        if not self._parking_engaged and self._should_engage_parking(robot_x, robot_y):
            logger.info("Parking engaged in corridor %s", self._current_corridor)
            self._parking_engaged = True

        if not self._parking_engaged:
            return False  # Keep navigating until at the staging point.

        if pc.is_done:
            self._gateway.publish_velocity(Velocity(linear=0.0, angular=0.0))
            return True

        cmd = pc.update((robot_x, robot_y), robot_yaw)
        linear = cmd.linear
        # Forward-clearance gate so the staging vector never drives into a wall.
        lidar_data = self._gateway.get_lidar_scan()
        if lidar_data:
            fwd = self._collision_controller.compute_forward_clearance(lidar_data[0], lidar_data[1])
            if fwd < self._tuning.clearance.CONTACT_DIST:
                linear = 0.0
        self._gateway.publish_velocity(Velocity(linear=linear, angular=cmd.steering))
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
        """Reverse out of a stuck state, but never back into an unseen wall."""
        logger.warning("Robot stuck - triggering escape")
        rear_clear = 10.0
        lidar_data = self._gateway.get_lidar_scan()
        if lidar_data:
            rear_clear = self._collision_controller.compute_rear_clearance(
                lidar_data[0], lidar_data[1],
            )
        if rear_clear < self._tuning.clearance.CONTACT_DIST:
            logger.warning("Stuck escape blocked: rear clearance %.2f m - holding", rear_clear)
            self._gateway.publish_velocity(Velocity(linear=0.0, angular=0.0))
        else:
            self._gateway.publish_velocity(
                Velocity(
                    linear=self._tuning.escape.REV_SPEED,
                    angular=self._tuning.escape.REV_STEERING_SCALE,
                ),
            )
        self._stuck_detector.reset()
