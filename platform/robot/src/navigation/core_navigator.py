"""Core Navigator decoupled from ROS2.

This class implements the navigation loop purely in Python using the
HardwareGateway interface. This achieves Dependency Inversion (SOLID),
making the navigation logic trivial to test and execute in a simulator.
"""

from __future__ import annotations

import logging
import math
from typing import Any

from shared.domain.models import Velocity
from shared.domain.enums import RiskLevel

from src.hardware.gateway import HardwareGateway
from src.navigation.controllers import (
    CollisionAvoidanceController,
    StuckDetector,
    WaypointController,
)
from shared.config.navigation_tuning import NavigationTuning

logger = logging.getLogger(__name__)


class CoreNavigator:
    """Orchestrates navigation by requesting states from a HardwareGateway
    and sending Velocity commands.
    """

    def __init__(
        self,
        gateway: HardwareGateway,
        waypoints: list[tuple[float, float]],
        num_laps: int = 3,
        tuning: NavigationTuning | None = None,
    ) -> None:
        self._gateway = gateway
        self._waypoints = waypoints
        self._num_laps = num_laps
        self._tuning = tuning or NavigationTuning()

        self._waypoint_index = 0
        self._laps_completed = 0
        self._waypoint_threshold = 0.20

        # Controllers
        self._waypoint_controller = WaypointController(
            max_steering_angle=0.5236,  # RobotSpecs.MAX_STEERING_ANGLE
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

    def step(self) -> None:
        """Execute one control step.

        Pulls current state from the gateway, calculates commands,
        and pushes them back to the gateway.
        """
        pose = self._gateway.get_current_pose()
        if not pose:
            return  # No pose available yet

        # Guard: check lap completion
        if self._laps_completed >= self._num_laps:
            self._gateway.publish_velocity(Velocity(linear=0.0, angular=0.0))
            logger.info("All laps complete!")
            return

        robot_x, robot_y = pose.x, pose.y
        robot_yaw = pose.yaw

        # Update stuck detector
        self._stuck_detector.update((robot_x, robot_y))
        if self._stuck_detector.is_stuck:
            logger.warning("Robot stuck - triggering escape")
            self._gateway.publish_velocity(Velocity(linear=-0.2, angular=0.5))
            return

        # Get current target waypoint
        if self._waypoint_index >= len(self._waypoints):
            self._laps_completed += 1
            self._waypoint_index = 0
            self._stuck_detector.reset()
            logger.info(f"Lap {self._laps_completed} complete")
            return

        target_wp = self._waypoints[self._waypoint_index]

        # Get LIDAR ranges from gateway
        lidar_data = self._gateway.get_lidar_scan()
        if lidar_data:
            ranges, angles = lidar_data
            forward_clearance = self._collision_controller.compute_forward_clearance(ranges)
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

        # Escape maneuvers if critical
        if risk == RiskLevel.CRITICAL and lidar_data:
            threat_dir = self._collision_controller.detect_threat_direction(lidar_data[0])
            maneuver = self._collision_controller.compute_escape_maneuver(risk, threat_dir)
            if maneuver:
                self._gateway.publish_velocity(Velocity(linear=maneuver.speed, angular=maneuver.steering))
                return

        # Normal publish
        self._gateway.publish_velocity(Velocity(linear=speed, angular=steering_normalized))
