"""WRO Track Navigator — Ackermann waypoint-following controller.

Navigates the robot around the WRO 2026 track for a configurable number of
laps using pure-pursuit waypoint following and LIDAR-based collision avoidance.

Usage:
    python -m src.navigation.navigator --metadata scenario_0000_metadata.json --laps 3
"""

from __future__ import annotations

import argparse
import json
import logging
import math
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from src.config.constants import DictKeys, RobotSpecs
from src.config.enums import ScenarioType
from src.navigation.collision import (
    assess_collision_risk,
    clamp_lidar_scan,
    measure_distance_in_direction,
    update_fwd_critical_count,
)
from src.navigation.waypoints import calculate_waypoints

logger = logging.getLogger(__name__)

# ── Navigation tuning ─────────────────────────────────────────────────────────

# Forward clearance thresholds for speed scaling (metres).
_FWD_CONTACT_DIST = 0.10
_FWD_SLOW_DIST = 0.25
_FWD_MEDIUM_DIST = 0.50
_FWD_FAST_DIST = 1.00

# Heading error thresholds for speed scaling (radians).
_ERR_CRAWL = 1.0
_ERR_SLOW = 0.7
_ERR_MEDIUM = 0.4

# Pure-pursuit lookahead distances (metres).
_LOOKAHEAD_SHORT = 0.30  # close to a corner (forward clearance < 0.15 m)
_LOOKAHEAD_LONG = 0.70   # normal straight driving

# Escape maneuver constants.
_ESCAPE_REV_SPEED = -0.20   # m/s during wall K-turn reverse phase
_ESCAPE_STEER_SCALE = 0.8   # fraction of max_steering_angle
_OBS_REV_SPEED = -0.25
_OBS_FWD_SPEED = 0.15
_OBS_STEER_SCALE = 0.70
_OBS_FWD_STEER_SCALE = 0.50
_OBS_REVERSE_FRACTION = 0.70  # fraction of frames spent reversing

# Side-correction gain (mild push away from close walls).
_SIDE_GAIN = 0.08
# Forward-obstacle avoidance gain (active on straights when sides clear).
_OBSTACLE_GAIN = 0.30

# Stuck detection parameters.
_STUCK_MOVE_THRESHOLD = 0.03  # metres — less than this → robot is stuck
_STUCK_ESCAPE_DURATION = 15   # frames (~0.75 s)
_STUCK_CLOSE_WALL = 0.18      # metres — escape direction safety override

# Speed fractions applied when forward clearance is in each zone.
_SPEED_CONTACT = 0.15   # very close (< _FWD_CONTACT_DIST) — creep forward
_SPEED_SLOW = 0.35      # slow zone
_SPEED_MEDIUM = 0.50    # medium zone
_SPEED_FAST = 0.70      # fast zone
_SPEED_FULL = 1.00      # clear path

# Speed fractions applied when heading error is large.
_SPEED_ERR_CRAWL = 0.25   # worst-case misalignment
_SPEED_ERR_SLOW = 0.35    # large heading error
_SPEED_ERR_MEDIUM = 0.55  # moderate heading error

# Escape duration clamping when computing wall-limited frames.
_ESCAPE_DUR_MIN = 6          # minimum escape frames
_ESCAPE_DUR_MAX = 12         # maximum escape frames
_ESCAPE_DUR_DIST_STEP = 0.03  # metres per escape frame

# Pure-pursuit steering P-gain.
_STEER_KP = 1.5

# Forward clearance below this uses _LOOKAHEAD_SHORT (approaching a corner).
_FWD_SHORT_LOOKAHEAD_DIST = 0.15

# Obstacle correction thresholds (function _compute_obstacle_correction).
_OBS_STRAIGHT_THRESHOLD = 0.4   # heading error below this = travelling straight (~23°)
_OBS_CLEAR_SIDES_DIST = 0.25    # side clearance above this = open corridor
_OBS_ACTIVE_FWD_DIST = 0.20     # forward dist below this = obstacle correction fires


class TrackNavigator(Node):
    """Navigate the WRO robot around the track using waypoint following.

    Args:
        metadata_path: Path to the scenario metadata JSON file.
        num_laps: Total number of laps the robot must complete.
        params_path: Optional path to a JSON file with runtime parameter
            overrides (see ``_apply_param_overrides``).
    """

    def __init__(
        self,
        metadata_path: str | Path,
        num_laps: int = 3,
        params_path: str | Path | None = None,
    ) -> None:
        super().__init__("track_navigator")

        self._metadata = _load_json(metadata_path)
        self._num_laps = num_laps
        self._is_open_challenge = (
            self._metadata.get(DictKeys.CHALLENGE_TYPE, ScenarioType.OPEN) == ScenarioType.OPEN
        )

        # ── Ackermann geometry ────────────────────────────────────────
        self._max_steering_angle: float = RobotSpecs.MAX_STEERING_ANGLE
        self._max_linear_speed: float = 0.50
        self._min_forward_speed: float = 0.08
        self._waypoint_threshold: float = 0.20

        # ── World-frame origin (odometry starts at 0,0 each run) ──────
        start_cond = self._metadata[DictKeys.STARTING_CONDITIONS]
        self._start_x: float = start_cond[DictKeys.POSITION][DictKeys.X]
        self._start_y: float = start_cond[DictKeys.POSITION][DictKeys.Y]
        self._start_yaw: float = start_cond[DictKeys.YAW]

        # ── Robot state ───────────────────────────────────────────────
        self._current_pos: tuple[float, float] | None = None
        self._current_yaw: float | None = None

        # ── LIDAR state ───────────────────────────────────────────────
        self._lidar_ranges: np.ndarray | None = None
        self._lidar_angles: np.ndarray | None = None

        # ── Collision avoidance parameters ────────────────────────────
        self._critical_distance: float = 0.07
        self._safe_distance: float = 0.15
        self._fwd_critical_lidar_count: int = 0
        self._fwd_critical_lidar_threshold: int = 2

        # ── Wall escape state ─────────────────────────────────────────
        self._escape_mode: bool = False
        self._escape_counter: int = 0
        self._escape_duration: int = 10
        self._escape_steer_sign: int = 1

        # ── Obstacle escape state ─────────────────────────────────────
        self._obstacle_escape: bool = False
        self._obstacle_escape_counter: int = 0
        self._obstacle_escape_duration: int = 12
        self._obstacle_escape_sign: int = 0

        # ── Repeat-escape detection ───────────────────────────────────
        self._obstacle_escape_positions: list[tuple[float, float]] = []
        self._obstacle_repeat_radius: float = 0.15
        self._obstacle_repeat_limit: int = 2

        self._critical_escape_positions: list[tuple[float, float]] = []
        self._critical_repeat_radius: float = 0.25
        self._critical_repeat_limit: int = 3

        # ── Waypoint tracking ─────────────────────────────────────────
        self._waypoints: list[tuple[float, float]] = calculate_waypoints(
            self._metadata, num_laps,
        )
        self._waypoint_index: int = 0
        self._prev_waypoint_dist: float = float("inf")
        self._dist_increasing_count: int = 0

        # ── Stuck detection ───────────────────────────────────────────
        self._stuck_check_pos: tuple[float, float] | None = None
        self._stuck_seconds: int = 0

        # ── Debug counter ─────────────────────────────────────────────
        self._log_counter: int = 0

        # ── ROS2 interfaces ───────────────────────────────────────────
        self._vel_publisher = self.create_publisher(
            Twist, "/wro_robot/cmd_vel", 10,
        )
        self.create_subscription(
            Odometry, "/wro_robot/odom", self._odom_callback, 10,
        )
        self.create_subscription(
            LaserScan, "/lidar", self._lidar_callback, 10,
        )
        self.create_timer(0.05, self._control_loop)  # 20 Hz

        if params_path is not None:
            self._apply_param_overrides(params_path)

        self.get_logger().info(
            f"Navigator ready: {len(self._waypoints)} waypoints, {num_laps} lap(s)",
        )

    # ── ROS2 callbacks ─────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        """Transform odometry from robot frame to world frame."""
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)

        cos_sy = math.cos(self._start_yaw)
        sin_sy = math.sin(self._start_yaw)
        self._current_pos = (
            self._start_x + odom_x * cos_sy - odom_y * sin_sy,
            self._start_y + odom_x * sin_sy + odom_y * cos_sy,
        )
        self._current_yaw = odom_yaw + self._start_yaw

    def _lidar_callback(self, msg: LaserScan) -> None:
        """Clamp raw LIDAR scan and update forward-critical counter."""
        raw = np.array(msg.ranges)
        self._lidar_ranges = clamp_lidar_scan(raw, RobotSpecs.LIDAR_MAX_RANGE)
        self._lidar_angles = np.linspace(
            msg.angle_min, msg.angle_max, len(self._lidar_ranges),
        )
        self._fwd_critical_lidar_count = update_fwd_critical_count(
            self._lidar_ranges,
            self._lidar_angles,
            self._critical_distance,
            self._fwd_critical_lidar_count,
        )

    # ── Control loop ───────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        """20 Hz control loop: waypoint following + collision avoidance."""
        if self._current_pos is None or self._current_yaw is None:
            return

        if self._waypoint_index >= len(self._waypoints):
            self.get_logger().info(f"Completed {self._num_laps} lap(s). Stopping.")
            self._publish_stop()
            rclpy.shutdown()
            return

        robot_x, robot_y = self._current_pos
        self._check_stuck(robot_x, robot_y)

        target_x, target_y = self._waypoints[self._waypoint_index]
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)

        # Closest-approach tracking (always update, freeze during escape).
        if distance > self._prev_waypoint_dist + 0.01:
            self._dist_increasing_count += 1
        else:
            self._dist_increasing_count = 0
        self._prev_waypoint_dist = distance

        distance, dx, dy, target_x, target_y = self._maybe_advance_waypoint(
            robot_x, robot_y, distance, dx, dy, target_x, target_y,
        )

        angle_error, steer_index = self._compute_lookahead_error(
            robot_x, robot_y, distance,
        )

        steer_x, steer_y = self._waypoints[steer_index]

        self.get_logger().info(
            f"NAV: pos=({robot_x:.2f},{robot_y:.2f}) "
            f"wp={self._waypoint_index}→({target_x:.2f},{target_y:.2f}) "
            f"look={steer_index}→({steer_x:.2f},{steer_y:.2f}) "
            f"dist={distance:.2f}m err={math.degrees(angle_error):.1f}°",
            throttle_duration_sec=1.0,
        )

        vel_msg = self._build_velocity_command(
            robot_x, robot_y, dx, dy, distance, angle_error,
        )
        self._vel_publisher.publish(vel_msg)
        self._log_counter += 1

    # ── Velocity command builder ────────────────────────────────────────────────

    def _build_velocity_command(
        self,
        robot_x: float,
        robot_y: float,
        dx: float,
        dy: float,
        distance: float,
        angle_error: float,
    ) -> Twist:
        """Select escape maneuver or normal navigation and build Twist message."""
        if self._lidar_ranges is None:
            return _make_twist(self._min_forward_speed, 0.0)

        risk_level, distances = assess_collision_risk(
            self._lidar_ranges,
            self._lidar_angles,
            self._critical_distance,
            self._fwd_critical_lidar_count,
            self._fwd_critical_lidar_threshold,
            self._is_open_challenge,
            angle_error,
        )

        if self._escape_mode:
            return self._execute_wall_escape(robot_x, robot_y)

        if self._obstacle_escape:
            return self._execute_obstacle_escape(robot_x, robot_y)

        if risk_level == "critical":
            return self._enter_wall_escape(
                robot_x, robot_y, dx, dy, distances, angle_error,
            )

        if risk_level == "obstacle":
            return self._enter_obstacle_escape(distances)

        return self._navigate_normally(dx, dy, distances, angle_error)

    # ── Escape maneuver logic ──────────────────────────────────────────────────

    def _execute_wall_escape(self, robot_x: float, robot_y: float) -> Twist:
        """Reverse-only K-turn away from a wall."""
        self._escape_counter += 1
        steer = self._escape_steer_sign * self._max_steering_angle * _ESCAPE_STEER_SCALE
        self.get_logger().warning(
            f"ESCAPE REV [{self._escape_counter}/{self._escape_duration}]: "
            f"steer={steer:.2f}",
        )
        if self._escape_counter >= self._escape_duration:
            self._escape_mode = False
            self._escape_counter = 0
            self._escape_duration = 10
            self._prev_waypoint_dist = float("inf")
            self._dist_increasing_count = 0
            self.get_logger().info("Wall escape complete")
        return _make_twist(_ESCAPE_REV_SPEED, steer)

    def _execute_obstacle_escape(self, robot_x: float, robot_y: float) -> Twist:
        """Two-phase mini K-turn for a traffic-sign obstacle."""
        self._obstacle_escape_counter += 1
        steer = self._obstacle_escape_sign * self._max_steering_angle * _OBS_STEER_SCALE
        reverse_frames = int(self._obstacle_escape_duration * _OBS_REVERSE_FRACTION)

        if self._obstacle_escape_counter <= reverse_frames:
            velocity = _OBS_REV_SPEED
            angular = steer
            phase = "REV"
        else:
            velocity = _OBS_FWD_SPEED
            angular = -steer * _OBS_FWD_STEER_SCALE
            phase = "FWD"

        self.get_logger().warning(
            f"OBS-ESCAPE {phase} "
            f"[{self._obstacle_escape_counter}/{self._obstacle_escape_duration}]: "
            f"v={velocity:.2f} steer={angular:.2f}",
        )

        if self._obstacle_escape_counter >= self._obstacle_escape_duration:
            self._finish_obstacle_escape(robot_x, robot_y)

        return _make_twist(velocity, angular)

    def _enter_wall_escape(
        self,
        robot_x: float,
        robot_y: float,
        dx: float,
        dy: float,
        distances: dict[str, float],
        angle_error: float,
    ) -> Twist:
        """Trigger a wall K-turn and return the initial Twist."""
        self._obstacle_escape = False
        left_dist = distances["left"]
        right_dist = distances["right"]

        self._critical_escape_positions.append((robot_x, robot_y))
        nearby_count = _count_nearby(
            self._critical_escape_positions, robot_x, robot_y,
            self._critical_repeat_radius,
        )

        self._escape_steer_sign = _choose_escape_sign(
            dx, dy, self._current_yaw, left_dist, right_dist, nearby_count,
        )
        self._escape_steer_sign = _apply_wall_safety_override(
            self._escape_steer_sign, left_dist, right_dist, nearby_count,
        )

        escape_side_dist = (
            left_dist if self._escape_steer_sign > 0 else right_dist
        )
        wall_limited = max(
            _ESCAPE_DUR_MIN,
            min(_ESCAPE_DUR_MAX, int(escape_side_dist / _ESCAPE_DUR_DIST_STEP)),
        )
        turning = abs(angle_error) > 0.25
        self._escape_duration = min(8, wall_limited) if turning else wall_limited

        if nearby_count >= self._critical_repeat_limit:
            self._skip_waypoint_and_clear_critical()

        self.get_logger().warning(
            f"CRITICAL #{nearby_count}: F={distances['forward']:.2f}m "
            f"{'RIGHT' if self._escape_steer_sign > 0 else 'LEFT'} "
            f"dur={self._escape_duration}",
        )
        self._escape_mode = True
        self._escape_counter = 0
        return _make_twist(_ESCAPE_REV_SPEED, 0.0)

    def _enter_obstacle_escape(self, distances: dict[str, float]) -> Twist:
        """Trigger an obstacle escape and return the initial Twist."""
        left_dist = distances["left"]
        right_dist = distances["right"]
        self._obstacle_escape_sign = 1 if right_dist > left_dist else -1
        self._obstacle_escape_positions.append(self._current_pos)
        self._obstacle_escape = True
        self._obstacle_escape_counter = 0
        steer = self._obstacle_escape_sign * self._max_steering_angle * _OBS_STEER_SCALE
        self.get_logger().warning(
            f"OBSTACLE → {'RIGHT' if self._obstacle_escape_sign > 0 else 'LEFT'} escape "
            f"(F={distances['forward']:.2f} L={left_dist:.2f} R={right_dist:.2f})",
        )
        return _make_twist(_OBS_REV_SPEED, steer)

    def _finish_obstacle_escape(
        self, robot_x: float, robot_y: float,
    ) -> None:
        """Finalise obstacle escape and skip waypoint if looping."""
        self._obstacle_escape = False
        self._obstacle_escape_counter = 0
        self._prev_waypoint_dist = float("inf")
        self._dist_increasing_count = 0

        nearby = _count_nearby(
            self._obstacle_escape_positions, robot_x, robot_y,
            self._obstacle_repeat_radius,
        )
        if nearby >= self._obstacle_repeat_limit:
            self.get_logger().warning(
                f"Obstacle loop ({nearby} escapes) — skipping waypoint "
                f"{self._waypoint_index}",
            )
            self._waypoint_index += 1
            self._obstacle_escape_positions.clear()
        else:
            self.get_logger().info(f"Obstacle escape complete ({nearby + 1} at spot)")

    # ── Normal navigation ──────────────────────────────────────────────────────

    def _navigate_normally(
        self,
        dx: float,
        dy: float,
        distances: dict[str, float],
        angle_error: float,
    ) -> Twist:
        """Pure-pursuit steering with speed scaling and mild side correction."""
        forward_dist = distances["forward"]
        left_dist = distances["left"]
        right_dist = distances["right"]

        speed = self._compute_speed(forward_dist, angle_error, dx, dy)
        waypoint_steer = _proportional_steer(
            angle_error, self._max_steering_angle,
        )
        side_correction = _compute_side_correction(
            left_dist, right_dist, self._safe_distance,
        )
        obstacle_correction = _compute_obstacle_correction(
            forward_dist, left_dist, right_dist, angle_error,
        )

        total_steer = float(np.clip(
            waypoint_steer + side_correction + obstacle_correction,
            -self._max_steering_angle,
            self._max_steering_angle,
        ))
        return _make_twist(speed, total_steer)

    def _compute_speed(
        self,
        forward_dist: float,
        angle_error: float,
        dx: float,
        dy: float,
    ) -> float:
        """Scale linear speed by forward clearance and heading error.

        Only called after the None-guard in _control_loop — yaw is always set.
        """
        if forward_dist < _FWD_CONTACT_DIST:
            speed_fwd = _SPEED_CONTACT
        elif forward_dist < _FWD_SLOW_DIST:
            speed_fwd = _SPEED_SLOW
        elif forward_dist < _FWD_MEDIUM_DIST:
            speed_fwd = _SPEED_MEDIUM
        elif forward_dist < _FWD_FAST_DIST:
            speed_fwd = _SPEED_FAST
        else:
            speed_fwd = _SPEED_FULL

        current_angle = math.atan2(dy, dx)
        current_err = _wrap_angle(current_angle - self._current_yaw)
        worst_err = max(abs(angle_error), abs(current_err))

        if worst_err > _ERR_CRAWL:
            speed_angle = _SPEED_ERR_CRAWL
        elif worst_err > _ERR_SLOW:
            speed_angle = _SPEED_ERR_SLOW
        elif worst_err > _ERR_MEDIUM:
            speed_angle = _SPEED_ERR_MEDIUM
        else:
            speed_angle = _SPEED_FULL

        raw_speed = self._max_linear_speed * min(speed_fwd, speed_angle)
        return max(raw_speed, self._min_forward_speed)

    # ── Waypoint tracking helpers ──────────────────────────────────────────────

    def _has_passed_waypoint(
        self,
        robot_x: float,
        robot_y: float,
        distance: float,
        dx: float,
        dy: float,
    ) -> bool:
        """Return True when the robot has overshot the current waypoint.

        Checks if the robot is facing away from the target (heading error > 90°)
        AND the next waypoint is closer, which indicates the current one was passed.

        Only called after the None-guard in _control_loop — yaw is always set.
        """
        target_angle = math.atan2(dy, dx)
        heading_err = _wrap_angle(target_angle - self._current_yaw)
        if abs(heading_err) <= math.pi / 2:
            return False
        next_idx = self._waypoint_index + 1
        if next_idx >= len(self._waypoints):
            return False
        nx, ny = self._waypoints[next_idx]
        next_dist = math.sqrt((nx - robot_x) ** 2 + (ny - robot_y) ** 2)
        return next_dist < distance

    def _maybe_advance_waypoint(
        self,
        robot_x: float,
        robot_y: float,
        distance: float,
        dx: float,
        dy: float,
        target_x: float,
        target_y: float,
    ) -> tuple[float, float, float, float, float]:
        """Advance the waypoint index if reached or passed; return updated geometry.

        Only called after the None-guard in _control_loop — yaw is always set.
        """
        if self._escape_mode or self._obstacle_escape:
            return distance, dx, dy, target_x, target_y

        should_skip = (
            distance < self._waypoint_threshold
            or self._dist_increasing_count >= 10
            or self._has_passed_waypoint(robot_x, robot_y, distance, dx, dy)
        )

        if should_skip:
            self._waypoint_index += 1
            self._prev_waypoint_dist = float("inf")
            self._dist_increasing_count = 0
            self._obstacle_escape_positions.clear()
            self._critical_escape_positions.clear()
            self.get_logger().info(
                f"Waypoint {self._waypoint_index} → ({target_x:.2f},{target_y:.2f})",
            )

        if self._waypoint_index < len(self._waypoints):
            target_x, target_y = self._waypoints[self._waypoint_index]
            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.sqrt(dx**2 + dy**2)

        return distance, dx, dy, target_x, target_y

    def _compute_lookahead_error(
        self,
        robot_x: float,
        robot_y: float,
        distance: float,
    ) -> tuple[float, int]:
        """Compute pure-pursuit heading error and the lookahead waypoint index.

        Only called after the None-guard in _control_loop — yaw is always set.
        """
        fwd_dist = float("inf")
        if self._lidar_ranges is not None:
            fwd_dist = measure_distance_in_direction(
                self._lidar_ranges, self._lidar_angles, target_angle=0.0,
            )
        lookahead = _LOOKAHEAD_SHORT if fwd_dist < _FWD_SHORT_LOOKAHEAD_DIST else _LOOKAHEAD_LONG

        steer_idx = self._waypoint_index
        cumulative = 0.0
        while steer_idx + 1 < len(self._waypoints) and cumulative < lookahead:
            wx, wy = self._waypoints[steer_idx]
            nx, ny = self._waypoints[steer_idx + 1]
            cumulative += math.sqrt((nx - wx)**2 + (ny - wy)**2)
            steer_idx += 1

        steer_x, steer_y = self._waypoints[steer_idx]
        angle_error = _wrap_angle(
            math.atan2(steer_y - robot_y, steer_x - robot_x) - self._current_yaw,
        )
        return angle_error, steer_idx

    def _check_stuck(self, robot_x: float, robot_y: float) -> None:
        """Detect and escape physics-stuck situations (embedded in wall)."""
        if self._log_counter % 20 != 0:
            return

        if self._stuck_check_pos is not None:
            moved = math.sqrt(
                (robot_x - self._stuck_check_pos[0])**2
                + (robot_y - self._stuck_check_pos[1])**2,
            )
            if moved < _STUCK_MOVE_THRESHOLD:
                self._stuck_seconds += 1
                if self._stuck_seconds >= 2 and not self._escape_mode:
                    self._trigger_stuck_escape(robot_x, robot_y)
            else:
                self._stuck_seconds = 0

        self._stuck_check_pos = (robot_x, robot_y)

    def _trigger_stuck_escape(self, robot_x: float, robot_y: float) -> None:
        """Choose escape direction and start wall-escape mode."""
        left_dist = right_dist = float("inf")
        if self._lidar_ranges is not None:
            left_dist = measure_distance_in_direction(
                self._lidar_ranges, self._lidar_angles,
                math.pi / 2, filter_self_detection=True,
            )
            right_dist = measure_distance_in_direction(
                self._lidar_ranges, self._lidar_angles,
                -math.pi / 2, filter_self_detection=True,
            )

        self._escape_steer_sign = _choose_escape_sign_from_waypoint(
            robot_x, robot_y, self._waypoint_index, self._waypoints,
            self._current_yaw, left_dist, right_dist,
        )
        self._escape_steer_sign = _apply_wall_safety_override(
            self._escape_steer_sign, left_dist, right_dist, nearby_count=0,
        )

        self._obstacle_escape = False
        self._escape_mode = True
        self._escape_counter = 0
        self._escape_duration = _STUCK_ESCAPE_DURATION
        self._stuck_seconds = 0
        self.get_logger().warning(
            f"STUCK at ({robot_x:.2f},{robot_y:.2f}) — triggering escape "
            f"{'RIGHT' if self._escape_steer_sign > 0 else 'LEFT'}",
        )

    def _skip_waypoint_and_clear_critical(self) -> None:
        """Skip the current waypoint and reset critical escape tracking."""
        self.get_logger().warning(
            f"CRITICAL LOOP — skipping waypoint {self._waypoint_index}",
        )
        self._waypoint_index += 1
        self._prev_waypoint_dist = float("inf")
        self._dist_increasing_count = 0
        self._critical_escape_positions.clear()

    # ── Stop and parameter helpers ─────────────────────────────────────────────

    def _publish_stop(self) -> None:
        """Publish a zero-velocity Twist."""
        self._vel_publisher.publish(_make_twist(0.0, 0.0))

    def _apply_param_overrides(self, params_path: str | Path) -> None:
        """Override runtime parameters from a JSON file.

        Supported keys: critical_distance, fwd_critical_lidar_threshold,
        critical_repeat_limit, critical_repeat_radius, waypoint_threshold,
        max_linear_speed.
        """
        overrides = _load_json(params_path)
        param_map = {
            "critical_distance": "_critical_distance",
            "fwd_critical_lidar_threshold": "_fwd_critical_lidar_threshold",
            "critical_repeat_limit": "_critical_repeat_limit",
            "critical_repeat_radius": "_critical_repeat_radius",
            "waypoint_threshold": "_waypoint_threshold",
            "max_linear_speed": "_max_linear_speed",
        }
        for key, attr in param_map.items():
            if key in overrides:
                setattr(self, attr, overrides[key])
        self.get_logger().info(f"Loaded param overrides: {overrides}")


# ── Pure helper functions ──────────────────────────────────────────────────────

def _load_json(path: str | Path) -> dict[str, Any]:
    with open(path) as fh:
        return json.load(fh)


def _make_twist(linear_x: float, angular_z: float) -> Twist:
    msg = Twist()
    msg.linear.x = float(linear_x)
    msg.angular.z = float(angular_z)
    return msg


def _wrap_angle(angle: float) -> float:
    """Wrap angle to (-π, π]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _proportional_steer(angle_error: float, max_angle: float) -> float:
    """P-controller for Ackermann steering angle."""
    return float(np.clip(_STEER_KP * angle_error, -max_angle, max_angle))


def _compute_side_correction(
    left_dist: float, right_dist: float, safe_distance: float,
) -> float:
    """Return a mild angular correction that pushes robot away from close walls."""
    if left_dist < safe_distance:
        ratio = 1.0 - left_dist / safe_distance
        return -_SIDE_GAIN * ratio
    if right_dist < safe_distance:
        ratio = 1.0 - right_dist / safe_distance
        return _SIDE_GAIN * ratio
    return 0.0


def _compute_obstacle_correction(
    forward_dist: float, left_dist: float, right_dist: float, angle_error: float,
) -> float:
    """Steer toward the open side when an obstacle is close ahead on a straight."""
    is_straight = abs(angle_error) < _OBS_STRAIGHT_THRESHOLD
    sides_clear = max(left_dist, right_dist) > _OBS_CLEAR_SIDES_DIST
    if forward_dist >= _OBS_ACTIVE_FWD_DIST or not sides_clear or not is_straight:
        return 0.0
    urgency = 1.0 - forward_dist / _OBS_ACTIVE_FWD_DIST
    strength = _OBSTACLE_GAIN * urgency
    # Negative angular.z = steer right; positive = steer left.
    return -strength if right_dist > left_dist else strength


def _count_nearby(
    positions: list[tuple[float, float]],
    robot_x: float,
    robot_y: float,
    radius: float,
) -> int:
    """Count how many stored positions are within radius of the robot."""
    return sum(
        1 for px, py in positions
        if math.sqrt((robot_x - px)**2 + (robot_y - py)**2) < radius
    )


def _choose_escape_sign(
    dx: float,
    dy: float,
    current_yaw: float,
    left_dist: float,
    right_dist: float,
    nearby_count: int,
) -> int:
    """Choose +1 (CW/right) or -1 (CCW/left) K-turn direction.

    First escape uses the current-waypoint heading to rotate toward the target.
    On the second escape from the same spot, use side clearance instead (the
    waypoint-heading direction already failed).
    """
    if nearby_count < 2 and (abs(dx) > 0.001 or abs(dy) > 0.001):
        wp_err = _wrap_angle(math.atan2(dy, dx) - current_yaw)
        if abs(wp_err) > 0.1:
            return -1 if wp_err > 0 else 1
    return 1 if right_dist > left_dist else -1


def _choose_escape_sign_from_waypoint(
    robot_x: float,
    robot_y: float,
    waypoint_index: int,
    waypoints: list[tuple[float, float]],
    current_yaw: float,
    left_dist: float,
    right_dist: float,
) -> int:
    """Choose escape direction for stuck recovery using waypoint heading."""
    if waypoint_index < len(waypoints):
        wp_x, wp_y = waypoints[waypoint_index]
        wp_err = _wrap_angle(math.atan2(wp_y - robot_y, wp_x - robot_x) - current_yaw)
        if abs(wp_err) > 0.1:
            return -1 if wp_err > 0 else 1
    return 1 if right_dist > left_dist else -1


def _apply_wall_safety_override(
    escape_sign: int,
    left_dist: float,
    right_dist: float,
    nearby_count: int,
) -> int:
    """Flip escape direction if the chosen side curves the robot into a close wall.

    Skipped on the second-or-later escape from the same spot because by then
    the direction is room-based and flipping it reverts to the failed heading.
    """
    if nearby_count >= 2:
        return escape_sign
    curve_side = left_dist if escape_sign > 0 else right_dist
    other_side = right_dist if escape_sign > 0 else left_dist
    if curve_side < _STUCK_CLOSE_WALL and other_side > curve_side:
        return -escape_sign
    return escape_sign


# ── CLI entry point ────────────────────────────────────────────────────────────

def main() -> None:
    """Parse arguments and run the TrackNavigator node."""
    parser = argparse.ArgumentParser(
        description="Navigate the WRO robot using waypoint following.",
    )
    parser.add_argument(
        "--metadata", required=True,
        help="Path to the scenario metadata JSON file.",
    )
    parser.add_argument(
        "--laps", type=int, default=3,
        help="Number of laps to complete (default: 3).",
    )
    parser.add_argument(
        "--params",
        help="Optional path to navigator_params.json for runtime overrides.",
    )
    args = parser.parse_args()

    metadata_path = Path(args.metadata)
    if not metadata_path.exists():
        logger.error("Metadata file not found: %s", metadata_path)
        raise SystemExit(1)

    rclpy.init()
    navigator: TrackNavigator | None = None
    try:
        navigator = TrackNavigator(
            metadata_path=metadata_path,
            num_laps=args.laps,
            params_path=args.params,
        )
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        if navigator is not None:
            navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
