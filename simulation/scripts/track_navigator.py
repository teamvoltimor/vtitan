#!/usr/bin/env python3
"""
WRO Track Navigator - Completes Open Challenge (Ackermann Steering)

Navigates robot around track for 3 laps using waypoint following.
Uses metadata to understand corridor layout and widths.
Sends steering angles via angular.z (Ackermann convention).

Usage:
    python3 track_navigator.py --metadata scenario_0000_metadata.json --laps 3
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import json
import argparse
from pathlib import Path
import numpy as np

from constants import RobotSpecs


class TrackNavigator(Node):
    """Navigate robot around WRO track using waypoint following (Ackermann)"""

    def __init__(self, metadata_path, num_laps=3):
        super().__init__('track_navigator')

        # Load metadata
        self.metadata = self.load_metadata(metadata_path)
        self.num_laps = num_laps

        # Ackermann geometry from RobotSpecs
        self.max_steering_angle = RobotSpecs.MAX_STEERING_ANGLE
        self.wheelbase = RobotSpecs.WHEELBASE

        # Calculate waypoints around track
        self.waypoints = self.calculate_waypoints()
        self.current_waypoint_idx = 0
        self.laps_completed = 0

        # Robot state (world frame)
        self.current_pos = None
        self.current_yaw = None

        # Challenge type: 'open' (no traffic signs) or 'obstacles' (has traffic signs)
        self.is_open_challenge = self.metadata.get('challenge_type', 'open') == 'open'

        # Odom→World transform: odom starts at (0,0,yaw=0), but robot spawns at (start_x, start_y, start_yaw)
        start_cond = self.metadata['starting_conditions']
        self.start_x = start_cond['position']['x']
        self.start_y = start_cond['position']['y']
        self.start_yaw = start_cond['yaw']

        # LIDAR data for collision avoidance (Slamtec C1: 0.05-12m range)
        self.lidar_ranges = None
        self.lidar_angles = None
        self.lidar_max_range = RobotSpecs.LIDAR_MAX_RANGE

        # Collision avoidance parameters
        self.danger_distance = 0.20  # meters - start slowing down
        self.critical_distance = 0.07  # meters - escape only at near-contact
        self.safe_distance = 0.15  # meters - side wall warning

        # Wall escape state: reverse-only bump with steering.
        # No FWD phase — in all observed cases the FWD phase drives back into
        # the wall, undoing rotational/lateral progress from the REV phase.
        self.escape_mode = False
        self.escape_counter = 0
        self.escape_duration = 10  # total frames (~0.5s at 20Hz)
        self.escape_steer_sign = 1  # +1 or -1, chosen at escape start based on LIDAR

        # Obstacle escape state: committed multi-frame maneuver (mini K-turn for traffic signs)
        # Prevents the rapid REVERSE↔STEER oscillation that occurs when F hovers around threshold.
        self.obstacle_escape = False
        self.obstacle_escape_counter = 0
        self.obstacle_escape_duration = 12  # total frames (~0.6s at 20Hz)
        self.obstacle_escape_sign = 0  # +1 (nose right) or -1 (nose left), chosen on entry

        # Repeated-obstacle detection: if robot triggers obstacle escape N times
        # near the same position, skip the current waypoint (it's behind the sign).
        self.obstacle_escape_positions = []  # list of (x,y) where escapes triggered
        self.obstacle_repeat_radius = 0.15  # same-spot threshold (meters)
        self.obstacle_repeat_limit = 2  # skip waypoint after this many escapes

        # Repeated-critical detection: if robot keeps hitting the SAME wall
        # (backs up → drives forward → same wall), skip the current waypoint.
        # Each escape backs up ~12cm; if the robot returns to the same wall 3×
        # it is clearly stuck and the waypoint is unreachable from this angle.
        self.critical_escape_positions = []  # list of (x,y) where critical triggered
        self.critical_repeat_radius = 0.25   # same-wall threshold (meters)
        self.critical_repeat_limit = 3       # skip waypoint after this many wall hits

        # Control parameters (Ackermann)
        self.max_linear_speed = 0.50  # m/s
        self.min_forward_speed = 0.08  # m/s - Ackermann needs forward motion to steer
        self.waypoint_threshold = 0.20  # meters - reach radius for waypoints

        # Closest-approach tracking: skip waypoint if distance increasing
        self.prev_waypoint_dist = float('inf')
        self.dist_increasing_count = 0

        # Stuck detection: if robot hasn't moved for 2+ seconds, force escape
        self.stuck_check_pos = None
        self.stuck_seconds = 0

        # Debug logging
        self.log_counter = 0

        # Publishers and subscribers
        self.vel_publisher = self.create_publisher(Twist, '/wro_robot/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/wro_robot/odom',
            self.odom_callback,
            10
        )
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10
        )

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f'Navigator initialized: {len(self.waypoints)} waypoints, {num_laps} laps')
        self.get_logger().info(f'Starting section: {self.metadata["starting_conditions"]["section"]}')
        self.get_logger().info(f'Direction: {self.metadata["starting_conditions"]["direction"]}')

    def load_metadata(self, metadata_path):
        """Load scenario metadata"""
        with open(metadata_path, 'r') as f:
            return json.load(f)

    def compute_steering_angle(self, angle_error):
        """Compute clamped steering angle from angle error (proportional control)"""
        kp = 1.5
        steer = kp * angle_error
        return float(np.clip(steer, -self.max_steering_angle, self.max_steering_angle))

    def calculate_waypoints(self):
        """Calculate waypoints around the track based on corridor layout.

        Uses circular arc waypoints at corners to ensure turns are physically
        feasible for the Ackermann robot (min turning radius ~0.3m).
        """
        corridor_widths = self.metadata['corridor_widths']
        direction = self.metadata['starting_conditions']['direction']

        # Convert widths from mm to meters
        north_width = corridor_widths['north']['width_mm'] / 1000.0
        south_width = corridor_widths['south']['width_mm'] / 1000.0
        east_width = corridor_widths['east']['width_mm'] / 1000.0
        west_width = corridor_widths['west']['width_mm'] / 1000.0

        # Track bounds
        track_max = 3.0
        track_center = 1.5

        # Bias corridor centers toward outer walls for inner wall clearance
        OUTER_WALL_BIAS = 0.05

        north_cy = track_max - north_width / 2 + OUTER_WALL_BIAS   # toward Y=3.0
        south_cy = south_width / 2 - OUTER_WALL_BIAS                # toward Y=0.0
        east_cx = track_max - east_width / 2 + OUTER_WALL_BIAS      # toward X=3.0
        west_cx = west_width / 2 - OUTER_WALL_BIAS                  # toward X=0.0

        # Arc radius for corners (must exceed min turning radius of ~0.294m).
        # Larger radius starts turn earlier, giving clearance from inner wall corners.
        R = 0.45

        # ── Corner arc generation ──────────────────────────────────────
        # For CW: all corners are RIGHT turns. For CCW: all LEFT turns.
        # Arc is a 90° circular arc connecting the two corridor centerlines.
        # ICR (instant center of rotation) is computed per corner.

        def arc_points(cx, cy, r, theta_start, theta_end, n=3):
            """Generate n intermediate points along an arc (excluding endpoints)."""
            pts = []
            for i in range(1, n + 1):
                t = i / (n + 1)
                theta = theta_start + t * (theta_end - theta_start)
                pts.append((round(cx + r * math.cos(theta), 3),
                            round(cy + r * math.sin(theta), 3)))
            return pts

        # For CW right turns, arc goes CW (decreasing theta).
        # SE corner: East(south) → South(west). ICR = (east_cx - R, south_cy + R)
        se_icr = (east_cx - R, south_cy + R)
        se_entry = (east_cx, south_cy + R)       # on east centerline
        se_exit = (east_cx - R, south_cy)         # on south centerline
        se_arc_cw = [se_entry] + arc_points(
            *se_icr, R, 0, -math.pi/2, n=3) + [se_exit]

        # SW corner: South(west) → West(north). ICR = (west_cx + R, south_cy + R)
        sw_icr = (west_cx + R, south_cy + R)
        sw_entry = (west_cx + R, south_cy)        # on south centerline
        sw_exit = (west_cx, south_cy + R)          # on west centerline
        sw_arc_cw = [sw_entry] + arc_points(
            *sw_icr, R, -math.pi/2, -math.pi, n=3) + [sw_exit]

        # NW corner: West(north) → North(east). ICR = (west_cx + R, north_cy - R)
        nw_icr = (west_cx + R, north_cy - R)
        nw_entry = (west_cx, north_cy - R)         # on west centerline
        nw_exit = (west_cx + R, north_cy)           # on north centerline
        nw_arc_cw = [nw_entry] + arc_points(
            *nw_icr, R, math.pi, math.pi/2, n=3) + [nw_exit]

        # NE corner: North(east) → East(south). ICR = (east_cx - R, north_cy - R)
        ne_icr = (east_cx - R, north_cy - R)
        ne_entry = (east_cx - R, north_cy)         # on north centerline
        ne_exit = (east_cx, north_cy - R)           # on east centerline
        ne_arc_cw = [ne_entry] + arc_points(
            *ne_icr, R, math.pi/2, 0, n=3) + [ne_exit]

        # For CCW, reverse each arc (left turns use same geometry, opposite direction)
        se_arc_ccw = list(reversed(se_arc_cw))
        sw_arc_ccw = list(reversed(sw_arc_cw))
        nw_arc_ccw = list(reversed(nw_arc_cw))
        ne_arc_ccw = list(reversed(ne_arc_cw))

        # ── Straight corridor waypoints ────────────────────────────────
        # Each corridor goes from one arc exit to the next arc entry.
        # Spacing ~0.3m along the corridor centerline.

        def straight_waypoints(fixed_coord, is_x, start_val, end_val, n=5):
            """Generate evenly spaced waypoints along a corridor centerline.
            is_x=True: fixed_coord is X, vary Y from start_val to end_val.
            is_x=False: fixed_coord is Y, vary X from start_val to end_val.
            """
            pts = []
            for i in range(n):
                t = i / (n - 1) if n > 1 else 0.5
                val = start_val + t * (end_val - start_val)
                if is_x:
                    pts.append((round(fixed_coord, 3), round(val, 3)))
                else:
                    pts.append((round(val, 3), round(fixed_coord, 3)))
            return pts

        # CW corridor segments (direction of travel):
        # East: heading south (Y decreasing) from NE exit to SE entry
        east_cw = straight_waypoints(east_cx, True, north_cy - R, south_cy + R, n=8)
        # South: heading west (X decreasing) from SE exit to SW entry
        south_cw = straight_waypoints(south_cy, False, east_cx - R, west_cx + R, n=8)
        # West: heading north (Y increasing) from SW exit to NW entry
        west_cw = straight_waypoints(west_cx, True, south_cy + R, north_cy - R, n=8)
        # North: heading east (X increasing) from NW exit to NE entry
        north_cw = straight_waypoints(north_cy, False, west_cx + R, east_cx - R, n=8)

        # CCW corridors (opposite direction)
        east_ccw = list(reversed(east_cw))
        south_ccw = list(reversed(south_cw))
        west_ccw = list(reversed(west_cw))
        north_ccw = list(reversed(north_cw))

        # ── Assemble full loop ─────────────────────────────────────────
        # CW order: East→SE→South→SW→West→NW→North→NE (then repeats)
        # Each segment: [corridor] + [corner_arc]
        if direction == 'clockwise':
            segments = {
                'east':  east_cw + se_arc_cw,
                'south': south_cw + sw_arc_cw,
                'west':  west_cw + nw_arc_cw,
                'north': north_cw + ne_arc_cw,
            }
            order = ['east', 'south', 'west', 'north']
        else:
            segments = {
                'east':  east_ccw + ne_arc_ccw,
                'south': south_ccw + se_arc_ccw,
                'west':  west_ccw + sw_arc_ccw,
                'north': north_ccw + nw_arc_ccw,
            }
            order = ['east', 'north', 'west', 'south']

        # Rotate order so starting section is first
        start_section = self.metadata['starting_conditions']['section'].lower()
        while order[0] != start_section:
            order.append(order.pop(0))

        # Build one full loop
        full_loop = []
        for sec in order:
            full_loop.extend(segments[sec])

        # Find closest waypoint to starting position in the first segment
        start_pos = self.metadata['starting_conditions']['position']
        start_x, start_y = start_pos['x'], start_pos['y']

        first_seg = segments[order[0]]
        min_dist = float('inf')
        start_idx = 0
        for i, (wx, wy) in enumerate(first_seg):
            dist = math.sqrt((wx - start_x)**2 + (wy - start_y)**2)
            if dist < min_dist:
                min_dist = dist
                start_idx = i

        # Build waypoint list: partial first segment + rest of first lap
        first_seg_partial = first_seg[start_idx:]
        rest_of_lap = []
        for sec in order[1:]:
            rest_of_lap.extend(segments[sec])

        waypoints = first_seg_partial + rest_of_lap

        # Additional full laps
        for _ in range(self.num_laps - 1):
            waypoints.extend(full_loop)

        # Complete final partial: the skipped start of first segment
        if start_idx > 0:
            waypoints.extend(first_seg[:start_idx])

        # Deduplicate consecutive waypoints (straight endpoints overlap arc
        # entry/exit points, wasting lookahead slots)
        deduped = [waypoints[0]]
        for wp in waypoints[1:]:
            if abs(wp[0] - deduped[-1][0]) > 0.001 or abs(wp[1] - deduped[-1][1]) > 0.001:
                deduped.append(wp)
        return deduped

    def odom_callback(self, msg):
        """Update robot position from odometry, transformed to world frame"""
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Transform odom → world: rotate by start_yaw then translate by start position
        cos_sy = math.cos(self.start_yaw)
        sin_sy = math.sin(self.start_yaw)
        self.current_pos = (
            self.start_x + odom_x * cos_sy - odom_y * sin_sy,
            self.start_y + odom_x * sin_sy + odom_y * cos_sy,
        )
        self.current_yaw = odom_yaw + self.start_yaw

    def lidar_callback(self, msg):
        """Update LIDAR data for collision avoidance (Slamtec C1)"""
        self.lidar_ranges = np.array(msg.ranges)
        num_points = len(self.lidar_ranges)
        self.lidar_angles = np.linspace(msg.angle_min, msg.angle_max, num_points)
        num_inf = np.sum(np.isinf(self.lidar_ranges))
        # Clamp below-min readings to real sensor minimum (Slamtec C1 returns ~50mm)
        below_min = self.lidar_ranges < RobotSpecs.LIDAR_MIN_RANGE
        self.lidar_ranges[below_min] = RobotSpecs.LIDAR_MIN_RANGE
        self.lidar_ranges[np.isinf(self.lidar_ranges)] = self.lidar_max_range

        self.get_logger().info(
            f'LIDAR received: {num_points} points, {num_inf} inf values, '
            f'angle range: [{msg.angle_min:.2f}, {msg.angle_max:.2f}]',
            throttle_duration_sec=2.0
        )

    def get_min_distance_in_direction(self, target_angle, tolerance=0.3,
                                      filter_self_detection=False):
        """Get minimum LIDAR distance in a specific direction (in robot frame)

        Args:
            filter_self_detection: If True, discard readings at LIDAR_MIN_RANGE.
                Only use for side directions (±π/2) where gpu_lidar detects
                the robot's own chassis. Forward (0) should NOT filter because
                a wall at 0.05m is a real obstacle.
        """
        if self.lidar_ranges is None or self.lidar_angles is None:
            return float('inf')

        # Compute shortest angular distance (wrapping-safe for any angle range)
        diff = self.lidar_angles - target_angle
        angle_diffs = np.abs(np.arctan2(np.sin(diff), np.cos(diff)))

        mask = angle_diffs < tolerance
        if np.any(mask):
            valid_ranges = self.lidar_ranges[mask]
            if filter_self_detection:
                # Filter readings near LIDAR minimum range — self-detection of
                # robot body (chassis, wheels, camera). gpu_lidar renders ALL
                # visuals including the same model's body. Use 0.08m threshold
                # (chassis half-width is ~0.075m) to catch all self-reflections.
                valid_ranges = valid_ranges[valid_ranges > 0.08]
            if len(valid_ranges) > 0:
                return float(np.min(valid_ranges))
        return float('inf')

    def check_collision_risk(self, angle_error=0.0):
        """Check for collision risk using LIDAR data.

        Args:
            angle_error: Current heading error to steer target (radians).
                Large values indicate the robot is actively turning. During turns,
                close forward readings are walls (not obstacles) since the robot
                is heading into a corridor side/end wall while turning.
        """
        if self.lidar_ranges is None:
            return 'safe', {
                'forward': float('inf'),
                'left': float('inf'),
                'right': float('inf')
            }

        forward_dist = self.get_min_distance_in_direction(0, tolerance=0.25)
        # Side: filter_self_detection=True because gpu_lidar side rays clip the
        # robot's own chassis at ~0.05m. Forward doesn't need filtering because
        # the LIDAR is mounted ahead of the chassis front face.
        left_dist = self.get_min_distance_in_direction(
            math.pi/2, tolerance=0.25, filter_self_detection=True)
        right_dist = self.get_min_distance_in_direction(
            -math.pi/2, tolerance=0.25, filter_self_detection=True)

        distances = {
            'forward': forward_dist,
            'left': left_dist,
            'right': right_dist
        }

        self.get_logger().info(
            f'LIDAR: F={forward_dist:.2f}m L={left_dist:.2f}m R={right_dist:.2f}m',
            throttle_duration_sec=0.5
        )

        # Wall clipping detection: forward > 4m is impossible inside the 3×3m
        # track. The LIDAR has passed through a thin wall into open space.
        if forward_dist > 4.0:
            self.get_logger().warning(
                f'WALL CLIP: F={forward_dist:.1f}m (impossible in 3m track)')
            return 'critical', distances

        # Determine if robot is actively turning based on heading error.
        # Used only for obstacle vs wall classification (obstacles challenge).
        abs_err = abs(angle_error)
        turning = abs_err > 0.25  # ~14 degrees

        # Determine if robot is boxed in (wall/corner) vs free-standing obstacle.
        # Use MIN side: if the closer wall is < 0.28m, the robot is in a corridor
        # near a wall/corner → K-turn needed. Only classify as 'obstacle' when
        # BOTH sides have room (traffic sign in wide corridor).
        # 600mm corridor with 0.05m outer-wall bias: near side ≈ 0.25m → boxed_in.
        # 1000mm corridor: near side ≈ 0.45m → obstacle escape (traffic sign).
        near_side = min(left_dist, right_dist)
        boxed_in = near_side < 0.28

        # In the open challenge there are no traffic signs — every forward
        # obstruction is a wall/corner. Always use 'critical' (K-turn/bump).
        # 'obstacle' classification is only for the obstacles challenge where
        # traffic signs can appear in wide corridors with room on both sides.
        can_be_obstacle = (not self.is_open_challenge
                           and not boxed_in and not turning)

        if forward_dist <= RobotSpecs.LIDAR_MIN_RANGE + 0.01:
            if can_be_obstacle:
                self.get_logger().warning(
                    f'OBSTACLE CONTACT: F={forward_dist:.3f}m '
                    f'(L={left_dist:.2f} R={right_dist:.2f})')
                return 'obstacle', distances
            else:
                self.get_logger().warning(
                    f'WALL CONTACT: F={forward_dist:.3f}m '
                    f'(boxed_in={boxed_in} turning={turning})')
                return 'critical', distances

        if forward_dist < self.critical_distance:
            if can_be_obstacle:
                return 'obstacle', distances
            else:
                return 'critical', distances

        return 'safe', distances

    def control_loop(self):
        """Main control loop - waypoint following with Ackermann steering"""
        if self.current_pos is None or self.current_yaw is None:
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info(f'Completed {self.num_laps} laps! Stopping.')
            self.stop_robot()
            rclpy.shutdown()
            return

        robot_x, robot_y = self.current_pos

        # Stuck detection: every second check if robot moved at least 3cm.
        # Catches physics-stuck situations (robot embedded in wall).
        if self.log_counter % 20 == 0:
            if self.stuck_check_pos is not None:
                moved = math.sqrt(
                    (robot_x - self.stuck_check_pos[0])**2 +
                    (robot_y - self.stuck_check_pos[1])**2)
                if moved < 0.03:
                    self.stuck_seconds += 1
                    if self.stuck_seconds >= 2 and not self.escape_mode:
                        # Use current-waypoint heading for escape direction.
                        # The robot is stuck near the target, so rotating toward
                        # it is more likely to free it than the L/R room heuristic.
                        left_d = self.get_min_distance_in_direction(
                            math.pi/2, tolerance=0.25, filter_self_detection=True)
                        right_d = self.get_min_distance_in_direction(
                            -math.pi/2, tolerance=0.25, filter_self_detection=True)
                        if self.current_waypoint_idx < len(self.waypoints):
                            wp_x, wp_y = self.waypoints[self.current_waypoint_idx]
                            stuck_wp_angle = math.atan2(wp_y - robot_y, wp_x - robot_x)
                            stuck_wp_err = stuck_wp_angle - self.current_yaw
                            while stuck_wp_err > math.pi: stuck_wp_err -= 2 * math.pi
                            while stuck_wp_err < -math.pi: stuck_wp_err += 2 * math.pi
                            if abs(stuck_wp_err) > 0.1:
                                self.escape_steer_sign = -1 if stuck_wp_err > 0 else 1
                            else:
                                self.escape_steer_sign = 1 if right_d > left_d else -1
                        else:
                            self.escape_steer_sign = 1 if right_d > left_d else -1
                        self.obstacle_escape = False  # overrides obstacle escape
                        self.get_logger().warning(
                            f'STUCK DETECTED ({robot_x:.2f},{robot_y:.2f}) '
                            f'for {self.stuck_seconds}s - ESCAPE '
                            f'(L={left_d:.2f} R={right_d:.2f})')
                        self.escape_mode = True
                        self.escape_counter = 0
                        self.escape_duration = 15  # ~0.75s (longer for stuck)
                        self.stuck_seconds = 0
                else:
                    self.stuck_seconds = 0
            self.stuck_check_pos = (robot_x, robot_y)

        # Get current target waypoint
        target_x, target_y = self.waypoints[self.current_waypoint_idx]

        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)

        # Check if we reached waypoint OR if waypoint is behind/passed
        def advance_waypoint(reason):
            self.current_waypoint_idx += 1
            self.prev_waypoint_dist = float('inf')
            self.dist_increasing_count = 0
            self.obstacle_escape_positions.clear()
            self.critical_escape_positions.clear()
            wp_total = len(self.waypoints)
            self.get_logger().info(
                f'{reason} waypoint {self.current_waypoint_idx}/{wp_total} '
                f'at ({target_x:.2f}, {target_y:.2f})')

        if distance < self.waypoint_threshold:
            advance_waypoint('Reached')
        else:
            # 1. Skip if waypoint is behind robot (angle > 90°) and next is closer
            target_angle_check = math.atan2(dy, dx)
            err = target_angle_check - self.current_yaw
            while err > math.pi:
                err -= 2 * math.pi
            while err < -math.pi:
                err += 2 * math.pi

            should_skip = False
            if abs(err) > math.pi / 2:  # > 90° — waypoint is behind
                if self.current_waypoint_idx + 1 < len(self.waypoints):
                    next_x, next_y = self.waypoints[self.current_waypoint_idx + 1]
                    next_dist = math.sqrt((next_x - robot_x)**2 + (next_y - robot_y)**2)
                    if next_dist < distance:
                        should_skip = True

            # 2. Closest-approach detection: if distance has been increasing
            #    for several frames, we've passed the waypoint
            if distance > self.prev_waypoint_dist + 0.01:
                self.dist_increasing_count += 1
            else:
                self.dist_increasing_count = 0
            self.prev_waypoint_dist = distance

            if self.dist_increasing_count >= 10 and distance > self.waypoint_threshold:
                should_skip = True

            if should_skip:
                advance_waypoint('Skipped (passed)')

        if self.current_waypoint_idx < len(self.waypoints):
            target_x, target_y = self.waypoints[self.current_waypoint_idx]
            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.sqrt(dx**2 + dy**2)

        # Distance-based steering lookahead (pure pursuit): steer toward a
        # point ~0.7m ahead along the waypoint path. This adapts to waypoint
        # density — on dense arc sections it looks many waypoints ahead,
        # on sparse straights fewer — giving consistent corner anticipation.
        lookahead_dist = 0.7  # meters along path
        steer_idx = self.current_waypoint_idx
        cumulative = 0.0
        while steer_idx + 1 < len(self.waypoints) and cumulative < lookahead_dist:
            wx, wy = self.waypoints[steer_idx]
            nx, ny = self.waypoints[steer_idx + 1]
            cumulative += math.sqrt((nx - wx)**2 + (ny - wy)**2)
            steer_idx += 1
        steer_x, steer_y = self.waypoints[steer_idx]
        steer_dx = steer_x - robot_x
        steer_dy = steer_y - robot_y

        target_angle = math.atan2(steer_dy, steer_dx)
        angle_error = target_angle - self.current_yaw

        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        self.get_logger().info(
            f'NAV: pos=({robot_x:.2f},{robot_y:.2f}) yaw={math.degrees(self.current_yaw):.1f} '
            f'wp={self.current_waypoint_idx}→({target_x:.2f},{target_y:.2f}) '
            f'look={steer_idx}→({steer_x:.2f},{steer_y:.2f}) '
            f'dist={distance:.2f}m err={math.degrees(angle_error):.1f}°',
            throttle_duration_sec=1.0
        )

        # Check for collision risk using LIDAR
        risk_level, distances = self.check_collision_risk(angle_error)

        vel_msg = Twist()

        # WALL ESCAPE: reverse-only bump when facing a wall/corner.
        # Back up with strong steering toward the side with more room.
        # Ackermann reverse: steer LEFT → nose swings RIGHT, steer RIGHT → nose swings LEFT.
        if self.escape_mode:
            self.escape_counter += 1

            # Reverse-only with strong steering. No FWD phase — it always
            # drives back into the wall, undoing all rotational/lateral progress.
            # After this, waypoint steering resumes and guides the robot onward.
            steer = self.escape_steer_sign * self.max_steering_angle * 0.8
            vel_msg.linear.x = -0.20
            vel_msg.angular.z = steer

            self.get_logger().warning(
                f'ESCAPE REV [{self.escape_counter}/{self.escape_duration}]: '
                f'v={vel_msg.linear.x:.2f}, steer={vel_msg.angular.z:.2f}')

            if self.escape_counter >= self.escape_duration:
                self.escape_mode = False
                self.escape_counter = 0
                self.escape_duration = 10  # reset to default
                self.get_logger().info('Escape complete')

        # OBSTACLE ESCAPE: committed multi-frame maneuver for traffic signs.
        # Phase 1 (70% frames): reverse at high speed with steering to create gap.
        # Phase 2 (30% frames): forward with opposite steering to re-orient.
        # Runs to completion regardless of per-frame LIDAR readings.
        elif self.obstacle_escape:
            self.obstacle_escape_counter += 1
            steer = self.obstacle_escape_sign * self.max_steering_angle * 0.7
            reverse_frames = int(self.obstacle_escape_duration * 0.7)

            if self.obstacle_escape_counter <= reverse_frames:
                # Phase 1: reverse with steering (Ackermann: steer LEFT → nose RIGHT)
                vel_msg.linear.x = -0.25
                vel_msg.angular.z = steer
                phase = 'REV'
            else:
                # Phase 2: forward with opposite steering to straighten
                vel_msg.linear.x = 0.15
                vel_msg.angular.z = -steer * 0.5
                phase = 'FWD'

            self.get_logger().warning(
                f'OBS-ESCAPE {phase} [{self.obstacle_escape_counter}/{self.obstacle_escape_duration}]: '
                f'v={vel_msg.linear.x:.2f}, steer={vel_msg.angular.z:.2f}')

            if self.obstacle_escape_counter >= self.obstacle_escape_duration:
                self.obstacle_escape = False
                self.obstacle_escape_counter = 0
                # Check if we've been escaping from the same spot repeatedly.
                # If so, the target waypoint is behind/past the obstacle — skip it.
                nearby = sum(
                    1 for ox, oy in self.obstacle_escape_positions
                    if math.sqrt((robot_x - ox)**2 + (robot_y - oy)**2)
                    < self.obstacle_repeat_radius)
                if nearby >= self.obstacle_repeat_limit:
                    self.get_logger().warning(
                        f'Obstacle loop detected ({nearby} escapes near '
                        f'({robot_x:.2f},{robot_y:.2f})) — skipping waypoint '
                        f'{self.current_waypoint_idx}')
                    self.current_waypoint_idx += 1
                    self.prev_waypoint_dist = float('inf')
                    self.dist_increasing_count = 0
                    self.obstacle_escape_positions.clear()
                else:
                    self.get_logger().info(
                        f'Obstacle escape complete ({nearby + 1} at this spot)')

        # COLLISION AVOIDANCE LOGIC
        elif risk_level == 'critical':
            # Wall/corner collision — choose escape strategy based on context.
            left_d = distances.get('left', 0)
            right_d = distances.get('right', 0)
            abs_err = abs(angle_error)

            # Determine escape direction using CURRENT WAYPOINT angle, not the
            # lookahead. The lookahead often points into/past the corner arc,
            # giving the wrong rotation direction.
            #
            # Example: at the NE corner, lookahead angle points NNW (toward the
            # arc and north corridor) → escape LEFT → robot drifts west into the
            # inner wall. But the current waypoint (2.75, 2.07) is to the NNE →
            # escape RIGHT → robot rotates back toward east and drives clear ✓
            #
            # For K-turn RIGHT (escape_steer_sign=+1):
            #   REV: angular.z = +0.42 (LEFT steer) → Ackermann reverse → nose CW
            #   → heading decreases → robot faces toward target (to the right) ✓
            # For K-turn LEFT (escape_steer_sign=-1):
            #   REV: angular.z = -0.42 (RIGHT steer) → Ackermann reverse → nose CCW
            #   → heading increases → robot faces toward target (to the left) ✓
            current_wp_angle = math.atan2(dy, dx)
            current_wp_err = current_wp_angle - self.current_yaw
            while current_wp_err > math.pi: current_wp_err -= 2 * math.pi
            while current_wp_err < -math.pi: current_wp_err += 2 * math.pi

            if abs(current_wp_err) > 0.1:
                # Rotate toward current waypoint heading
                self.escape_steer_sign = -1 if current_wp_err > 0 else 1
            elif right_d > left_d:
                self.escape_steer_sign = 1
            else:
                self.escape_steer_sign = -1

            self.obstacle_escape = False  # escape overrides obstacle maneuver

            # Repeated-critical detection: robot backs up N cm then drives
            # straight into the same wall again — the waypoint is unreachable
            # from this angle. Record position and escalate response.
            self.critical_escape_positions.append((robot_x, robot_y))
            nearby_critical = sum(
                1 for ox, oy in self.critical_escape_positions
                if math.sqrt((robot_x - ox)**2 + (robot_y - oy)**2)
                < self.critical_repeat_radius)

            # Escalate escape duration on repeated contacts at the same wall.
            if nearby_critical >= 2:
                self.escape_duration = 20  # ~1.0s — more lateral displacement
            elif abs_err > 0.25:
                self.escape_duration = 8   # ~0.4s (corner, waypoint steering handles rest)
            else:
                self.escape_duration = 12  # ~0.6s (straight/slight correction)

            # After N escapes from the same wall, skip the current waypoint.
            # The target is behind the wall from the robot's angle — advance
            # to the next waypoint which should be past the corner junction.
            if nearby_critical >= self.critical_repeat_limit:
                self.get_logger().warning(
                    f'CRITICAL LOOP ({nearby_critical} escapes near '
                    f'({robot_x:.2f},{robot_y:.2f})) — skipping waypoint '
                    f'{self.current_waypoint_idx}')
                self.current_waypoint_idx += 1
                self.prev_waypoint_dist = float('inf')
                self.dist_increasing_count = 0
                self.critical_escape_positions.clear()

            self.get_logger().warning(
                f'CRITICAL #{nearby_critical}: F={distances["forward"]:.2f}m - ESCAPE '
                f'(L={left_d:.2f} R={right_d:.2f} look_err={math.degrees(angle_error):.0f}° '
                f'wp_err={math.degrees(current_wp_err):.0f}° '
                f'→ {"RIGHT/CW" if self.escape_steer_sign > 0 else "LEFT/CCW"} '
                f'dur={self.escape_duration})')
            self.escape_mode = True
            self.escape_counter = 0
            vel_msg.linear.x = -0.20
            vel_msg.angular.z = 0.0

        elif risk_level == 'obstacle':
            # Obstacle ahead with room on sides — initiate obstacle escape maneuver.
            # This is a committed multi-frame sequence, not per-frame reactive.
            left_d = distances.get('left', 0)
            right_d = distances.get('right', 0)
            self.obstacle_escape_sign = 1 if right_d > left_d else -1
            direction = "RIGHT" if self.obstacle_escape_sign > 0 else "LEFT"
            # Record position for repeated-obstacle detection
            self.obstacle_escape_positions.append((robot_x, robot_y))
            self.get_logger().warning(
                f'OBSTACLE → ESCAPE {direction} '
                f'(F={distances["forward"]:.2f} L={left_d:.2f} R={right_d:.2f}) '
                f'[attempt {len(self.obstacle_escape_positions)}]')
            self.obstacle_escape = True
            self.obstacle_escape_counter = 0
            # Start reversing immediately
            vel_msg.linear.x = -0.25
            vel_msg.angular.z = self.obstacle_escape_sign * self.max_steering_angle * 0.7

        else:

            # ── WAYPOINT-BASED NAVIGATION ──────────────────────────────
            # Arc waypoints handle corners; LIDAR only modulates speed and
            # applies mild side corrections. No LIDAR-based turn direction
            # override — the waypoints know which way to turn.
            forward_dist = distances['forward']
            left_dist = distances['left']
            right_dist = distances['right']

            # ── LINEAR VELOCITY ──
            # Slow down based on forward clearance and heading error.
            # Use BOTH the lookahead error (for steering anticipation) and
            # the current-waypoint error (for off-path detection). The worse
            # of the two determines speed — prevents full-speed oscillation
            # when the robot exits a corner off-center.
            if forward_dist < 0.10:
                speed_fwd = 0.15
            elif forward_dist < 0.25:
                speed_fwd = 0.35
            elif forward_dist < 0.5:
                speed_fwd = 0.5
            elif forward_dist < 1.0:
                speed_fwd = 0.7
            else:
                speed_fwd = 1.0

            # Angle error to current waypoint (detects off-path condition)
            current_angle = math.atan2(dy, dx)
            current_err = current_angle - self.current_yaw
            while current_err > math.pi:
                current_err -= 2 * math.pi
            while current_err < -math.pi:
                current_err += 2 * math.pi
            worst_err = max(abs(angle_error), abs(current_err))

            if worst_err > 1.0:
                speed_angle = 0.25
            elif worst_err > 0.7:
                speed_angle = 0.35
            elif worst_err > 0.4:
                speed_angle = 0.55
            else:
                speed_angle = 1.0

            vel_msg.linear.x = self.max_linear_speed * min(speed_fwd, speed_angle)
            vel_msg.linear.x = max(vel_msg.linear.x, self.min_forward_speed)

            # ── STEERING: PURE WAYPOINT FOLLOWING ──
            waypoint_steer = self.compute_steering_angle(angle_error)

            # Mild side wall correction (proportional push away from close walls)
            side_correction = 0.0
            if left_dist < self.safe_distance:
                ratio = 1.0 - left_dist / self.safe_distance
                side_correction = -0.08 * ratio
            elif right_dist < self.safe_distance:
                ratio = 1.0 - right_dist / self.safe_distance
                side_correction = 0.08 * ratio

            # Forward obstacle avoidance: when something is close ahead and
            # sides have room, steer toward the more open side. Stronger as
            # the obstacle gets closer. Only activates on straights (low
            # heading error) to avoid fighting waypoint steering during turns.
            obstacle_correction = 0.0
            if forward_dist < 0.20 and max(left_dist, right_dist) > 0.25 \
                    and abs(angle_error) < 0.4:
                urgency = 1.0 - forward_dist / 0.20  # 0→1 as obstacle approaches
                steer_strength = 0.3 * urgency
                if right_dist > left_dist:
                    obstacle_correction = -steer_strength  # steer right (negative)
                else:
                    obstacle_correction = steer_strength   # steer left (positive)

            total_steer = waypoint_steer + side_correction + obstacle_correction
            vel_msg.angular.z = float(np.clip(total_steer,
                                              -self.max_steering_angle,
                                              self.max_steering_angle))

        # Log velocity commands
        self.get_logger().info(
            f'CMD: v={vel_msg.linear.x:.3f} m/s, steer={vel_msg.angular.z:.3f} rad, '
            f'risk={risk_level}, waypoint={self.current_waypoint_idx}',
            throttle_duration_sec=0.5
        )

        self.vel_publisher.publish(vel_msg)

        # Periodic debug logging
        self.log_counter += 1
        if self.log_counter % 40 == 0:
            lidar_info = ""
            if distances:
                lidar_info = f', LIDAR[F:{distances["forward"]:.2f} L:{distances["left"]:.2f} R:{distances["right"]:.2f}]'

            self.get_logger().info(
                f'Pos: ({robot_x:.2f}, {robot_y:.2f}), '
                f'Target WP{self.current_waypoint_idx}: ({target_x:.2f}, {target_y:.2f}), '
                f'Dist: {distance:.2f}m, '
                f'Vel: {vel_msg.linear.x:.2f} m/s, Steer: {vel_msg.angular.z:.2f} rad'
                f'{lidar_info}'
            )

    def stop_robot(self):
        """Stop the robot"""
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.vel_publisher.publish(vel_msg)


def main():
    parser = argparse.ArgumentParser(description='Navigate WRO track using waypoint following')
    parser.add_argument('--metadata', type=str, required=True,
                       help='Path to scenario metadata JSON file')
    parser.add_argument('--laps', type=int, default=3,
                       help='Number of laps to complete (default: 3)')

    args = parser.parse_args()

    if not Path(args.metadata).exists():
        print(f"ERROR: Metadata file not found: {args.metadata}")
        return 1

    rclpy.init()

    try:
        navigator = TrackNavigator(
            metadata_path=args.metadata,
            num_laps=args.laps
        )
        rclpy.spin(navigator)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            navigator.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
