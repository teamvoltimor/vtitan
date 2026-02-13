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
        self.danger_distance = 0.25  # meters - start slowing down
        self.critical_distance = 0.18  # meters - emergency maneuver
        self.safe_distance = 0.30  # meters - minimum safe distance

        # Escape maneuver state
        self.escape_mode = False
        self.escape_counter = 0
        self.escape_duration = 20  # frames (~1 second at 20Hz)

        # Control parameters (Ackermann)
        self.max_linear_speed = 0.25  # m/s
        self.min_forward_speed = 0.05  # m/s - Ackermann needs forward motion to steer
        self.waypoint_threshold = 0.15  # meters

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
        kp = 2.0
        steer = kp * angle_error
        return float(np.clip(steer, -self.max_steering_angle, self.max_steering_angle))

    def calculate_waypoints(self):
        """Calculate waypoints around the track based on corridor layout"""
        # Get corridor widths from metadata
        corridor_widths = self.metadata['corridor_widths']
        direction = self.metadata['starting_conditions']['direction']

        # Convert widths from mm to meters
        north_width = corridor_widths['north']['width_mm'] / 1000.0
        south_width = corridor_widths['south']['width_mm'] / 1000.0
        east_width = corridor_widths['east']['width_mm'] / 1000.0
        west_width = corridor_widths['west']['width_mm'] / 1000.0

        # Track bounds
        track_min = 0.0
        track_max = 3.0
        track_center = 1.5

        # Calculate corridor centers (middle of each corridor)
        north_center_y = track_max - north_width / 2
        south_center_y = south_width / 2
        east_center_x = track_max - east_width / 2
        west_center_x = west_width / 2

        # Safety margins
        corner_entry = 0.6
        corner_exit = 0.7

        # Helper function to create corner transition waypoint
        def corner_waypoint(from_x, from_y, to_x, to_y, blend=0.5):
            return (from_x * blend + to_x * (1-blend),
                    from_y * blend + to_y * (1-blend))

        # North corridor: Enter from East, exit to West (X decreases)
        north_corridor_cw = [
            (track_max - corner_entry, north_center_y),
            (track_center + 0.6, north_center_y),
            (track_center + 0.3, north_center_y),
            (track_center, north_center_y),
            (track_center - 0.3, north_center_y),
            (track_center - 0.6, north_center_y),
            (track_min + corner_exit, north_center_y),
        ]
        north_to_west_corner = corner_waypoint(
            track_min + corner_exit, north_center_y,
            west_center_x, track_max - corner_entry,
            blend=0.6
        )

        # East corridor: Enter from South, exit to North (Y increases)
        east_corridor_cw = [
            (east_center_x, track_min + corner_entry),
            (east_center_x, track_center - 0.6),
            (east_center_x, track_center - 0.3),
            (east_center_x, track_center),
            (east_center_x, track_center + 0.3),
            (east_center_x, track_center + 0.6),
            (east_center_x, track_max - corner_exit),
        ]
        east_to_north_corner = corner_waypoint(
            east_center_x, track_max - corner_exit,
            track_max - corner_entry, north_center_y,
            blend=0.6
        )

        # South corridor: Enter from West, exit to East (X increases)
        south_corridor_cw = [
            (track_min + corner_entry, south_center_y),
            (track_center - 0.6, south_center_y),
            (track_center - 0.3, south_center_y),
            (track_center, south_center_y),
            (track_center + 0.3, south_center_y),
            (track_center + 0.6, south_center_y),
            (track_max - corner_exit, south_center_y),
        ]
        south_to_east_corner = corner_waypoint(
            track_max - corner_exit, south_center_y,
            east_center_x, track_min + corner_entry,
            blend=0.6
        )

        # West corridor: Enter from North, exit to South (Y decreases)
        west_corridor_cw = [
            (west_center_x, track_max - corner_entry),
            (west_center_x, track_center + 0.6),
            (west_center_x, track_center + 0.3),
            (west_center_x, track_center),
            (west_center_x, track_center - 0.3),
            (west_center_x, track_center - 0.6),
            (west_center_x, track_min + corner_exit),
        ]
        west_to_south_corner = corner_waypoint(
            west_center_x, track_min + corner_exit,
            track_min + corner_entry, south_center_y,
            blend=0.6
        )

        # Get starting section
        start_section = self.metadata['starting_conditions']['section'].lower()
        start_pos = self.metadata['starting_conditions']['position']
        start_x, start_y = start_pos['x'], start_pos['y']

        # Order corridors and corners based on starting section and direction
        # NOTE: The *_corridor_cw lists are ordered for CCW traversal (W→E, S→N, etc.)
        # For actual CW traversal, we reverse them. Variable names kept for compatibility.
        if direction == 'clockwise':
            if start_section == 'north':
                all_corridors = [list(reversed(north_corridor_cw)), [east_to_north_corner],
                               list(reversed(east_corridor_cw)), [south_to_east_corner],
                               list(reversed(south_corridor_cw)), [west_to_south_corner],
                               list(reversed(west_corridor_cw)), [north_to_west_corner]]
            elif start_section == 'east':
                all_corridors = [list(reversed(east_corridor_cw)), [south_to_east_corner],
                               list(reversed(south_corridor_cw)), [west_to_south_corner],
                               list(reversed(west_corridor_cw)), [north_to_west_corner],
                               list(reversed(north_corridor_cw)), [east_to_north_corner]]
            elif start_section == 'south':
                all_corridors = [list(reversed(south_corridor_cw)), [west_to_south_corner],
                               list(reversed(west_corridor_cw)), [north_to_west_corner],
                               list(reversed(north_corridor_cw)), [east_to_north_corner],
                               list(reversed(east_corridor_cw)), [south_to_east_corner]]
            else:  # west
                all_corridors = [list(reversed(west_corridor_cw)), [north_to_west_corner],
                               list(reversed(north_corridor_cw)), [east_to_north_corner],
                               list(reversed(east_corridor_cw)), [south_to_east_corner],
                               list(reversed(south_corridor_cw)), [west_to_south_corner]]
        else:  # counterclockwise
            if start_section == 'north':
                all_corridors = [north_corridor_cw, [north_to_west_corner], west_corridor_cw,
                               [west_to_south_corner], south_corridor_cw, [south_to_east_corner],
                               east_corridor_cw, [east_to_north_corner]]
            elif start_section == 'east':
                all_corridors = [east_corridor_cw, [east_to_north_corner], north_corridor_cw,
                               [north_to_west_corner], west_corridor_cw, [west_to_south_corner],
                               south_corridor_cw, [south_to_east_corner]]
            elif start_section == 'south':
                all_corridors = [south_corridor_cw, [south_to_east_corner], east_corridor_cw,
                               [east_to_north_corner], north_corridor_cw, [north_to_west_corner],
                               west_corridor_cw, [west_to_south_corner]]
            else:  # west
                all_corridors = [west_corridor_cw, [west_to_south_corner], south_corridor_cw,
                               [south_to_east_corner], east_corridor_cw, [east_to_north_corner],
                               north_corridor_cw, [north_to_west_corner]]

        # Find closest waypoint in the FIRST corridor (starting corridor)
        first_corridor = all_corridors[0]
        min_dist = float('inf')
        start_idx = 0
        for i, (wx, wy) in enumerate(first_corridor):
            dist = math.sqrt((wx - start_x)**2 + (wy - start_y)**2)
            if dist < min_dist:
                min_dist = dist
                start_idx = i

        # Build final waypoint list (no wrap-around in first corridor)
        # Partial first corridor: from start position to corridor end
        waypoints = list(first_corridor[start_idx:])

        # Rest of first lap: corners and remaining corridors
        for corridor in all_corridors[1:]:
            waypoints.extend(corridor)

        # Full loops for additional laps
        full_loop = []
        for corridor in all_corridors:
            full_loop.extend(corridor)

        for _ in range(self.num_laps - 1):
            waypoints.extend(full_loop)

        # Complete final lap: traverse the skipped start of first corridor
        if start_idx > 0:
            waypoints.extend(first_corridor[:start_idx])

        return waypoints

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
        self.lidar_ranges[np.isinf(self.lidar_ranges)] = self.lidar_max_range

        self.get_logger().info(
            f'LIDAR received: {num_points} points, {num_inf} inf values, '
            f'angle range: [{msg.angle_min:.2f}, {msg.angle_max:.2f}]',
            throttle_duration_sec=2.0
        )

    def get_min_distance_in_direction(self, target_angle, tolerance=0.3):
        """Get minimum LIDAR distance in a specific direction (in robot frame)"""
        if self.lidar_ranges is None or self.lidar_angles is None:
            return float('inf')

        target_angle = target_angle % (2 * math.pi)
        angle_diffs = np.abs(self.lidar_angles - target_angle)
        angle_diffs = np.minimum(angle_diffs, 2*math.pi - angle_diffs)

        mask = angle_diffs < tolerance
        if np.any(mask):
            return np.min(self.lidar_ranges[mask])
        return float('inf')

    def check_collision_risk(self):
        """Check for collision risk using LIDAR data"""
        if self.lidar_ranges is None:
            return 'safe', {
                'forward': float('inf'),
                'left': float('inf'),
                'right': float('inf')
            }

        forward_dist = self.get_min_distance_in_direction(0, tolerance=0.5)
        left_dist = self.get_min_distance_in_direction(math.pi/2, tolerance=0.4)
        right_dist = self.get_min_distance_in_direction(-math.pi/2, tolerance=0.4)

        distances = {
            'forward': forward_dist,
            'left': left_dist,
            'right': right_dist
        }

        self.get_logger().info(
            f'LIDAR: F={forward_dist:.2f}m L={left_dist:.2f}m R={right_dist:.2f}m',
            throttle_duration_sec=0.5
        )

        if forward_dist < self.critical_distance:
            return 'critical', distances
        elif forward_dist < self.danger_distance:
            return 'danger', distances
        elif left_dist < self.critical_distance or right_dist < self.critical_distance:
            return 'side_critical', distances
        elif left_dist < self.safe_distance or right_dist < self.safe_distance:
            return 'side_warning', distances

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

        # Get current target waypoint
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        robot_x, robot_y = self.current_pos

        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)

        # Check if we reached waypoint
        if distance < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx}/{len(self.waypoints)} at ({target_x:.2f}, {target_y:.2f})')

            if self.current_waypoint_idx % 16 == 0:
                self.laps_completed = self.current_waypoint_idx // 16
                self.get_logger().info(f'Lap {self.laps_completed}/{self.num_laps} completed')

            if self.current_waypoint_idx < len(self.waypoints):
                target_x, target_y = self.waypoints[self.current_waypoint_idx]
                dx = target_x - robot_x
                dy = target_y - robot_y
                distance = math.sqrt(dx**2 + dy**2)

        # Calculate angle to waypoint
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.current_yaw

        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        self.get_logger().info(
            f'NAV: pos=({robot_x:.2f},{robot_y:.2f}) yaw={math.degrees(self.current_yaw):.1f} '
            f'target=({target_x:.2f},{target_y:.2f}) dist={distance:.2f}m angle_err={math.degrees(angle_error):.1f}',
            throttle_duration_sec=1.0
        )

        # Check for collision risk using LIDAR
        risk_level, distances = self.check_collision_risk()

        vel_msg = Twist()

        # ESCAPE MODE: reverse with steering
        # NOTE: Ackermann reverse inverts the steering effect on yaw.
        # Reverse + left steer → nose swings RIGHT; reverse + right steer → nose swings LEFT.
        # So steer TOWARD the close wall while reversing to swing nose AWAY from it.
        if self.escape_mode:
            self.escape_counter += 1
            left_space = distances['left']
            right_space = distances['right']

            if left_space > right_space:
                # Wall on right → steer right while reversing → nose goes left (away from right wall)
                turn_dir = "RIGHT"
                vel_msg.angular.z = -self.max_steering_angle * 0.8
            else:
                # Wall on left → steer left while reversing → nose goes right (away from left wall)
                turn_dir = "LEFT"
                vel_msg.angular.z = self.max_steering_angle * 0.8

            self.get_logger().warn(
                f'ESCAPE MODE [{self.escape_counter}/{self.escape_duration}]: '
                f'L={left_space:.2f}m R={right_space:.2f}m -> Turning {turn_dir}'
            )
            vel_msg.linear.x = -0.20

            if self.escape_counter >= self.escape_duration:
                self.escape_mode = False
                self.escape_counter = 0
                self.get_logger().info('Escape maneuver complete')

        # COLLISION AVOIDANCE LOGIC
        elif risk_level == 'critical':
            self.get_logger().warn(f'CRITICAL: Forward distance {distances["forward"]:.2f}m - ENTERING ESCAPE MODE!')
            self.escape_mode = True
            self.escape_counter = 0
            vel_msg.linear.x = -0.20
            vel_msg.angular.z = 0.0

        elif risk_level == 'danger':
            self.get_logger().warn(f'DANGER: Forward distance {distances["forward"]:.2f}m - SLOWING!')
            vel_msg.linear.x = max(0.05, self.min_forward_speed)
            vel_msg.angular.z = self.compute_steering_angle(angle_error)

        elif risk_level == 'side_critical':
            if distances['left'] < self.critical_distance:
                self.get_logger().warn(f'Side collision risk: LEFT {distances["left"]:.2f}m')
                correction = -0.15  # Steer right (away from left wall)
            else:
                self.get_logger().warn(f'Side collision risk: RIGHT {distances["right"]:.2f}m')
                correction = 0.15   # Steer left (away from right wall)

            vel_msg.linear.x = self.max_linear_speed * 0.5
            steer = self.compute_steering_angle(angle_error) + correction
            vel_msg.angular.z = float(np.clip(steer,
                                              -self.max_steering_angle,
                                              self.max_steering_angle))

        else:
            # INTELLIGENT NAVIGATION with Ackermann steering
            forward_dist = distances['forward']
            left_dist = distances['left']
            right_dist = distances['right']

            # PREDICTIVE TURNING
            predictive_steer = 0.0

            if forward_dist < 1.5:
                space_diff = left_dist - right_dist

                if abs(space_diff) > 0.08:
                    turn_urgency = (1.5 - forward_dist) / 1.5

                    if forward_dist < 0.4:
                        predictive_steer = np.sign(space_diff) * 0.4 * turn_urgency
                    elif forward_dist < 0.7:
                        predictive_steer = np.sign(space_diff) * 0.35 * turn_urgency
                    elif forward_dist < 1.0:
                        predictive_steer = np.sign(space_diff) * 0.25 * turn_urgency
                    else:
                        predictive_steer = np.sign(space_diff) * 0.15 * turn_urgency

                    self.get_logger().info(
                        f'Predictive: F={forward_dist:.2f}m, L={left_dist:.2f}m, '
                        f'R={right_dist:.2f}m, Diff={space_diff:.2f}m, Steer={predictive_steer:.2f}',
                        throttle_duration_sec=0.5
                    )

            # Linear velocity
            if forward_dist < 0.4:
                vel_msg.linear.x = self.max_linear_speed * 0.3
            elif forward_dist < 0.7:
                vel_msg.linear.x = self.max_linear_speed * 0.5
            elif abs(angle_error) > 0.5:
                vel_msg.linear.x = self.max_linear_speed * 0.4
            elif abs(angle_error) > 0.2:
                vel_msg.linear.x = self.max_linear_speed * 0.6
            else:
                vel_msg.linear.x = self.max_linear_speed

            if distance < 0.3:
                vel_msg.linear.x *= 0.7

            # Enforce minimum forward speed for Ackermann
            vel_msg.linear.x = max(vel_msg.linear.x, self.min_forward_speed)

            # Side warning correction
            side_correction = 0.0
            if risk_level == 'side_warning':
                if left_dist < self.safe_distance:
                    side_correction = -0.1   # Steer right (away from left wall)
                elif right_dist < self.safe_distance:
                    side_correction = 0.1    # Steer left (away from right wall)

            # Steering angle: combine waypoint + predictive + side correction
            waypoint_steer = self.compute_steering_angle(angle_error)

            if abs(predictive_steer) > 0.05:
                waypoint_weight = 0.2
                obstacle_weight = 0.8
                total_steer = (waypoint_weight * waypoint_steer +
                               obstacle_weight * predictive_steer +
                               side_correction)
            else:
                total_steer = waypoint_steer + side_correction

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
