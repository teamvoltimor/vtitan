#!/usr/bin/env python3
"""
WRO Track Navigator - Completes Open Challenge

Navigates robot around track for 3 laps using waypoint following.
Uses metadata to understand corridor layout and widths.

Usage:
    python3 track_navigator.py --metadata scenario_0000_metadata.json --laps 3
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import json
import argparse
from pathlib import Path
import numpy as np


class TrackNavigator(Node):
    """Navigate robot around WRO track using waypoint following"""

    def __init__(self, metadata_path, num_laps=3):
        super().__init__('track_navigator')

        # Load metadata
        self.metadata = self.load_metadata(metadata_path)
        self.num_laps = num_laps

        # Calculate waypoints around track
        self.waypoints = self.calculate_waypoints()
        self.current_waypoint_idx = 0
        self.laps_completed = 0

        # Robot state
        self.current_pos = None
        self.current_yaw = None

        # Control parameters (tuned for accurate tracking)
        self.max_linear_speed = 0.20  # m/s (reduced for better control)
        self.max_angular_speed = 0.5  # rad/s (reduced for smoother turns)
        self.waypoint_threshold = 0.12  # meters - tighter threshold for accuracy

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

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f'Navigator initialized: {len(self.waypoints)} waypoints, {num_laps} laps')
        self.get_logger().info(f'Starting section: {self.metadata["starting_conditions"]["section"]}')
        self.get_logger().info(f'Direction: {self.metadata["starting_conditions"]["direction"]}')

    def load_metadata(self, metadata_path):
        """Load scenario metadata"""
        with open(metadata_path, 'r') as f:
            return json.load(f)

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

        # Define corridor waypoint groups
        # Clockwise goes: North->East->South->West
        # North corridor: Enter from East, exit to West (X decreases)
        north_corridor_cw = [
            (track_max - 0.3, north_center_y),  # East end (enter here)
            (track_center + 0.5, north_center_y),
            (track_center, north_center_y),
            (track_center - 0.5, north_center_y),
            (track_min + 0.3, north_center_y),  # West end (exit here)
        ]

        # East corridor: Enter from South, exit to North (Y increases)
        east_corridor_cw = [
            (east_center_x, track_min + 0.3),  # South end (enter here)
            (east_center_x, track_center - 0.5),
            (east_center_x, track_center),
            (east_center_x, track_center + 0.5),
            (east_center_x, track_max - 0.3),  # North end (exit here)
        ]

        # South corridor: Enter from West, exit to East (X increases)
        south_corridor_cw = [
            (track_min + 0.3, south_center_y),  # West end (enter here)
            (track_center - 0.5, south_center_y),
            (track_center, south_center_y),
            (track_center + 0.5, south_center_y),
            (track_max - 0.3, south_center_y),  # East end (exit here)
        ]

        # West corridor: Enter from North, exit to South (Y decreases)
        west_corridor_cw = [
            (west_center_x, track_max - 0.3),  # North end (enter here)
            (west_center_x, track_center + 0.5),
            (west_center_x, track_center),
            (west_center_x, track_center - 0.5),
            (west_center_x, track_min + 0.3),  # South end (exit here)
        ]

        # Get starting section
        start_section = self.metadata['starting_conditions']['section']

        # Get starting position
        start_pos = self.metadata['starting_conditions']['position']
        start_x, start_y = start_pos['x'], start_pos['y']

        # Order corridors based on starting section and direction
        if direction == 'clockwise':
            if start_section == 'north':
                all_corridors = [north_corridor_cw, east_corridor_cw, south_corridor_cw, west_corridor_cw]
            elif start_section == 'east':
                all_corridors = [east_corridor_cw, south_corridor_cw, west_corridor_cw, north_corridor_cw]
            elif start_section == 'south':
                all_corridors = [south_corridor_cw, west_corridor_cw, north_corridor_cw, east_corridor_cw]
            else:  # west
                all_corridors = [west_corridor_cw, north_corridor_cw, east_corridor_cw, south_corridor_cw]
        else:  # counterclockwise: North->West->South->East
            if start_section == 'north':
                all_corridors = [list(reversed(north_corridor_cw)), list(reversed(west_corridor_cw)),
                               list(reversed(south_corridor_cw)), list(reversed(east_corridor_cw))]
            elif start_section == 'west':
                all_corridors = [list(reversed(west_corridor_cw)), list(reversed(south_corridor_cw)),
                               list(reversed(east_corridor_cw)), list(reversed(north_corridor_cw))]
            elif start_section == 'south':
                all_corridors = [list(reversed(south_corridor_cw)), list(reversed(east_corridor_cw)),
                               list(reversed(north_corridor_cw)), list(reversed(west_corridor_cw))]
            else:  # east
                all_corridors = [list(reversed(east_corridor_cw)), list(reversed(north_corridor_cw)),
                               list(reversed(west_corridor_cw)), list(reversed(south_corridor_cw))]

        # Find closest waypoint in the FIRST corridor (starting corridor)
        first_corridor = all_corridors[0]
        min_dist = float('inf')
        start_idx = 0
        for i, (wx, wy) in enumerate(first_corridor):
            dist = math.sqrt((wx - start_x)**2 + (wy - start_y)**2)
            if dist < min_dist:
                min_dist = dist
                start_idx = i

        # Build final waypoint list: start from closest point in first corridor, then rest of corridors
        waypoints = first_corridor[start_idx:] + first_corridor[:start_idx]  # Reorder first corridor
        for corridor in all_corridors[1:]:  # Add remaining corridors in order
            waypoints.extend(corridor)

        # Repeat waypoints for number of laps
        waypoints = waypoints * self.num_laps

        return waypoints

    def odom_callback(self, msg):
        """Update robot position from odometry"""
        # Extract position
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """Main control loop - waypoint following"""
        # Wait for first odometry message
        if self.current_pos is None or self.current_yaw is None:
            return

        # Check if we've completed all waypoints
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info(f'Completed {self.num_laps} laps! Stopping.')
            self.stop_robot()
            rclpy.shutdown()
            return

        # Get current target waypoint
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        robot_x, robot_y = self.current_pos

        # Calculate distance to waypoint
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)

        # Check if we reached waypoint
        if distance < self.waypoint_threshold:
            self.current_waypoint_idx += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx}/{len(self.waypoints)} at ({target_x:.2f}, {target_y:.2f})')

            # Track lap completion (every 16 waypoints = 1 lap)
            if self.current_waypoint_idx % 16 == 0:
                self.laps_completed = self.current_waypoint_idx // 16
                self.get_logger().info(f'Lap {self.laps_completed}/{self.num_laps} completed')

            # Get next waypoint if available
            if self.current_waypoint_idx < len(self.waypoints):
                target_x, target_y = self.waypoints[self.current_waypoint_idx]
                dx = target_x - robot_x
                dy = target_y - robot_y
                distance = math.sqrt(dx**2 + dy**2)

        # Calculate angle to waypoint
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.current_yaw

        # Normalize angle to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Simple proportional control
        vel_msg = Twist()

        # Linear velocity (slow down when turning)
        if abs(angle_error) > 0.5:  # Large angle error
            vel_msg.linear.x = self.max_linear_speed * 0.3  # Slow down
        elif abs(angle_error) > 0.2:  # Medium angle error
            vel_msg.linear.x = self.max_linear_speed * 0.6
        else:  # Small angle error
            vel_msg.linear.x = self.max_linear_speed

        # Also slow down when close to waypoint
        if distance < 0.3:
            vel_msg.linear.x *= 0.7

        # Angular velocity (proportional to angle error)
        kp_angular = 1.5  # Proportional gain (reduced for smoother control)
        vel_msg.angular.z = np.clip(kp_angular * angle_error,
                                    -self.max_angular_speed,
                                    self.max_angular_speed)

        # Publish velocity
        self.vel_publisher.publish(vel_msg)

        # Debug logging (every 2 seconds)
        self.log_counter += 1
        if self.log_counter % 40 == 0:  # 20 Hz * 2 sec = 40
            self.get_logger().info(
                f'Pos: ({robot_x:.2f}, {robot_y:.2f}), '
                f'Target WP{self.current_waypoint_idx}: ({target_x:.2f}, {target_y:.2f}), '
                f'Dist: {distance:.2f}m, '
                f'Vel: {vel_msg.linear.x:.2f} m/s, {vel_msg.angular.z:.2f} rad/s'
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

    # Check if metadata file exists
    if not Path(args.metadata).exists():
        print(f"ERROR: Metadata file not found: {args.metadata}")
        return 1

    # Initialize ROS2
    rclpy.init()

    try:
        # Create and run navigator
        navigator = TrackNavigator(
            metadata_path=args.metadata,
            num_laps=args.laps
        )

        # Spin until navigation completes
        rclpy.spin(navigator)

    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
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
