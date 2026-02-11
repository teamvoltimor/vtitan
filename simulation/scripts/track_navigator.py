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
from sensor_msgs.msg import LaserScan
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

        # LIDAR data for collision avoidance (RPLIDAR C1: 0.05-12m range)
        self.lidar_ranges = None
        self.lidar_angles = None
        self.lidar_max_range = 12.0  # RPLIDAR C1 maximum range

        # Collision avoidance parameters
        self.danger_distance = 0.25  # meters - start slowing down (increased)
        self.critical_distance = 0.18  # meters - emergency maneuver (increased)
        self.safe_distance = 0.30  # meters - minimum safe distance

        # Escape maneuver state
        self.escape_mode = False
        self.escape_counter = 0
        self.escape_duration = 20  # frames (~1 second at 20Hz)

        # Control parameters (balanced for speed and safety)
        self.max_linear_speed = 0.25  # m/s (faster for competition)
        self.max_angular_speed = 1.0  # rad/s (much faster turns - ~57 deg/s)
        self.waypoint_threshold = 0.15  # meters - switch waypoints earlier to avoid overshooting

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

        # Define corridor waypoint groups with smooth corner transitions
        # Add intermediate corner waypoints to create gradual turns
        # Clockwise goes: North->East->South->West

        # Safety margins
        corner_entry = 0.6   # Distance from corner when entering (more conservative)
        corner_exit = 0.7    # Distance from corner when exiting

        # Helper function to create corner transition waypoint
        def corner_waypoint(from_x, from_y, to_x, to_y, blend=0.5):
            """Create intermediate waypoint between corridors"""
            return (from_x * blend + to_x * (1-blend),
                    from_y * blend + to_y * (1-blend))

        # North corridor: Enter from East, exit to West (X decreases)
        north_corridor_cw = [
            (track_max - corner_entry, north_center_y),  # East entry
            (track_center + 0.6, north_center_y),
            (track_center + 0.3, north_center_y),
            (track_center, north_center_y),  # Center
            (track_center - 0.3, north_center_y),
            (track_center - 0.6, north_center_y),
            (track_min + corner_exit, north_center_y),  # West exit (pre-corner)
        ]
        # Add smooth corner transition to West corridor
        north_to_west_corner = corner_waypoint(
            track_min + corner_exit, north_center_y,  # Last North point
            west_center_x, track_max - corner_entry,   # First West point
            blend=0.6  # Closer to North side
        )

        # East corridor: Enter from South, exit to North (Y increases)
        east_corridor_cw = [
            (east_center_x, track_min + corner_entry),  # South entry
            (east_center_x, track_center - 0.6),
            (east_center_x, track_center - 0.3),
            (east_center_x, track_center),  # Center
            (east_center_x, track_center + 0.3),
            (east_center_x, track_center + 0.6),
            (east_center_x, track_max - corner_exit),  # North exit (pre-corner)
        ]
        # Add smooth corner transition to North corridor
        east_to_north_corner = corner_waypoint(
            east_center_x, track_max - corner_exit,         # Last East point
            track_max - corner_entry, north_center_y,       # First North point
            blend=0.6
        )

        # South corridor: Enter from West, exit to East (X increases)
        south_corridor_cw = [
            (track_min + corner_entry, south_center_y),  # West entry
            (track_center - 0.6, south_center_y),
            (track_center - 0.3, south_center_y),
            (track_center, south_center_y),  # Center
            (track_center + 0.3, south_center_y),
            (track_center + 0.6, south_center_y),
            (track_max - corner_exit, south_center_y),  # East exit (pre-corner)
        ]
        # Add smooth corner transition to East corridor
        south_to_east_corner = corner_waypoint(
            track_max - corner_exit, south_center_y,    # Last South point
            east_center_x, track_min + corner_entry,    # First East point
            blend=0.6
        )

        # West corridor: Enter from North, exit to South (Y decreases)
        west_corridor_cw = [
            (west_center_x, track_max - corner_entry),  # North entry
            (west_center_x, track_center + 0.6),
            (west_center_x, track_center + 0.3),
            (west_center_x, track_center),  # Center
            (west_center_x, track_center - 0.3),
            (west_center_x, track_center - 0.6),
            (west_center_x, track_min + corner_exit),  # South exit (pre-corner)
        ]
        # Add smooth corner transition to South corridor
        west_to_south_corner = corner_waypoint(
            west_center_x, track_min + corner_exit,     # Last West point
            track_min + corner_entry, south_center_y,   # First South point
            blend=0.6
        )

        # Get starting section
        start_section = self.metadata['starting_conditions']['section']

        # Get starting position
        start_pos = self.metadata['starting_conditions']['position']
        start_x, start_y = start_pos['x'], start_pos['y']

        # Order corridors and corners based on starting section and direction
        if direction == 'clockwise':
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
        else:  # counterclockwise: North->West->South->East
            if start_section == 'north':
                all_corridors = [list(reversed(north_corridor_cw)), [east_to_north_corner],
                               list(reversed(east_corridor_cw)), [south_to_east_corner],
                               list(reversed(south_corridor_cw)), [west_to_south_corner],
                               list(reversed(west_corridor_cw)), [north_to_west_corner]]
            elif start_section == 'west':
                all_corridors = [list(reversed(west_corridor_cw)), [north_to_west_corner],
                               list(reversed(north_corridor_cw)), [east_to_north_corner],
                               list(reversed(east_corridor_cw)), [south_to_east_corner],
                               list(reversed(south_corridor_cw)), [west_to_south_corner]]
            elif start_section == 'south':
                all_corridors = [list(reversed(south_corridor_cw)), [west_to_south_corner],
                               list(reversed(west_corridor_cw)), [north_to_west_corner],
                               list(reversed(north_corridor_cw)), [east_to_north_corner],
                               list(reversed(east_corridor_cw)), [south_to_east_corner]]
            else:  # east
                all_corridors = [list(reversed(east_corridor_cw)), [south_to_east_corner],
                               list(reversed(south_corridor_cw)), [west_to_south_corner],
                               list(reversed(west_corridor_cw)), [north_to_west_corner],
                               list(reversed(north_corridor_cw)), [east_to_north_corner]]

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

    def lidar_callback(self, msg):
        """Update LIDAR data for collision avoidance (RPLIDAR C1)"""
        self.lidar_ranges = np.array(msg.ranges)
        num_points = len(self.lidar_ranges)
        self.lidar_angles = np.linspace(msg.angle_min, msg.angle_max, num_points)
        # Replace inf with max range (out of detection = far away)
        num_inf = np.sum(np.isinf(self.lidar_ranges))
        self.lidar_ranges[np.isinf(self.lidar_ranges)] = self.lidar_max_range

        # Debug: Log LIDAR data reception
        self.get_logger().info(
            f'LIDAR received: {num_points} points, {num_inf} inf values, '
            f'angle range: [{msg.angle_min:.2f}, {msg.angle_max:.2f}]',
            throttle_duration_sec=2.0
        )

    def get_min_distance_in_direction(self, target_angle, tolerance=0.3):
        """Get minimum LIDAR distance in a specific direction (in robot frame)"""
        if self.lidar_ranges is None or self.lidar_angles is None:
            return float('inf')

        # Normalize target angle to [0, 2π]
        target_angle = target_angle % (2 * math.pi)

        # Find measurements in the target direction
        angle_diffs = np.abs(self.lidar_angles - target_angle)
        # Handle wraparound
        angle_diffs = np.minimum(angle_diffs, 2*math.pi - angle_diffs)

        mask = angle_diffs < tolerance
        if np.any(mask):
            return np.min(self.lidar_ranges[mask])
        return float('inf')

    def check_collision_risk(self):
        """Check for collision risk using LIDAR data"""
        if self.lidar_ranges is None:
            # Return default safe distances when LIDAR not ready
            return 'safe', {
                'forward': float('inf'),
                'left': float('inf'),
                'right': float('inf')
            }

        # Check distances in key directions with wider tolerance for better wall detection
        # CRITICAL: LIDAR angle convention - need to verify if left/right are swapped
        # Forward (0°), Left (should be 90° or 270°?), Right (should be 270° or 90°?)
        forward_dist = self.get_min_distance_in_direction(0, tolerance=0.5)  # Wider forward cone
        # SWAPPED: Try opposite angles if behavior is inverted
        left_dist = self.get_min_distance_in_direction(3*math.pi/2, tolerance=0.4)  # Was π/2, now 3π/2
        right_dist = self.get_min_distance_in_direction(math.pi/2, tolerance=0.4)  # Was 3π/2, now π/2

        distances = {
            'forward': forward_dist,
            'left': left_dist,
            'right': right_dist
        }

        # Log distances for debugging
        self.get_logger().info(
            f'LIDAR: F={forward_dist:.2f}m L={left_dist:.2f}m R={right_dist:.2f}m',
            throttle_duration_sec=0.5
        )

        # Determine risk level
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

        # Debug: Log navigation state
        self.get_logger().info(
            f'NAV: pos=({robot_x:.2f},{robot_y:.2f}) yaw={math.degrees(self.current_yaw):.1f}° '
            f'target=({target_x:.2f},{target_y:.2f}) dist={distance:.2f}m angle_err={math.degrees(angle_error):.1f}°',
            throttle_duration_sec=1.0
        )

        # Check for collision risk using LIDAR
        risk_level, distances = self.check_collision_risk()

        # Create velocity command
        vel_msg = Twist()

        # ESCAPE MODE: Execute escape maneuver if triggered
        if self.escape_mode:
            self.escape_counter += 1

            # ESCAPE MANEUVER: Reverse while turning away from obstacle
            left_space = distances['left']
            right_space = distances['right']

            # Determine turn direction
            if left_space > right_space:
                turn_dir = "LEFT"
                vel_msg.angular.z = 0.6  # Turn left while reversing
            else:
                turn_dir = "RIGHT"
                vel_msg.angular.z = -0.6  # Turn right while reversing

            self.get_logger().warn(
                f'ESCAPE MODE [{self.escape_counter}/{self.escape_duration}]: '
                f'L={left_space:.2f}m R={right_space:.2f}m → Turning {turn_dir}'
            )
            vel_msg.linear.x = -0.20  # Reverse faster

            # Exit escape mode after duration
            if self.escape_counter >= self.escape_duration:
                self.escape_mode = False
                self.escape_counter = 0
                self.get_logger().info('Escape maneuver complete')

        # COLLISION AVOIDANCE LOGIC
        elif risk_level == 'critical':
            # EMERGENCY: Too close to wall ahead - ENTER ESCAPE MODE!
            self.get_logger().warn(f'CRITICAL: Forward distance {distances["forward"]:.2f}m - ENTERING ESCAPE MODE!')
            self.escape_mode = True
            self.escape_counter = 0
            vel_msg.linear.x = -0.20  # Start reversing
            vel_msg.angular.z = 0.0

        elif risk_level == 'danger':
            # DANGER: Slow down significantly
            self.get_logger().warn(f'DANGER: Forward distance {distances["forward"]:.2f}m - SLOWING!')
            vel_msg.linear.x = 0.05  # Very slow forward
            # Turn more aggressively toward waypoint to avoid obstacle
            kp_angular = 2.0
            vel_msg.angular.z = np.clip(kp_angular * angle_error,
                                        -self.max_angular_speed,
                                        self.max_angular_speed)

        elif risk_level == 'side_critical':
            # Too close to side wall - steer away
            if distances['left'] < self.critical_distance:
                self.get_logger().warn(f'Side collision risk: LEFT {distances["left"]:.2f}m')
                correction = 0.3  # Steer right
            else:
                self.get_logger().warn(f'Side collision risk: RIGHT {distances["right"]:.2f}m')
                correction = -0.3  # Steer left

            vel_msg.linear.x = self.max_linear_speed * 0.5
            kp_angular = 1.5
            vel_msg.angular.z = np.clip(kp_angular * angle_error + correction,
                                        -self.max_angular_speed,
                                        self.max_angular_speed)

        else:
            # INTELLIGENT NAVIGATION: Use LIDAR to predict and avoid obstacles
            forward_dist = distances['forward']
            left_dist = distances['left']
            right_dist = distances['right']

            # PREDICTIVE TURNING: Look ahead and turn toward open space
            predictive_turn = 0.0

            # Start turning earlier (1.5m) and more aggressively
            if forward_dist < 1.5:  # Forward obstacle within 1.5 meters
                # Calculate which side has more space
                space_diff = left_dist - right_dist

                if abs(space_diff) > 0.08:  # Lower threshold (8cm) for earlier response
                    # Turn toward the side with more space
                    # Strength increases as forward distance decreases
                    turn_urgency = (1.5 - forward_dist) / 1.5  # 0 to 1 scale

                    if forward_dist < 0.4:
                        # Very close: turn aggressively
                        predictive_turn = np.sign(space_diff) * 1.2 * turn_urgency
                    elif forward_dist < 0.7:
                        # Close obstacle: strong turn
                        predictive_turn = np.sign(space_diff) * 1.0 * turn_urgency
                    elif forward_dist < 1.0:
                        # Medium distance: moderate turn
                        predictive_turn = np.sign(space_diff) * 0.7 * turn_urgency
                    else:
                        # Far but detected: gentle turn
                        predictive_turn = np.sign(space_diff) * 0.4 * turn_urgency

                    self.get_logger().info(
                        f'Predictive: F={forward_dist:.2f}m, L={left_dist:.2f}m, '
                        f'R={right_dist:.2f}m, Diff={space_diff:.2f}m, Turn={predictive_turn:.2f}',
                        throttle_duration_sec=0.5
                    )

            # NORMAL NAVIGATION: Proportional control toward waypoint
            # Linear velocity (slow down based on forward obstacle and turning)
            if forward_dist < 0.4:
                vel_msg.linear.x = self.max_linear_speed * 0.3  # Very slow when close
            elif forward_dist < 0.7:
                vel_msg.linear.x = self.max_linear_speed * 0.5  # Slow when approaching
            elif abs(angle_error) > 0.5:  # Large angle error
                vel_msg.linear.x = self.max_linear_speed * 0.4  # Slow down for sharp turns
            elif abs(angle_error) > 0.2:  # Medium angle error
                vel_msg.linear.x = self.max_linear_speed * 0.6
            else:  # Small angle error and clear ahead
                vel_msg.linear.x = self.max_linear_speed

            # Also slow down when close to waypoint
            if distance < 0.3:
                vel_msg.linear.x *= 0.7

            # Apply side warning correction if needed
            side_correction = 0.0
            if risk_level == 'side_warning':
                if left_dist < self.safe_distance:
                    side_correction = 0.2  # Stronger steer right
                elif right_dist < self.safe_distance:
                    side_correction = -0.2  # Stronger steer left

            # Angular velocity: Combine waypoint following + predictive turning + side correction
            kp_angular = 2.0  # Increased gain for faster turning response

            # Blend waypoint navigation with obstacle avoidance
            if abs(predictive_turn) > 0.1:
                # Prioritize obstacle avoidance over waypoint when obstacle detected
                waypoint_weight = 0.2  # Further reduce waypoint influence
                obstacle_weight = 0.8  # Increase obstacle avoidance dominance
                total_angular = (waypoint_weight * kp_angular * angle_error +
                               obstacle_weight * predictive_turn +
                               side_correction)
            else:
                # Normal waypoint following
                total_angular = kp_angular * angle_error + side_correction

            vel_msg.angular.z = np.clip(total_angular,
                                        -self.max_angular_speed,
                                        self.max_angular_speed)

        # Log velocity commands for debugging
        self.get_logger().info(
            f'CMD: v={vel_msg.linear.x:.3f} m/s, ω={vel_msg.angular.z:.3f} rad/s, '
            f'risk={risk_level}, waypoint={self.current_waypoint_idx}',
            throttle_duration_sec=0.5
        )

        # Publish velocity
        self.vel_publisher.publish(vel_msg)

        # Debug logging (every 2 seconds)
        self.log_counter += 1
        if self.log_counter % 40 == 0:  # 20 Hz * 2 sec = 40
            lidar_info = ""
            if distances:
                lidar_info = f', LIDAR[F:{distances["forward"]:.2f} L:{distances["left"]:.2f} R:{distances["right"]:.2f}]'

            self.get_logger().info(
                f'Pos: ({robot_x:.2f}, {robot_y:.2f}), '
                f'Target WP{self.current_waypoint_idx}: ({target_x:.2f}, {target_y:.2f}), '
                f'Dist: {distance:.2f}m, '
                f'Vel: {vel_msg.linear.x:.2f} m/s, {vel_msg.angular.z:.2f} rad/s'
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
