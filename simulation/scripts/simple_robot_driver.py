#!/usr/bin/env python3
"""
Simple Robot Driver for Training Video Recording

Drives the robot around the WRO track using simple wall-following behavior.
Designed to work with the training_camera (static camera at starting position).

Usage:
    python3 simple_robot_driver.py --direction clockwise --duration 30
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time
import argparse


class SimpleRobotDriver(Node):
    """Simple robot driver that drives forward with basic control"""

    def __init__(self, direction='clockwise', duration=30):
        super().__init__('simple_robot_driver')

        self.direction = direction
        self.duration = duration
        self.start_time = time.time()

        # Create velocity publisher
        self.vel_publisher = self.create_publisher(
            Twist,
            '/wro_robot/cmd_vel',
            10
        )

        # Control parameters
        self.forward_speed = 0.3  # m/s (30 cm/s - slow and steady)
        self.angular_speed = 0.5  # rad/s for turning

        # Simple state machine
        self.state = 'forward'
        self.state_timer = 0

        # Timer for control loop (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(f'Robot driver started: {direction} direction, {duration}s duration')

    def control_loop(self):
        """Simple control loop - drive forward with periodic turns"""
        elapsed = time.time() - self.start_time

        # Stop after duration
        if elapsed >= self.duration:
            self.stop_robot()
            self.get_logger().info('Driving duration reached, stopping')
            rclpy.shutdown()
            return

        # Create velocity command
        vel_msg = Twist()

        # Simple behavior: drive forward, turn at intervals
        # This simulates going around the track
        self.state_timer += 0.02

        if self.state == 'forward':
            # Drive forward for 3 seconds
            vel_msg.linear.x = self.forward_speed
            vel_msg.angular.z = 0.0

            if self.state_timer >= 3.0:
                self.state = 'turning'
                self.state_timer = 0

        elif self.state == 'turning':
            # Turn for 1.5 seconds
            vel_msg.linear.x = self.forward_speed * 0.5  # Slow down while turning

            # Turn direction based on clockwise/counterclockwise
            if self.direction == 'clockwise':
                vel_msg.angular.z = -self.angular_speed  # Turn right
            else:
                vel_msg.angular.z = self.angular_speed   # Turn left

            if self.state_timer >= 1.5:
                self.state = 'forward'
                self.state_timer = 0

        # Publish velocity
        self.vel_publisher.publish(vel_msg)

    def stop_robot(self):
        """Stop the robot"""
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.vel_publisher.publish(vel_msg)


def main():
    parser = argparse.ArgumentParser(description='Simple robot driver for video recording')
    parser.add_argument('--direction', type=str, default='clockwise',
                       choices=['clockwise', 'counterclockwise'],
                       help='Driving direction around track')
    parser.add_argument('--duration', type=int, default=30,
                       help='Driving duration in seconds')

    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()

    try:
        # Create and run driver
        driver = SimpleRobotDriver(
            direction=args.direction,
            duration=args.duration
        )

        # Spin until duration expires
        rclpy.spin(driver)

    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        try:
            driver.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
