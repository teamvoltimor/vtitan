#!/usr/bin/env python3
"""
Simple Robot Driver for Training Video Recording (Ackermann Steering)

Drives the robot around the WRO track using simple timed behavior.
Designed to work with the training_camera (static camera at starting position).
Sends steering angles via angular.z (Ackermann convention).

Usage:
    python3 simple_robot_driver.py --direction clockwise --duration 30
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time
import argparse

from src.config.constants import RobotSpecs


class SimpleRobotDriver(Node):
    """Simple robot driver using Ackermann steering angles"""

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
        self.forward_speed = 0.3  # m/s
        self.steering_angle = 0.35  # rad (~20 deg) for turns

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

        self.state_timer += 0.02

        if self.state == 'forward':
            # Drive forward for 3 seconds
            vel_msg.linear.x = self.forward_speed
            vel_msg.angular.z = 0.0

            if self.state_timer >= 3.0:
                self.state = 'turning'
                self.state_timer = 0

        elif self.state == 'turning':
            # Ackermann needs forward speed to turn (no pivot)
            vel_msg.linear.x = self.forward_speed * 0.5

            # Steering angle based on direction
            if self.direction == 'clockwise':
                vel_msg.angular.z = -self.steering_angle  # Steer right
            else:
                vel_msg.angular.z = self.steering_angle   # Steer left

            # Slightly longer turn for Ackermann (wider turning radius)
            if self.state_timer >= 2.0:
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
        driver = SimpleRobotDriver(
            direction=args.direction,
            duration=args.duration
        )
        rclpy.spin(driver)

    except KeyboardInterrupt:
        pass
    finally:
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
