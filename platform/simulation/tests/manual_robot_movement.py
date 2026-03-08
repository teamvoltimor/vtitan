#!/usr/bin/env python3
"""Manual Gazebo integration script — verifies Ackermann steering over ROS2.

Requires a live Gazebo session with the robot model loaded.
Run with: pixi run python simulation/tests/manual_robot_movement.py

NOT a pytest test — does not run in CI.
"""

import time

import rclpy  # type: ignore[import]
from geometry_msgs.msg import Twist  # type: ignore[import]
from rclpy.node import Node  # type: ignore[import]


class RobotTester(Node):
    def __init__(self) -> None:
        super().__init__("robot_tester")
        self.vel_publisher = self.create_publisher(Twist, "/wro_robot/cmd_vel", 10)
        self.get_logger().info("Robot tester initialized")

        # Wait for connections
        time.sleep(1)

    def test_turn_left(self) -> None:
        """Test turning left (positive steering angle + forward speed)"""
        self.get_logger().info("Testing TURN LEFT for 3 seconds (steer + forward)...")
        msg = Twist()
        msg.linear.x = 0.15  # Forward speed required for Ackermann
        msg.angular.z = 0.4  # Steer left

        for _i in range(30):  # 3 seconds at 10Hz
            self.vel_publisher.publish(msg)
            time.sleep(0.1)

        self.stop()

    def test_turn_right(self) -> None:
        """Test turning right (negative steering angle + forward speed)"""
        self.get_logger().info("Testing TURN RIGHT for 3 seconds (steer + forward)...")
        msg = Twist()
        msg.linear.x = 0.15  # Forward speed required for Ackermann
        msg.angular.z = -0.4  # Steer right

        for _i in range(30):  # 3 seconds at 10Hz
            self.vel_publisher.publish(msg)
            time.sleep(0.1)

        self.stop()

    def test_forward(self) -> None:
        """Test moving forward"""
        self.get_logger().info("Testing FORWARD for 2 seconds...")
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0

        for _i in range(20):  # 2 seconds at 10Hz
            self.vel_publisher.publish(msg)
            time.sleep(0.1)

        self.stop()

    def stop(self) -> None:
        """Stop the robot"""
        self.get_logger().info("STOPPING")
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.vel_publisher.publish(msg)
        time.sleep(0.5)


def main() -> None:
    rclpy.init()
    tester = RobotTester()

    try:
        # Run tests sequentially
        tester.get_logger().info("=" * 50)
        tester.get_logger().info("STARTING ROBOT MOVEMENT TESTS (Ackermann)")
        tester.get_logger().info("Watch Gazebo to see if robot moves!")
        tester.get_logger().info("=" * 50)

        time.sleep(2)

        # Test 1: Turn left (with forward motion)
        tester.test_turn_left()
        time.sleep(1)

        # Test 2: Turn right (with forward motion)
        tester.test_turn_right()
        time.sleep(1)

        # Test 3: Forward
        tester.test_forward()

        tester.get_logger().info("=" * 50)
        tester.get_logger().info("TESTS COMPLETE!")
        tester.get_logger().info("If robot did NOT move, Ackermann steering plugin has issues")
        tester.get_logger().info("=" * 50)

    except KeyboardInterrupt:
        pass
    finally:
        tester.stop()
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
