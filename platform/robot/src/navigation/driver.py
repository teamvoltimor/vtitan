#!/usr/bin/env python3
"""Simple timed robot driver for training video recording (Ackermann steering).

Drives the robot around the WRO track using alternating forward and turning
phases.  Designed to work alongside the static training camera at the
starting position.  Steering angles are sent via ``angular.z`` following
the Ackermann convention.

Usage:
    python3 simple_robot_driver.py --direction clockwise --duration 30
"""

from __future__ import annotations

import argparse
import logging
import time
from typing import Literal

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

logger = logging.getLogger(__name__)

# Duration of each timed phase (seconds).
_FORWARD_PHASE_DURATION = 3.0
_TURNING_PHASE_DURATION = 2.0

# Control-loop period (seconds) — 50 Hz.
_LOOP_PERIOD = 0.02

# Speed fraction applied while turning (Ackermann needs forward velocity to steer).
_TURNING_SPEED_FRACTION = 0.5

# Fixed steering angle used during turning phases (~20°).
_STEERING_ANGLE = 0.35


class SimpleRobotDriver(Node):
    """Drives the robot in alternating forward / turning phases.

    Args:
        direction: Traversal direction around the track.
        duration: Total wall-clock seconds before the node stops itself.
    """

    def __init__(
        self,
        direction: Literal["clockwise", "counterclockwise"] = "clockwise",
        duration: float = 30.0,
    ) -> None:
        super().__init__("simple_robot_driver")

        self._direction = direction
        self._duration = duration
        self._start_time = time.time()

        self.declare_parameter("cmd_vel_topic", "/wro_robot/cmd_vel")
        self.shutdown_requested = False
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value

        self._vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

        self._forward_speed: float = 0.3  # m/s
        self._state: Literal["forward", "turning"] = "forward"
        self._state_timer: float = 0.0

        self.create_timer(_LOOP_PERIOD, self._control_loop)
        self.get_logger().info(
            "Robot driver started: %s direction, %ss duration",
            direction,
            duration,
        )

    def _control_loop(self) -> None:
        """Alternate between forward and turning phases until duration expires."""
        elapsed = time.time() - self._start_time
        if elapsed >= self._duration:
            self._stop_robot()
            self.get_logger().info("Driving duration reached, stopping")
            self.shutdown_requested = True
            return

        vel_msg = Twist()
        self._state_timer += _LOOP_PERIOD

        if self._state == "forward":
            vel_msg.linear.x = self._forward_speed
            vel_msg.angular.z = 0.0
            if self._state_timer >= _FORWARD_PHASE_DURATION:
                self._state = "turning"
                self._state_timer = 0.0

        else:  # turning
            vel_msg.linear.x = self._forward_speed * _TURNING_SPEED_FRACTION
            # Ackermann convention: negative angular.z steers right (CW).
            vel_msg.angular.z = -_STEERING_ANGLE if self._direction == "clockwise" else _STEERING_ANGLE
            if self._state_timer >= _TURNING_PHASE_DURATION:
                self._state = "forward"
                self._state_timer = 0.0

        self._vel_publisher.publish(vel_msg)

    def _stop_robot(self) -> None:
        """Publish a zero-velocity command."""
        self._vel_publisher.publish(Twist())


def main() -> None:
    """Parse arguments and run the SimpleRobotDriver node."""
    parser = argparse.ArgumentParser(description="Simple robot driver for video recording")
    parser.add_argument(
        "--direction",
        type=str,
        default="clockwise",
        choices=["clockwise", "counterclockwise"],
        help="Driving direction around track",
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=30,
        help="Driving duration in seconds",
    )
    args = parser.parse_args()

    rclpy.init()
    driver: SimpleRobotDriver | None = None
    try:
        driver = SimpleRobotDriver(direction=args.direction, duration=args.duration)
        while rclpy.ok() and not getattr(driver, "shutdown_requested", False):
            rclpy.spin_once(driver, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if driver is not None:
            try:
                driver.destroy_node()
            except RuntimeError:
                logger.warning("Exception during node teardown", exc_info=True)
        try:
            rclpy.shutdown()
        except RuntimeError:
            logger.warning("Exception during rclpy shutdown", exc_info=True)


if __name__ == "__main__":
    main()
