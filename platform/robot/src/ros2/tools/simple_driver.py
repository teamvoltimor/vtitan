#!/usr/bin/env python3
"""Simple timed robot driver for training video recording (Ackermann steering).

Drives the robot around the WRO track using alternating forward and turning
phases.  Designed to work alongside the static training camera at the
starting position.  Publishes AckermannDriveStamped on /ackermann_cmd, same
as every other drive-command source (ackermann_motor_node's own subscription
and state_machine_node's echo-back both expect this type).

Usage:
    python3 simple_robot_driver.py --direction clockwise --duration 30
"""

from __future__ import annotations

import argparse
import logging
import time
from typing import Literal

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from shared.config.ros_topics import RosTopicConfig

from src.ros2.params import declare_and_get_str_param
from src.ros2.qos import QOS_ACKERMANN_CMD

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

        self.shutdown_requested = False
        # Single source of truth for the drive command topic: Ackermann, not the
        # pre-Ackermann cmd_vel name that used to dead-end commands at a topic
        # nothing subscribes to.
        ackermann_cmd_topic = declare_and_get_str_param(
            self,
            "cmd_vel_topic",
            RosTopicConfig.load_default().commands.ackermann_cmd,
        )

        # AckermannDriveStamped, not geometry_msgs/Twist -- the topic name was
        # migrated to /ackermann_cmd but the message type never was, so this
        # never actually matched ackermann_motor_node's subscription (DDS
        # requires matching types, not just matching topic names -- confirmed
        # this publisher had zero subscribers on real hardware).
        self._drive_publisher = self.create_publisher(AckermannDriveStamped, ackermann_cmd_topic, QOS_ACKERMANN_CMD)

        self._forward_speed: float = 0.3  # m/s
        self._state: Literal["forward", "turning"] = "forward"
        self._state_timer: float = 0.0

        self.create_timer(_LOOP_PERIOD, self._control_loop)
        self.get_logger().info(
            f"Robot driver started: {direction} direction, {duration}s duration",
        )

    def _control_loop(self) -> None:
        """Alternate between forward and turning phases until duration expires."""
        elapsed = time.time() - self._start_time
        if elapsed >= self._duration:
            self._stop_robot()
            self.get_logger().info("Driving duration reached, stopping")
            self.shutdown_requested = True
            return

        drive_msg = AckermannDriveStamped()
        self._state_timer += _LOOP_PERIOD

        if self._state == "forward":
            drive_msg.drive.speed = self._forward_speed
            drive_msg.drive.steering_angle = 0.0
            if self._state_timer >= _FORWARD_PHASE_DURATION:
                self._state = "turning"
                self._state_timer = 0.0

        else:  # turning
            drive_msg.drive.speed = self._forward_speed * _TURNING_SPEED_FRACTION
            # Positive steering_angle steers left; negative steers right (CW).
            drive_msg.drive.steering_angle = -_STEERING_ANGLE if self._direction == "clockwise" else _STEERING_ANGLE
            if self._state_timer >= _TURNING_PHASE_DURATION:
                self._state = "forward"
                self._state_timer = 0.0

        self._drive_publisher.publish(drive_msg)

    def _stop_robot(self) -> None:
        """Publish a zero-velocity command."""
        self._drive_publisher.publish(AckermannDriveStamped())


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
