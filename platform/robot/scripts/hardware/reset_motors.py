#!/usr/bin/env python3
"""Immediately zero the drive command and recenter steering.

ackermann_motor_node's watchdog already stops the drive motor 1s after
commands stop arriving, but it never recenters steering (see
_watchdog_check in ackermann_motor_node.py -- it only calls
drive.stop_drive()). Run this right after switching between drive-navigation
and drive-controller (or killing either with Ctrl+C) so steering snaps back
to center immediately instead of sitting wherever the last command left it.

Run ON Pi 5 (needs ROS2 discovery to the Pi Zero's ackermann_motor_node,
same as scripts/hardware/test_motors.py).

Usage:
    python3 scripts/hardware/reset_motors.py
"""

from __future__ import annotations

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from shared.config.ros_topics import RosTopicConfig

from scripts.common.motor_hold import publish_hold

# Repeated for a short window (not one-shot) so the command lands even if
# discovery/matching between this short-lived process and
# ackermann_motor_node's subscription hasn't settled yet on the first publish.
_HOLD_S = 0.5
_PUBLISH_INTERVAL_S = 0.05


def main() -> None:
    """Publish a zero-speed, zero-steering command for a short hold, then exit."""
    rclpy.init()
    node = Node("reset_motors")
    pub = node.create_publisher(AckermannDriveStamped, RosTopicConfig.load_default().commands.ackermann_cmd, 10)

    stop_msg = AckermannDriveStamped()
    stop_msg.drive.speed = 0.0
    stop_msg.drive.steering_angle = 0.0

    publish_hold(node, pub, stop_msg, _HOLD_S, publish_interval_s=0.0, spin_timeout_s=_PUBLISH_INTERVAL_S)

    node.destroy_node()
    rclpy.shutdown()
    print("Sent stop + center-steering command.")


if __name__ == "__main__":
    main()
