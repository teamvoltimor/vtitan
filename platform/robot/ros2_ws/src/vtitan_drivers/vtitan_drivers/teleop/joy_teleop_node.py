"""ROS2 node that maps a joystick (e.g. 8BitDo Ultimate 2 over Bluetooth) to Ackermann drive commands.

Run on: Raspberry Pi 5 (paired to the controller over Bluetooth), reaching
the Pi Zero's ackermann_motor_node over the network via /ackermann_cmd --
the same cross-board path scripts/hardware/test_motors.py already uses.

Not part of the competition stack: this is a bench-testing tool for driving
the steering/drive motors by hand instead of via a scripted sweep. Requires
the stock `joy` package's `joy_node` to already be publishing /joy (see
joy_teleop_launch.py, which starts both together).

Topics:
    Subscribed:
        - /joy (sensor_msgs/Joy) - raw controller axes/buttons
    Published:
        - /ackermann_cmd (ackermann_msgs/AckermannDriveStamped) - drive commands

Environment Variables:
    See src.teleop.config.Config (JOY_TELEOP_* prefix) for axis/button
    mapping, speed/steering limits, and safety timeouts -- the exact
    axis/button indices are unverified for the 8BitDo Ultimate 2 and should
    be confirmed with `ros2 topic echo /joy` after pairing.
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy

from src.teleop.config import Config
from src.teleop.mapping import compute_command

if TYPE_CHECKING:
    from rclpy.publisher import Publisher
    from rclpy.subscription import Subscription
    from rclpy.timer import Timer

NODE_NAME = "joy_teleop_node"


class JoyTeleopNode(Node):
    """Publishes /ackermann_cmd from /joy, gated by a dead-man button and a link-staleness check."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.config = Config()
        self.get_logger().info(
            f"Joy teleop configured: steering_axis={self.config.steering_axis_index}, "
            f"throttle_axis={self.config.throttle_axis_index}, "
            f"deadman_button={self.config.deadman_button_index}, "
            f"max_speed={self.config.max_speed_mps} m/s, max_steering={self.config.max_steering_deg} deg",
        )

        self._latest_joy: Joy | None = None
        self._last_joy_time: float = 0.0
        self._drive_armed = False

        self.joy_sub: Subscription = self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self.cmd_pub: Publisher = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        self.publish_timer: Timer = self.create_timer(1.0 / self.config.publish_rate_hz, self._publish_command)

    def _joy_callback(self, msg: Joy) -> None:
        self._latest_joy = msg
        self._last_joy_time = time.monotonic()

    def _publish_command(self) -> None:
        if self._latest_joy is None:
            return

        joy_is_fresh = (time.monotonic() - self._last_joy_time) <= self.config.joy_timeout_s
        speed_mps, steering_deg = compute_command(
            list(self._latest_joy.axes),
            list(self._latest_joy.buttons),
            self.config,
            joy_is_fresh=joy_is_fresh,
        )

        drive_armed = joy_is_fresh and bool(self._latest_joy.buttons[self.config.deadman_button_index])
        if drive_armed != self._drive_armed:
            self._drive_armed = drive_armed
            self.get_logger().info(f"Drive {'ARMED (dead-man held)' if drive_armed else 'disarmed'}")

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed_mps
        msg.drive.steering_angle = math.radians(steering_deg)
        self.cmd_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Run the joystick teleop node."""
    rclpy.init(args=args)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
