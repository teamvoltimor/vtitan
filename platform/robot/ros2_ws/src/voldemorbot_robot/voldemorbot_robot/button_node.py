"""ROS2 node that publishes GPIO button events to /button/event.

Run on: Raspberry Pi Zero 2W

Decouples physical button hardware from the state machine. The Pi Zero
owns GPIO and publishes events over the network; the Pi 5 state machine
subscribes instead of polling GPIO directly.

Usage:
    ros2 run voldemorbot_robot button_node

Topics:
    Published:
        - /button/event (std_msgs/String) — ButtonEvent value on each event
"""

from typing import TYPE_CHECKING, override

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from src.hardware.button.gpio import Driver as ButtonDriver

if TYPE_CHECKING:
    from rclpy.publisher import Publisher
    from rclpy.timer import Timer

NODE_NAME = "button_node"
BUTTON_EVENT_TOPIC = "/button/event"
DEFAULT_QUEUE_DEPTH = 10
BUTTON_POLL_HZ = 20.0
BUTTON_POLL_PERIOD_S = 1.0 / BUTTON_POLL_HZ


class ButtonNode(Node):
    """Polls the physical button at 20 Hz and publishes events to /button/event."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.get_logger().info("Initializing Button Node")

        self.pub: Publisher[String] = self.create_publisher(
            String, BUTTON_EVENT_TOPIC, DEFAULT_QUEUE_DEPTH
        )

        try:
            self.driver = ButtonDriver()
            self.driver.connect()
            self.get_logger().info("Button driver connected")
        except (RuntimeError, OSError, ValueError, ImportError) as e:
            self.get_logger().error(f"Failed to connect button driver: {e}")
            self.driver = None

        self.timer: Timer = self.create_timer(BUTTON_POLL_PERIOD_S, self._poll)

        self.get_logger().info("Button Node ready")

    def _poll(self) -> None:
        """Check for a new button event and publish if one occurred."""
        if self.driver is None:
            return
        state = self.driver.get_state()
        if state.last_event:
            msg = String()
            msg.data = state.last_event.value
            self.pub.publish(msg)

    @override
    def destroy_node(self) -> None:
        if self.driver is not None:
            try:
                self.driver.close()
            except (RuntimeError, OSError) as e:
                self.get_logger().error(f"Error closing button driver: {e}")
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ButtonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
