"""ROS2 lifecycle node that publishes GPIO button events to /button/event.

Run on: Raspberry Pi Zero 2W

Decouples physical button hardware from the state machine. The Pi Zero
owns GPIO and publishes events over the network; the Pi 5 state machine
subscribes instead of polling GPIO directly.

Hardware connects in on_configure() and polling starts in on_activate(), so
the node can be reconfigured or retried (e.g. after a driver failure) without
a process restart. Auto-configures and auto-activates on launch (see
rpi_zero_nodes.launch.py) so deployed behavior is unchanged: the node still
starts polling immediately, same as a plain Node would.

Usage:
    ros2 run voldemorbot_drivers button_node

Topics:
    Published:
        - /button/event (std_msgs/String) — ButtonEvent value on each event
"""

from __future__ import annotations

from typing import TYPE_CHECKING, override

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import String

from src.hardware.button.gpio import Driver as ButtonDriver

if TYPE_CHECKING:
    from rclpy.lifecycle.node import LifecycleState
    from rclpy.lifecycle.publisher import Publisher
    from rclpy.timer import Timer

NODE_NAME = "button_node"
BUTTON_EVENT_TOPIC = "/button/event"
DEFAULT_QUEUE_DEPTH = 10
BUTTON_POLL_HZ = 20.0
BUTTON_POLL_PERIOD_S = 1.0 / BUTTON_POLL_HZ


class ButtonNode(LifecycleNode):
    """Polls the physical button at 20 Hz and publishes events to /button/event."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.get_logger().info("Button Node constructed (unconfigured)")
        self.driver: ButtonDriver | None = None
        self.pub: Publisher | None = None
        self.timer: Timer | None = None

    @override
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Connect the GPIO driver and create the publisher."""
        self.get_logger().info("Configuring Button Node")

        self.pub = self.create_lifecycle_publisher(String, BUTTON_EVENT_TOPIC, DEFAULT_QUEUE_DEPTH)

        try:
            self.driver = ButtonDriver()
            self.driver.connect()
            self.get_logger().info("Button driver connected")
        except (RuntimeError, OSError, ValueError, ImportError) as e:
            self.get_logger().error(f"Failed to connect button driver: {e}")
            self.driver = None

        return TransitionCallbackReturn.SUCCESS

    @override
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Start the poll timer."""
        self.get_logger().info("Activating Button Node")
        self.timer = self.create_timer(BUTTON_POLL_PERIOD_S, self._poll)
        return super().on_activate(state)

    @override
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop the poll timer."""
        self.get_logger().info("Deactivating Button Node")
        self._destroy_timer()
        return super().on_deactivate(state)

    @override
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Disconnect the driver and tear down the publisher."""
        self.get_logger().info("Cleaning up Button Node")
        self._disconnect_driver()
        if self.pub is not None:
            self.destroy_publisher(self.pub)
            self.pub = None
        return TransitionCallbackReturn.SUCCESS

    @override
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Tear down whatever exists, regardless of which state shutdown was triggered from."""
        self.get_logger().info("Shutting down Button Node")
        self._destroy_timer()
        self._disconnect_driver()
        if self.pub is not None:
            self.destroy_publisher(self.pub)
            self.pub = None
        return TransitionCallbackReturn.SUCCESS

    def _destroy_timer(self) -> None:
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

    def _disconnect_driver(self) -> None:
        if self.driver is not None:
            try:
                self.driver.close()
            except Exception as e:  # noqa: BLE001 - cleanup must never fail node teardown
                self.get_logger().error(f"Error closing button driver: {e}")
            self.driver = None

    def _poll(self) -> None:
        """Check for a new button event and publish if one occurred."""
        if self.driver is None or self.pub is None:
            return
        state = self.driver.get_state()
        if state.last_event:
            msg = String()
            msg.data = state.last_event.value
            self.pub.publish(msg)

    @override
    def destroy_node(self) -> None:
        """Release hardware directly rather than trigger an on_shutdown transition.

        Handles a node destroyed without a clean lifecycle shutdown (e.g.
        process killed mid-active, or a test that never triggers shutdown) --
        sidesteps the lifecycle state machine entirely rather than risk an
        invalid-transition error.
        """
        self._destroy_timer()
        self._disconnect_driver()
        self.pub = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the button node, auto-configuring and auto-activating on launch."""
    rclpy.init(args=args)
    node = ButtonNode()
    try:
        # Auto-configure + auto-activate on launch so deployed behavior matches
        # a plain Node: the button starts polling immediately, no manual
        # `ros2 lifecycle set` step required.
        node.trigger_configure()
        node.trigger_activate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
