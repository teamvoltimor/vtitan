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
    ros2 run vtitan_drivers button_node

Topics:
    Published:
        - /button/event (std_msgs/String) — ButtonEvent value on each event
        - /button/hold (std_msgs/String) — JSON progress while the button is
          held, so the OLED can show the operator what a longer hold will do
"""

from __future__ import annotations


from typing import TYPE_CHECKING, override

import rclpy
from pydantic import AliasChoices, Field
from pydantic_settings import SettingsConfigDict
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from shared.config.ros_topics import RosTopicConfig
from std_msgs.msg import String

from src.hardware.button.gpio import Driver as ButtonDriver
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings
from src.ros2.wire_models import ButtonHoldThreshold, ButtonHoldWire

if TYPE_CHECKING:
    from rclpy.lifecycle.node import LifecycleState
    from rclpy.lifecycle.publisher import Publisher
    from rclpy.timer import Timer

    from src.hardware.button.state import ButtonState

NODE_NAME = "button_node"
DEFAULT_QUEUE_DEPTH = 10


class NodeConfig(HardwareBaseSettings):
    """Node-level timing, configurable via config/hardware/button/button_node.toml.

    Matches every hardware driver's Config pattern -- separate from
    gpio.toml/mcp2221.toml alongside it, which are the driver's own
    debounce/threshold config, not this node's poll rate.
    """

    model_config = SettingsConfigDict(env_prefix="", toml_file=CONFIG_DIR / "button" / "button_node.toml")

    poll_hz: float = Field(default=20.0, validation_alias=AliasChoices("POLL_HZ", "poll_hz"))
    """Rate the physical button is sampled at."""


_node_config = NodeConfig()
BUTTON_POLL_HZ = _node_config.poll_hz
BUTTON_POLL_PERIOD_S = 1.0 / BUTTON_POLL_HZ

# Names the *kind* of each threshold, not what it does. What a hold does
# depends on the robot state -- past the long threshold it stops a running
# robot but restarts a finished one -- and this node has no idea which state
# the machine is in. It publishes when things happen; the display, which does
# subscribe to /robot_state, decides what to call them.
_HOLD_KINDS = ("long", "shutdown")


class ButtonNode(LifecycleNode):
    """Polls the physical button at 20 Hz and publishes events to /button/event."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.get_logger().info("Button Node constructed (unconfigured)")
        self.driver: ButtonDriver | None = None
        self.pub: Publisher | None = None
        self.pub_hold: Publisher | None = None
        self.timer: Timer | None = None
        self._was_pressed: bool = False
        """Tracks the press so release publishes one clearing frame, not a stream of zeros."""
        self.driver_fault: str | None = None
        """Why the driver is unavailable, replayed by _poll so the reason is in
        the log at press time rather than only in a startup line."""

    @override
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Connect the GPIO driver and create the publisher."""
        self.get_logger().info("Configuring Button Node")

        topics = RosTopicConfig.load_default()
        self.pub = self.create_lifecycle_publisher(String, topics.button.event, DEFAULT_QUEUE_DEPTH)
        # Depth 1: this is a liveness readout at 20Hz, and a subscriber that
        # fell behind wants the current hold time, never a backlog of old ones.
        self.pub_hold = self.create_lifecycle_publisher(String, topics.button.hold, 1)

        try:
            self.driver = ButtonDriver()
            self.driver.connect()
            self.driver_fault = None
            self.get_logger().info("Button driver connected")
        except (RuntimeError, OSError, ValueError, ImportError) as e:
            # ValueError covers pydantic's ValidationError too, so a .env
            # missing BUTTON_GPIO_PIN / BUTTON__* lands here and looks exactly
            # like a wiring fault. Keep the text -- it names the missing key.
            self.driver_fault = f"{type(e).__name__}: {e}"
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
        if self.pub_hold is not None:
            self.destroy_publisher(self.pub_hold)
            self.pub_hold = None
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
        if self.pub_hold is not None:
            self.destroy_publisher(self.pub_hold)
            self.pub_hold = None
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
            # on_configure returns SUCCESS even when connect() fails, so a dead
            # driver leaves this node ACTIVE with a button that can never emit:
            # the state machine simply never hears an event and the OLED goes on
            # showing a healthy "Press to START". Repeat the fault while it
            # lasts, so it is in the log at the moment the operator presses --
            # not only in a startup line that scrolled away minutes ago.
            self.get_logger().error(
                f"Button driver not connected - no button events will be published "
                f"({self.driver_fault or 'driver never configured'})",
                throttle_duration_sec=5.0,
            )
            return
        state = self.driver.get_state()
        if state.last_event:
            msg = String()
            msg.data = state.last_event.value
            self.pub.publish(msg)
        self._publish_hold_progress(state)

    def _publish_hold_progress(self, state: ButtonState) -> None:
        """Tell the display how long the button has been held and what comes next.

        Ten seconds is a long time to hold a button with no feedback -- long
        enough to doubt whether the press registered at all and let go a second
        early. This is what lets the OLED count it out.

        Only published while the button is down, plus one final frame on
        release so the display clears instead of freezing on the last number.
        """
        if self.pub_hold is None:
            return
        if not state.is_pressed:
            if self._was_pressed:
                self._was_pressed = False
                msg = String()
                msg.data = ButtonHoldWire().model_dump_json()
                self.pub_hold.publish(msg)
            return

        self._was_pressed = True
        if self.driver is None:
            return
        thresholds = (
            self.driver.config.button.long_press_threshold_sec,
            self.driver.config.button.shutdown_press_threshold_sec,
        )
        msg = String()
        msg.data = ButtonHoldWire(
            held_sec=round(state.press_duration, 1),
            # All of them, not just the next one: the display skips any
            # whose action is meaningless in the current state (holding
            # from READY does nothing at the long threshold), and it can
            # only do that if it can see past the first.
            thresholds=[
                ButtonHoldThreshold(at=at, kind=kind) for at, kind in zip(thresholds, _HOLD_KINDS, strict=True)
            ],
        ).model_dump_json()
        self.pub_hold.publish(msg)

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
        self.pub_hold = None
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
