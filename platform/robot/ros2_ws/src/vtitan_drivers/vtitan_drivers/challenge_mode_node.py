"""Publishes the challenge-mode jumper state from the Pi Zero.

The jumper is wired to the ZERO's GPIO23, but the consumer
(``state_machine_node``) runs on the Pi 5. It previously read the pin through
``ChallengeModeDriver`` directly, i.e. Pi 5's own GPIO23 -- which nothing is
connected to. With ``pull_up=True`` an unconnected pin reads HIGH, so it
latched "jumper absent" -> Open Challenge on every boot, silently, and the
Obstacle Challenge could never be selected.

So the read happens here, on the board the wire is actually attached to, and
crosses to the Pi 5 as a topic.

Published:
    /challenge_mode/jumper_inserted (std_msgs/Bool) -- true when the jumper
        shorts the pin to GND (Obstacle Challenge).

The topic is TRANSIENT_LOCAL so the state machine gets the current value the
moment it subscribes, however long after the Zero it started -- the mode is
latched once at race setup, so a subscriber that joins late must not have to
wait for the next periodic publish to learn it.
"""

from __future__ import annotations

import rclpy
from pydantic import AliasChoices, Field
from pydantic_settings import SettingsConfigDict
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shared.config.ros_topics import RosTopicConfig
from std_msgs.msg import Bool

from src.hardware.challenge_mode.driver import Driver as ChallengeModeDriver
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings

NODE_NAME = "challenge_mode_node"


class NodeConfig(HardwareBaseSettings):
    """Node-level timing, configurable via config/hardware/challenge_mode_node.toml.

    Matches every hardware driver's Config pattern -- separate from
    challenge_mode.toml alongside it, which is the GPIO driver's own config
    (pin number), not this node's publish rate.
    """

    model_config = SettingsConfigDict(env_prefix="", toml_file=CONFIG_DIR / "challenge_mode_node.toml")

    publish_rate_hz: float = Field(default=2.0, validation_alias=AliasChoices("PUBLISH_RATE_HZ", "publish_rate_hz"))
    """Republish rate.

    The jumper is a boot-time setting, not a live control input, so this only
    has to be frequent enough that the state machine's 3-sample consistency
    check settles quickly at startup. TRANSIENT_LOCAL covers late
    subscribers, so this is really just a liveness heartbeat.
    """


_node_config = NodeConfig()
PUBLISH_RATE_HZ = _node_config.publish_rate_hz


class ChallengeModeNode(Node):
    """Reads the challenge-mode jumper and publishes it for the Pi 5.

    A plain Node rather than a LifecycleNode (unlike its siblings in
    ``pi_zero_peripherals_node``): it owns one input with no meaningful
    configured/active distinction, and the state machine needs the value as
    early as possible rather than gated behind an activation transition.
    """

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        topics = RosTopicConfig.load_default()
        self._publisher = self.create_publisher(Bool, topics.challenge_mode.jumper_inserted, qos)

        # The driver connects lazily, so constructing it touches no GPIO and
        # cannot fail here; a wiring or permissions fault surfaces on the first
        # read instead.
        self._driver = ChallengeModeDriver()
        self._logged_fault = False
        self.get_logger().info(f"Challenge-mode jumper on GPIO {self._driver.config.gpio_pin}")

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)

    def _publish(self) -> None:
        try:
            inserted = self._driver.is_jumper_inserted()
        except Exception as e:  # noqa: BLE001 - any GPIO read fault falls back to Open
            # Publish nothing on fault: the state machine falls back to Open
            # Challenge when it hears nothing. Logged once (not every 500 ms)
            # but at error level, because a wiring fault would otherwise be
            # indistinguishable from "Open Challenge was selected". RcutilsLogger
            # has no exception() (only debug/info/warning/error/fatal), unlike
            # Python's stdlib logger -- error() takes a plain string, not
            # exc_info, so the exception is folded into the message instead.
            if not self._logged_fault:
                self.get_logger().error(
                    f"Failed to read challenge-mode jumper; falling back to Open Challenge: "
                    f"{type(e).__name__}: {e}",
                )
                self._logged_fault = True
            return
        if self._logged_fault:
            self.get_logger().info("Challenge-mode jumper readable again")
            self._logged_fault = False
        self._publisher.publish(Bool(data=inserted))

    def destroy_node(self) -> None:
        """Release the GPIO line.

        ``pi_zero_peripherals_node`` calls this expecting the same cleanup
        ``ButtonNode``/``OLEDDisplayNode`` do on their own teardown -- without
        an override here that call was a no-op, leaving the jumper's
        ``InputDevice`` reserved against gpiozero's pin factory.
        """
        self._driver.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the challenge-mode node standalone (normally hosted by pi_zero_peripherals_node)."""
    rclpy.init(args=args)
    node = ChallengeModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
