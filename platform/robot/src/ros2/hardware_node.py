"""Base class for lifecycle-managed hardware driver ROS2 nodes."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, override

from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

from src.ros2.params import declare_param, get_float_param, get_str_param

if TYPE_CHECKING:
    from rclpy.lifecycle.node import LifecycleState
    from rclpy.lifecycle.publisher import Publisher
    from rclpy.timer import Timer


class LifecycleHardwareNode(LifecycleNode, ABC):
    """Abstract base for hardware driver ROS2 lifecycle nodes.

    Hardware connects in ``on_configure()`` and publishing starts in
    ``on_activate()`` -- the pattern shared by every hardware node in this
    package. This base owns that state machine plumbing (timer/publisher
    lifecycle, ``destroy_node``); subclasses only supply the driver-specific
    parts: ``_create_driver``, ``_configure_driver``, ``publish_imu``, and
    (optionally) ``_on_driver_disconnected`` for extra teardown state.
    """

    def __init__(
        self,
        node_name: str,
        message_type: type,
        *,
        publish_rate_default: float,
        topic_default: str,
        frame_id_default: str,
    ) -> None:
        """Construct the node (unconfigured -- no hardware I/O yet)."""
        super().__init__(node_name)
        self.get_logger().info(f"{node_name} constructed (unconfigured)")

        declare_param(self, "publish_rate", publish_rate_default)
        declare_param(self, "frame_id", frame_id_default)
        declare_param(self, "topic", topic_default)

        self._message_type = message_type
        self.driver: object | None = None
        self.publisher_: Publisher | None = None
        self.timer: Timer | None = None
        self.frame_id: str = ""

    @abstractmethod
    def _create_driver(self) -> object:
        """Instantiate the hardware driver (no I/O yet)."""

    @abstractmethod
    def _configure_driver(self, driver: object) -> TransitionCallbackReturn:
        """Connect/start ``driver``, handling driver-specific failure modes."""

    @abstractmethod
    def publish_imu(self) -> None:
        """Read from the driver and publish one message."""

    def _on_driver_disconnected(self) -> None:
        """Hook for subclasses to reset driver-specific readiness state."""

    @override
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Connect the driver and create the publisher."""
        self.get_logger().info(f"Configuring {self.get_name()}")

        self.frame_id = get_str_param(self, "frame_id")
        topic = get_str_param(self, "topic")
        self.publisher_ = self.create_lifecycle_publisher(self._message_type, topic, 10)

        self.driver = self._create_driver()
        return self._configure_driver(self.driver)

    @override
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Start the publish timer."""
        self.get_logger().info(f"Activating {self.get_name()}")
        publish_rate = get_float_param(self, "publish_rate")
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_imu)
        return super().on_activate(state)

    @override
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop the publish timer."""
        self.get_logger().info(f"Deactivating {self.get_name()}")
        self._destroy_timer()
        return super().on_deactivate(state)

    @override
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Disconnect the driver and tear down the publisher."""
        self.get_logger().info(f"Cleaning up {self.get_name()}")
        self._disconnect_driver()
        self._destroy_publisher()
        return TransitionCallbackReturn.SUCCESS

    @override
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Tear down whatever exists, regardless of which state shutdown was triggered from."""
        self.get_logger().info(f"Shutting down {self.get_name()}")
        self._destroy_timer()
        self._disconnect_driver()
        self._destroy_publisher()
        return TransitionCallbackReturn.SUCCESS

    def _destroy_timer(self) -> None:
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

    def _destroy_publisher(self) -> None:
        if self.publisher_ is not None:
            self.destroy_publisher(self.publisher_)
            self.publisher_ = None

    def _disconnect_driver(self) -> None:
        if self.driver is not None:
            try:
                self.driver.close()
            except Exception as e:  # noqa: BLE001 - cleanup must never fail node teardown
                self.get_logger().error(f"Error closing driver: {e}")
            self.driver = None
        self._on_driver_disconnected()

    @override
    def destroy_node(self) -> None:
        """Release hardware directly rather than trigger an on_shutdown transition.

        Handles a node destroyed without a clean lifecycle shutdown (e.g.
        process killed mid-active, or a test that never triggers shutdown).
        """
        self._destroy_timer()
        self._disconnect_driver()
        self.publisher_ = None
        return super().destroy_node()
