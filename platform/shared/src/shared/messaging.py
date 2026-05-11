"""Message broker abstraction for robot hardware I/O.

Provides a protocol for pub/sub message transport, allowing robot logic
to be independent of ROS2/MQTT/file transports. Supports:

- Async-friendly message publishing and subscription
- Topic-based communication with type hints
- Provider implementations (ROS2, mock, file-based)
- Middleware hooks for filtering/transforming

Robot components (IMU, motors, vision) publish telemetry to topics.
Backend consumes via subscriptions or polling. Decouples hardware I/O
from domain logic, making components testable without ROS2.

Usage:
    # Inject provider (ROS2 in prod, mock in tests)
    provider: MessageProvider = ROS2MessageProvider() | MockMessageProvider()

    with provider:
        provider.publish("/robot/imu", imu_reading)
        msg = provider.receive("/robot/vision", timeout=1.0)
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Protocol

from .domain.exceptions import MessagingError, ProviderNotConnectedError

if TYPE_CHECKING:
    from collections.abc import Callable


@dataclass(slots=True, frozen=True)
class Message:
    """Generic message for pub/sub communication.

    Attributes:
            topic: Topic name (e.g., "/robot/imu", "/cmd/motors").
            data: Message payload (serializable object).
            timestamp: Message creation time (seconds since epoch).
            message_type: Optional ROS message type hint (e.g., "geometry_msgs/Twist").
    """

    topic: str
    data: Any
    timestamp: float = 0.0
    message_type: str | None = None


class MessageProvider(Protocol):
    """Protocol for pub/sub message brokers.

    Implementations can use ROS2, MQTT, mock in-memory, file-based, etc.
    Used by robot components (IMU, motors, vision) to emit telemetry.

    Any class implementing this protocol (duck typing) is a valid provider.
    """

    def __enter__(self) -> MessageProvider:
        """Enter context manager (connect/initialize)."""
        ...

    def __exit__(self, *_: object) -> None:
        """Exit context manager (disconnect/cleanup)."""
        ...

    def publish(self, topic: str, data: Any, message_type: str | None = None) -> None:
        """Publish a message to a topic.

        Args:
                topic: Topic name.
                data: Message payload (dict, dataclass, etc.).
                message_type: Optional ROS message type hint.

        Raises:
                ProviderNotConnectedError: If provider not connected.
        """
        ...

    def subscribe(
        self,
        topic: str,
        callback: Callable[[Message], None],
        message_type: str | None = None,
    ) -> str:
        """Subscribe to a topic with callback handler.

        Args:
                topic: Topic name.
                callback: Called on each message received.
                message_type: Optional ROS message type hint.

        Returns:
                Subscription ID (for later unsubscribe).

        Raises:
                ProviderNotConnectedError: If provider not connected.
        """
        ...

    def unsubscribe(self, subscription_id: str) -> None:
        """Unsubscribe from a topic.

        Args:
                subscription_id: ID returned from subscribe().
        """
        ...

    def receive(self, topic: str, timeout: float = 1.0) -> Message | None:
        """Receive one message from topic (blocking).

        Args:
                topic: Topic name.
                timeout: Maximum wait time in seconds.

        Returns:
                Message if received, None if timeout.

        Raises:
                ProviderNotConnectedError: If provider not connected.
        """
        ...

    def spin_once(self) -> None:
        """Process pending messages once (non-blocking)."""
        ...

    def is_connected(self) -> bool:
        """Check if provider is connected and ready."""
        ...


class MockMessageProvider:
    """In-memory message provider for testing.

    Implements MessageProvider protocol without external dependencies.
    Useful for unit tests of robot components without ROS2.

    Features:
            - Synchronous message delivery
            - Message history tracking (test utility)
            - Callback-based and polling-based consumption
    """

    def __init__(self) -> None:
        """Initialize mock provider."""
        self._connected = False
        self._messages: dict[str, list[Message]] = {}
        self._callbacks: dict[str, list[Callable[[Message], None]]] = {}
        self._subscription_counter = 0
        self._subscriptions: dict[str, tuple[str, Callable[[Message], None]]] = {}

    def __enter__(self) -> MockMessageProvider:
        """Connect (set connected flag)."""
        self._connected = True
        return self

    def __exit__(self, *_: object) -> None:
        """Disconnect and clear state."""
        self._connected = False
        self._messages.clear()
        self._callbacks.clear()
        self._subscriptions.clear()

    def publish(self, topic: str, data: Any, message_type: str | None = None) -> None:
        """Publish message and immediately deliver to subscribers.

        Args:
                topic: Topic name.
                data: Message payload.
                message_type: Optional ROS message type hint.

        Raises:
                ProviderNotConnectedError: If not connected.
        """
        if not self._connected:
            raise ProviderNotConnectedError("Provider not connected")

        msg = Message(
            topic=topic, data=data, timestamp=time.time(), message_type=message_type
        )

        if topic not in self._messages:
            self._messages[topic] = []
        self._messages[topic].append(msg)

        if topic in self._callbacks:
            for callback in self._callbacks[topic]:
                callback(msg)

    def subscribe(
        self,
        topic: str,
        callback: Callable[[Message], None],
        message_type: str | None = None,
    ) -> str:
        """Subscribe to topic.

        Args:
                topic: Topic name.
                callback: Called on each publish to this topic.
                message_type: Unused in mock provider.

        Returns:
                Subscription ID for later unsubscribe.

        Raises:
                ProviderNotConnectedError: If not connected.
        """
        if not self._connected:
            raise ProviderNotConnectedError("Provider not connected")

        if topic not in self._callbacks:
            self._callbacks[topic] = []

        self._callbacks[topic].append(callback)

        sub_id = f"sub_{self._subscription_counter}"
        self._subscription_counter += 1
        self._subscriptions[sub_id] = (topic, callback)
        return sub_id

    def unsubscribe(self, subscription_id: str) -> None:
        """Unsubscribe from topic.

        Args:
                subscription_id: ID returned from subscribe().
        """
        if subscription_id in self._subscriptions:
            topic, callback = self._subscriptions.pop(subscription_id)
            if topic in self._callbacks:
                self._callbacks[topic].remove(callback)

    def receive(self, topic: str, timeout: float = 1.0) -> Message | None:
        """Receive latest message from topic.

        Note: Returns immediately with latest message (ignores timeout in mock).

        Args:
                topic: Topic name.
                timeout: Unused in mock (always non-blocking).

        Returns:
                Latest message on topic, or None if never published.

        Raises:
                ProviderNotConnectedError: If not connected.
        """
        if not self._connected:
            raise ProviderNotConnectedError("Provider not connected")

        if topic in self._messages and self._messages[topic]:
            return self._messages[topic][-1]
        return None

    def spin_once(self) -> None:
        """Process pending messages (no-op for mock)."""

    def is_connected(self) -> bool:
        """Check connection status."""
        return self._connected

    def get_message_history(self, topic: str) -> list[Message]:
        """Retrieve all messages ever published to a topic (test utility).

        Args:
                topic: Topic name.

        Returns:
                List of messages in publish order.
        """
        return self._messages.get(topic, [])

    def clear_history(self) -> None:
        """Clear all message history (test utility)."""
        self._messages.clear()


class MessageRouter:
    """Routes messages between topics with transformation middleware.

    Useful for bridging between robot components and backend telemetry.
    Transforms and filters messages as they flow through topics.
    """

    def __init__(self, provider: MessageProvider) -> None:
        """Initialize router with a message provider.

        Args:
                provider: Message broker (must implement MessageProvider protocol).
        """
        self.provider = provider
        self._routes: dict[
            str, list[tuple[str, Callable[[Message], Message | None]]]
        ] = {}

    def add_route(
        self,
        from_topic: str,
        to_topic: str,
        transformer: Callable[[Message], Message | None] | None = None,
    ) -> None:
        """Add a route that forwards messages between topics.

        Args:
                from_topic: Source topic to subscribe to.
                to_topic: Destination topic to publish to.
                transformer: Optional transformer (returns None to drop message).
        """
        if from_topic not in self._routes:
            self._routes[from_topic] = []

        self._routes[from_topic].append((to_topic, transformer or (lambda m: m)))

    def start_routing(self) -> None:
        """Activate all routes (subscribe to source topics)."""
        for from_topic in self._routes:

            def make_callback(src_topic: str) -> Callable[[Message], None]:
                def callback(msg: Message) -> None:
                    for dst_topic, transformer in self._routes[src_topic]:
                        transformed = transformer(msg)
                        if transformed:
                            self.provider.publish(
                                dst_topic,
                                transformed.data,
                                transformed.message_type,
                            )

                return callback

            self.provider.subscribe(from_topic, make_callback(from_topic))

    def stop_routing(self) -> None:
        """Deactivate all routes."""
        self._routes.clear()


class ROS2MessageProvider:
    """ROS2-based message provider implementation.

    Maps MessageProvider protocol to ROS2 publisher/subscriber API.
    Used by robot components to emit hardware telemetry to topics.

    Requires rclpy and ROS2 to be installed and initialized.
    """

    def __init__(self) -> None:
        """Initialize ROS2 provider (not yet connected)."""
        self._node: Any = None
        self._publishers: dict[str, Any] = {}
        self._subscriptions: dict[str, tuple[Any, str]] = {}
        self._subscription_counter = 0

    def __enter__(self) -> ROS2MessageProvider:
        """Connect to ROS2.

        Initializes rclpy if needed and creates a node.

        Returns:
                Self for use in with statement.

        Raises:
                MessagingError: If ROS2 (rclpy) not installed.
        """
        try:
            import rclpy
            from rclpy.node import Node

            if not rclpy.ok():
                rclpy.init()

            self._node = Node("telemetry_provider")
            return self
        except ImportError as e:
            raise MessagingError("ROS2 (rclpy) not installed") from e

    def __exit__(self, *_: object) -> None:
        """Disconnect from ROS2.

        Destroys the node and cleans up publishers/subscribers.
        """
        if self._node:
            for pub in self._publishers.values():
                self._node.destroy_publisher(pub)
            for sub, _ in self._subscriptions.values():
                self._node.destroy_subscription(sub)
            self._publishers.clear()
            self._subscriptions.clear()
            self._node.destroy_node()
            self._node = None

    def publish(self, topic: str, data: Any, message_type: str | None = None) -> None:
        """Publish a message to a ROS2 topic.

        Args:
                topic: Topic name (e.g., "/robot/imu").
                data: Message payload (dict or ROS message object).
                message_type: ROS message type (e.g., "sensor_msgs/Imu").

        Raises:
                ProviderNotConnectedError: If not connected.
        """
        if not self._node:
            raise ProviderNotConnectedError("ROS2 provider not connected")

        if topic not in self._publishers:
            self._create_publisher(topic, message_type)

        publisher = self._publishers[topic]
        publisher.publish(data)

    def subscribe(
        self,
        topic: str,
        callback: Callable[[Message], None],
        message_type: str | None = None,
    ) -> str:
        """Subscribe to a ROS2 topic with callback.

        Args:
                topic: Topic name.
                callback: Called with Message wrapping each ROS message received.
                message_type: ROS message type hint.

        Returns:
                Subscription ID for later unsubscribe.

        Raises:
                ProviderNotConnectedError: If not connected.
        """
        if not self._node:
            raise ProviderNotConnectedError("ROS2 provider not connected")

        def ros2_callback(ros_msg: Any) -> None:
            msg = Message(
                topic=topic,
                data=ros_msg,
                timestamp=time.time(),
                message_type=message_type,
            )
            callback(msg)

        subscription = self._create_subscription(topic, ros2_callback, message_type)
        sub_id = f"ros2_sub_{self._subscription_counter}"
        self._subscription_counter += 1
        self._subscriptions[sub_id] = (subscription, topic)
        return sub_id

    def unsubscribe(self, subscription_id: str) -> None:
        """Unsubscribe from a topic.

        Args:
                subscription_id: ID returned from subscribe().
        """
        if subscription_id in self._subscriptions:
            subscription, topic = self._subscriptions.pop(subscription_id)
            if self._node:
                self._node.destroy_subscription(subscription)

    def receive(self, topic: str, timeout: float = 1.0) -> Message | None:
        """Receive one message from topic (blocking).

        Uses rclpy.spin_until_future_complete to wait for a message.
        Not recommended for real-time; prefer subscribe() + callback.

        Args:
                topic: Topic name.
                timeout: Maximum wait time in seconds.

        Returns:
                Message if received before timeout, None otherwise.

        Raises:
                ProviderNotConnectedError: If not connected.
        """
        if not self._node:
            raise ProviderNotConnectedError("ROS2 provider not connected")

        from rclpy.executors import SingleThreadedExecutor

        received_msg = None

        def callback(ros_msg: Any) -> None:
            nonlocal received_msg
            received_msg = Message(topic=topic, data=ros_msg, timestamp=time.time())

        sub_id = self.subscribe(topic, callback)
        executor = SingleThreadedExecutor()
        executor.add_node(self._node)

        try:
            executor.spin_once(timeout_sec=timeout)
        finally:
            executor.shutdown()
            self.unsubscribe(sub_id)

        return received_msg

    def spin_once(self) -> None:
        """Process pending ROS2 callbacks (non-blocking).

        Must be called regularly to deliver subscribed messages.
        Typically called in a spin loop or timer.
        """
        if self._node:
            import rclpy

            rclpy.spin_once(self._node, timeout_sec=0.0)

    def is_connected(self) -> bool:
        """Check if connected to ROS2.

        Returns:
                True if __enter__ called and __exit__ not yet called.
        """
        return self._node is not None

    def _create_publisher(self, topic: str, message_type: str | None) -> None:
        """Create a ROS2 publisher for the topic.

        Args:
                topic: Topic name.
                message_type: ROS message type (required for type safety).

        Raises:
                MessagingError: If message_type not provided or type unknown.
        """
        if not message_type:
            raise MessagingError(f"message_type required for publisher on {topic}")

        try:
            ros_msg_class = self._resolve_message_type(message_type)
            publisher = self._node.create_publisher(ros_msg_class, topic, 10)
            self._publishers[topic] = publisher
        except Exception as e:
            raise MessagingError(f"Failed to create publisher on {topic}: {e}") from e

    def _create_subscription(
        self,
        topic: str,
        callback: Callable[[Any], None],
        message_type: str | None,
    ) -> Any:
        """Create a ROS2 subscription for the topic.

        Args:
                topic: Topic name.
                callback: Called with each ROS message received.
                message_type: ROS message type hint (optional).

        Returns:
                ROS2 subscription object.

        Raises:
                MessagingError: If subscription fails.
        """
        try:
            if message_type:
                ros_msg_class = self._resolve_message_type(message_type)
            else:
                ros_msg_class = self._infer_message_type(topic)

            return self._node.create_subscription(ros_msg_class, topic, callback, 10)
        except Exception as e:
            raise MessagingError(
                f"Failed to create subscription to {topic}: {e}"
            ) from e

    @staticmethod
    def _resolve_message_type(message_type: str) -> Any:
        """Resolve ROS message type string to actual class.

        Args:
                message_type: Type string (e.g., "geometry_msgs/Twist").

        Returns:
                ROS message class.

        Raises:
                MessagingError: If type not found.
        """
        try:
            parts = message_type.split("/")
            if len(parts) != 2:
                raise ValueError("Invalid message type format")

            pkg, msg = parts
            module = __import__(f"{pkg}.msg", fromlist=[msg])
            return getattr(module, msg)
        except Exception as e:
            raise MessagingError(f"Unknown message type: {message_type}") from e

    @staticmethod
    def _infer_message_type(topic: str) -> Any:
        """Infer ROS message type from topic name.

        Heuristic for common patterns (imu -> sensor_msgs/Imu, etc.).

        Args:
                topic: Topic name.

        Returns:
                ROS message class.

        Raises:
                MessagingError: If type cannot be inferred.
        """
        import re

        topic_lower = topic.lower()

        inferences = {
            r"imu": "sensor_msgs/Imu",
            r"twist|cmd_vel": "geometry_msgs/Twist",
            r"odometry|odom": "nav_msgs/Odometry",
            r"laser|scan": "sensor_msgs/LaserScan",
            r"camera|image": "sensor_msgs/Image",
        }

        for pattern, msg_type in inferences.items():
            if re.search(pattern, topic_lower):
                return ROS2MessageProvider._resolve_message_type(msg_type)

        raise MessagingError(
            f"Cannot infer message type for {topic} (provide message_type)"
        )
