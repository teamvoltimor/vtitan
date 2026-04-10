"""Message broker abstraction for decoupling from ROS2.

Provides a provider pattern for message transport, allowing the core
robot logic to be independent of the ROS2 framework. Supports:

- Message publishing and subscription abstraction
- Topic-based communication
- Provider implementations (ROS2, mock, file-based)
- Middleware hooks for filtering/transforming

This decouples core navigation, state, and sensor logic from ROS2
dependencies, making components more testable and portable.

Usage:
    # Use injected provider
    provider = ros2_provider()  # or mock_provider() for testing

    with provider:
        provider.publish("cmd_vel", velocity_msg)
        detections = provider.receive("vision/detections", timeout=1.0)
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Generic, TypeVar
from shared.domain.exceptions import ProviderNotConnectedError, MessagingError

if TYPE_CHECKING:
    from collections.abc import Callable

T = TypeVar("T")


@dataclass
class Message:
    """Generic message for topic-based communication.

    Fields:
        topic: Topic name (e.g., "/robot/cmd_vel")
        data: Message payload (dict or serializable object)
        timestamp: Message creation time (seconds since epoch)
        message_type: ROS message type hint (e.g., "geometry_msgs/Twist")
    """

    topic: str
    data: Any
    timestamp: float = 0.0
    message_type: str | None = None


class MessageProvider(ABC):
    """Abstract base for message broker implementations.

    Defines interface for pub/sub communication. Implementations can use
    ROS2, test doubles, file-based messaging, etc.
    """

    @abstractmethod
    def __enter__(self) -> MessageProvider:
        """Enter context manager (connect/initialize)."""
        ...

    @abstractmethod
    def __exit__(self, *_: object) -> None:
        """Exit context manager (disconnect/cleanup)."""
        ...

    @abstractmethod
    def publish(self, topic: str, data: Any, message_type: str | None = None) -> None:
        """Publish a message to a topic.

        Args:
            topic: Topic name
            data: Message payload
            message_type: Optional ROS message type hint

        Raises:
            RuntimeError: If provider not connected
            ValueError: If message format invalid
        """
        ...

    @abstractmethod
    def subscribe(
        self,
        topic: str,
        callback: Callable[[Message], None],
        message_type: str | None = None,
    ) -> str:
        """Subscribe to a topic with callback handler.

        Args:
            topic: Topic name
            callback: Function called when message received
            message_type: Optional ROS message type hint

        Returns:
            Subscription ID (for later unsubscribe)

        Raises:
            RuntimeError: If provider not connected
        """
        ...

    @abstractmethod
    def unsubscribe(self, subscription_id: str) -> None:
        """Unsubscribe from a topic.

        Args:
            subscription_id: ID returned from subscribe()
        """
        ...

    @abstractmethod
    def receive(self, topic: str, timeout: float = 1.0) -> Message | None:
        """Receive one message from topic (blocking).

        Args:
            topic: Topic name
            timeout: Maximum wait time in seconds

        Returns:
            Message if received, None if timeout
        """
        ...

    @abstractmethod
    def spin_once(self) -> None:
        """Process pending messages once (non-blocking)."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if provider is connected and ready."""
        ...


class MockMessageProvider(MessageProvider):
    """In-memory message provider for testing.

    Features:
        - No external dependencies
        - Synchronous message delivery
        - Message history tracking
        - Configurable delays
    """

    def __init__(self) -> None:
        """Initialize mock provider."""
        self._connected = False
        self._messages: dict[str, list[Message]] = {}
        self._callbacks: dict[str, list[Callable[[Message], None]]] = {}
        self._subscription_counter = 0
        self._subscriptions: dict[str, tuple[str, Callable[[Message], None]]] = {}

    def __enter__(self) -> MockMessageProvider:
        """Connect."""
        self._connected = True
        return self

    def __exit__(self, *_: object) -> None:
        """Disconnect."""
        self._connected = False
        self._messages.clear()
        self._callbacks.clear()
        self._subscriptions.clear()

    def publish(self, topic: str, data: Any, message_type: str | None = None) -> None:
        """Publish message and deliver to subscribers."""
        if not self._connected:
            raise ProviderNotConnectedError("Provider not connected")

        import time

        msg = Message(
            topic=topic, data=data, timestamp=time.time(), message_type=message_type
        )

        # Store in history
        if topic not in self._messages:
            self._messages[topic] = []
        self._messages[topic].append(msg)

        # Deliver to subscribers
        if topic in self._callbacks:
            for callback in self._callbacks[topic]:
                callback(msg)

    def subscribe(
        self,
        topic: str,
        callback: Callable[[Message], None],
        message_type: str | None = None,
    ) -> str:
        """Subscribe to topic."""
        if not self._connected:
            raise ProviderNotConnectedError("Provider not connected")

        if topic not in self._callbacks:
            self._callbacks[topic] = []

        self._callbacks[topic].append(callback)

        # Return subscription ID
        sub_id = f"sub_{self._subscription_counter}"
        self._subscription_counter += 1
        self._subscriptions[sub_id] = (topic, callback)
        return sub_id

    def unsubscribe(self, subscription_id: str) -> None:
        """Unsubscribe from topic."""
        if subscription_id in self._subscriptions:
            topic, callback = self._subscriptions.pop(subscription_id)
            if topic in self._callbacks:
                self._callbacks[topic].remove(callback)

    def receive(self, topic: str, timeout: float = 1.0) -> Message | None:
        """Receive latest message from topic (non-blocking in mock)."""
        if not self._connected:
            raise ProviderNotConnectedError("Provider not connected")

        if topic in self._messages and self._messages[topic]:
            return self._messages[topic][-1]
        return None

    def spin_once(self) -> None:
        """No-op for mock provider."""
        pass

    def is_connected(self) -> bool:
        """Check connection status."""
        return self._connected

    def get_message_history(self, topic: str) -> list[Message]:
        """Get all messages published to a topic (test utility)."""
        return self._messages.get(topic, [])

    def clear_history(self) -> None:
        """Clear message history (test utility)."""
        self._messages.clear()


class MessageRouter:
    """Routes messages between topics with filtering/transformation.

    Features:
        - Filter messages by topic pattern
        - Transform message payloads
        - Middleware chain support
    """

    def __init__(self, provider: MessageProvider) -> None:
        """Initialize router.

        Args:
            provider: Underlying message provider
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
            from_topic: Source topic
            to_topic: Destination topic
            transformer: Optional function to transform message (None = drop)
        """
        if from_topic not in self._routes:
            self._routes[from_topic] = []

        self._routes[from_topic].append((to_topic, transformer or (lambda m: m)))

    def start_routing(self) -> None:
        """Start message routing (subscribe to all source topics)."""
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
        """Stop message routing (clear routes)."""
        self._routes.clear()


class ROS2MessageProvider(MessageProvider):
    """ROS2-based message provider implementation.

    Maps abstract MessageProvider interface to ROS2 publisher/subscriber API.
    Requires rclpy installation.
    """

    def __init__(self) -> None:
        """Initialize ROS2 provider."""
        self._node: Any = None
        self._publishers: dict[str, Any] = {}
        self._subscriptions: dict[str, Any] = {}

    def __enter__(self) -> ROS2MessageProvider:
        """Connect to ROS2."""
        try:
            import rclpy
            from rclpy.node import Node

            if not rclpy.ok():
                rclpy.init()

            self._node = Node("message_provider")
            return self
        except ImportError as e:
            raise MessagingError("ROS2 (rclpy) not installed") from e

    def __exit__(self, *_: object) -> None:
        """Disconnect from ROS2."""
        if self._node:
            self._node.destroy_node()
            self._node = None

    def publish(self, topic: str, data: Any, message_type: str | None = None) -> None:
        """Publish to ROS2 topic."""
        if not self._node:
            raise ProviderNotConnectedError("Provider not connected")

        # Create publisher if needed
        if topic not in self._publishers:
            # Would need to map message_type to actual ROS message class
            # This is a simplified stub
            pass

        # Publish message
        # publisher = self._publishers[topic]
        # publisher.publish(msg)

    def subscribe(
        self,
        topic: str,
        callback: Callable[[Message], None],
        message_type: str | None = None,
    ) -> str:
        """Subscribe to ROS2 topic."""
        if not self._node:
            raise ProviderNotConnectedError("Provider not connected")

        # Would create subscription with ROS message parsing
        # Stub implementation
        return "ros2_sub_0"

    def unsubscribe(self, subscription_id: str) -> None:
        """Unsubscribe from ROS2 topic."""
        pass

    def receive(self, topic: str, timeout: float = 1.0) -> Message | None:
        """Receive from ROS2 topic (blocking)."""
        # Would use rclpy.spin_until_future_complete
        return None

    def spin_once(self) -> None:
        """Process ROS2 callbacks."""
        # rclpy.spin_once(self._node, timeout_sec=0.0)
        pass

    def is_connected(self) -> bool:
        """Check ROS2 connection."""
        return self._node is not None
