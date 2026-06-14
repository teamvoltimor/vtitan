"""Shared utilities for voldemorbot-platform.

Central module for configuration, types, enums, I/O utilities, and messaging
abstractions shared across backend, robot, simulation, and frontend services.

Imports:
    config: Configuration, constants, types, enums
    io: JSONL file utilities
    messaging: Message broker abstraction (decouples from ROS2)
"""

from shared.io import JsonlReader, JsonlValidator, JsonlWriter
from shared.messaging import (
    Message,
    MessageProvider,
    MessageRouter,
    MockMessageProvider,
    ROS2MessageProvider,
)

__all__ = [
    # I/O Utilities
    "JsonlReader",
    "JsonlValidator",
    "JsonlWriter",
    # Messaging
    "Message",
    "MessageProvider",
    "MessageRouter",
    "MockMessageProvider",
    "ROS2MessageProvider",
]
