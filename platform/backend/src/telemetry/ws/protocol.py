"""WebSocket message protocol types (Type Safety Pillar).

Defines typed message envelopes to replace raw JSON strings and ensure
all broadcast messages conform to a single schema.
"""

from __future__ import annotations

import time
from typing import Literal

from pydantic import BaseModel, Field

from src.telemetry.models import RobotSnapshot, TopicsSnapshot


class TelemetryMessage(BaseModel):
    """Typed message envelope for WebSocket communication.

    This ensures all broadcast messages conform to a single schema,
    preventing protocol violations and making the API self-documenting
    (Type Safety Pillar: Type-Safe Modeling).
    """

    message_type: Literal["snapshot", "topics", "heartbeat"]
    """Type of message being sent."""

    timestamp: float = Field(default_factory=time.time)
    """Unix timestamp when message was created."""

    payload: RobotSnapshot | TopicsSnapshot | dict | None = None
    """Message payload (structure depends on message_type)."""
