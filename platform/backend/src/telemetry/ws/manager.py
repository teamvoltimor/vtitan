"""WebSocket connection management for real-time telemetry streaming."""

from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING

from fastapi import WebSocket

if TYPE_CHECKING:
    from src.telemetry.ws.protocol import TelemetryMessage

logger = logging.getLogger(__name__)


class ConnectionManager:
    """Manages WebSocket connection lifecycle and broadcasting.

    Single Responsibility (SOLID): Only manages connections, not message protocol.
    Typed error handling replaces generic exception swallowing.
    """

    def __init__(self) -> None:
        """Initialize connection pool."""
        self.active_connections: list[WebSocket] = []
        self._metrics = {
            "broadcast_failures": 0,
            "client_disconnects": 0,
            "broadcast_timeouts": 0,
        }

    def has_active_connections(self) -> bool:
        """Check if any clients are connected.

        Returns:
            True if at least one active connection exists.
        """
        return len(self.active_connections) > 0

    async def connect(self, websocket: WebSocket) -> None:
        """Accept a new WebSocket connection.

        Args:
            websocket: FastAPI WebSocket instance.
        """
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.debug(
            "Client connected",
            extra={"client": websocket.client, "total_connections": len(self.active_connections)},
        )

    def disconnect(self, websocket: WebSocket) -> None:
        """Remove a WebSocket connection from the pool.

        Args:
            websocket: FastAPI WebSocket instance.
        """
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            logger.debug(
                "Client disconnected",
                extra={"client": websocket.client, "total_connections": len(self.active_connections)},
            )

    async def broadcast(self, data: str) -> None:
        """Broadcast JSON string to all connected clients concurrently using asyncio.TaskGroup.

        Handles timeouts, disconnections, and unexpected errors with specific
        logging for observability (Error Handling Pillar).

        Args:
            data: JSON string to broadcast.
        """
        if not self.active_connections:
            return

        disconnected: list[WebSocket] = []

        async def _send_to_client(connection: WebSocket) -> None:
            try:
                await asyncio.wait_for(
                    connection.send_text(data),
                    timeout=5.0,
                )
            except asyncio.TimeoutError:
                logger.warning(
                    "Broadcast timeout for client",
                    extra={"client": connection.client},
                )
                disconnected.append(connection)
                self._metrics["broadcast_timeouts"] += 1
            except RuntimeError as exc:
                if "connection is not established" in str(exc):
                    logger.debug("Client already disconnected")
                    disconnected.append(connection)
                    self._metrics["client_disconnects"] += 1
                else:
                    logger.error(
                        "Unexpected runtime error during broadcast",
                        exc_info=exc,
                        extra={"client": connection.client},
                    )
                    disconnected.append(connection)
                    self._metrics["broadcast_failures"] += 1
            except Exception as exc:
                logger.error(
                    "Unexpected error broadcasting to client",
                    exc_info=exc,
                    extra={"client": connection.client},
                )
                disconnected.append(connection)
                self._metrics["broadcast_failures"] += 1

        try:
            async with asyncio.TaskGroup() as tg:
                for connection in list(self.active_connections):
                    tg.create_task(_send_to_client(connection))
        except Exception as exc:
            logger.error("Error in broadcast task group", exc_info=exc)

        # Clean up disconnected clients (single responsibility)
        for connection in disconnected:
            self.disconnect(connection)

    def get_metrics(self) -> dict:
        """Return broadcast metrics for monitoring.

        Returns:
            Dictionary with broadcast statistics.
        """
        return self._metrics.copy()
