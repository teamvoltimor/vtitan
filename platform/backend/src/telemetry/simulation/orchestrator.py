"""Simulation loop orchestration and coordination.

Separates simulation driving logic from HTTP routing (Single Responsibility).
"""

from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.telemetry.generator_sim import TelemetryGenerator
    from src.telemetry.ws.manager import ConnectionManager

logger = logging.getLogger(__name__)


class SimulationOrchestrator:
    """Drives simulation loop and coordinates broadcasts.

    Single Responsibility: Only manages simulation + broadcast coordination,
    not HTTP routing or connection management.
    """

    def __init__(
        self,
        generator: TelemetryGenerator,
        connection_manager: ConnectionManager,
        poll_interval_ms: float = 2500,
    ) -> None:
        """Initialize orchestrator.

        Args:
            generator: Telemetry generator instance.
            connection_manager: WebSocket connection pool.
            poll_interval_ms: Milliseconds between simulation frames.
        """
        self.generator = generator
        self.connection_manager = connection_manager
        self.poll_interval_sec = poll_interval_ms / 1000.0
        self._error_count = 0
        self._max_consecutive_errors = 5

    async def run_loop(self) -> None:
        """Main simulation loop with error resilience.

        Drives the simulation and broadcasts frames to connected clients.
        Implements exponential backoff on errors (Error Handling Pillar).

        Raises:
            asyncio.CancelledError: When gracefully shut down.
            RuntimeError: If errors exceed threshold.
        """
        logger.info("Simulation loop started")

        try:
            while True:
                try:
                    # Guard clause: only broadcast if clients connected (Logic-Cleaner Rule 1)
                    if self.connection_manager.has_active_connections():
                        snapshot = self.generator.latest_snapshot()
                        json_data = snapshot.model_dump_json()
                        await self.connection_manager.broadcast(json_data)
                        self._error_count = 0  # Reset on success

                    await asyncio.sleep(self.poll_interval_sec)

                except asyncio.CancelledError:
                    logger.info("Simulation loop cancelled")
                    raise

                except Exception as exc:
                    self._error_count += 1
                    logger.exception(
                        "Simulation loop error (attempt %s/%s)",
                        self._error_count,
                        self._max_consecutive_errors,
                        exc_info=exc,
                    )

                    # Fail fast after max retries (Logic-Cleaner Rule 1: Guard Clauses)
                    if self._error_count >= self._max_consecutive_errors:
                        logger.critical(
                            "Simulation loop exhausted error retries (%s)",
                            self._max_consecutive_errors,
                            exc_info=exc,
                        )
                        msg = f"Simulation loop failed {self._max_consecutive_errors} times"
                        raise RuntimeError(msg) from exc

                    # Exponential backoff before retry (Error Handling: Resiliency)
                    backoff_seconds = min(2**self._error_count, 30)
                    logger.warning("Retrying simulation after %ss backoff", backoff_seconds)
                    await asyncio.sleep(backoff_seconds)

        except asyncio.CancelledError:
            logger.info("Simulation loop cancelled during error handling")
        except Exception as exc:
            logger.critical(
                "Simulation loop terminated with unrecovered error",
                exc_info=exc,
            )
            raise
