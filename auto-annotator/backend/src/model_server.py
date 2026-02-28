"""Helpers for interacting with the SAM TCP model server."""

from __future__ import annotations

from src.constants import SERVER_PORT, SERVER_PORT_SOURCE
from src.sam_client import ModelServerClient
from src.utils import get_logger

logger = get_logger(__name__)


def connect_to_model_server() -> ModelServerClient | None:
    """Return a connected client, or ``None`` when no server is reachable."""
    candidate = ModelServerClient()
    if candidate.ping():
        logger.info(
            "Connected to model server on port %s (%s).",
            SERVER_PORT,
            SERVER_PORT_SOURCE,
        )
        return candidate

    logger.info(
        "Model server not reachable on port %s (%s); falling back to local inference.",
        SERVER_PORT,
        SERVER_PORT_SOURCE,
    )
    return None
