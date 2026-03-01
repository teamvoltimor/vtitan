"""Helpers for connecting to the SAM TCP model server."""

from __future__ import annotations

import time

from src.constants import SERVER_PORT, SERVER_PORT_SOURCE
from src.sam_client import ModelServerClient
from src.utils import get_logger

logger = get_logger(__name__)

_RETRY_INTERVAL_SECONDS = 2
_DEFAULT_TIMEOUT_SECONDS = 60


def connect_to_model_server(timeout: float = _DEFAULT_TIMEOUT_SECONDS) -> ModelServerClient | None:
    """Return a connected client, retrying until *timeout* seconds have elapsed.

    The TCP model server binds its socket before loading the SAM model, but
    Python and CUDA imports still take ~15 seconds before that point.  Retrying
    here avoids an unnecessary fallback to local inference that would load a
    second copy of the model into VRAM.

    Args:
        timeout: Maximum seconds to wait before giving up and returning ``None``.

    Returns:
        A live :class:`~src.sam_client.ModelServerClient`, or ``None`` when the
        server is not reachable within *timeout* seconds.
    """
    deadline = time.monotonic() + timeout
    attempt = 0

    while time.monotonic() < deadline:
        candidate = ModelServerClient()
        if candidate.ping():
            logger.info(
                "Connected to model server on port %s (%s) after %d attempt(s).",
                SERVER_PORT,
                SERVER_PORT_SOURCE,
                attempt + 1,
            )
            return candidate

        attempt += 1
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(min(_RETRY_INTERVAL_SECONDS, remaining))

    logger.warning(
        "Model server not reachable on port %s (%s) after %.0fs; falling back to local inference.",
        SERVER_PORT,
        SERVER_PORT_SOURCE,
        timeout,
    )
    return None
