"""src.utils – Logging setup."""

from __future__ import annotations

import json
import logging
import sys
from datetime import UTC, datetime
from typing import Any


class _JsonFormatter(logging.Formatter):
    """Emit each log record as a single JSON line.

    Adds a ``ts`` (ISO-8601 UTC timestamp), ``lvl``, ``logger``, and ``msg``
    field.  Any dict stored in ``record._extra`` is merged into the payload,
    and exception tracebacks are included under ``exc`` when present.
    """

    def format(self, record: logging.LogRecord) -> str:
        """Format *record* as a JSON string.

        Args:
            record: Log record to format.

        Returns:
            Single-line JSON string representing the log entry.
        """
        payload: dict[str, Any] = {
            "timestamp": datetime.now(UTC).isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }
        extra = getattr(record, "_extra", None)
        if extra:
            payload.update(extra)
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload)


def get_logger(name: str) -> logging.Logger:
    """Return a JSON-formatting logger attached to stdout.

    Safe to call multiple times with the same *name*; handlers are only
    added on the first call.

    Args:
        name: Logger name, typically the module's ``__name__``.

    Returns:
        Configured :class:`logging.Logger` instance.
    """
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(_JsonFormatter())
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        logger.propagate = False
    return logger