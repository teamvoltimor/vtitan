"""Structured JSON logging helper."""

import logging
from typing import override

from src.logger.config import LOG_LEVEL, is_competition_mode
from src.logger.payload import LogPayload


class NullHandler(logging.Handler):
    """Handler that does nothing - for competition mode."""

    @override
    def emit(self, record: logging.LogRecord) -> None:
        """Do nothing."""


class JSONFormatter(logging.Formatter):
    """Format records as single-line JSON."""

    @override
    def format(self, record: logging.LogRecord) -> str:
        """
        Format a log record as JSON with keys.

        Includes the following fields:
        - timestamp: ISO 8601 UTC timestamp of the log event
        - level: log level name (e.g. "info", "error")
        - logger: logger name (e.g. "myapp.module")
        - message: the log message with any args already interpolated
        - details: optional dict of additional structured data (if record.details is a non-empty dict)
        - exc_info: optional string of the formatted exception info (if record.exc_info is set)

        Args:
            record: The LogRecord to format.

        Returns:
            A JSON string representing the log record.
        """
        payload = LogPayload(record)
        return payload.to_json()


def configure_json_logging(level: int | None = None) -> logging.Logger:
    """
    Attach a JSON stream handler to the root logger.

    Args:
        level: The minimum log level to output. Defaults to LOG_LEVEL env var.

    Returns:
        The configured root logger.
    """
    root = logging.getLogger()

    # If competition mode is enabled, disable all logging by attaching a NullHandler and setting level above CRITICAL.
    if is_competition_mode():
        root.handlers = [NullHandler()]
        root.setLevel(logging.CRITICAL + 1)
        return root

    # Otherwise, configure JSON logging with the specified level or LOG_LEVEL env var.
    level_str = LOG_LEVEL.upper() if level is None else str(level)
    logging_level = getattr(logging, level_str, logging.INFO)

    handler = logging.StreamHandler()
    handler.setFormatter(JSONFormatter())

    root.handlers = [handler]
    root.setLevel(logging_level)
    return root
