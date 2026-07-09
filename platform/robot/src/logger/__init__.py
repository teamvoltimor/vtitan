"""logger package.

Structured JSON logging utilities.

Public API:
    - COMPETITION_MODE: whether competition mode is enabled (disables all logging).
    - LOG_LEVEL: the configured log level.
    - is_competition_mode: Check if competition mode is enabled.
    - JSONFormatter: A logging.Formatter subclass that formats log records as single-line JSON with structured fields.
    - configure_json_logging: A helper function to set up JSON logging on the root logger with a JSONFormatter.
    - LogPayload: A dataclass representing the structured log entry, responsible for extracting relevant information from a LogRecord and converting it to JSON.
"""

from src.logger.config import (
    COMPETITION_MODE,
    LOG_LEVEL,
    is_competition_mode,
)
from src.logger.json import JSONFormatter, configure_json_logging
from src.logger.payload import LogPayload

__all__ = [
    "COMPETITION_MODE",
    "LOG_LEVEL",
    "JSONFormatter",
    "LogPayload",
    "configure_json_logging",
    "is_competition_mode",
]
