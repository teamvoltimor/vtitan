"""Logging configuration constants."""

import logging

from src.env import EnvVar, bool_cast

logger = logging.getLogger(__name__)

LOG_LEVEL = EnvVar[str](key="LOG_LEVEL", default="INFO")
"""
Logging level for the application. Supported levels are: DEBUG, INFO, WARNING, ERROR, CRITICAL.
"""

COMPETITION_MODE = EnvVar[bool](key="COMPETITION_MODE", default=False, cast=bool_cast)
"""
Whether to enable competition mode. When enabled, the logger will use a more compact format suitable for competition environments.
"""


def is_competition_mode() -> bool:
    """Check if competition mode is enabled."""
    return COMPETITION_MODE.value
