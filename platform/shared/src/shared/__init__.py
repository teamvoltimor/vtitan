"""Shared utilities for voldemorbot-platform.

Central module for configuration, enums, and I/O utilities. Currently
consumed only by ``platform/robot`` (Python); the Go backend and TypeScript
frontend do not import it.

Imports:
    config: Configuration, constants, enums
    io: JSONL file utilities
"""

from shared.io import JsonlReader, JsonlValidator, JsonlWriter

__all__ = [
    # I/O Utilities
    "JsonlReader",
    "JsonlValidator",
    "JsonlWriter",
]
