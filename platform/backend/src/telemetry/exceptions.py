"""Custom exception hierarchy for telemetry operations.

All exceptions inherit from TelemetryError for easy filtering and handling.
Follows Python-Architect Rule 6: Custom exception hierarchy.
"""

from __future__ import annotations


class TelemetryError(Exception):
    """Base exception for all telemetry operations."""


class ConfigurationError(TelemetryError):
    """Configuration loading or validation failed."""


class RecorderError(TelemetryError):
    """Recording operation failed."""


class SessionNotFoundError(RecorderError):
    """Requested replay session does not exist."""


class CorruptedSessionError(RecorderError):
    """Session file contains corrupted or invalid entries."""


class BroadcastError(TelemetryError):
    """WebSocket broadcast operation failed."""


class SimulationError(TelemetryError):
    """Simulation loop encountered unrecoverable error."""
