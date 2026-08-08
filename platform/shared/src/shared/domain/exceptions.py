"""Centralized exception hierarchy for the vTitan V2 platform.

Provides domain-specific error handling instead of generic catch-alls.
"""

from __future__ import annotations


class PlatformError(Exception):
    """Base exception for all custom platform errors."""


class HardwareError(PlatformError):
    """Base exception for hardware communication failures."""


class I2CCommunicationError(HardwareError):
    """Raised when an I2C transaction fails (e.g. IMU or Motor disconnect)."""


class UARTCommunicationError(HardwareError):
    """Raised when UART serial read/write fails."""


class MessagingError(PlatformError):
    """Base exception for messaging and broker failures."""


class ProviderNotConnectedError(MessagingError):
    """Raised when attempting to use a message provider before connecting."""


class TopicFormatError(MessagingError):
    """Raised when an invalid topic name or message structure is used."""


class ConfigError(PlatformError):
    """Base exception for configuration and startup validation failures."""


class ScenarioFormatError(ConfigError):
    """Raised when simulation scenario JSON metadata is invalid."""


class TelemetryParseError(PlatformError):
    """Raised when incoming telemetry or WebSocket data violates the expected schema."""
