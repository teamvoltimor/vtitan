"""Centralized exception hierarchy for the Klevor V2 platform.

Provides domain-specific error handling instead of generic catch-alls.
"""

from __future__ import annotations


class PlatformError(Exception):
    """Base exception for all custom platform errors."""

    pass


class HardwareError(PlatformError):
    """Base exception for hardware communication failures."""

    pass


class I2CCommunicationError(HardwareError):
    """Raised when an I2C transaction fails (e.g. IMU or Motor disconnect)."""

    pass


class UARTCommunicationError(HardwareError):
    """Raised when UART serial read/write fails."""

    pass


class MessagingError(PlatformError):
    """Base exception for messaging and broker failures."""

    pass


class ProviderNotConnectedError(MessagingError):
    """Raised when attempting to use a message provider before connecting."""

    pass


class TopicFormatError(MessagingError):
    """Raised when an invalid topic name or message structure is used."""

    pass


class ConfigError(PlatformError):
    """Base exception for configuration and startup validation failures."""

    pass


class ScenarioFormatError(ConfigError):
    """Raised when simulation scenario JSON metadata is invalid."""

    pass


class TelemetryParseError(PlatformError):
    """Raised when incoming telemetry or WebSocket data violates the expected schema."""

    pass
