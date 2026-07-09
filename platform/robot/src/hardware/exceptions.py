"""Hardware-specific exception hierarchy for graceful error handling."""

from shared.domain.exceptions import HardwareError as _PlatformHardwareError


class HardwareError(_PlatformHardwareError):
    """Base class for all hardware errors raised by this robot's drivers.

    Subclasses the platform-wide shared.domain.exceptions.HardwareError so
    code catching that shared base (e.g. resiliency.with_retry's default
    exception tuple) also catches every driver-specific error below.
    """


class MotorConnectionError(HardwareError):
    """Motor failed to connect (USB/I2C issue)."""

    def __init__(self, port: str | list[str], reason: str):
        """Initialize motor connection error.

        Args:
            port: Motor port identifier (e.g., 'A', 'B', or device path) or list of ports if multiple motors are involved.
            reason: Human-readable reason for connection failure.
        """
        self.port = port
        self.reason = reason
        super().__init__(f"Motor on port {port}: {reason}")


class MotorTimeoutError(HardwareError):
    """Motor command timed out."""


class IMUConnectionError(HardwareError):
    """IMU failed to connect (USB/I2C issue)."""

    def __init__(self, port: str, reason: str):
        """Initialize IMU connection error.

        Args:
            port: IMU port identifier.
            reason: Human-readable reason for connection failure.
        """
        self.port = port
        self.reason = reason
        super().__init__(f"IMU on port {port}: {reason}")


class CameraConnectionError(HardwareError):
    """Camera failed to connect."""
