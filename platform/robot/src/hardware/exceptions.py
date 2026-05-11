"""Hardware-specific exception hierarchy for graceful error handling."""


class HardwareError(Exception):
    """Base class for all hardware errors."""


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
