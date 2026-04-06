"""Hardware-specific exception hierarchy for graceful error handling."""


class HardwareError(Exception):
    """Base class for all hardware errors."""

    pass


class MotorConnectionError(HardwareError):
    """Motor failed to connect (USB/I2C issue)."""

    def __init__(self, port: str, reason: str):
        """Initialize motor connection error.

        Args:
            port: Motor port identifier (e.g., 'A', 'B', or device path).
            reason: Human-readable reason for connection failure.
        """
        self.port = port
        self.reason = reason
        super().__init__(f"Motor on port {port}: {reason}")


class MotorCalibrationError(HardwareError):
    """Calibration data missing or invalid."""

    pass


class MotorTimeoutError(HardwareError):
    """Motor command timed out."""

    pass


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


class IMUCalibrationError(HardwareError):
    """IMU calibration data missing or invalid."""

    pass


class CameraConnectionError(HardwareError):
    """Camera failed to connect."""

    pass
