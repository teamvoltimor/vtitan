from abc import ABC, abstractmethod

from src.hardware.imu.readings import (
    AccelerometerReading,
    EulerReading,
    GyroscopeReading,
    LinearAccelelerometerReading,
    MagnetometerReading,
    QuaternionReading,
    RVCReading,
)


class Driver(ABC):
    """Abstract IMU driver."""

    @abstractmethod
    def connect(self) -> None:
        """Connect to IMU."""

    @abstractmethod
    def get_accelerometer(self) -> AccelerometerReading:
        """Get accelerometer data (m/s²)."""

    @abstractmethod
    def get_gyroscope(self) -> GyroscopeReading:
        """Get gyroscope data (rad/s)."""

    @abstractmethod
    def get_magnetometer(self) -> MagnetometerReading:
        """Get magnetometer data (µT)."""

    @abstractmethod
    def get_quaternion(self) -> QuaternionReading:
        """Get fused quaternion (w, x, y, z)."""

    @abstractmethod
    def get_euler(self) -> EulerReading:
        """Get fused Euler angles (pitch, roll, yaw) in degrees."""

    @abstractmethod
    def get_linear_acceleration(self) -> LinearAccelelerometerReading:
        """Get linear acceleration (m/s², gravity removed)."""

    @abstractmethod
    def close(self) -> None:
        """Close connection."""


class RVCDriver(ABC):
    """Abstract IMU Robot Vacuum Cleaner driver (reduced data set)."""

    @abstractmethod
    def connect(self) -> None:
        """Connect to IMU."""

    @abstractmethod
    def start_polling(self) -> None:
        """Start background polling."""

    @abstractmethod
    def stop_polling(self) -> None:
        """Stop background polling."""

    @abstractmethod
    def get_data(self) -> RVCReading | None:
        """Get latest sensor data."""

    @abstractmethod
    def close(self) -> None:
        """Close connection."""
