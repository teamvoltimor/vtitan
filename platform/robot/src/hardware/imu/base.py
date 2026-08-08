from abc import ABC, abstractmethod
from dataclasses import dataclass

from src.hardware.imu.readings import (
    AccelerometerReading,
    EulerReading,
    GyroscopeReading,
    LinearAccelelerometerReading,
    MagnetometerReading,
    QuaternionReading,
    RVCReading,
)


@dataclass(slots=True)
class Data:
    """Aggregate IMU sensor reading returned by ``Driver.get_all_data()``."""

    accelerometer: tuple[float, float, float]
    gyroscope: tuple[float, float, float]
    magnetometer: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    euler: tuple[float, float, float]
    linear_accel: tuple[float, float, float]


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
    def enable_sensors(self) -> None:
        """Enable all sensor feature reports on the IMU."""

    @abstractmethod
    def get_all_data(self) -> Data:
        """Get all sensor Readings in one aggregate snapshot."""

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
