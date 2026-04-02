from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class Data:
    """IMU sensor data."""

    accelerometer: tuple[float, float, float]
    gyroscope: tuple[float, float, float]
    magnetometer: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    euler: tuple[float, float, float]
    linear_accel: tuple[float, float, float]


@dataclass
class RVCData:
    """IMU RVC mode data (reduced set)."""

    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    x_accel: float
    y_accel: float
    z_accel: float
    quaternion: tuple[float, float, float, float]


class Driver(ABC):
    """Abstract IMU driver."""

    @abstractmethod
    def connect(self) -> None:
        """Connect to IMU."""

    @abstractmethod
    def get_accelerometer(self) -> tuple[float, float, float]:
        """Get accelerometer data (m/s²)."""

    @abstractmethod
    def get_gyroscope(self) -> tuple[float, float, float]:
        """Get gyroscope data (rad/s)."""

    @abstractmethod
    def get_magnetometer(self) -> tuple[float, float, float]:
        """Get magnetometer data (µT)."""

    @abstractmethod
    def get_quaternion(self) -> tuple[float, float, float, float]:
        """Get fused quaternion (w, x, y, z)."""

    @abstractmethod
    def get_euler(self) -> tuple[float, float, float]:
        """Get fused Euler angles (pitch, roll, yaw) in degrees."""

    @abstractmethod
    def get_linear_acceleration(self) -> tuple[float, float, float]:
        """Get linear acceleration (m/s², gravity removed)."""

    @abstractmethod
    def get_all_data(self) -> Data:
        """Get all sensor data."""


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
    def get_data(self) -> RVCData | None:
        """Get latest sensor data."""

    @abstractmethod
    def close(self) -> None:
        """Close connection."""
