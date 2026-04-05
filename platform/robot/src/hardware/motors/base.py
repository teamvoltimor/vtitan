"""Abstract base classes for motor implementations."""

from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class CalibrationData:
    """Motor calibration data."""

    left_limit: float
    right_limit: float
    center: float = 0.0


@dataclass
class Config:
    """Motor configuration."""

    steering_port: str
    drive_port: str
    default_speed: int
    test_duration: float


class Driver(ABC):
    """Abstract motor driver."""

    @abstractmethod
    def connect(self) -> None:
        """Connect to motors."""

    @abstractmethod
    def get_steering_position(self) -> float:
        """Get current steering position in degrees."""

    @abstractmethod
    def get_drive_position(self) -> float:
        """Get current drive position in degrees."""

    @abstractmethod
    def get_steering_speed(self) -> float:
        """Get current steering speed in degrees/s."""

    @abstractmethod
    def get_drive_speed(self) -> float:
        """Get current drive speed in degrees/s."""

    @abstractmethod
    def run_drive_forward(self, speed: int | None = None) -> None:
        """Run drive motor forward."""

    @abstractmethod
    def run_drive_reverse(self, speed: int | None = None) -> None:
        """Run drive motor in reverse."""

    @abstractmethod
    def stop_drive(self) -> None:
        """Stop drive motor."""

    @abstractmethod
    def move_steering_to(self, position: float, speed: int = 20) -> None:
        """Move steering to absolute position."""

    @abstractmethod
    def center_steering(self) -> None:
        """Center steering wheels."""
