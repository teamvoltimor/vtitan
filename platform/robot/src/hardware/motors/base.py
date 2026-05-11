"""Abstract base classes for motor implementations."""

from abc import ABC, abstractmethod


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

    @abstractmethod
    def move_steering_to_right_from_center(self, position: float, speed: int = 20) -> None:
        """Move steering to right relative position in degrees from center position."""

    @abstractmethod
    def move_steering_to_left_from_center(self, position: float, speed: int = 20) -> None:
        """Move steering to left relative position in degrees from center position."""
