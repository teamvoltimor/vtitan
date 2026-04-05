"""Abstract base classes for camera implementations."""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    """Captured camera frame."""

    frame: np.ndarray
    timestamp: float
    width: int
    height: int


@dataclass
class Config:
    """Camera configuration."""

    device: str
    width: int
    height: int
    fps: int


class Driver(ABC):
    """Abstract camera driver."""

    @abstractmethod
    def connect(self) -> None:
        """Connect to camera device."""

    @abstractmethod
    def capture_frame(self) -> Frame:
        """Capture a single frame."""

    @abstractmethod
    def get_resolution(self) -> tuple[int, int]:
        """Get current resolution."""

    @abstractmethod
    def close(self) -> None:
        """Close camera."""
