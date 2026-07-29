"""Abstract base classes for camera implementations."""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np
from shared.domain.models import CameraSize


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
    def get_resolution(self) -> CameraSize:
        """Get current resolution and orientation metadata."""

    @abstractmethod
    def close(self) -> None:
        """Close camera."""

    @staticmethod
    def to_rgb(frame: np.ndarray) -> np.ndarray:
        """Convert a captured frame to RGB channel order for the detector.

        Both camera backends decode into BGR order (cv2.imdecode for rpicam,
        Picamera2's packed "RGB888" for camera_module_3 -- named for byte
        layout, not numpy axis order). Feeding either straight to the
        detector unconverted swaps red and blue, which reads red prisms as
        green -- the failure that inverts the WRO pass-side rule, and which
        produces no error at all. Shared here since both backends' reversal
        was byte-identical.
        """
        return frame[:, :, ::-1]
