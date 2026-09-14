"""Abstract base classes for camera implementations."""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np
from pydantic import AliasChoices, Field
from shared.config.generated.hardware.camera.config_schema import HardwareCameraConfig
from shared.domain.models import CameraSize, ImageRotation

from src.hardware.settings_base import HardwareBaseSettings


@dataclass(slots=True)
class Frame:
    """Captured camera frame."""

    frame: np.ndarray
    timestamp: float
    width: int
    height: int


class Config(HardwareBaseSettings, HardwareCameraConfig):
    """Fields and orientation logic shared by every concrete camera backend.

    Subclasses the generated generic camera DTO for the TOML-backed capture
    keys. ``inverted`` is the one extra fact this shared layer owns (an
    upside-down mount), kept as wrapper behaviour; concrete backends add their
    own backend-specific fields (device path, autofocus, ...) and their own
    ``model_config`` (env prefix, toml file).
    """

    inverted: bool = Field(default=False, validation_alias=AliasChoices("CAMERA_INVERTED", "camera_inverted"))
    """
    True when the camera is mounted upside-down. Applies a 180 degree rotation, which matters beyond looking right: an unrotated frame mirrors which side of the image a sign falls on, so a sign to be passed on the left is reported to the right of centre.
    """

    def resolved_flips(self) -> tuple[bool, bool]:
        """Effective (hflip, vflip) once `inverted` is folded in.

        An upside-down mount is a 180 degree rotation, which is exactly both
        mirrors at once. Expressing it that way rather than as an explicit
        "Rotation" control keeps it composable with an extra rotation and
        works on sensors whose driver exposes the flips but not arbitrary
        rotation.
        """
        return self.hflip != self.inverted, self.vflip != self.inverted

    def get_resolution(self) -> CameraSize:
        """Get current resolution and orientation metadata."""
        return CameraSize(
            width_px=self.width,
            height_px=self.height,
            rotation_deg=ImageRotation.CW_180 if self.inverted else ImageRotation.NONE,
            hflip=self.hflip,
            vflip=self.vflip,
        )


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
