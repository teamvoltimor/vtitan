"""Abstract base classes for camera implementations."""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np
from pydantic import AliasChoices, Field
from shared.domain.models import CameraSize, ImageRotation

from src.hardware.settings_base import HardwareBaseSettings


@dataclass(slots=True)
class Frame:
    """Captured camera frame."""

    frame: np.ndarray
    timestamp: float
    width: int
    height: int


class Config(HardwareBaseSettings):
    """Fields and orientation logic shared by every concrete camera backend.

    Subclasses add their own backend-specific fields (device path, autofocus,
    ...) and their own ``model_config`` (env prefix, toml file).
    """

    width: int = Field(default=1536, validation_alias=AliasChoices("CAMERA_WIDTH", "camera_width"))
    height: int = Field(default=864, validation_alias=AliasChoices("CAMERA_HEIGHT", "camera_height"))
    fps: int = Field(default=30, validation_alias=AliasChoices("CAMERA_FPS", "camera_fps"))

    inverted: bool = Field(default=False, validation_alias=AliasChoices("CAMERA_INVERTED", "camera_inverted"))
    """
    True when the camera is mounted upside-down. Applies a 180 degree rotation, which matters beyond looking right: an unrotated frame mirrors which side of the image a sign falls on, so a sign to be passed on the left is reported to the right of centre.
    """

    hflip: bool = Field(default=False, validation_alias=AliasChoices("CAMERA_HFLIP", "camera_hflip"))
    """
    Mirror horizontally. Note a horizontal flip alone also swaps left and right in the detections.
    """

    vflip: bool = Field(default=False, validation_alias=AliasChoices("CAMERA_VFLIP", "camera_vflip"))
    """
    Mirror vertically. Prefer `inverted` for an upside-down mount: a 180 degree rotation is hflip and vflip together, and setting only one of them mirrors the scene rather than righting it.
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
