"""RPi Camera Module 3 Wide driver implementation with Picamera2."""

import logging
import time
from collections.abc import Generator
from typing import Any, ClassVar

import numpy as np
from picamera2 import Picamera2
from pydantic import field_validator
from pydantic_settings import SettingsConfigDict
from shared.config.defaults_model import DefaultsModel
from shared.config.generated.hardware.camera.rpi_camera_module_3_schema import (
    HardwareCameraRpiCameraModule3,
)
from shared.domain.models import CameraSize, ImageRotation

from src.hardware.camera.base import (
    Driver as CameraDriver,
    Frame,
)
from src.hardware.camera.frame_streamer import FrameStreamer
from src.hardware.camera.rpi.camera_module_3.enums import (
    AeExposureMode,
    AfMode,
    AfSpeed,
    AwbMode,
    NoiseReductionMode,
)
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings
from src.logger import configure_json_logging
from src.logger.constants import DETAILS_KEY

configure_json_logging()


_COLOUR_NDIM = 3
_RGBA_CHANNELS = 4


class Config(DefaultsModel, HardwareBaseSettings, HardwareCameraRpiCameraModule3):
    """Camera configuration for RPi Camera Module 3.

    Subclasses the generated DTO for the TOML-backed ``camera_*`` keys. The
    shared orientation logic (``resolved_flips`` / ``get_resolution``) is kept
    here, and the two never-committed tuning knobs (autofocus speed, manual
    shutter) stay wrapper-only. The generated fields are all optional because a
    hardware-profile overlay only declares ``camera_awb_mode``; the shipped
    defaults the old hand-written model carried are re-applied as wrapper
    fallbacks (never as DTO fields), and the string-valued enum keys are
    re-cast to their typed enums.
    """

    model_config = SettingsConfigDict(
        env_prefix="",
        toml_file=CONFIG_DIR / "camera" / "rpi_camera_module_3.toml",
    )

    _DEFAULTS: ClassVar[dict[str, Any]] = {
        "camera_device": "/dev/video0",
        "camera_width": 1536,
        "camera_height": 864,
        "camera_fps": 30,
        "camera_inverted": False,
        "camera_rotation": 0,
        "camera_hflip": False,
        "camera_vflip": False,
        "camera_af_mode": "continuous",
        "camera_lens_position": None,
        "camera_ae_exposure_mode": "normal",
        "camera_analogue_gain": 1.0,
        "camera_awb_mode": "auto",
        "camera_noise_reduction_mode": "fast",
        "camera_sharpness": 1.0,
    }

    camera_af_speed: AfSpeed = AfSpeed.NORMAL
    """Passed to libcamera. Only applies when `camera_af_mode` is AUTO or CONTINUOUS.
    Never committed to the TOML (the key is commented out there), so it is an
    env-only wrapper field."""

    camera_exposure_time_us: int | None = None
    """
    Manual exposure time in microseconds. Set together with `camera_analogue_gain` to disable auto-exposure entirely; leave unset to keep AE enabled. Never committed to the TOML (the key is commented out there), so it is an env-only wrapper field.
    """

    @field_validator("camera_af_mode")
    @classmethod
    def _as_af_mode(cls, value: str | None) -> AfMode | None:
        return None if value is None else AfMode(value)

    @field_validator("camera_ae_exposure_mode")
    @classmethod
    def _as_ae_exposure_mode(cls, value: str | None) -> AeExposureMode | None:
        return None if value is None else AeExposureMode(value)

    @field_validator("camera_awb_mode")
    @classmethod
    def _as_awb_mode(cls, value: str | None) -> AwbMode | None:
        return None if value is None else AwbMode(value)

    @field_validator("camera_noise_reduction_mode")
    @classmethod
    def _as_noise_reduction_mode(cls, value: str | None) -> NoiseReductionMode | None:
        return None if value is None else NoiseReductionMode(value)

    def resolved_flips(self) -> tuple[bool, bool]:
        """Effective (hflip, vflip) once `camera_inverted` is folded in.

        An upside-down mount is a 180 degree rotation, which is exactly both
        mirrors at once. Expressing it that way rather than as an explicit
        "Rotation" control keeps it composable with an extra rotation and
        works on sensors whose driver exposes the flips but not arbitrary
        rotation.
        """
        return self.camera_hflip != self.camera_inverted, self.camera_vflip != self.camera_inverted

    def get_resolution(self) -> CameraSize:
        """Get current resolution and orientation metadata."""
        return CameraSize(
            width_px=self.camera_width,
            height_px=self.camera_height,
            rotation_deg=ImageRotation.CW_180 if self.camera_inverted else ImageRotation.NONE,
            hflip=self.camera_hflip,
            vflip=self.camera_vflip,
        )


class Driver(CameraDriver):
    """Driver for RPi Camera Module 3 Wide using Picamera2."""

    def __init__(self, config: Config | None = None):
        self.config = config or Config()
        self._picamera2: Picamera2 | None = None
        self.logger = logging.getLogger(__name__)
        self._streamer: FrameStreamer[np.ndarray] = FrameStreamer(
            self._capture_array,
            maxsize=2,
            error_message="Capture error",
            logger=self.logger,
        )

    def connect(self) -> None:
        """Open camera device."""
        self.logger.info(
            "Opening Picamera2",
            extra={
                "details": {
                    "device": self.config.camera_device,
                    "resolution": (self.config.camera_width, self.config.camera_height),
                    "fps": self.config.camera_fps,
                },
            },
        )

        self._picamera2 = Picamera2(self.config.camera_device)

        # Pin the format. Without it Picamera2 defaults to XBGR8888 and
        # capture_array() returns four channels, which every downstream
        # consumer here assumes is three. Note the naming is a trap:
        # Picamera2's "RGB888" hands back B,G,R in numpy order -- see to_rgb().
        config = self._picamera2.create_video_configuration(
            main={"size": (self.config.camera_width, self.config.camera_height), "format": "RGB888"},
            controls={"FrameRate": self.config.camera_fps, **self._build_detection_controls()},
        )
        self._picamera2.configure(config)

        hflip, vflip = self.config.resolved_flips()
        if self.config.camera_rotation:
            self._picamera2.set_controls({"Rotation": self.config.camera_rotation})
        if hflip:
            self._picamera2.set_controls({"HFlip": True})
        if vflip:
            self._picamera2.set_controls({"VFlip": True})

        self._picamera2.start()
        self.logger.info("Camera opened")

    def _build_detection_controls(self) -> dict[str, object]:
        """Build the libcamera controls dict driving focus/exposure/colour/noise for detection.

        Split out of connect() so it's obvious this, and only this, is where
        Config's typed enum fields turn into libcamera control values.
        """
        cfg = self.config
        controls: dict[str, object] = {
            "AfMode": cfg.camera_af_mode.libcamera_value,
            "AfSpeed": cfg.camera_af_speed.libcamera_value,
            "AeExposureMode": cfg.camera_ae_exposure_mode.libcamera_value,
            "AwbMode": cfg.camera_awb_mode.libcamera_value,
            "NoiseReductionMode": cfg.camera_noise_reduction_mode.libcamera_value,
            "Sharpness": cfg.camera_sharpness,
        }

        if cfg.camera_af_mode == AfMode.MANUAL and cfg.camera_lens_position is not None:
            controls["LensPosition"] = cfg.camera_lens_position

        if cfg.camera_exposure_time_us is not None:
            controls["AeEnable"] = False
            controls["ExposureTime"] = cfg.camera_exposure_time_us
            controls["AnalogueGain"] = cfg.camera_analogue_gain

        return controls

    @property
    def picamera2(self) -> Picamera2:
        """Get Picamera2 instance."""
        if self._picamera2 is None:
            self.connect()
        return self._picamera2

    def _capture_array(self) -> np.ndarray:
        """Raw array capture used as the streamer's producer callback."""
        frame: np.ndarray = self.picamera2.capture_array()
        return frame

    def capture_frame(self) -> Frame:
        """Capture a single frame."""
        frame = self.picamera2.capture_array()
        timestamp = time.time()
        height, width = frame.shape[:2]

        self.logger.debug("Frame captured", extra={DETAILS_KEY: {"width": width, "height": height}})
        return Frame(frame=frame, timestamp=timestamp, width=width, height=height)

    @staticmethod
    def to_rgb(frame: np.ndarray) -> np.ndarray:
        """Convert a captured frame to RGB channel order.

        Picamera2's ``"RGB888"`` is named for the packed byte layout, not the
        numpy axis order: the array comes back B,G,R. Feeding that to the
        detector unconverted swaps red and blue, which reads red prisms as
        green -- the failure that inverts the WRO pass-side rule, and which
        produces no error at all.

        Args:
            frame: Array straight from ``capture_array()``.

        Returns:
            The same pixels in R,G,B order. Four-channel captures are narrowed
            to three first.
        """
        if frame.ndim == _COLOUR_NDIM and frame.shape[2] == _RGBA_CHANNELS:
            frame = frame[:, :, :3]
        return frame[:, :, ::-1]

    def get_resolution(self) -> CameraSize:
        """Get current resolution and orientation metadata."""
        return self.config.get_resolution()

    def start_streaming(self) -> None:
        """Start continuous frame capture in background thread."""
        if self._picamera2 is None:
            self.connect()

        self._streamer.start()
        self.logger.info("Streaming started")

    def stop_streaming(self) -> None:
        """Stop continuous frame capture."""
        self._streamer.stop()
        self.logger.info("Streaming stopped")

    def get_latest_frame(self) -> np.ndarray | None:
        """Get latest frame without blocking."""
        return self._streamer.get_nowait()

    def stream(self) -> Generator[np.ndarray, None, None]:
        """Generator that yields continuous frames."""
        self.start_streaming()
        try:
            while self._streamer.running:
                yield self._streamer.get()
        finally:
            self.stop_streaming()

    def measure_fps(self, num_frames: int = 30) -> float:
        """Measure actual FPS."""
        start_time = time.time()

        for _ in range(num_frames):
            self.picamera2.capture_array()

        elapsed = time.time() - start_time
        fps = num_frames / elapsed

        self.logger.info("FPS measured", extra={DETAILS_KEY: {"fps": fps, "num_frames": num_frames}})
        return fps

    def measure_latency(self, num_frames: int = 10) -> float:
        """Measure average frame capture latency in seconds."""
        latencies = []

        for _ in range(num_frames):
            start = time.time()
            self.picamera2.capture_array()
            latencies.append(time.time() - start)

        avg_latency = sum(latencies) / len(latencies)
        self.logger.info("Latency measured", extra={DETAILS_KEY: {"avg_latency_ms": avg_latency * 1000}})
        return avg_latency

    def close(self) -> None:
        """Close camera."""
        self.stop_streaming()
        if self._picamera2:
            self._picamera2.stop()
            self.logger.info("Camera closed")
