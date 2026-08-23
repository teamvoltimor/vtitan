"""RPi Camera Module 3 Wide driver implementation with Picamera2."""

import logging
import time
from collections.abc import Generator

import numpy as np
from picamera2 import Picamera2
from pydantic import AliasChoices, Field
from pydantic_settings import SettingsConfigDict
from shared.domain.models import CameraSize

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


class Config(HardwareBaseSettings):
    """Camera configuration for RPi Camera Module 3.

    Structurally mirrors src.hardware.camera.base.Config's shape (device,
    width, height, fps) rather than subclassing it -- that base is a plain
    dataclass, and mixing dataclass/pydantic-settings inheritance is fragile.
    """

    model_config = SettingsConfigDict(
        env_prefix="",
        toml_file=CONFIG_DIR / "camera" / "rpi_camera_module_3.toml",
    )

    device: str = Field(default="/dev/video0", validation_alias=AliasChoices("CAMERA_DEVICE", "camera_device"))
    width: int = Field(default=1536, validation_alias=AliasChoices("CAMERA_WIDTH", "camera_width"))
    height: int = Field(default=864, validation_alias=AliasChoices("CAMERA_HEIGHT", "camera_height"))
    fps: int = Field(default=30, validation_alias=AliasChoices("CAMERA_FPS", "camera_fps"))

    inverted: bool = Field(default=False, validation_alias=AliasChoices("CAMERA_INVERTED", "camera_inverted"))
    """
    True when the camera is mounted upside-down, as the LIDAR already is. Applies a 180 degree rotation so frames come out the right way up. Without it the image is not merely upside-down for a human: it flips which side of the frame a sign appears on, so a sign the robot should pass on its left is reported to the right of centre.
    """

    rotation: int = Field(default=0, validation_alias=AliasChoices("CAMERA_ROTATION", "camera_rotation"))
    """
    Extra rotation in degrees, applied on top of `inverted` for mounts that are neither upright nor a clean 180.
    """

    hflip: bool = Field(default=False, validation_alias=AliasChoices("CAMERA_HFLIP", "camera_hflip"))
    """
    Mirror horizontally. Note a horizontal flip alone also swaps left and right in the detections.
    """

    vflip: bool = Field(default=False, validation_alias=AliasChoices("CAMERA_VFLIP", "camera_vflip"))
    """
    Mirror vertically. Prefer `inverted` for an upside-down mount: a 180 degree rotation is hflip and vflip together, and setting only one of them mirrors the scene rather than righting it.
    """

    af_mode: AfMode = Field(
        default=AfMode.CONTINUOUS, validation_alias=AliasChoices("CAMERA_AF_MODE", "camera_af_mode")
    )
    """
    Continuous AF can hunt (and blur) mid-detection; switch to MANUAL with `lens_position` set once the working distance to signs/obstacles is known.
    """

    lens_position: float | None = Field(
        default=None, validation_alias=AliasChoices("CAMERA_LENS_POSITION", "camera_lens_position")
    )
    """
    Dioptres (1/distance_m) used when `af_mode = MANUAL`. Ignored otherwise.
    """

    af_speed: AfSpeed = Field(
        default=AfSpeed.NORMAL, validation_alias=AliasChoices("CAMERA_AF_SPEED", "camera_af_speed")
    )
    """Only applies when `af_mode` is AUTO or CONTINUOUS."""

    ae_exposure_mode: AeExposureMode = Field(
        default=AeExposureMode.NORMAL,
        validation_alias=AliasChoices("CAMERA_AE_EXPOSURE_MODE", "camera_ae_exposure_mode"),
    )
    """SHORT biases auto-exposure toward shorter exposure times (less motion blur, more noise)."""

    exposure_time_us: int | None = Field(
        default=None, validation_alias=AliasChoices("CAMERA_EXPOSURE_TIME_US", "camera_exposure_time_us")
    )
    """
    Manual exposure time in microseconds. Set together with `analogue_gain` to disable auto-exposure entirely; leave unset to keep AE enabled.
    """

    analogue_gain: float = Field(
        default=1.0, validation_alias=AliasChoices("CAMERA_ANALOGUE_GAIN", "camera_analogue_gain")
    )
    """Sensor gain. Only fixed when `exposure_time_us` is also set; otherwise AE is free to adjust it."""

    awb_mode: AwbMode = Field(default=AwbMode.AUTO, validation_alias=AliasChoices("CAMERA_AWB_MODE", "camera_awb_mode"))
    """
    Sign colour classification (red vs green) is threshold-based, so a fixed mode avoids AWB drift shifting hue readings under changing venue lighting.
    """

    noise_reduction_mode: NoiseReductionMode = Field(
        default=NoiseReductionMode.FAST,
        validation_alias=AliasChoices("CAMERA_NOISE_REDUCTION_MODE", "camera_noise_reduction_mode"),
    )
    """HIGH_QUALITY adds latency the control loop can't afford."""

    sharpness: float = Field(default=1.0, validation_alias=AliasChoices("CAMERA_SHARPNESS", "camera_sharpness"))
    """libcamera sharpness multiplier; 1.0 is the sensor default."""


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
                    "device": self.config.device,
                    "resolution": (self.config.width, self.config.height),
                    "fps": self.config.fps,
                },
            },
        )

        self._picamera2 = Picamera2(self.config.device)

        # Pin the format. Without it Picamera2 defaults to XBGR8888 and
        # capture_array() returns four channels, which every downstream
        # consumer here assumes is three. Note the naming is a trap:
        # Picamera2's "RGB888" hands back B,G,R in numpy order -- see to_rgb().
        config = self._picamera2.create_video_configuration(
            main={"size": (self.config.width, self.config.height), "format": "RGB888"},
            controls={"FrameRate": self.config.fps, **self._build_detection_controls()},
        )
        self._picamera2.configure(config)

        hflip, vflip = self.config.resolved_flips()
        if self.config.rotation:
            self._picamera2.set_controls({"Rotation": self.config.rotation})
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
            "AfMode": cfg.af_mode.libcamera_value,
            "AfSpeed": cfg.af_speed.libcamera_value,
            "AeExposureMode": cfg.ae_exposure_mode.libcamera_value,
            "AwbMode": cfg.awb_mode.libcamera_value,
            "NoiseReductionMode": cfg.noise_reduction_mode.libcamera_value,
            "Sharpness": cfg.sharpness,
        }

        if cfg.af_mode == AfMode.MANUAL and cfg.lens_position is not None:
            controls["LensPosition"] = cfg.lens_position

        if cfg.exposure_time_us is not None:
            controls["AeEnable"] = False
            controls["ExposureTime"] = cfg.exposure_time_us
            controls["AnalogueGain"] = cfg.analogue_gain

        return controls

    @property
    def picamera2(self) -> Picamera2:
        """Get Picamera2 instance."""
        if self._picamera2 is None:
            self.connect()
        return self._picamera2

    def _capture_array(self) -> np.ndarray:
        """Raw array capture used as the streamer's producer callback."""
        return self.picamera2.capture_array()

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
