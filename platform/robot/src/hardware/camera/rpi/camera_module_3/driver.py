"""RPi Camera Module 3 Wide driver implementation with Picamera2."""

import logging
import threading
import time
from collections.abc import Generator
from contextlib import suppress
from queue import Empty, Queue

import numpy as np
from picamera2 import Picamera2
from pydantic import AliasChoices, Field
from pydantic_settings import SettingsConfigDict
from shared.domain.models import CameraSize, ImageRotation

from src.hardware.camera.base import (
    Driver as CameraDriver,
    Frame,
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


class Driver(CameraDriver):
    """Driver for RPi Camera Module 3 Wide using Picamera2."""

    def __init__(self, config: Config | None = None):
        self.config = config or Config()
        self._picamera2: Picamera2 | None = None
        self._running = False
        self._capture_thread: threading.Thread | None = None
        self._frame_queue: Queue[np.ndarray] = Queue(maxsize=2)
        self.logger = logging.getLogger(__name__)

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
            controls={
                "AnalogueGain": 1.0,
                "FrameRate": self.config.fps,
            },
        )
        self._picamera2.configure(config)

        # An upside-down mount is a 180 degree rotation, which is exactly both
        # mirrors at once. Expressing it that way rather than as "Rotation"
        # keeps it composable with an explicit rotation and works on sensors
        # whose driver exposes the flips but not arbitrary rotation.
        hflip = self.config.hflip != self.config.inverted
        vflip = self.config.vflip != self.config.inverted
        if self.config.rotation:
            self._picamera2.set_controls({"Rotation": self.config.rotation})
        if hflip:
            self._picamera2.set_controls({"HFlip": True})
        if vflip:
            self._picamera2.set_controls({"VFlip": True})

        self._picamera2.start()
        self.logger.info("Camera opened")

    @property
    def picamera2(self) -> Picamera2:
        """Get Picamera2 instance."""
        if self._picamera2 is None:
            self.connect()
        return self._picamera2

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
        return CameraSize(
            width_px=self.config.width,
            height_px=self.config.height,
            rotation_deg=ImageRotation.CW_180 if self.config.inverted else ImageRotation.NONE,
            hflip=self.config.hflip,
            vflip=self.config.vflip,
        )

    def _capture_loop(self) -> None:
        """Continuous capture loop for streaming."""
        while self._running:
            try:
                frame = self.picamera2.capture_array()

                if self._frame_queue.full():
                    with suppress(Empty):
                        self._frame_queue.get_nowait()

                self._frame_queue.put(frame)
            except Exception:
                self.logger.exception("Capture error")
                time.sleep(0.1)

    def start_streaming(self) -> None:
        """Start continuous frame capture in background thread."""
        if self._running:
            return

        if self._picamera2 is None:
            self.connect()

        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        self.logger.info("Streaming started")

    def stop_streaming(self) -> None:
        """Stop continuous frame capture."""
        self._running = False

        if self._capture_thread:
            self._capture_thread.join(timeout=2.0)

        self.logger.info("Streaming stopped")

    def get_latest_frame(self) -> np.ndarray | None:
        """Get latest frame without blocking."""
        try:
            return self._frame_queue.get_nowait()
        except Empty:
            return None

    def stream(self) -> Generator[np.ndarray, None, None]:
        """Generator that yields continuous frames."""
        self.start_streaming()
        try:
            while self._running:
                frame = self._frame_queue.get()
                yield frame
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
