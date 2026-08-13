"""Generic camera streaming driver."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Self

if TYPE_CHECKING:
    from collections.abc import Generator

    import numpy as np

from src.hardware.camera.config import Config as CameraConfig
from src.hardware.camera.frame_streamer import FrameStreamer
from src.hardware.camera.rpi.camera_module_3.driver import (
    Config as RPiCameraConfig,
    Driver as CameraDriver,
)
from src.logger import configure_json_logging

configure_json_logging()


def build_rpi_camera_config(config: CameraConfig) -> RPiCameraConfig:
    """Translate the generic camera Config into the Picamera2 driver's Config."""
    return RPiCameraConfig(
        device=config.device,
        width=config.width,
        height=config.height,
        fps=config.fps,
        rotation=config.rotation,
        hflip=config.hflip,
        vflip=config.vflip,
    )


class StreamingDriver:
    """Generic streaming driver that wraps camera-specific drivers."""

    def __init__(self, config: CameraConfig | None = None):
        self.config = config or CameraConfig()
        self._camera_driver: CameraDriver = self._create_driver()
        self._logger = logging.getLogger(__name__)
        self._streamer: FrameStreamer[np.ndarray] = FrameStreamer(
            self._camera_driver.get_latest_frame,
            maxsize=2,
            idle_sleep=0.001,
            error_message="Capture error",
            logger=self._logger,
        )

    def _create_driver(self) -> CameraDriver:
        """Create the appropriate camera driver based on config or device detection."""
        return CameraDriver(build_rpi_camera_config(self.config))

    def connect(self) -> None:
        """Connect to camera."""
        self._logger.info(
            "Connecting to camera",
            extra={
                "details": {
                    "device": self.config.device,
                    "resolution": (self.config.width, self.config.height),
                    "fps": self.config.fps,
                },
            },
        )
        self._camera_driver.connect()
        self._logger.info("Camera connected")

    def start_streaming(self) -> None:
        """Start continuous frame capture in background thread."""
        if self._streamer.running:
            return

        self.connect()
        self._streamer.start()
        self._logger.info("Streaming started")

    def stop_streaming(self) -> None:
        """Stop continuous frame capture."""
        self._streamer.stop()
        self._logger.info("Streaming stopped")

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

    def capture_frame(self) -> np.ndarray:
        """Capture a single frame."""
        return self._camera_driver.capture_frame().frame

    def get_resolution(self) -> tuple[int, int]:
        """Get current resolution."""
        return self.config.width, self.config.height

    def measure_fps(self, num_frames: int = 30) -> float:
        """Measure actual FPS."""
        return self._camera_driver.measure_fps(num_frames)

    def measure_latency(self, num_frames: int = 10) -> float:
        """Measure average frame capture latency in seconds."""
        return self._camera_driver.measure_latency(num_frames)

    def close(self) -> None:
        """Close camera."""
        self.stop_streaming()
        self._camera_driver.close()
        self._logger.info("Camera closed")

    def __enter__(self) -> Self:
        self.start_streaming()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: object,
    ) -> None:
        self.close()
