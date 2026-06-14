"""Generic camera streaming driver."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Generator
from contextlib import suppress
from queue import Empty, Queue
from typing import TYPE_CHECKING, Self

if TYPE_CHECKING:
    from collections.abc import Generator

    import numpy as np

from src.hardware.camera.config import Config as CameraConfig
from src.hardware.camera.rpi.camera_module_3.driver import (
    Config as RPiCameraConfig,
    Driver as CameraDriver,
)
from src.logger import configure_json_logging

configure_json_logging()


class StreamingDriver:
    """Generic streaming driver that wraps camera-specific drivers."""

    def __init__(self, config: CameraConfig | None = None):
        self.config = config or CameraConfig()
        self._camera_driver: CameraDriver = self._create_driver()
        self._running = False
        self._capture_thread: threading.Thread | None = None
        self._frame_queue: Queue[np.ndarray] = Queue(maxsize=2)
        self._logger = logging.getLogger(__name__)

    def _create_driver(self) -> CameraDriver:
        """Create the appropriate camera driver based on config or device detection."""
        camera_config = RPiCameraConfig(
            device=self.config.device,
            width=self.config.width,
            height=self.config.height,
            fps=self.config.fps,
            rotation=self.config.rotation,
            hflip=self.config.hflip,
            vflip=self.config.vflip,
        )
        return CameraDriver(camera_config)

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

    def _capture_loop(self) -> None:
        """Continuous capture loop."""
        while self._running:
            try:
                frame = self._camera_driver.get_latest_frame()

                if frame is None:
                    time.sleep(0.001)
                    continue

                if self._frame_queue.full():
                    with suppress(Empty):
                        self._frame_queue.get_nowait()

                self._frame_queue.put(frame)
            except Exception as e:
                self._logger.exception("Capture error: %s", e)  # noqa: TRY401
                time.sleep(0.1)

    def start_streaming(self) -> None:
        """Start continuous frame capture in background thread."""
        if self._running:
            return

        self.connect()

        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        self._logger.info("Streaming started")

    def stop_streaming(self) -> None:
        """Stop continuous frame capture."""
        self._running = False

        if self._capture_thread:
            self._capture_thread.join(timeout=2.0)

        self._logger.info("Streaming stopped")

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
