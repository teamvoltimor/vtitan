"""RPi Camera Module 3 Wide driver implementation."""

import logging
import time
from dataclasses import dataclass

import numpy as np

from src.env import EnvVar
from src.hardware.camera.base import (
    Config as BaseConfig,
    Driver as CameraDriver,
    Frame,
)
from src.logger import configure_json_logging

configure_json_logging()

CAMERA_DEVICE = EnvVar[str](key="CAMERA_DEVICE", default="/dev/video0")
CAMERA_WIDTH = EnvVar[int](key="CAMERA_WIDTH", default=1536, cast=int)
CAMERA_HEIGHT = EnvVar[int](key="CAMERA_HEIGHT", default=864, cast=int)
CAMERA_FPS = EnvVar[int](key="CAMERA_FPS", default=30, cast=int)


@dataclass
class Config(BaseConfig):
    """Camera configuration for RPi Camera Module 3."""

    device: str = CAMERA_DEVICE.value
    width: int = CAMERA_WIDTH.value
    height: int = CAMERA_HEIGHT.value
    fps: int = CAMERA_FPS.value


class Driver(CameraDriver):
    """Driver for RPi Camera Module 3 Wide."""

    def __init__(self, config: Config | None = None):
        self.config = config or Config()
        self._capture = None
        self.logger = logging.getLogger(__name__)

    def connect(self) -> None:
        """Open camera device."""
        import cv2

        self.logger.info("Opening camera", extra={"details": {"device": self.config.device}})
        self._capture = cv2.VideoCapture(self.config.device)

        if not self._capture.isOpened():
            raise RuntimeError(f"Cannot open camera {self.config.device}")

        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        self._capture.set(cv2.CAP_PROP_FPS, self.config.fps)

        self.logger.info("Camera opened")

    @property
    def capture(self):
        """Get capture instance."""
        if self._capture is None:
            self.connect()
        return self._capture

    def capture_frame(self) -> Frame:
        """Capture a single frame."""
        ret, frame = self.capture.read()

        if not ret:
            raise RuntimeError("Failed to capture frame")

        timestamp = time.time()
        height, width = frame.shape[:2]

        self.logger.debug("Frame captured", extra={"details": {"width": width, "height": height}})
        return Frame(frame=frame, timestamp=timestamp, width=width, height=height)

    def get_resolution(self) -> tuple[int, int]:
        """Get current resolution."""
        frame = self.capture_frame()
        return frame.width, frame.height

    def measure_fps(self, num_frames: int = 30) -> float:
        """Measure actual FPS."""
        start_time = time.time()

        for _ in range(num_frames):
            self.capture.read()

        elapsed = time.time() - start_time
        fps = num_frames / elapsed

        self.logger.info("FPS measured", extra={"details": {"fps": fps, "num_frames": num_frames}})
        return fps

    def measure_latency(self, num_frames: int = 10) -> float:
        """Measure average frame capture latency in seconds."""
        latencies = []

        for _ in range(num_frames):
            start = time.time()
            self.capture.read()
            latencies.append(time.time() - start)

        avg_latency = sum(latencies) / len(latencies)
        self.logger.info("Latency measured", extra={"details": {"avg_latency_ms": avg_latency * 1000}})
        return avg_latency

    def close(self) -> None:
        """Close camera."""
        if self._capture:
            self._capture.release()
            self.logger.info("Camera closed")
