"""RPi Camera Module 3 Wide driver implementation with Picamera2."""

import logging
import threading
import time
from queue import Queue
from dataclasses import dataclass, field
from typing import Generator, Optional

import numpy as np
from picamera2 import Picamera2

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
    rotation: int = 0
    hflip: bool = False
    vflip: bool = False


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
                }
            },
        )

        self._picamera2 = Picamera2(self.config.device)

        config = self._picamera2.create_video_configuration(
            main={"size": (self.config.width, self.config.height)},
            controls={
                "AnalogueGain": 1.0,
                "FrameRate": self.config.fps,
            },
        )
        self._picamera2.configure(config)

        if self.config.rotation:
            self._picamera2.set_controls({"Rotation": self.config.rotation})
        if self.config.hflip:
            self._picamera2.set_controls({"HFlip": True})
        if self.config.vflip:
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

        self.logger.debug("Frame captured", extra={"details": {"width": width, "height": height}})
        return Frame(frame=frame, timestamp=timestamp, width=width, height=height)

    def get_resolution(self) -> tuple[int, int]:
        """Get current resolution."""
        return self.config.width, self.config.height

    def _capture_loop(self) -> None:
        """Continuous capture loop for streaming."""
        while self._running:
            try:
                frame = self.picamera2.capture_array()

                if self._frame_queue.full():
                    try:
                        self._frame_queue.get_nowait()
                    except Exception:
                        pass

                self._frame_queue.put(frame)
            except Exception as e:
                self.logger.error(f"Capture error: {e}")
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
        except Exception:
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

        self.logger.info("FPS measured", extra={"details": {"fps": fps, "num_frames": num_frames}})
        return fps

    def measure_latency(self, num_frames: int = 10) -> float:
        """Measure average frame capture latency in seconds."""
        latencies = []

        for _ in range(num_frames):
            start = time.time()
            self.picamera2.capture_array()
            latencies.append(time.time() - start)

        avg_latency = sum(latencies) / len(latencies)
        self.logger.info("Latency measured", extra={"details": {"avg_latency_ms": avg_latency * 1000}})
        return avg_latency

    def close(self) -> None:
        """Close camera."""
        self.stop_streaming()
        if self._picamera2:
            self._picamera2.stop()
            self.logger.info("Camera closed")
