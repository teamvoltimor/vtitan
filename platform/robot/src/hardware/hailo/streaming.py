"""Continuous Hailo inference with camera streaming."""

from __future__ import annotations

import logging
from queue import Empty
from typing import TYPE_CHECKING, Self

import cv2
import numpy as np

if TYPE_CHECKING:
    from collections.abc import Generator

    from src.hardware.hailo.base import Config as HailoConfig
    from src.hardware.hailo.config import StreamingConfig
    from src.hardware.hailo.inferences import InferenceResult

from src.hardware.camera.frame_streamer import FrameStreamer
from src.hardware.camera.streaming import StreamingDriver as CameraStreamingDriver
from src.hardware.hailo.hailo_8.driver import Driver as HailoDriver
from src.logger import configure_json_logging

configure_json_logging()


def preprocess(frame: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
    """Resize a frame to the model's input size.

    Deliberately no scaling to [0, 1]: the HEF's input tensor is UINT8 and the
    compiled graph carries its own ``normalization`` layer, so dividing here
    would both mismatch the dtype and normalize twice.
    """
    resized = cv2.resize(frame, (target_width, target_height))
    return resized.astype(np.uint8)


class StreamingDriver:
    """Continuous inference driver combining camera and Hailo NPU."""

    def __init__(
        self,
        config: StreamingConfig,
        hailo_config: HailoConfig | None = None,
    ):
        self.config = config
        self._hailo_driver = HailoDriver(hailo_config)
        self._camera_streaming_driver: CameraStreamingDriver | None = None
        self._logger = logging.getLogger(__name__)
        self._capture_streamer: FrameStreamer[np.ndarray] = FrameStreamer(
            self._get_latest_camera_frame,
            maxsize=config.queue_size,
            idle_sleep=0.001,
            error_message="Capture error",
            logger=self._logger,
        )
        self._inference_streamer: FrameStreamer[InferenceResult] = FrameStreamer(
            self._run_inference,
            maxsize=config.queue_size,
            error_message="Inference error",
            logger=self._logger,
        )

    def connect(self) -> None:
        """Connect to camera and Hailo device."""
        self._logger.info("Connecting to Hailo NPU")
        self._hailo_driver.connect()
        self._hailo_driver.load_model()
        self._logger.info("Hailo NPU connected")

    def _start_camera(self) -> None:
        """Start camera capture using CameraStreamingDriver."""
        self._logger.info(
            "Starting camera",
            extra={
                "details": {
                    "resolution": (
                        self.config.width,
                        self.config.height,
                    ),
                    "fps": self.config.fps,
                },
            },
        )

        self._camera_streaming_driver = CameraStreamingDriver(self.config)
        self._camera_streaming_driver.start_streaming()
        self._logger.info("Camera started")

    def _get_latest_camera_frame(self) -> np.ndarray | None:
        """Pull the newest frame off the camera driver's own stream."""
        # Only started (see start()) after the camera driver is set.
        assert self._camera_streaming_driver is not None
        return self._camera_streaming_driver.get_latest_frame()

    def _run_inference(self) -> InferenceResult | None:
        """Block for the next captured frame and run inference on it."""
        try:
            frame = self._capture_streamer.get(timeout=1.0)
        except Empty:
            return None

        input_data = preprocess(
            frame,
            self.config.model_input_width,
            self.config.model_input_height,
        )

        return self._hailo_driver.infer_with_image_size(
            input_data,
            original_width=frame.shape[1],
            original_height=frame.shape[0],
            image=frame,
        )

    def start(self) -> None:
        """Start continuous inference."""
        if self._inference_streamer.running:
            return

        self.connect()
        self._start_camera()

        self._capture_streamer.start()
        self._inference_streamer.start()

        self._logger.info("Streaming started")

    def stop(self) -> None:
        """Stop continuous inference."""
        self._capture_streamer.stop()
        self._inference_streamer.stop()

        if self._camera_streaming_driver:
            self._camera_streaming_driver.close()
            self._logger.info("Camera stopped")

        self._logger.info("Streaming stopped")

    def get_latest(self) -> InferenceResult | None:
        """Get latest inference result without blocking."""
        return self._inference_streamer.get_nowait()

    @property
    def camera_driver(self) -> CameraStreamingDriver | None:
        """Get camera driver for direct access if needed."""
        return self._camera_streaming_driver

    @property
    def hailo_driver(self) -> HailoDriver:
        """Get Hailo driver for direct access if needed."""
        return self._hailo_driver

    def run(self) -> Generator[InferenceResult, None, None]:
        """Generator that yields continuous inference results."""
        self.start()
        try:
            while self._inference_streamer.running:
                yield self._inference_streamer.get()
        finally:
            self.stop()

    def __enter__(self) -> Self:
        self.start()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: object,
    ) -> None:
        self.stop()
