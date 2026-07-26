"""Hailo 8 NPU driver implementation."""

import contextlib
import logging
import threading
import time
from typing import Any

import numpy as np
from hailo_platform import VDevice
from hailo_platform.pyhailort.pyhailort import ConfiguredInferModel, InferModel

from src.hardware.hailo.base import (
    Config,
    Driver as ABC_Driver,
)
from src.hardware.hailo.inferences import (
    InferenceResult,
)
from src.logger import configure_json_logging

configure_json_logging()


class Driver(ABC_Driver):
    """Driver for Hailo 8 NPU."""

    def __init__(self, config: Config | None = None):
        self.config: Config = config or Config()
        self._vdevice: VDevice | None = None
        self._infer_model: InferModel | None = None
        self._configured_model: ConfiguredInferModel | None = None
        self._bindings = None
        self._lock: threading.Lock = threading.Lock()
        self.logger = logging.getLogger(__name__)

    def connect(self) -> None:
        """Connect to Hailo device."""
        self.logger.info("Connecting to Hailo 8 NPU")

        # Create virtual device with default parameters (can be customized if needed)
        params = VDevice.create_params()
        self._vdevice = VDevice(params)
        self.logger.info(
            "Connected to Hailo 8 NPU",
            extra={"details": {"device_ids": self._vdevice.get_physical_devices_ids()}},
        )

    @property
    def vdevice(self) -> VDevice:
        """Get Hailo virtual device."""
        if self._vdevice is None:
            self.connect()
        return self._vdevice

    def load_model(self, model_path: str | None = None) -> None:
        """Load a .hef model."""
        path = model_path or self.config.model_path
        self.logger.info("Loading HEF model", extra={"details": {"model_path": path}})

        # Load the model onto the virtual device
        self._infer_model = self.vdevice.create_infer_model(path)
        self.logger.info("HEF model loaded")

    @property
    def infer_model(self) -> InferModel:
        """Get infer model instance."""
        if self._infer_model is None:
            self.load_model()
        return self._infer_model

    @property
    def configured_model(self) -> ConfiguredInferModel:
        """Get the configured, activated infer model.

        Activation is explicit and held for the driver's lifetime. HailoRT does
        not activate on ``configure()``, and re-activating per frame would pay
        that cost on every inference.
        """
        if self._configured_model is None:
            configured = self.infer_model.configure()
            configured.activate()
            self._configured_model = configured
        return self._configured_model

    def get_input_shape(self) -> tuple[int, ...]:
        """Get model input shape."""
        return tuple(self.infer_model.input().shape)

    def get_output_shape(self) -> tuple[int, ...]:
        """Get model output shape."""
        return tuple(self.infer_model.output().shape)

    def infer(self, input_data: np.ndarray) -> Any:
        """Run inference and return the raw output buffer.

        For an NMS-by-class model that buffer is a list of per-class arrays of
        ``(n_boxes, 5)``; decode it with
        :func:`~src.hardware.hailo.inferences.iter_nms_by_class` rather than
        indexing it directly, since the box count differs per class.

        Args:
            input_data: HWC frame matching the model's input shape and dtype.

        Returns:
            The raw output buffer as HailoRT hands it back.
        """
        configured = self.configured_model
        bindings = configured.create_bindings()
        bindings.input().set_buffer(np.ascontiguousarray(input_data))
        # The output binding must be given a buffer up front: without one
        # HailoRT refuses the run with "not configured as view".
        bindings.output().set_buffer(np.empty(self.get_output_shape(), dtype=np.float32))
        configured.run_async([bindings]).wait(self.config.inference_timeout_ms)
        return bindings.output().get_buffer()

    def close(self) -> None:
        """Deactivate the model and release the device.

        Activation is held for the driver's lifetime, and an activated model
        keeps a non-daemon HailoRT thread alive: without this the interpreter
        never exits, which looks like a hang rather than a leak. Safe to call
        more than once.
        """
        if self._configured_model is not None:
            with contextlib.suppress(Exception):
                self._configured_model.deactivate()
            self._configured_model = None
        self._infer_model = None
        if self._vdevice is not None:
            with contextlib.suppress(Exception):
                self._vdevice.release()
            self._vdevice = None
        self.logger.info("Released Hailo 8 NPU")

    def infer_with_timing(self, input_data: np.ndarray) -> InferenceResult:
        """Run inference and measure latency."""
        start = time.perf_counter()
        self.infer(input_data)
        latency_ms = (time.perf_counter() - start) * 1000

        self.logger.info("Inference completed", extra={"details": {"latency_ms": latency_ms}})
        return InferenceResult(detections=[], latency_ms=latency_ms)

    def infer_with_image_size(
        self,
        input_data: np.ndarray,
        original_width: int,
        original_height: int,
        image: np.ndarray | None = None,
    ) -> InferenceResult:
        """Run inference with image dimensions for scaling bounding boxes."""
        start = time.perf_counter()
        raw_output = self.infer(input_data)
        latency_ms = (time.perf_counter() - start) * 1000

        return InferenceResult.parse_yolo_nms_output(
            raw_tensor=raw_output,
            img_width=original_width,
            img_height=original_height,
            class_map=self.config.class_map,
            latency_ms=latency_ms,
            conf_threshold=self.config.min_confidence,
            image=image,
        )

    def benchmark_latency(self, num_iterations: int | None = None) -> float:
        """Benchmark inference latency."""
        iterations = num_iterations or self.config.benchmark_iterations
        input_shape = self.get_input_shape()
        dummy_input = np.random.rand(*input_shape).astype(np.float32)

        latencies = []
        for _ in range(iterations):
            start = time.time()
            self.infer(dummy_input)
            latencies.append((time.time() - start) * 1000)

        avg_latency = sum(latencies) / len(latencies)
        self.logger.info(
            "Latency benchmark",
            extra={"details": {"avg_latency_ms": avg_latency, "iterations": iterations}},
        )
        return avg_latency

    def get_temperature(self) -> float | None:
        """Get NPU temperature in Celsius."""
        try:
            devices = self.vdevice.get_physical_devices()
            if devices:
                temp = devices[0].control.get_device_temperature()
                self.logger.info("Temperature read", extra={"details": {"temperature_c": temp}})
                return float(temp)
        except (RuntimeError, OSError, ValueError):
            return None
        else:
            return None

    def get_power_usage(self) -> int | None:
        """Get power usage in mW."""
        try:
            devices = self.vdevice.get_physical_devices()
            if devices:
                power = devices[0].get_power_usage()
                self.logger.info("Power usage read", extra={"details": {"power_mw": power}})
                return int(power)
        except (RuntimeError, OSError, ValueError):
            return None
        else:
            return None
