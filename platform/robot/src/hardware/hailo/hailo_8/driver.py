"""Hailo 8 NPU driver implementation."""

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
        """Get configured infer model (activates hardware)."""
        if self._configured_model is None:
            self._configured_model = self.infer_model.configure()
        return self._configured_model

    def get_input_shape(self) -> tuple[int, ...]:
        """Get model input shape."""
        input_info = self.infer_model.inputs()[0]
        return tuple(input_info.shape)

    def get_output_shape(self) -> tuple[int, ...]:
        """Get model output shape."""
        output_info = self.infer_model.outputs()[0]
        return tuple(output_info.shape)

    def infer(self, input_data: np.ndarray) -> Any:
        """Run inference on input data."""
        with self.configured_model as configured:
            bindings = configured.create_bindings()
            bindings.input().set_buffer(input_data)
            configured.run_async(bindings).wait()
            return bindings.output().get_buffer()

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

        with self.configured_model as configured:
            bindings = configured.create_bindings()
            bindings.input().set_buffer(input_data)
            configured.run_async(bindings).wait()
            raw_output = bindings.output().get_buffer()

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
