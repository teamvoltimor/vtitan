"""
Hardware driver for Hailo 8 NPU.

Used by: Raspberry Pi 5
"""

import logging
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from src.env import EnvVar
from src.logger import configure_json_logging

configure_json_logging()

HAILO_MODEL_PATH = EnvVar[str](key="HAILO_MODEL_PATH", default="/usr/local/hailo/models/yolo11n.hef")
HAILO_BENCHMARK_ITERATIONS = EnvVar[int](key="HAILO_BENCHMARK_ITERATIONS", default=10, cast=int)


@dataclass
class HailoConfig:
    """Hailo NPU configuration."""

    model_path: str = HAILO_MODEL_PATH.value
    benchmark_iterations: int = HAILO_BENCHMARK_ITERATIONS.value


@dataclass
class InferenceResult:
    """YOLO inference result."""

    detections: List[Dict[str, Any]]
    latency_ms: float


class HailoDriver:
    """Driver for Hailo 8 NPU."""

    def __init__(self, config: Optional[HailoConfig] = None):
        self.config = config or HailoConfig()
        self._device = None
        self._network = None
        self.logger = logging.getLogger(__name__)

    def connect(self) -> None:
        """Connect to Hailo device."""
        import hailo

        self.logger.info("Connecting to Hailo 8 NPU")
        self._device = hailo.HailoDevice()
        self.logger.info("Connected to Hailo 8 NPU", extra={"details": {"device": str(self._device)}})

    @property
    def device(self):
        """Get Hailo device."""
        if self._device is None:
            self.connect()
        return self._device

    def load_model(self, model_path: Optional[str] = None) -> None:
        """Load a .hef model."""
        path = model_path or self.config.model_path
        self.logger.info("Loading HEF model", extra={"details": {"model_path": path}})
        self._network = self.device.create_network_group(path)
        self.logger.info("HEF model loaded")

    @property
    def network(self):
        """Get network instance."""
        if self._network is None:
            self.load_model()
        return self._network

    def get_input_shape(self) -> Tuple[int, ...]:
        """Get model input shape."""
        return self.network.get_input_shape()

    def get_output_shape(self) -> Tuple[int, ...]:
        """Get model output shape."""
        return self.network.get_output_shape()

    def infer(self, input_data: np.ndarray) -> Any:
        """Run inference on input data."""
        return self.network.infer(input_data)

    def infer_with_timing(self, input_data: np.ndarray) -> InferenceResult:
        """Run inference and measure latency."""
        start = time.time()
        output = self.infer(input_data)
        latency_ms = (time.time() - start) * 1000

        self.logger.info("Inference completed", extra={"details": {"latency_ms": latency_ms}})
        return InferenceResult(detections=[], latency_ms=latency_ms)

    def benchmark_latency(self, num_iterations: Optional[int] = None) -> float:
        """Benchmark inference latency."""
        import numpy as np

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
            "Latency benchmark", extra={"details": {"avg_latency_ms": avg_latency, "iterations": iterations}}
        )
        return avg_latency

    def get_temperature(self) -> Optional[float]:
        """Get NPU temperature in Celsius."""
        try:
            temp = self.device.get_device_temperature()
            self.logger.info("Temperature read", extra={"details": {"temperature_c": temp}})
            return temp
        except AttributeError:
            return None

    def get_power_usage(self) -> Optional[int]:
        """Get power usage in mW."""
        try:
            power = self.device.get_power_usage()
            self.logger.info("Power usage read", extra={"details": {"power_mw": power}})
            return power
        except AttributeError:
            return None
