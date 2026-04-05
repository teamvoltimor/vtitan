"""Abstract base classes for Hailo NPU implementations."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class InferenceResult:
    """Inference result."""

    detections: list[dict[str, Any]]
    latency_ms: float


@dataclass
class Config:
    """Hailo configuration."""

    model_path: str
    benchmark_iterations: int


class Driver(ABC):
    """Abstract Hailo NPU driver."""

    @abstractmethod
    def connect(self) -> None:
        """Connect to Hailo device."""

    @abstractmethod
    def load_model(self, model_path: str | None = None) -> None:
        """Load a .hef model."""

    @abstractmethod
    def get_input_shape(self) -> tuple[int, ...]:
        """Get model input shape."""

    @abstractmethod
    def get_output_shape(self) -> tuple[int, ...]:
        """Get model output shape."""

    @abstractmethod
    def infer(self, input_data: np.ndarray) -> Any:
        """Run inference on input data."""

    @abstractmethod
    def infer_with_timing(self, input_data: np.ndarray) -> InferenceResult:
        """Run inference and measure latency."""

    @abstractmethod
    def benchmark_latency(self, num_iterations: int | None = None) -> float:
        """Benchmark inference latency."""
