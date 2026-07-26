"""Vision and YOLO detection module."""

from __future__ import annotations

import contextlib
from abc import ABC, abstractmethod
from enum import Enum
from typing import TYPE_CHECKING, Self

import cv2
import numpy as np
from pydantic import BaseModel
from shared.domain.models import Detection

if TYPE_CHECKING:
    from src.hardware.hailo.base import Driver as HailoDriver


class TrafficSignColor(Enum):
    """Enumerated traffic sign colors for type-safe detection."""

    RED = "red"
    GREEN = "green"
    MAGENTA = "magenta"

    def __str__(self) -> str:
        """Return the string value of the color."""
        return self.value


class BBoxFormat(Enum):
    """Output bounding-box coordinate convention."""

    NORMALIZED = "normalized"
    ABSOLUTE = "absolute"


class SignDetection(BaseModel):
    """A detected traffic sign or parking block."""

    color: TrafficSignColor
    bbox: tuple[float, float, float, float]  # (x1, y1, x2, y2)
    confidence: float
    position_estimate: tuple[float, float] | None = None

    def to_dict(self) -> dict:
        """Convert detection to a dictionary for JSON serialization."""
        return self.model_dump(include={"color", "bbox", "confidence"})

    def to_detection(self) -> Detection:
        """Convert to the shared domain Detection for interop with non-vision modules."""
        x1, y1, x2, y2 = self.bbox
        w = x2 - x1
        h = y2 - y1
        return Detection(
            class_name=str(self.color),
            confidence=self.confidence,
            bbox=self.bbox,
            x=(x1 + x2) / 2,
            y=(y1 + y2) / 2,
            width=w,
            height=h,
            area=w * h,
        )


@dataclass(frozen=True)
class DetectorConfig:
    """Configuration for detector initialization.

    This dataclass injects model path and class-to-color mapping,
    decoupling the model from hardcoded color names.
    """

    model_path: str
    class_to_color: dict[int, TrafficSignColor]
    min_confidence: float = 0.25
    output_format: BBoxFormat = BBoxFormat.NORMALIZED

    def get_color(self, class_id: int) -> TrafficSignColor | None:
        """Get color for a class ID.

        Args:
            class_id: Model output class ID.

        Returns:
            TrafficSignColor if mapping exists, None otherwise.
        """
        return self.class_to_color.get(class_id)


class DetectorBase(ABC):
    """Base class for vision detectors."""

    @abstractmethod
    def detect(self, image: np.ndarray) -> list[SignDetection]:
        """Detect objects in an RGB image."""


class LocalYoloDetector(DetectorBase):
    """Local YOLO detector using ultralytics (for simulation/dev)."""

    def __init__(self, config: DetectorConfig | None = None):
        """Initialize YOLO detector with injected configuration.

        Args:
            config: DetectorConfig with model path and class mappings.
                If None, uses defaults.
        """
        from ultralytics import YOLO  # noqa: PLC0415

        if config is None:
            config = DetectorConfig(
                model_path="yolov8n.pt",
                class_to_color={
                    0: TrafficSignColor.RED,
                    1: TrafficSignColor.GREEN,
                    2: TrafficSignColor.MAGENTA,
                },
            )

        self.config = config
        self.model = YOLO(config.model_path)

    def detect(self, image: np.ndarray) -> list[SignDetection]:
        """Detect objects using Ultralytics YOLO."""
        # Perform inference
        results = self.model.predict(source=image, verbose=False)

        detections: list[SignDetection] = []
        if not results:
            return detections

        for box in results[0].boxes:
            class_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            color = self.config.get_color(class_id)
            if color is not None:
                detections.append(SignDetection(color, (x1, y1, x2, y2), conf))

        return detections


class HailoDetector(DetectorBase):
    """Hailo 8 NPU detector backed by the hardware driver (create_infer_model API).

    Use as a context manager: __enter__ connects and loads the model;
    __exit__ releases the PCIe device handle.
    """

    def __init__(self, driver: HailoDriver, config: DetectorConfig) -> None:
        self._driver = driver
        self._config = config

    def __enter__(self) -> Self:
        self._driver.connect()
        self._driver.load_model(self._config.model_path)
        return self

    def __exit__(self, *args: object) -> None:
        vdevice = getattr(self._driver, "_vdevice", None)
        if vdevice is not None:
            with contextlib.suppress(Exception):
                vdevice.release()

    def detect(self, image: np.ndarray) -> list[SignDetection]:
        """Detect objects using Hailo 8 NPU."""
        shape = self._driver.get_input_shape()  # (H, W, C)
        h, w = shape[0], shape[1]
        img_resized = cv2.resize(image, (w, h))
        output = self._driver.infer(np.expand_dims(img_resized, axis=0))

        detections = []
        for box in output:
            conf = float(box[4])
            if conf < self._config.min_confidence:
                continue
            color = self._config.get_color(int(box[5]))
            if color is None:
                continue

            # Hailo NMS output rows: [y_min, x_min, y_max, x_max, confidence, class_id]
            if self._config.output_format is BBoxFormat.NORMALIZED:
                y1 = box[0] * image.shape[0]
                x1 = box[1] * image.shape[1]
                y2 = box[2] * image.shape[0]
                x2 = box[3] * image.shape[1]
            else:
                scale_y = image.shape[0] / h
                scale_x = image.shape[1] / w
                y1 = box[0] * scale_y
                x1 = box[1] * scale_x
                y2 = box[2] * scale_y
                x2 = box[3] * scale_x

            detections.append(SignDetection(color, (x1, y1, x2, y2), conf))

        return detections
