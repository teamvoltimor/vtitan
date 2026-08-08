"""Vision and YOLO detection module."""

from __future__ import annotations

import contextlib
from abc import ABC, abstractmethod
from enum import Enum
from typing import TYPE_CHECKING, Self

import cv2
from pydantic import BaseModel
from pydantic_settings import SettingsConfigDict
from shared.domain.enums import GMR_CLASS_NAMES
from shared.domain.models import Detection

from src.hardware.hailo.inferences import iter_nms_by_class
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings

if TYPE_CHECKING:
    import numpy as np

    from src.hardware.hailo.base import Driver as HailoDriver


class TrafficSignColor(Enum):
    """Enumerated traffic sign colors for type-safe detection."""

    RED = "red"
    GREEN = "green"
    MAGENTA = "magenta"

    def __str__(self) -> str:
        """Return the string value of the color."""
        return self.value


# Derived from the one declaration of the detector's class order, rather than
# restated here -- this map and the driver's id-to-name map drifted apart from
# it once already, and a mismatch swaps red for green silently.
DEFAULT_CLASS_TO_COLOR: dict[int, TrafficSignColor] = {
    class_id: TrafficSignColor(name) for class_id, name in GMR_CLASS_NAMES.items()
}

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
        """Convert detection to a dictionary for JSON serialization.

        ``mode="json"`` is what makes this JSON-serializable: a plain
        ``model_dump()`` leaves ``color`` as a ``TrafficSignColor`` member, and
        the vision node's ``json.dumps`` then raises on every frame.
        """
        return self.model_dump(mode="json", include={"color", "bbox", "confidence"})

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


class DetectorConfig(HardwareBaseSettings):
    """Configuration for detector initialization.

    Injects model path and class-to-color mapping, decoupling the model from
    hardcoded color names. ``class_to_color`` is always passed explicitly by
    callers (it's derived from the model's class order). ``model_path`` is
    normally passed explicitly too (real construction always names a
    specific, backend-derived model), but its default is sourced from
    config/hardware/vision/detector.toml rather than a bare literal, for the
    test/debug callers that build a detector with no config at all (see
    ``LocalYoloDetector.__init__`` and ``create_detector``) -- previously a
    module constant (``DEFAULT_YOLO_MODEL_PATH = "yolov8n.pt"``) whose own
    docstring already flagged it as a placeholder pending config sourcing.
    """

    model_config = SettingsConfigDict(env_prefix="detector_", toml_file=CONFIG_DIR / "vision" / "detector.toml")

    model_path: str = "yolov8n.pt"
    class_to_color: dict[int, TrafficSignColor]
    min_confidence: float = 0.45
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
            config = DetectorConfig(class_to_color=DEFAULT_CLASS_TO_COLOR)

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
                detections.append(SignDetection(color=color, bbox=(x1, y1, x2, y2), confidence=conf))

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
        # Deactivates the model as well as releasing the device; releasing the
        # VDevice alone leaves an activated model holding a live HailoRT thread.
        with contextlib.suppress(Exception):
            self._driver.close()

    def detect(self, image: np.ndarray) -> list[SignDetection]:
        """Detect objects using Hailo 8 NPU."""
        shape = self._driver.get_input_shape()  # (H, W, C)
        h, w = shape[0], shape[1]
        # The HEF's input is UINT8 and the graph carries its own normalization,
        # so the resized frame is fed through unscaled.
        img_resized = cv2.resize(image, (w, h))
        # HailoRT takes the frame as HWC; a batch axis is rejected.
        output = self._driver.infer(img_resized)

        detections = []
        for class_id, conf, (ymin, xmin, ymax, xmax) in iter_nms_by_class(output):
            if conf < self._config.min_confidence:
                continue
            color = self._config.get_color(class_id)
            if color is None:
                continue

            if self._config.output_format is BBoxFormat.NORMALIZED:
                y1, x1 = ymin * image.shape[0], xmin * image.shape[1]
                y2, x2 = ymax * image.shape[0], xmax * image.shape[1]
            else:
                scale_y, scale_x = image.shape[0] / h, image.shape[1] / w
                y1, x1 = ymin * scale_y, xmin * scale_x
                y2, x2 = ymax * scale_y, xmax * scale_x

            detections.append(SignDetection(color=color, bbox=(x1, y1, x2, y2), confidence=conf))

        return detections
