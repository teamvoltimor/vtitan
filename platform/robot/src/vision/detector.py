"""Vision and YOLO detection module."""

from __future__ import annotations

import contextlib
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Self

import cv2
import numpy as np
from pydantic_settings import SettingsConfigDict
from shared.domain.enums import GMR_CLASS_NAMES
from shared.domain.models import Detection, SignColor

from src.hardware.hailo.inferences import iter_nms_by_class
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings

if TYPE_CHECKING:
    from src.hardware.hailo.base import Driver as HailoDriver

# Grey fill for letterbox padding -- matches hailo/src/constants.py's
# LETTERBOX_PAD_COLOR and hailo/eval/metrics.py's LETTERBOX_PAD_COLOR, which
# is what the HEF was calibrated, quantized and its mAP measured against.
_LETTERBOX_PAD_VALUE = 114


# Derived from the one declaration of the detector's class order, rather than
# restated here -- this map and the driver's id-to-name map drifted apart from
# it once already, and a mismatch swaps red for green silently.
DEFAULT_CLASS_TO_COLOR: dict[int, SignColor] = {
    class_id: SignColor(name) for class_id, name in GMR_CLASS_NAMES.items()
}


class BBoxFormat(Enum):
    """Output bounding-box coordinate convention."""

    NORMALIZED = "normalized"
    ABSOLUTE = "absolute"


@dataclass(frozen=True, slots=True)
class LetterboxTransform:
    """A letterboxed frame plus what's needed to map its detections back to the source frame."""

    image: np.ndarray
    scale: float
    pad_x: float
    pad_y: float


def letterbox(image: np.ndarray, target_h: int, target_w: int) -> LetterboxTransform:
    """Resize onto a ``target_h`` x ``target_w`` grey canvas without distorting aspect ratio.

    A plain ``cv2.resize`` to a square input stretches a 16:9 camera frame
    non-uniformly, distorting every sign's proportions relative to what the
    HEF was trained, quantized and mAP-measured against -- both
    ``hailo/src/image.py``'s ``letterbox()`` (used to build the calibration/
    eval sets) and Ultralytics' own preprocessing (used by
    :class:`LocalYoloDetector`) letterbox instead. This mirrors that
    transform for the Hailo runtime path.

    Args:
        image: Source frame, any resolution.
        target_h: Model input height.
        target_w: Model input width.

    Returns:
        The padded canvas plus the scale and pixel padding needed to map a
        detection in canvas space back to *image*'s space.
    """
    h, w = image.shape[:2]
    scale = min(target_h / h, target_w / w)
    new_h, new_w = round(h * scale), round(w * scale)
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    pad_x, pad_y = (target_w - new_w) / 2, (target_h - new_h) / 2
    top, left = int(pad_y), int(pad_x)  # pad_x/pad_y are >= 0, so truncation is floor
    canvas = np.full((target_h, target_w, image.shape[2]), _LETTERBOX_PAD_VALUE, dtype=image.dtype)
    canvas[top : top + new_h, left : left + new_w] = resized
    return LetterboxTransform(image=canvas, scale=scale, pad_x=pad_x, pad_y=pad_y)


def _detection_from_bbox(color: SignColor, bbox: tuple[float, float, float, float], confidence: float) -> Detection:
    """Build a Detection from a raw (x1, y1, x2, y2) bbox and its detected color.

    Shared by both detector backends so the bbox-center/width/height/area
    derivation exists in exactly one place.
    """
    x1, y1, x2, y2 = bbox
    w = x2 - x1
    h = y2 - y1
    return Detection(
        class_name=color,
        confidence=confidence,
        bbox=bbox,
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
    class_to_color: dict[int, SignColor]
    min_confidence: float = 0.45
    output_format: BBoxFormat = BBoxFormat.NORMALIZED

    def get_color(self, class_id: int) -> SignColor | None:
        """Get color for a class ID.

        Args:
            class_id: Model output class ID.

        Returns:
            SignColor if mapping exists, None otherwise.
        """
        return self.class_to_color.get(class_id)


class DetectorBase(ABC):
    """Base class for vision detectors."""

    @abstractmethod
    def detect(self, image: np.ndarray) -> list[Detection]:
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

    def detect(self, image: np.ndarray) -> list[Detection]:
        """Detect objects using Ultralytics YOLO."""
        # Perform inference
        results = self.model.predict(source=image, verbose=False)

        detections: list[Detection] = []
        if not results:
            return detections

        for box in results[0].boxes:
            class_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            color = self.config.get_color(class_id)
            if color is not None:
                detections.append(_detection_from_bbox(color, (x1, y1, x2, y2), conf))

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

    def detect(self, image: np.ndarray) -> list[Detection]:
        """Detect objects using Hailo 8 NPU."""
        shape = self._driver.get_input_shape()  # (H, W, C)
        target_h, target_w = shape[0], shape[1]
        # Letterbox rather than stretch: the HEF was calibrated, quantized and
        # its mAP measured against aspect-ratio-preserving, padded input (see
        # letterbox()'s docstring), not a distorted square. The HEF's input is
        # UINT8 and the graph carries its own normalization, so the canvas is
        # fed through unscaled.
        transform = letterbox(image, target_h, target_w)
        # HailoRT takes the frame as HWC; a batch axis is rejected.
        output = self._driver.infer(transform.image)

        detections = []
        for class_id, conf, (ymin, xmin, ymax, xmax) in iter_nms_by_class(output):
            if conf < self._config.min_confidence:
                continue
            color = self._config.get_color(class_id)
            if color is None:
                continue

            if self._config.output_format is BBoxFormat.NORMALIZED:
                y1_px, x1_px = ymin * target_h, xmin * target_w
                y2_px, x2_px = ymax * target_h, xmax * target_w
            else:
                y1_px, x1_px = ymin, xmin
                y2_px, x2_px = ymax, xmax

            x1 = (x1_px - transform.pad_x) / transform.scale
            y1 = (y1_px - transform.pad_y) / transform.scale
            x2 = (x2_px - transform.pad_x) / transform.scale
            y2 = (y2_px - transform.pad_y) / transform.scale
            x1, x2 = np.clip((x1, x2), 0, image.shape[1])
            y1, y2 = np.clip((y1, y2), 0, image.shape[0])

            detections.append(_detection_from_bbox(color, (float(x1), float(y1), float(x2), float(y2)), conf))

        return detections
