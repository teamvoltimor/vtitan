"""Shared types, enums, exceptions, model registry, logger, and image utilities."""
from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, NewType

import cv2
import numpy as np

# Domain types
ModelName = NewType("ModelName", str)
FilePath = NewType("FilePath", str)


# Logging

class _JsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, str] = {
            "ts": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "level": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload)


def get_logger(name: str) -> logging.Logger:
    """Return a module-level logger.

    Args:
        name: Typically ``__name__`` from the calling module.

    Returns:
        A standard :class:`logging.Logger` bound to the JSON handler
        configured in :func:`configure_logging`.
    """
    return logging.getLogger(name)


def configure_logging(level: str = "INFO") -> None:
    """Wire the root logger to emit structured JSON lines.

    Args:
        level: One of ``DEBUG``, ``INFO``, ``WARNING``, ``ERROR``.
    """
    handler = logging.StreamHandler()
    handler.setFormatter(_JsonFormatter())
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        handlers=[handler],
        force=True,
    )


# Enums

class Backend(str, Enum):
    """Inference backend for the ``test`` command."""

    PT = "pt"
    ONNX = "onnx"
    ULTRAONNX = "ultraonnx"

    def __str__(self) -> str:
        return self.value


class Task(str, Enum):
    """Inference task type."""

    DETECT = "detect"
    SEGMENT = "segment"

    def __str__(self) -> str:
        return self.value


class HWArch(str, Enum):
    """Hailo target hardware architecture."""

    HAILO8 = "hailo8"
    HAILO8L = "hailo8l"

    def __str__(self) -> str:
        return self.value


# Exceptions

class HailoError(Exception):
    """Base error for the Hailo pipeline."""


class ModelNotFoundError(HailoError):
    """Raised when a requested model file does not exist on disk."""


class CalibrationDataError(HailoError):
    """Raised when calibration data is missing or malformed."""


class CompileError(HailoError):
    """Raised when the Hailo DFC fails to compile a model."""


# Directory constants

DATA_DIR = "data"
SHARED_WITH_DOCKER = "shared_with_docker"
DOCKER_SHARED_MOUNT = "/local/shared_with_docker"

# Docker container identity
DOCKER_CONTAINER = "hailo8_ai_sw_suite_2025-10_container"
DOCKER_IMAGE = "hailo8_ai_sw_suite_2025-10:1"


# Model registry

@dataclass(slots=True, frozen=True)
class ModelEntry:
    """Immutable descriptor for a registered YOLO model variant.

    Args:
        pt_file: Path to the PyTorch checkpoint (under ``data/``).
        onnx_file: Path to the exported ONNX graph (under ``data/``).
        task: Whether this model performs detection or segmentation.
        opset: Default ONNX opset for Hailo compatibility.
        export_extras: Additional keyword arguments forwarded to
            ``YOLO.export()``, stored as an immutable tuple of pairs.
        zoo_name: Hailo Model Zoo identifier used by ``hailomz`` CLI
            (e.g. ``"yolov11s"``). ``None`` if the model is not in the zoo.
    """

    pt_file: str
    onnx_file: str
    task: Task
    opset: int
    export_extras: tuple[tuple[str, Any], ...]
    zoo_name: str | None = None

    def extra_kwargs(self) -> dict[str, Any]:
        """Materialise ``export_extras`` back into a plain dict."""
        return dict(self.export_extras)


MODEL_REGISTRY: dict[str, ModelEntry] = {
    "yolo11n": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo11n.pt",
        onnx_file=f"{DATA_DIR}/yolo11n.onnx",
        task=Task.DETECT,
        opset=13,
        export_extras=(),
        zoo_name="yolov11n",
    ),
    "yolo11s": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo11s.pt",
        onnx_file=f"{DATA_DIR}/yolo11s.onnx",
        task=Task.DETECT,
        opset=13,
        export_extras=(),
        zoo_name="yolov11s",
    ),
    "yolo12n": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo12n.pt",
        onnx_file=f"{DATA_DIR}/yolo12n.onnx",
        task=Task.DETECT,
        opset=11,
        export_extras=(("simplify", True), ("nms", False), ("optimize", False)),
        zoo_name="yolov12n",
    ),
    "yolo26n": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo26n.pt",
        onnx_file=f"{DATA_DIR}/yolo26n.onnx",
        task=Task.DETECT,
        opset=11,
        export_extras=(("simplify", True), ("nms", False), ("optimize", False)),
        zoo_name=None,
    ),
    "yolo26l": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo26l.pt",
        onnx_file=f"{DATA_DIR}/yolo26l.onnx",
        task=Task.DETECT,
        opset=11,
        export_extras=(("simplify", True), ("nms", False)),
        zoo_name=None,
    ),
    "yolo26l-seg": ModelEntry(
        pt_file=f"{DATA_DIR}/yolo26l-seg.pt",
        onnx_file=f"{DATA_DIR}/yolo26l-seg.onnx",
        task=Task.SEGMENT,
        opset=11,
        export_extras=(("simplify", True), ("nms", False)),
        zoo_name=None,
    ),
}


# COCO labels

COCO_CLASSES: list[str] = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table",
    "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
]

# Seeded so colours are stable across runs
_rng = np.random.default_rng(42)
COLORS: list[tuple[int, ...]] = [
    tuple(int(x) for x in _rng.integers(0, 255, 3))
    for _ in range(len(COCO_CLASSES))
]


# Image utilities

def letterbox(
    img: np.ndarray,
    new_shape: tuple[int, int] = (640, 640),
    pad_color: tuple[int, int, int] = (114, 114, 114),
) -> tuple[np.ndarray, float, float, float]:
    """Resize ``img`` with letterboxing to preserve aspect ratio.

    Args:
        img: BGR image array.
        new_shape: Target (height, width).
        pad_color: Padding fill value.

    Returns:
        ``(padded_img, scale_ratio, pad_width, pad_height)``
    """
    h, w = img.shape[:2]
    ratio = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad = (int(round(w * ratio)), int(round(h * ratio)))
    dw = (new_shape[1] - new_unpad[0]) / 2
    dh = (new_shape[0] - new_unpad[1]) / 2

    img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(np.floor(dh)), int(np.ceil(dh))
    left, right = int(np.floor(dw)), int(np.ceil(dw))
    img = cv2.copyMakeBorder(
        img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=pad_color
    )
    # Final resize ensures exact target dimensions despite float rounding
    img = cv2.resize(img, new_shape[::-1], interpolation=cv2.INTER_LINEAR)
    return img, ratio, dw, dh


def preprocess(
    img_path: str, size: int = 640
) -> tuple[np.ndarray, float, float, float, np.ndarray]:
    """Load, letterbox, and normalise an image for ONNX inference.

    Args:
        img_path: Path to the source image.
        size: Square input dimension expected by the model.

    Returns:
        ``(chw_batch, ratio, pad_w, pad_h, original_bgr)``
    """
    img0 = cv2.imread(img_path)
    img, ratio, dw, dh = letterbox(img0, new_shape=(size, size))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    img = np.expand_dims(np.transpose(img, (2, 0, 1)), 0)  # HWC → NCHW
    return img, ratio, dw, dh, img0


def scale_coords(
    boxes: np.ndarray,
    ratio: float,
    dw: float,
    dh: float,
    img_shape: tuple[int, ...],
) -> np.ndarray:
    """Map letterboxed xyxy coordinates back to the original image space.

    Args:
        boxes: Array of shape ``(N, 4)`` in xyxy format.
        ratio: Scale ratio returned by :func:`letterbox`.
        dw: Horizontal padding returned by :func:`letterbox`.
        dh: Vertical padding returned by :func:`letterbox`.
        img_shape: ``(H, W, ...)`` of the original image.

    Returns:
        Clipped xyxy boxes in original image coordinates.
    """
    boxes[:, [0, 2]] -= dw
    boxes[:, [1, 3]] -= dh
    boxes[:, :4] /= ratio
    boxes[:, 0] = np.clip(boxes[:, 0], 0, img_shape[1])
    boxes[:, 1] = np.clip(boxes[:, 1], 0, img_shape[0])
    boxes[:, 2] = np.clip(boxes[:, 2], 0, img_shape[1])
    boxes[:, 3] = np.clip(boxes[:, 3], 0, img_shape[0])
    return boxes


def draw_boxes(
    image: np.ndarray,
    boxes: np.ndarray,
    scores: np.ndarray,
    classes: np.ndarray,
) -> None:
    """Overlay bounding boxes and labels on ``image`` in-place.

    Args:
        image: BGR image to draw on.
        boxes: ``(N, 4)`` xyxy float array.
        scores: ``(N,)`` confidence scores.
        classes: ``(N,)`` class indices.
    """
    for box, score, cls in zip(boxes, scores, classes):
        x1, y1, x2, y2 = map(int, box)
        class_id = int(cls)
        color = COLORS[class_id % len(COLORS)]
        label = (
            f"{COCO_CLASSES[class_id]} {score:.2f}"
            if class_id < len(COCO_CLASSES)
            else f"{class_id} {score:.2f}"
        )
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            image, label, (x1, max(y1 - 10, 0)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
        )


def unletterbox_mask(
    mask: np.ndarray,
    orig_shape: tuple[int, ...],
    ratio: float,
    dw: float,
    dh: float,
) -> np.ndarray:
    """Crop letterbox padding from a segmentation mask and resize to original dimensions.

    Args:
        mask: ``(H, W)`` binary or greyscale mask in letterboxed space.
        orig_shape: ``(H, W, ...)`` of the original image.
        ratio: Scale ratio from :func:`letterbox`.
        dw: Horizontal padding from :func:`letterbox`.
        dh: Vertical padding from :func:`letterbox`.

    Returns:
        Mask resized to ``(orig_H, orig_W)``.
    """
    h, w = mask.shape
    top, bottom = int(np.floor(dh)), int(np.ceil(dh))
    left, right = int(np.floor(dw)), int(np.ceil(dw))
    y1, y2 = max(top, 0), max(h - bottom, top + 1)
    x1, x2 = max(left, 0), max(w - right, left + 1)
    cropped = mask[y1:y2, x1:x2]
    if cropped.size == 0:
        # Padding fully consumed the mask — fall back to the full letterboxed mask
        cropped = mask
    return cv2.resize(
        cropped, (orig_shape[1], orig_shape[0]), interpolation=cv2.INTER_NEAREST
    )


def infer_task(model_path: str) -> Task:
    """Infer detection vs segmentation from the model filename.

    Args:
        model_path: Path or filename of the model file.

    Returns:
        :attr:`Task.SEGMENT` if ``"seg"`` appears in the stem, else
        :attr:`Task.DETECT`.
    """
    return Task.SEGMENT if "seg" in Path(model_path).name else Task.DETECT


def iter_images(directory: str):
    """Yield ``(filename, full_path)`` for every image in *directory*.

    Args:
        directory: Path to search for ``.jpg``, ``.jpeg``, or ``.png`` files.

    Yields:
        ``(fname, abs_path)`` tuples.
    """
    for fname in os.listdir(directory):
        if fname.lower().endswith((".jpg", ".jpeg", ".png")):
            yield fname, os.path.join(directory, fname)
