"""Inference testing across all backends (PyTorch, ONNX, Ultralytics+ONNX)."""
from __future__ import annotations

from pathlib import Path
from typing import Protocol

import cv2
import numpy as np

from src.common import (
    Backend,
    HailoError,
    Task,
    _require_dep,
    get_logger,
)
from src.config import TestConfig  # noqa: TC001
from src.image import (
    _apply_boxes,
    draw_boxes,
    infer_task,
    iter_images,
    preprocess,
    unletterbox_mask,
)

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None  # type: ignore[assignment, misc]

try:
    import onnxruntime as ort
except ImportError:
    ort = None  # type: ignore[assignment]

log = get_logger(__name__)


class InferenceHandler(Protocol):
    """Protocol for backend-specific inference handlers via duck typing.

    Implementations should define setup() to initialize the model and
    infer() to run inference on a single image. The common iteration and
    I/O loop is handled by run_inference().
    """

    def setup(self) -> None:
        """Initialize the model or inference session."""
        ...

    def infer(self, img_path: str) -> np.ndarray:
        """Run inference on one image, returning the annotated result image."""
        ...


class _PTHandler:
    """PyTorch backend handler via Ultralytics."""

    def __init__(self, config: TestConfig):
        self.config = config

    def setup(self) -> None:
        _require_dep(YOLO, "ultralytics")
        self.model = YOLO(self.config.model)

    def infer(self, img_path: str) -> np.ndarray:
        results = self.model(img_path)
        orig = results[0].orig_img
        if results[0].boxes is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            scores = results[0].boxes.conf.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy()
            draw_boxes(orig, boxes, scores, classes)
        return orig


class _UltraONNXHandler:
    """ONNX backend handler via Ultralytics."""

    def __init__(self, config: TestConfig):
        self.config = config

    def setup(self) -> None:
        _require_dep(YOLO, "ultralytics")
        self.model = YOLO(self.config.model)

    def infer(self, img_path: str) -> np.ndarray:
        results = self.model(img_path)
        return results[0].plot()


class _ONNXDetectHandler:
    """Raw ONNX backend handler for detection task."""

    def __init__(self, config: TestConfig):
        self.config = config

    def setup(self) -> None:
        _require_dep(ort, "onnxruntime")
        self.session = ort.InferenceSession(self.config.model)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [o.name for o in self.session.get_outputs()]

    def infer(self, img_path: str) -> np.ndarray:
        img_input, ratio, dw, dh, orig = preprocess(img_path)
        outputs = self.session.run(self.output_names, {self.input_name: img_input})
        _apply_boxes(
            outputs[0][0] if outputs else None,
            self.config.conf,
            ratio,
            dw,
            dh,
            orig,
        )
        return orig


class _ONNXSegmentHandler:
    """Raw ONNX backend handler for segmentation task."""

    def __init__(self, config: TestConfig):
        self.config = config

    def setup(self) -> None:
        _require_dep(ort, "onnxruntime")
        self.session = ort.InferenceSession(self.config.model)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [o.name for o in self.session.get_outputs()]

    def infer(self, img_path: str) -> np.ndarray:
        img_input, ratio, dw, dh, orig = preprocess(img_path)
        outputs = self.session.run(self.output_names, {self.input_name: img_input})

        boxes_raw = outputs[0][0] if outputs else None
        mask_raw = outputs[1][0] if len(outputs) > 1 else None

        overlay = orig.copy()

        if mask_raw is not None:
            log.debug(
                "Mask shape=%s  min=%.3f  max=%.3f",
                mask_raw.shape,
                mask_raw.min(),
                mask_raw.max(),
            )
            for i in range(mask_raw.shape[0]):
                ch = (mask_raw[i] > 0.5).astype(np.uint8) * 255
                ch = unletterbox_mask(ch, orig.shape, ratio, dw, dh)
                cv2.imwrite(
                    str(Path(self.config.output) / f"mask_ch{i}_{Path(img_path).name}"),
                    ch,
                )
            argmax = np.argmax(mask_raw, axis=0).astype(np.uint8) * 255
            argmax = unletterbox_mask(argmax, orig.shape, ratio, dw, dh)
            cv2.imwrite(
                str(Path(self.config.output) / f"mask_argmax_{Path(img_path).name}"),
                argmax,
            )
            mask_img = (mask_raw[0] > 0.5).astype(np.uint8) * 255
            mask_img = unletterbox_mask(mask_img, orig.shape, ratio, dw, dh)
            color_mask = np.zeros_like(orig)
            color_mask[:, :, 1] = mask_img
            overlay = cv2.addWeighted(orig, 0.7, color_mask, 0.3, 0)

        _apply_boxes(boxes_raw, self.config.conf, ratio, dw, dh, overlay)
        return overlay


def _run_inference(config: TestConfig, handler: InferenceHandler) -> None:
    """Generic inference loop: setup, iterate, infer, save, log.

    Args:
        config: Test parameters (input dir, output dir, confidence threshold).
        handler: Backend-specific inference handler (initialized with config).
    """
    handler.setup()
    Path(config.output).mkdir(parents=True, exist_ok=True)

    count = 0
    for fname, img_path in iter_images(config.input):
        annotated = handler.infer(img_path)
        cv2.imwrite(str(Path(config.output) / fname), annotated)
        count += 1

    log.info("Saved %d annotated images → %s", count, config.output)


def run(config: TestConfig) -> None:
    """Dispatch an inference test to the appropriate backend handler.

    Args:
        config: Test parameters.

    Raises:
        HailoError: For unsupported backend/task combinations or import failures.
    """
    task = config.task or infer_task(config.model)

    handler_class = _DISPATCH.get((config.backend, task))
    if handler_class is None:
        msg = f"No handler for backend={config.backend!r}, task={task!r}."
        raise HailoError(msg)

    handler = handler_class(config)
    _run_inference(config, handler)


# Dispatch table maps (backend, task) → handler class
_DISPATCH: dict[tuple[Backend, Task], type[InferenceHandler]] = {
    (Backend.PT, Task.DETECT): _PTHandler,
    (Backend.PT, Task.SEGMENT): _PTHandler,
    (Backend.ULTRAONNX, Task.DETECT): _UltraONNXHandler,
    (Backend.ULTRAONNX, Task.SEGMENT): _UltraONNXHandler,
    (Backend.ONNX, Task.DETECT): _ONNXDetectHandler,
    (Backend.ONNX, Task.SEGMENT): _ONNXSegmentHandler,
}
