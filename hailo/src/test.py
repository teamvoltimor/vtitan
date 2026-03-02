"""Inference testing across all backends (PyTorch, ONNX, Ultralytics+ONNX)."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import cv2
import numpy as np

from src.common import (
    Backend,
    HailoError,
    Task,
    draw_boxes,
    get_logger,
    infer_task,
    iter_images,
    preprocess,
    scale_coords,
    unletterbox_mask,
)

if TYPE_CHECKING:
    from collections.abc import Callable

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None  # type: ignore[assignment, misc]

try:
    import onnxruntime as ort
except ImportError:
    ort = None  # type: ignore[assignment]

log = get_logger(__name__)


@dataclass(slots=True, frozen=True)
class TestConfig:
    """Parameters for an inference test run.

    Args:
        model: Path to the ``.pt`` or ``.onnx`` model file.
        backend: Which inference engine to use.
        task: Detection or segmentation. Inferred from filename when ``None``.
        input: Directory of test images.
        output: Directory where annotated images are written.
        conf: Confidence threshold; detections below this are discarded.
    """

    model: str
    backend: Backend
    task: Task | None
    input: str
    output: str
    conf: float


def run(config: TestConfig) -> None:
    """Dispatch an inference test to the appropriate backend handler.

    Args:
        config: Test parameters.

    Raises:
        HailoError: For unsupported backend/task combinations or import failures.
    """
    Path(config.output).mkdir(parents=True, exist_ok=True)
    task = config.task or infer_task(config.model)

    handler = _DISPATCH.get((config.backend, task))
    if handler is None:
        msg = f"No handler for backend={config.backend!r}, task={task!r}."
        raise HailoError(msg)
    handler(config, task)


# Private backend handlers

def _run_pt(config: TestConfig, _task: Task) -> None:
    """Run inference with the Ultralytics PyTorch backend."""
    if YOLO is None:
        msg = "ultralytics is not installed."
        raise HailoError(msg)

    model = YOLO(config.model)
    count = 0
    for fname, img_path in iter_images(config.input):
        results = model(img_path)
        orig = cv2.imread(img_path)
        if results[0].boxes is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            scores = results[0].boxes.conf.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy()
            draw_boxes(orig, boxes, scores, classes)
        cv2.imwrite(str(Path(config.output) / fname), orig)
        count += 1
    log.info("Saved %d annotated images → %s", count, config.output)


def _run_ultraonnx(config: TestConfig, _task: Task) -> None:
    """Run inference with the Ultralytics API over an ONNX model."""
    if YOLO is None:
        msg = "ultralytics is not installed."
        raise HailoError(msg)

    model = YOLO(config.model)
    count = 0
    for fname, img_path in iter_images(config.input):
        results = model(img_path)
        # Ultralytics handles mask + box rendering for both tasks
        results[0].save(filename=str(Path(config.output) / fname))
        count += 1
    log.info("Saved %d annotated images → %s", count, config.output)


def _run_onnx_detect(config: TestConfig, _task: Task) -> None:
    """Run detection inference via raw ONNXRuntime."""
    if ort is None:
        msg = "onnxruntime is not installed."
        raise HailoError(msg)

    session = ort.InferenceSession(config.model)
    input_name = session.get_inputs()[0].name
    output_names = [o.name for o in session.get_outputs()]

    count = 0
    for fname, img_path in iter_images(config.input):
        img_input, ratio, dw, dh, orig = preprocess(img_path)
        outputs = session.run(output_names, {input_name: img_input})

        raw = outputs[0][0] if outputs else None
        if raw is not None and raw.shape[0] > 0:
            xyxy, scores, classes = raw[:, :4], raw[:, 4], raw[:, 5]
            keep = scores > config.conf
            xyxy, scores, classes = xyxy[keep], scores[keep], classes[keep]
            if len(xyxy):
                xyxy = scale_coords(xyxy, ratio, dw, dh, orig.shape)
                draw_boxes(orig, xyxy, scores, classes)

        cv2.imwrite(str(Path(config.output) / fname), orig)
        count += 1
    log.info("Saved %d annotated images → %s", count, config.output)


def _run_onnx_segment(config: TestConfig, _task: Task) -> None:
    """Run segmentation inference via raw ONNXRuntime.

    Saves per-channel masks, an argmax composite, and a blended overlay
    for each input image.
    """
    if ort is None:
        msg = "onnxruntime is not installed."
        raise HailoError(msg)

    session = ort.InferenceSession(config.model)
    input_name = session.get_inputs()[0].name
    output_names = [o.name for o in session.get_outputs()]

    count = 0
    for fname, img_path in iter_images(config.input):
        img_input, ratio, dw, dh, orig = preprocess(img_path)
        outputs = session.run(output_names, {input_name: img_input})

        boxes_raw = outputs[0][0] if len(outputs) > 0 else None
        mask_raw = outputs[1][0] if len(outputs) > 1 else None

        overlay = orig.copy()

        if mask_raw is not None:
            log.debug(
                "Mask shape=%s  min=%.3f  max=%.3f",
                mask_raw.shape, mask_raw.min(), mask_raw.max(),
            )
            # Save individual channels for per-class debugging
            for i in range(mask_raw.shape[0]):
                ch = (mask_raw[i] > 0.5).astype(np.uint8) * 255
                ch = unletterbox_mask(ch, orig.shape, ratio, dw, dh)
                cv2.imwrite(
                    str(Path(config.output) / f"mask_ch{i}_{fname}"), ch,
                )
            # Argmax composite shows the dominant class per pixel
            argmax = np.argmax(mask_raw, axis=0).astype(np.uint8) * 255
            argmax = unletterbox_mask(argmax, orig.shape, ratio, dw, dh)
            cv2.imwrite(
                str(Path(config.output) / f"mask_argmax_{fname}"), argmax,
            )
            # Green overlay on the primary mask channel for the main output
            mask_img = (mask_raw[0] > 0.5).astype(np.uint8) * 255
            mask_img = unletterbox_mask(mask_img, orig.shape, ratio, dw, dh)
            color_mask = np.zeros_like(orig)
            color_mask[:, :, 1] = mask_img
            overlay = cv2.addWeighted(orig, 0.7, color_mask, 0.3, 0)

        if boxes_raw is not None and boxes_raw.shape[0] > 0:
            xyxy, scores, classes = boxes_raw[:, :4], boxes_raw[:, 4], boxes_raw[:, 5]
            keep = scores > config.conf
            xyxy, scores, classes = xyxy[keep], scores[keep], classes[keep]
            if len(xyxy):
                xyxy = scale_coords(xyxy, ratio, dw, dh, orig.shape)
                draw_boxes(overlay, xyxy, scores, classes)

        cv2.imwrite(str(Path(config.output) / fname), overlay)
        count += 1
    log.info("Saved %d annotated images → %s", count, config.output)


# Dispatch table maps (backend, task) → handler
_DISPATCH: dict[tuple[Backend, Task], Callable[[TestConfig, Task], None]] = {
    (Backend.PT,        Task.DETECT):  _run_pt,
    (Backend.PT,        Task.SEGMENT): _run_pt,
    (Backend.ULTRAONNX, Task.DETECT):  _run_ultraonnx,
    (Backend.ULTRAONNX, Task.SEGMENT): _run_ultraonnx,
    (Backend.ONNX,      Task.DETECT):  _run_onnx_detect,
    (Backend.ONNX,      Task.SEGMENT): _run_onnx_segment,
}
