"""Image processing utilities: loading, letterboxing, annotation, masking."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from src.coco import COCO_CLASSES, COLORS
from src.constants import (
    BOX_LINE_THICKNESS,
    DEFAULT_IMG_SIZE,
    DETECTION_BOX_COLS,
    DETECTION_CLASS_COL,
    DETECTION_OUTPUT_COLS,
    DETECTION_SCORE_COL,
    IMAGE_EXTENSIONS,
    LETTERBOX_PAD_COLOR,
    NORMALIZE_FACTOR,
    TEXT_FONT_SCALE,
    TEXT_THICKNESS,
    TEXT_Y_OFFSET,
    TRANSPOSE_HWC_TO_CHW,
)
from src.enums import Task
from src.errors import HailoError


def letterbox(
    img: np.ndarray,
    new_shape: tuple[int, int] = (DEFAULT_IMG_SIZE, DEFAULT_IMG_SIZE),
    pad_color: tuple[int, int, int] = LETTERBOX_PAD_COLOR,
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
    new_unpad = (round(w * ratio), round(h * ratio))
    dw = (new_shape[1] - new_unpad[0]) / 2
    dh = (new_shape[0] - new_unpad[1]) / 2

    img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(np.floor(dh)), int(np.ceil(dh))
    left, right = int(np.floor(dw)), int(np.ceil(dw))
    img = cv2.copyMakeBorder(
        img,
        top,
        bottom,
        left,
        right,
        cv2.BORDER_CONSTANT,
        value=pad_color,
    )
    return img, ratio, dw, dh


def preprocess(
    img_path: str,
    size: int = DEFAULT_IMG_SIZE,
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
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / NORMALIZE_FACTOR
    img = np.expand_dims(np.transpose(img, TRANSPOSE_HWC_TO_CHW), 0)
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


def _class_label(class_id: int, names: dict[int, str] | None) -> str:
    """Resolve a class index to a display name.

    Prefers the model's own ``names`` mapping (correct for custom-trained
    models), falling back to the COCO class list, then the bare index.
    """
    if names is not None and class_id in names:
        return names[class_id]
    if class_id < len(COCO_CLASSES):
        return COCO_CLASSES[class_id]
    return str(class_id)


def draw_boxes(
    image: np.ndarray,
    boxes: np.ndarray,
    scores: np.ndarray,
    classes: np.ndarray,
    names: dict[int, str] | None = None,
) -> None:
    """Overlay bounding boxes and labels on ``image`` in-place.

    Args:
        image: BGR image to draw on.
        boxes: ``(N, 4)`` xyxy float array.
        scores: ``(N,)`` confidence scores.
        classes: ``(N,)`` class indices.
        names: Optional ``{index: name}`` mapping from the model. When ``None``
            the COCO class list is used. Pass the model's own names to label
            custom-trained classes correctly.
    """
    for box, score, cls in zip(boxes, scores, classes, strict=False):
        x1, y1, x2, y2 = map(int, box)
        class_id = int(cls)
        color = COLORS[class_id % len(COLORS)]
        label = f"{_class_label(class_id, names)} {score:.2f}"
        cv2.rectangle(image, (x1, y1), (x2, y2), color, BOX_LINE_THICKNESS)
        cv2.putText(
            image,
            label,
            (x1, max(y1 - TEXT_Y_OFFSET, 0)),
            cv2.FONT_HERSHEY_SIMPLEX,
            TEXT_FONT_SCALE,
            color,
            TEXT_THICKNESS,
        )


def unletterbox_mask(
    mask: np.ndarray,
    orig_shape: tuple[int, ...],
    _ratio: float,
    dw: float,
    dh: float,
) -> np.ndarray:
    """Crop letterbox padding from a segmentation mask and resize to original dimensions.

    Args:
        mask: ``(H, W)`` binary or greyscale mask in letterboxed space.
        orig_shape: ``(H, W, ...)`` of the original image.
        _ratio: Unused; kept for call-site symmetry with :func:`preprocess`.
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
        cropped = mask
    return cv2.resize(
        cropped,
        (orig_shape[1], orig_shape[0]),
        interpolation=cv2.INTER_NEAREST,
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
    for path in Path(directory).iterdir():
        if path.suffix.lower() in IMAGE_EXTENSIONS:
            yield path.name, str(path)


def apply_boxes(
    raw: np.ndarray | None,
    conf: float,
    ratio: float,
    dw: float,
    dh: float,
    image: np.ndarray,
    names: dict[int, str] | None = None,
) -> None:
    """Filter by confidence, scale, and draw detected boxes onto image in-place.

    Expects ``raw`` in **post-NMS** ``(N, 6)`` layout
    (``xyxy``, score, class). Models exported with ``nms=False`` emit a raw
    ``(channels, anchors)`` detection tensor instead, which this function cannot
    decode.

    Raises:
        HailoError: If ``raw`` is not ``(N, 6)``.
    """
    if raw is None or raw.shape[0] == 0:
        return
    if raw.ndim != 2 or raw.shape[1] != DETECTION_OUTPUT_COLS:
        msg = (
            f"Raw ONNX output has shape {raw.shape}; expected post-NMS (N, {DETECTION_OUTPUT_COLS}). "
            "The registered models export with nms=False, so use "
            "`--backend ultraonnx` to test them."
        )
        raise HailoError(msg)
    xyxy = raw[:, :DETECTION_BOX_COLS]
    scores = raw[:, DETECTION_SCORE_COL]
    classes = raw[:, DETECTION_CLASS_COL]
    keep = scores > conf
    xyxy, scores, classes = xyxy[keep], scores[keep], classes[keep]
    if len(xyxy):
        xyxy = scale_coords(xyxy, ratio, dw, dh, image.shape)
        draw_boxes(image, xyxy, scores, classes, names)
