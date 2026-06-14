"""src.server.yoloe – YOLOE model loading and text-prompted inference.

YOLOE is an open-vocabulary detection + segmentation model from Ultralytics.
It accepts a list of class-name strings as text prompts (``set_classes``), then
runs standard detection on an image and returns per-detection segmentation masks.

Two classes are provided:

* :class:`_NoopPredictor` – sentinel that satisfies the ``ctx.predictor is not None``
  guard in the dispatch layer without exposing unsupported point-prompted inference.

* :class:`YOLOETextSegmenter` – wraps the YOLOE model and exposes the same
  ``segment_by_text`` interface as :class:`src.server.sam3.SAM3TextSegmenter`, so
  :func:`src.server.dispatch.handle_predict_text` can call it interchangeably.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

import cv2
import numpy as np

from src.server.constants import CFG_KEY_CHECKPOINT, CFG_KEY_SUPPORTS_TEXT, PROJECT_ROOT
from src.utils import get_logger

if TYPE_CHECKING:
    from src.server.context import ServerContext

logger = get_logger(__name__)


class _NoopPredictor:
    """Sentinel predictor that satisfies the dispatch guard without point-prompted inference.

    YOLOE does not support point-prompted segmentation (SAM-style click interaction).
    This class exists only so that ``ctx.predictor is not None`` passes in dispatch,
    while making unsupported calls fail with a clear error message.
    """

    def set_image(self, image: np.ndarray) -> None:
        """Accept a set_image call without storing anything (auto-detect models set image internally)."""

    def predict(self, **_kwargs: Any) -> None:
        """Raise NotImplementedError — YOLOE only supports text-prompted auto-annotation."""
        msg = "YOLOE does not support point-prompted inference. Use Auto-annotate instead."
        raise NotImplementedError(msg)


try:
    from ultralytics import YOLO as YOLOE  # type: ignore[import-untyped]
except ModuleNotFoundError as exc:
    error_msg = "ultralytics is required to load YOLOETextSegmenter"
    raise RuntimeError(error_msg) from exc


class YOLOETextSegmenter:
    """Open-vocabulary text-prompted segmenter powered by YOLOE.

    Used by the auto-annotate feature to detect and segment all instances of the
    provided class names in one forward pass.

    Args:
        checkpoint: Absolute path to the YOLOE ``.pt`` weights file.
        device:     Torch device string (``"cuda"`` or ``"cpu"``).
    """

    def __init__(self, checkpoint: str, device: str) -> None:
        self.model = YOLOE(checkpoint)
        self.device = device
        if device == "cuda":
            self.model.to(device)

    def segment_by_text(self, image: np.ndarray, class_names: list[str]) -> list[dict]:
        """Run text-prompted detection + segmentation for the given class names.

        YOLOE processes all classes in a single forward pass.  Results are grouped
        by class name and returned in the same format as
        :meth:`src.server.sam3.SAM3TextSegmenter.segment_by_text`.

        Args:
            image:       RGB uint8 numpy array of the image to segment.
            class_names: List of class-name strings used as text prompts.

        Returns:
            List of result dicts, one per class name, each containing:
            ``class_name``, ``masks`` (list of bool H×W arrays), ``scores``
            (list of floats), ``boxes`` (list of ``[x1, y1, x2, y2]`` floats).
            Classes with no detections have empty lists.
        """
        self.model.set_classes(class_names)

        results = self.model.predict(image, verbose=False)

        per_class: dict[str, dict] = {
            name: {"class_name": name, "masks": [], "scores": [], "boxes": []} for name in class_names
        }

        if not results:
            return list(per_class.values())

        result = results[0]
        orig_h, orig_w = result.orig_shape[:2]

        if result.masks is None or result.boxes is None:
            return list(per_class.values())

        for i in range(len(result.boxes)):
            cls_idx = int(result.boxes.cls[i].item())
            if cls_idx >= len(class_names):
                continue

            cls_name = class_names[cls_idx]

            mask_raw = result.masks.data[i].cpu().numpy()
            if mask_raw.shape != (orig_h, orig_w):
                mask_raw = cv2.resize(
                    mask_raw.astype(np.uint8),
                    (orig_w, orig_h),
                    interpolation=cv2.INTER_NEAREST,
                )
            mask_bool = mask_raw.astype(bool)

            score = float(result.boxes.conf[i].item())
            box = result.boxes.xyxy[i].cpu().numpy().tolist()

            per_class[cls_name]["masks"].append(mask_bool)
            per_class[cls_name]["scores"].append(score)
            per_class[cls_name]["boxes"].append(box)

        return list(per_class.values())


def load_yoloe(cfg: dict, ctx: ServerContext) -> None:
    """Load the YOLOE segmenter into *ctx*.

    Sets ``ctx.predictor`` to a :class:`_NoopPredictor` (satisfies the dispatch
    guard) and, when ``supports_text`` is enabled in the config, sets
    ``ctx.text_seg`` to a :class:`YOLOETextSegmenter`.

    Args:
        cfg: Model config dict from ``models.toml``; must contain ``checkpoint``
             and optionally ``supports_text`` (bool).
        ctx: Mutable server context; ``predictor`` and ``text_seg`` are updated in-place.
    """
    ckpt = Path(cfg[CFG_KEY_CHECKPOINT])
    if not ckpt.is_absolute():
        ckpt = PROJECT_ROOT / ckpt

    logger.info("Loading YOLOE from %s", ckpt)
    ctx.predictor = _NoopPredictor()
    ctx.text_seg = None

    if cfg.get(CFG_KEY_SUPPORTS_TEXT):
        ctx.text_seg = YOLOETextSegmenter(str(ckpt), ctx.device)
        logger.info("YOLOE text segmenter ready on %s", ctx.device)
