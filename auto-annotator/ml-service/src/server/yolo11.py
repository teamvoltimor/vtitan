"""src.server.yolo11 – YOLOv11 detection model loading and class-filtered inference.

YOLOv11 is a closed-vocabulary detection model trained on a fixed set of classes.
It accepts class-name filters via segment_by_text and returns bounding-box masks
(rectangular binary masks rasterised from predicted boxes), making it compatible
with the handle_predict_text dispatch path.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from src.server.constants import CFG_KEY_CHECKPOINT, PROJECT_ROOT
from src.utils import get_logger

if TYPE_CHECKING:
    from src.server.context import ServerContext

logger = get_logger(__name__)


class _NoopPredictor:
    """Sentinel that satisfies the dispatch guard without supporting point inference.

    YOLOv11 is a detection model and does not accept SAM-style point prompts.
    This class exists so that ``ctx.predictor is not None`` passes in dispatch
    while making unsupported calls fail with a clear error message.
    """

    def set_image(self, image: np.ndarray) -> None:
        """Accept set_image silently — YOLOv11 sets the image during predict."""

    def predict(self, **_kwargs: Any) -> None:
        """Raise NotImplementedError — YOLOv11 only supports auto-annotate."""
        msg = "YOLOv11 does not support point-prompted inference. Use Auto-annotate instead."
        raise NotImplementedError(msg)


try:
    from ultralytics import YOLO  # type: ignore[import-untyped]
except ModuleNotFoundError as exc:
    error_msg = "ultralytics is required to load Yolo11Detector"
    raise RuntimeError(error_msg) from exc


class Yolo11Detector:
    """Closed-vocabulary detector powered by YOLOv11.

    Runs standard object detection and returns bounding-box masks for detections
    whose class name matches the requested class_names filter. Boxes are rasterised
    into rectangular boolean masks so the result is compatible with the
    segment_by_text interface consumed by handle_predict_text.

    Args:
        checkpoint: Absolute path to the YOLOv11 ``.pt`` weights file.
        device:     Torch device string (``"cuda"`` or ``"cpu"``).
    """

    def __init__(self, checkpoint: str, device: str) -> None:
        self.model = YOLO(checkpoint)
        self.device = device
        if device == "cuda":
            self.model.to(device)
        self._class_names: list[str] = list(self.model.names.values())
        logger.info("YOLOv11 loaded: %d classes: %s", len(self._class_names), self._class_names)

    def segment_by_text(self, image: np.ndarray, class_names: list[str]) -> list[dict]:
        """Run detection and return bounding-box masks for matching classes.

        Each detected box is rasterised into a rectangular boolean mask (H×W).
        Classes not in the model's trained vocabulary return empty results.

        Args:
            image:       RGB uint8 numpy array.
            class_names: Class names to detect; names not in the trained vocabulary
                         return empty result dicts.

        Returns:
            List of result dicts, one per requested class name, each containing:
            ``class_name``, ``masks`` (list of bool H×W arrays), ``scores``
            (list of floats), ``boxes`` (list of ``[x1, y1, x2, y2]`` floats).
        """
        results = self.model.predict(image, verbose=False, device=self.device)

        per_class: dict[str, dict] = {
            name: {"class_name": name, "masks": [], "scores": [], "boxes": []}
            for name in class_names
        }

        if not results:
            return list(per_class.values())

        result = results[0]
        h, w = image.shape[:2]

        if result.boxes is None:
            return list(per_class.values())

        for i in range(len(result.boxes)):
            cls_idx = int(result.boxes.cls[i].item())
            cls_name = self.model.names.get(cls_idx, "")
            if cls_name not in per_class:
                continue

            x1, y1, x2, y2 = result.boxes.xyxy[i].cpu().numpy().tolist()
            score = float(result.boxes.conf[i].item())

            mask = np.zeros((h, w), dtype=bool)
            mask[max(0, int(y1)):min(h, int(y2)), max(0, int(x1)):min(w, int(x2))] = True

            per_class[cls_name]["masks"].append(mask)
            per_class[cls_name]["scores"].append(score)
            per_class[cls_name]["boxes"].append([x1, y1, x2, y2])

        return list(per_class.values())


def load_yolo11(cfg: dict, ctx: ServerContext) -> None:
    """Load a YOLOv11 detector into *ctx*.

    Sets ``ctx.predictor`` to a :class:`_NoopPredictor` (satisfies the dispatch
    guard) and ``ctx.text_seg`` to a :class:`Yolo11Detector` so the auto-annotate
    (``predict_text``) command works.

    Args:
        cfg: Model config dict from ``models.toml``; must contain ``checkpoint``.
        ctx: Mutable server context; ``predictor`` and ``text_seg`` are updated in-place.
    """
    ckpt = Path(cfg[CFG_KEY_CHECKPOINT])
    if not ckpt.is_absolute():
        ckpt = PROJECT_ROOT / ckpt

    logger.info("Loading YOLOv11 from %s", ckpt)
    ctx.predictor = _NoopPredictor()
    ctx.text_seg = Yolo11Detector(str(ckpt), ctx.device)
    logger.info("YOLOv11 detector ready on %s", ctx.device)
