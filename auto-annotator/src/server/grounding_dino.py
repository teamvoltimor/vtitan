"""src.server.grounding_dino – Grounding DINO + SAM model loading and inference.

Uses the autodistill ``GroundedSAM`` pipeline which combines Grounding DINO (open-vocabulary
object detection via text prompts) with SAM (instance segmentation masks).

Two classes are provided:

* :class:`_NoopPredictor` – re-exported from :mod:`src.server.yoloe` so the sentinel
  is not duplicated; satisfies ``ctx.predictor is not None`` in the dispatch layer.

* :class:`GroundingDINOTextSegmenter` – wraps ``GroundedSAM`` and exposes the same
  ``segment_by_text`` interface as :class:`src.server.sam3.SAM3TextSegmenter`.
"""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import TYPE_CHECKING

import cv2
import numpy as np

from src.server.constants import CFG_KEY_SUPPORTS_TEXT
from src.utils import get_logger

if TYPE_CHECKING:
    from src.server.context import ServerContext

logger = get_logger(__name__)


class GroundingDINOTextSegmenter:
    """Open-vocabulary text-prompted segmenter using Grounding DINO + SAM (autodistill).

    Class names are used directly as text prompts for Grounding DINO.  The model is
    rebuilt only when the set of class names changes, so repeated calls with the same
    classes reuse the loaded weights.

    autodistill's ``GroundedSAM.predict()`` expects a file path, so the input numpy
    image is written to a temporary JPEG, processed, and the temp file is deleted.

    Args:
        device: Torch device string (currently informational; autodistill manages device).
    """

    def __init__(self, device: str) -> None:
        self.device = device
        self._model: object | None = None
        self._ontology_classes: list[str] = []

    def _build_model(self, class_names: list[str]) -> None:
        from autodistill.detection import CaptionOntology  # type: ignore[import-untyped]
        from autodistill_grounded_sam import GroundedSAM  # type: ignore[import-untyped]

        ontology = CaptionOntology({name: name for name in class_names})
        self._model = GroundedSAM(ontology=ontology)
        self._ontology_classes = list(class_names)
        logger.info(
            "Grounding DINO ontology built",
            extra={"_extra": {"classes": class_names}},
        )

    def segment_by_text(self, image: np.ndarray, class_names: list[str]) -> list[dict]:
        """Run text-prompted detection + segmentation for the given class names.

        Args:
            image:       RGB uint8 numpy array of the image to segment.
            class_names: List of class-name strings used as text prompts.

        Returns:
            List of result dicts, one per class name, each containing:
            ``class_name``, ``masks`` (list of bool H×W arrays), ``scores``
            (list of floats), ``boxes`` (list of ``[x1, y1, x2, y2]`` floats).
            Classes with no detections have empty lists.
        """
        if class_names != self._ontology_classes or self._model is None:
            self._build_model(class_names)

        per_class: dict[str, dict] = {
            name: {"class_name": name, "masks": [], "scores": [], "boxes": []}
            for name in class_names
        }

        tmp_path: str | None = None
        try:
            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
                tmp_path = tmp.name

            bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(tmp_path, bgr)

            detections = self._model.predict(tmp_path)
        except Exception as exc:
            logger.warning(
                "Grounding DINO predict failed",
                extra={"_extra": {"err": str(exc)}},
            )
            return list(per_class.values())
        finally:
            if tmp_path is not None:
                Path(tmp_path).unlink(missing_ok=True)

        if detections is None:
            return list(per_class.values())

        n = len(detections) if hasattr(detections, "__len__") else 0
        masks = getattr(detections, "mask", None)
        confidences = getattr(detections, "confidence", None)
        class_ids = getattr(detections, "class_id", None)
        xyxy = getattr(detections, "xyxy", None)

        for i in range(n):
            cls_idx = int(class_ids[i]) if class_ids is not None else 0
            if cls_idx >= len(class_names):
                continue

            cls_name = class_names[cls_idx]

            if masks is not None and i < len(masks):
                mask_bool = masks[i].astype(bool)
                per_class[cls_name]["masks"].append(mask_bool)
            else:
                continue

            if confidences is not None and i < len(confidences):
                per_class[cls_name]["scores"].append(float(confidences[i]))

            if xyxy is not None and i < len(xyxy):
                per_class[cls_name]["boxes"].append(xyxy[i].tolist())

        return list(per_class.values())


def load_grounding_dino(cfg: dict, ctx: ServerContext) -> None:
    """Load the Grounding DINO + SAM segmenter into *ctx*.

    Sets ``ctx.predictor`` to a :class:`~src.server.yoloe._NoopPredictor` and,
    when ``supports_text`` is enabled, sets ``ctx.text_seg`` to a
    :class:`GroundingDINOTextSegmenter`.

    The underlying model weights are downloaded on first ``segment_by_text`` call
    (lazy init), so ``load_grounding_dino`` returns quickly.

    Args:
        cfg: Model config dict from ``models.toml``; optionally contains ``supports_text``.
        ctx: Mutable server context; ``predictor`` and ``text_seg`` are updated in-place.
    """
    from src.server.yoloe import _NoopPredictor

    ctx.predictor = _NoopPredictor()
    ctx.text_seg = None

    if cfg.get(CFG_KEY_SUPPORTS_TEXT):
        ctx.text_seg = GroundingDINOTextSegmenter(ctx.device)
        logger.info("Grounding DINO text segmenter initialised (weights load on first call)")
