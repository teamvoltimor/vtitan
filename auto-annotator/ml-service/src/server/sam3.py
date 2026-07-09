"""src.server.sam3 – SAM 3 model loading and text/point-prompted inference helpers.

Provides two classes:

* :class:`SAM3Predictor` – wraps a ``Sam3TrackerModel`` (or ``AutoModel`` fallback) to
  present the same ``set_image`` / ``predict`` interface as SAM 1 and SAM 2 predictors,
  so the dispatch layer can call them interchangeably.

* :class:`SAM3TextSegmenter` – uses the text-conditioned SAM 3 model variant to run
  open-vocabulary segmentation for a list of class-name strings (used by the
  auto-annotate feature).

Module-level imports are kept to a minimum; heavy ML frameworks (transformers, torch)
are imported inside try/except fallback blocks where the specific class names differ
between model versions, or at the top of this module since it is only ever imported
by the server process.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
import torch
from PIL import Image as _PIL

from src.server.constants import CFG_KEY_HF_REPO, CFG_KEY_SUPPORTS_TEXT

if TYPE_CHECKING:
    from src.server.context import ServerContext


class SAM3Predictor:
    """SAM 3 point-prompted predictor with a SAM2-compatible interface.

    Wraps ``Sam3TrackerModel`` (or an ``AutoModel`` fallback) so that the
    dispatch layer can call ``set_image()`` and ``predict()`` without knowing
    which SAM generation is active.

    Args:
        model:     Loaded transformer model (``Sam3TrackerModel`` or ``AutoModel``).
        processor: Matching image/prompt processor.
    """

    def __init__(self, model: Any, processor: Any) -> None:
        self.model = model
        self.processor = processor
        self._pil: _PIL.Image | None = None
        self._orig_hw: tuple[int, int] | None = None

    def set_image(self, image: np.ndarray) -> None:
        """Store the image for use in subsequent :meth:`predict` calls.

        Args:
            image: RGB uint8 numpy array of shape ``(H, W, 3)``.
        """
        self._pil = _PIL.fromarray(image)
        self._orig_hw = image.shape[:2]

    def predict(
        self,
        point_coords: np.ndarray,
        point_labels: np.ndarray,
        mask_input: np.ndarray | None = None,
        _multimask_output: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, Any]:
        """Run point-prompted mask prediction on the stored image.

        Args:
            point_coords:     Float array of shape ``(N, 2)`` with (x, y) pixel coords.
            point_labels:     Int array of shape ``(N,)`` with 1=positive, 0=negative.
            mask_input:       Optional logit mask from a previous call for iterative refinement.
            multimask_output: Ignored (kept for API compatibility with SAM 1/2).

        Returns:
            Three-tuple ``(masks_np, scores_np, logits)`` where:
            * ``masks_np`` is a boolean array of shape ``(K, H, W)``.
            * ``scores_np`` is a float32 array of shape ``(K,)`` with confidence scores.
            * ``logits`` is the raw pred_masks tensor (for iterative refinement), or ``None``.

        Raises:
            RuntimeError: When :meth:`set_image` has not been called first.
        """
        if self._pil is None:
            msg = "set_image() must be called before predict()"
            raise RuntimeError(msg)

        # Reformat coords and labels into the nested list shape the SAM3 processor expects.
        input_points = [[[[float(c[0]), float(c[1])] for c in point_coords]]]
        input_labels = [[[int(lb) for lb in point_labels]]]

        proc_kwargs: dict[str, Any] = dict(
            images=self._pil,
            input_points=input_points,
            input_labels=input_labels,
            return_tensors="pt",
        )
        if mask_input is not None:
            # Silently skip if this processor version does not support input_masks.
            import contextlib as _ctx

            with _ctx.suppress(Exception):
                proc_kwargs["input_masks"] = [[mask_input]]

        inputs = self.processor(**proc_kwargs).to(self.model.device)

        # Run forward pass.
        with torch.inference_mode():
            outputs = self.model(**inputs)

        # Post-process to pixel-space masks; fall back to bilinear interpolation
        # on older transformers versions that lack the helper method.
        try:
            masks_t = self.processor.post_process_masks(
                outputs.pred_masks,
                inputs["original_sizes"],
                inputs["reshaped_input_sizes"],
            )[0]
        except Exception:
            import torch.nn.functional as _F

            h, w = self._orig_hw
            raw = outputs.pred_masks[0, :, 0:1].float()
            masks_t = _F.interpolate(raw, (h, w), mode="bilinear")[:, 0] > 0

        if masks_t.ndim == 4:
            masks_t = masks_t[:, 0]

        masks_np = masks_t.cpu().bool().numpy()

        # Extract confidence scores; fall back to all-ones when the attribute is absent.
        if hasattr(outputs, "iou_scores"):
            scores_np = outputs.iou_scores[0].flatten().cpu().float().numpy()
        elif hasattr(outputs, "pred_scores"):
            scores_np = outputs.pred_scores[0].flatten().cpu().float().numpy()
        else:
            scores_np = np.ones(len(masks_np), dtype=np.float32)

        # Return raw pred_masks for iterative refinement on subsequent calls.
        logits = outputs.pred_masks[0].cpu().numpy() if hasattr(outputs, "pred_masks") else None
        return masks_np, scores_np, logits


class SAM3TextSegmenter:
    """Open-vocabulary text-prompted segmenter powered by SAM 3.

    Used by the auto-annotate feature to generate masks for a list of class
    names without any point prompts from the user.

    Args:
        hf_repo: HuggingFace repository ID for the SAM 3 model weights.
        device:  Torch device string (``"cuda"`` or ``"cpu"``).
    """

    def __init__(self, hf_repo: str, device: str) -> None:
        try:
            from transformers import Sam3Model, Sam3Processor  # type: ignore[import-untyped]

            self.processor = Sam3Processor.from_pretrained(hf_repo)
            self.model = Sam3Model.from_pretrained(hf_repo).to(device)
        except (ImportError, AttributeError):
            from transformers import AutoModel, AutoProcessor  # type: ignore[import-untyped]

            self.processor = AutoProcessor.from_pretrained(hf_repo)
            self.model = AutoModel.from_pretrained(hf_repo).to(device)
        self.device = device

    def segment_by_text(self, image: np.ndarray, class_names: list[str]) -> list[dict]:
        """Run text-prompted segmentation for each class name in *class_names*.

        Each class is segmented independently; failures are captured per-class and
        included in the result list with an ``"error"`` key so partial results are
        not lost.

        Args:
            image:       RGB uint8 numpy array of the image to segment.
            class_names: List of class-name strings used as text prompts.

        Returns:
            List of result dicts, one per class name, each containing:
            ``class_name``, ``masks`` (list of bool arrays), ``scores`` (list of
            floats), ``boxes`` (list of ``[x1, y1, x2, y2]`` floats), and
            optionally ``error`` (str) on failure.
        """
        pil = _PIL.fromarray(image)
        h, w = image.shape[:2]
        results = []

        # Segment each class independently; capture per-class failures to preserve partial results.
        for class_name in class_names:
            try:
                # Encode image and text prompt into model inputs.
                inputs = self.processor(
                    images=pil,
                    text=[class_name],
                    return_tensors="pt",
                ).to(self.device)

                # Run forward pass.
                with torch.inference_mode():
                    outputs = self.model(**inputs)

                # Post-process to pixel-space masks; fall back to bilinear interpolation
                # on older transformers versions that lack the helper method.
                try:
                    masks_t = self.processor.post_process_masks(
                        outputs.pred_masks,
                        inputs["original_sizes"],
                        inputs["reshaped_input_sizes"],
                    )[0]
                except Exception:
                    import torch.nn.functional as _F

                    raw = outputs.pred_masks[0, :, 0:1].float()
                    masks_t = _F.interpolate(raw, (h, w), mode="bilinear")[:, 0] > 0

                if masks_t.ndim == 4:
                    masks_t = masks_t[:, 0]
                masks_np = masks_t.cpu().bool().numpy()

                # Extract confidence scores; default to 1.0 when iou_scores is unavailable.
                scores = (
                    outputs.iou_scores[0].flatten().cpu().float().tolist()
                    if hasattr(outputs, "iou_scores")
                    else [1.0] * len(masks_np)
                )

                # Convert centre-format pred_boxes to [x1, y1, x2, y2] in pixels.
                boxes: list[list[float]] = []
                if hasattr(outputs, "pred_boxes"):
                    for box in outputs.pred_boxes[0].cpu().float().numpy():
                        cx, cy, bw, bh = box * np.array([w, h, w, h])
                        boxes.append(
                            [
                                float(cx - bw / 2),
                                float(cy - bh / 2),
                                float(cx + bw / 2),
                                float(cy + bh / 2),
                            ],
                        )

                results.append(
                    {
                        "class_name": class_name,
                        "masks": [masks_np[i] for i in range(len(masks_np))],
                        "scores": scores,
                        "boxes": boxes,
                    },
                )
            except Exception as e:
                # Capture per-class failure so other classes are not discarded.
                results.append(
                    {
                        "class_name": class_name,
                        "masks": [],
                        "scores": [],
                        "boxes": [],
                        "error": str(e),
                    },
                )

        return results


def load_sam3(cfg: dict, ctx: ServerContext) -> None:
    """Load the SAM 3 interactive predictor (and optional text segmenter) into *ctx*.

    Attempts to import ``Sam3TrackerModel`` and ``Sam3TrackerProcessor`` from
    ``transformers`` first; falls back to ``AutoModel`` / ``AutoProcessor`` for
    versions where the named classes are not yet available.

    Args:
        cfg: Model config dict from ``models.toml``; must contain ``hf_repo`` and
             optionally ``supports_text`` (bool).
        ctx: Mutable server context; ``predictor`` and ``text_seg`` are updated in-place.
    """
    hf_repo = cfg[CFG_KEY_HF_REPO]

    try:
        from transformers import Sam3TrackerModel, Sam3TrackerProcessor  # type: ignore[import-untyped]

        tracker = Sam3TrackerModel.from_pretrained(hf_repo).to(ctx.device)
        tracker_proc = Sam3TrackerProcessor.from_pretrained(hf_repo)
    except (ImportError, AttributeError):
        from transformers import AutoModel, AutoProcessor  # type: ignore[import-untyped]

        tracker = AutoModel.from_pretrained(hf_repo).to(ctx.device)
        tracker_proc = AutoProcessor.from_pretrained(hf_repo)

    ctx.predictor = SAM3Predictor(tracker, tracker_proc)
    ctx.text_seg = None

    if cfg.get(CFG_KEY_SUPPORTS_TEXT):
        import contextlib as _ctx

        with _ctx.suppress(Exception):
            ctx.text_seg = SAM3TextSegmenter(hf_repo, ctx.device)
