"""src.inference – SAM inference helpers (model-server or local fallback).

Iterative refinement: if ``pending_logits`` exist from a previous call they are
fed back as ``mask_input``, giving SAM a spatial prior for sharper successive masks.

Connected-component filtering: only blobs that contain at least one positive
click are kept, eliminating stray background regions.

Uses InferenceBackend protocol for pluggable adapters and typed exceptions
instead of bare exception catching.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import cv2
import numpy as np

from src.constants import INFERENCE_DEFAULT_MASK_SCORE, MASK_LABELS
from src.exceptions import (
    InferenceBadInput,
    InferenceBackendError,
    InferenceGPUMemory,
    InferenceModelUnavailable,
)
from src.inference_backends import get_backend, load_native_sam2, load_ultralytics_fallback
from src.models import AppState, InferenceContext, InferenceResult, Point

if TYPE_CHECKING:
    from src.sam_client import ModelServerClient

from src.utils import get_logger

logger = get_logger(__name__)


def initialize_inference(client: ModelServerClient | None, context: InferenceContext) -> None:
    """Load SAM locally into *context* when no model server is reachable.

    Tries the native ``sam2`` package first (preferred), then falls back to the
    ``ultralytics`` SAM wrapper. Does nothing when *client* is not ``None``
    (server mode is active) or when *context* already has a predictor loaded.

    Args:
        client:  Active :class:`~src.sam_client.ModelServerClient`, or ``None``
                 when running without a model server.
        context: :class:`~src.models.InferenceContext` to populate in-place.
    """
    if client is not None:
        return

    try:
        load_native_sam2(context)
    except InferenceBackendError as e:
        logger.info("Native SAM2 unavailable, trying ultralytics", extra={"_extra": {"err": str(e)}})
        try:
            load_ultralytics_fallback(context)
        except InferenceBackendError as fallback_err:
            logger.error("No SAM backend available", extra={"_extra": {"err": str(fallback_err)}})


def _filter_connected_components(mask: np.ndarray, positive_points: list[Point]) -> np.ndarray:
    """Keep only connected components that contain at least one positive click.

    Eliminates stray background blobs that SAM may include in the mask
    when the user has not placed points in those regions.

    Args:
        mask:            Boolean H×W mask array from SAM.
        positive_points: Points with ``label == 1`` (positive clicks).

    Returns:
        Filtered boolean H×W mask containing only components touched by a positive click.
        Returns *mask* unchanged when no components are selected (safety fallback).
    """
    if not mask.any():
        return mask

    _n_labels, label_map = cv2.connectedComponents(mask.astype(np.uint8))
    keep: set[int] = set()

    for point in positive_points:
        if point.label != 1:
            continue
        px = max(0, min(point.x, mask.shape[1] - 1))
        py = max(0, min(point.y, mask.shape[0] - 1))
        component_label = label_map[py, px]
        if component_label != 0:
            keep.add(component_label)

    if not keep:
        return mask

    return np.isin(label_map, list(keep))


def _empty_cache(context: InferenceContext) -> None:
    """Free CUDA cache if torch is loaded."""
    if context.torch_module is not None and context.torch_module.cuda.is_available():
        context.torch_module.cuda.empty_cache()


def run_sam_inference(
    state: AppState,
    client: ModelServerClient | None,
    context: InferenceContext,
) -> InferenceResult:
    """Run SAM on the current point buffer and return a structured result.

    Feeds ``state.pending_logits`` back as ``mask_input`` for iterative
    refinement when available. Applies connected-component filtering to remove
    stray background blobs.

    Args:
        state:   Current :class:`~src.models.AppState` with image and point buffer.
        client:  Active server client, or ``None`` for local inference.
        context: Local inference context (predictor, torch, OOM class).

    Returns:
        :class:`~src.models.InferenceResult` with ``ok == True`` on success.
    """
    if state.current_image is None:
        return InferenceResult(masks=None, best_idx=0, scores_str="", error="No image loaded.")
    if not state.point_buffer:
        return InferenceResult(masks=None, best_idx=0, scores_str="", error="No points in buffer.")
    if client is None and context.predictor is None:
        msg = "SAM model not loaded."
        return InferenceResult(masks=None, best_idx=0, scores_str="", error=msg)

    points = state.point_buffer
    coords = np.array([[p.x, p.y] for p in points], dtype=np.float32)
    labels = np.array([p.label for p in points], dtype=np.int32)

    mask_input: np.ndarray | None = None
    if state.pending_logits is not None:
        raw_logits = state.pending_logits[state.pending_mask_idx]
        mask_input = raw_logits[None] if raw_logits.ndim == 2 else raw_logits

    try:
        if client is not None:
            if not state.image_set:
                client.set_image(state.current_image)
                state.image_set = True
            all_masks, scores_flat, logits_out = client.predict(coords, labels, mask_input=mask_input)
            logits_out = np.array(logits_out) if logits_out else None
        else:
            backend = get_backend(context)
            if not state.image_set:
                backend.set_image(state.current_image)
                state.image_set = True
            all_masks, scores_flat, logits_out = backend.predict(coords, labels, mask_input=mask_input)

    except InferenceGPUMemory:
        _empty_cache(context)
        return InferenceResult(masks=None, best_idx=0, scores_str="", error="CUDA OOM — try a smaller image.")
    except InferenceBackendError as e:
        _empty_cache(context)
        return InferenceResult(masks=None, best_idx=0, scores_str="", error=f"Inference error: {e}")
    except Exception as e:
        _empty_cache(context)
        logger.error("Unexpected inference error", extra={"_extra": {"error": str(e)}})
        return InferenceResult(masks=None, best_idx=0, scores_str="", error=f"Unexpected error: {e}")
    finally:
        if client is None:
            _empty_cache(context)

    state.pending_logits = logits_out

    positive_points = [p for p in points if p.label == 1]
    all_masks = [_filter_connected_components(mask, positive_points) for mask in all_masks]

    best_index = int(np.argmax(scores_flat))
    scores_str = "  ".join(f"{MASK_LABELS[i]}: {scores_flat[i]:.3f}" for i in range(len(all_masks)))
    return InferenceResult(masks=tuple(all_masks), best_idx=best_index, scores_str=scores_str, error="")
