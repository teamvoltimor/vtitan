"""src.inference – SAM inference helpers (model-server or local fallback).

Iterative refinement: if ``pending_logits`` exist from a previous call they are
fed back as ``mask_input``, giving SAM a spatial prior for sharper successive masks.

Connected-component filtering: only blobs that contain at least one positive
click are kept, eliminating stray background regions.

All previously module-level globals (``_predictor``, ``_use_native``,
``_oom_error``, ``_torch``) are now fields on :class:`~src.models.InferenceContext`
which is passed explicitly through every function.  No global state is used.
"""

from __future__ import annotations

import contextlib
from typing import TYPE_CHECKING

import cv2
import numpy as np

from src.constants import (
    DEVICE_CPU,
    DEVICE_CUDA,
    INFERENCE_DEFAULT_MASK_SCORE,
    MASK_LABELS,
    MODELS_DIR,
    SAM2_DEFAULT_HF_REPO,
    SAM2_DEFAULT_HIERA_CONFIG,
    SAM2_LOCAL_CHECKPOINT_FILENAME,
)
from src.models import AppState, InferenceContext, InferenceResult, Point

if TYPE_CHECKING:
    from src.sam_client import ModelServerClient

from src.utils import get_logger

logger = get_logger(__name__)


def _empty_cache(context: InferenceContext) -> None:
    """Free the CUDA memory cache when a CUDA-capable torch module is loaded.

    Args:
        context: Inference context holding the lazy-loaded ``torch_module``.
    """
    if context.torch_module is not None and context.torch_module.cuda.is_available():
        context.torch_module.cuda.empty_cache()


def _autocast_ctx(context: InferenceContext) -> contextlib.AbstractContextManager:
    """Return a ``torch.autocast`` context for CUDA, or a no-op on CPU.

    Args:
        context: Inference context holding the lazy-loaded ``torch_module``.

    Returns:
        Context manager suitable for use in a ``with`` block.
    """
    if context.torch_module is not None and context.torch_module.cuda.is_available():
        return context.torch_module.autocast(DEVICE_CUDA, dtype=context.torch_module.bfloat16)
    return contextlib.nullcontext()


def initialize_inference(client: ModelServerClient | None, context: InferenceContext) -> None:
    """Load SAM locally into *context* when no model server is reachable.

    Tries the native ``sam2`` package first (preferred), then falls back to the
    ``ultralytics`` SAM wrapper.  Does nothing when *client* is not ``None``
    (server mode is active) or when *context* already has a predictor loaded.

    Args:
        client:  Active :class:`~src.sam_client.ModelServerClient`, or ``None``
                 when running without a model server.
        context: :class:`~src.models.InferenceContext` to populate in-place.
    """
    if client is not None:
        return

    import torch

    context.torch_module = torch
    context.oom_error = torch.cuda.OutOfMemoryError
    device = DEVICE_CUDA if torch.cuda.is_available() else DEVICE_CPU

    local_checkpoint = MODELS_DIR / SAM2_LOCAL_CHECKPOINT_FILENAME
    _load_sam2_predictor(context, device, local_checkpoint)


def _load_sam2_predictor(context: InferenceContext, device: str, local_checkpoint: object) -> None:
    """Populate *context* with a SAM2 or ultralytics predictor.

    Tries native sam2 first (preferred); falls back to ultralytics.

    Args:
        context:          Inference context to populate in-place.
        device:           Torch device string (``"cuda"`` or ``"cpu"``).
        local_checkpoint: Path to a local ``.pt`` checkpoint file.
    """
    try:
        from sam2.sam2_image_predictor import SAM2ImagePredictor  # type: ignore[import-untyped]

        if local_checkpoint.exists():
            from sam2.build_sam import build_sam2  # type: ignore[import-untyped]

            model = build_sam2(SAM2_DEFAULT_HIERA_CONFIG, str(local_checkpoint), device=device)
            context.predictor = SAM2ImagePredictor(model)
        else:
            context.predictor = SAM2ImagePredictor.from_pretrained(SAM2_DEFAULT_HF_REPO)
        context.use_native = True
    except Exception as native_error:
        logger.info("SAM2 native unavailable", extra={"_extra": {"err": str(native_error)}})
        _load_ultralytics_fallback(context)


def _load_ultralytics_fallback(context: InferenceContext) -> None:
    """Attempt to load the ultralytics SAM backend into *context*.

    Args:
        context: Inference context to populate in-place.
    """
    try:
        from ultralytics import SAM as UltralyticsSAM  # type: ignore[import-untyped]

        context.predictor = UltralyticsSAM(SAM2_LOCAL_CHECKPOINT_FILENAME)
        context.use_native = False
    except Exception as fallback_error:
        logger.info("No SAM backend available", extra={"_extra": {"err": str(fallback_error)}})


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


def _infer_via_server(
    state: AppState,
    client: ModelServerClient,
    coords: np.ndarray,
    labels: np.ndarray,
    mask_input: np.ndarray | None,
) -> tuple[list[np.ndarray], np.ndarray, np.ndarray | None]:
    """Delegate inference to the model-server TCP process.

    Args:
        state:      Current request state (``image_set`` flag mutated in-place).
        client:     Active model-server TCP client.
        coords:     Float32 point-coordinates array of shape ``(N, 2)``.
        labels:     Int32 point-labels array of shape ``(N,)``.
        mask_input: Optional logit mask for iterative refinement.

    Returns:
        Three-tuple ``(masks, scores, logits)``.
    """
    if not state.image_set:
        client.set_image(state.current_image)
        state.image_set = True

    all_masks, scores_list, logits = client.predict(coords, labels, mask_input=mask_input)
    return all_masks, np.array(scores_list), logits


def _infer_via_native_sam2(
    state: AppState,
    context: InferenceContext,
    coords: np.ndarray,
    labels: np.ndarray,
    mask_input: np.ndarray | None,
) -> tuple[list[np.ndarray], np.ndarray, np.ndarray | None]:
    """Run point-prompted inference using the native SAM2 package.

    Args:
        state:      Current request state (``image_set`` flag mutated in-place).
        context:    Local inference context holding the native predictor.
        coords:     Float32 point-coordinates array of shape ``(N, 2)``.
        labels:     Int32 point-labels array of shape ``(N,)``.
        mask_input: Optional logit mask for iterative refinement.

    Returns:
        Three-tuple ``(masks, scores, logits)``.
    """
    if not state.image_set:
        with context.torch_module.inference_mode(), _autocast_ctx(context):
            context.predictor.set_image(state.current_image)
        state.image_set = True

    with context.torch_module.inference_mode(), _autocast_ctx(context):
        masks, scores, logits = context.predictor.predict(
            point_coords=coords,
            point_labels=labels,
            mask_input=mask_input,
            multimask_output=True,
        )

    all_masks = [masks[index].astype(bool) for index in range(len(masks))]
    return all_masks, scores.flatten(), logits


def _infer_via_ultralytics(
    state: AppState,
    context: InferenceContext,
) -> tuple[list[np.ndarray], np.ndarray, None]:
    """Run inference using the ultralytics SAM wrapper (positive points only).

    Args:
        state:   Current request state with ``point_buffer`` and ``current_image``.
        context: Local inference context holding the ultralytics predictor.

    Returns:
        Three-tuple ``(masks, scores, None)`` — logits are not available.

    Raises:
        ValueError: When the model returns no masks.
    """
    positive_coords = [[point.x, point.y] for point in state.point_buffer if point.label == 1]
    positive_labels = [1] * len(positive_coords)

    results = context.predictor(state.current_image, points=[positive_coords], labels=[positive_labels])
    if not results or results[0].masks is None:
        msg = "SAM returned no mask"
        raise ValueError(msg)

    single_mask = results[0].masks.data[0].cpu().numpy().astype(bool)
    return [single_mask], np.array([INFERENCE_DEFAULT_MASK_SCORE]), None


def run_sam_inference(
    state: AppState,
    client: ModelServerClient | None,
    context: InferenceContext,
) -> InferenceResult:
    """Run SAM on the current point buffer and return a structured result.

    Feeds ``state.pending_logits`` back as ``mask_input`` for iterative
    refinement when available.  Applies connected-component filtering to remove
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
        return InferenceResult(masks=None, best_idx=0, scores_str="", error="SAM model not loaded.")

    points = state.point_buffer
    coords = np.array([[p.x, p.y] for p in points], dtype=np.float32)
    labels = np.array([p.label for p in points], dtype=np.int32)

    mask_input: np.ndarray | None = None
    if state.pending_logits is not None:
        raw_logits = state.pending_logits[state.pending_mask_idx]
        mask_input = raw_logits[None] if raw_logits.ndim == 2 else raw_logits

    logits_out: np.ndarray | None = None
    all_masks: list[np.ndarray]
    scores_flat: np.ndarray

    try:
        if client is not None:
            all_masks, scores_flat, logits_out = _infer_via_server(state, client, coords, labels, mask_input)
        elif context.use_native:
            all_masks, scores_flat, logits_out = _infer_via_native_sam2(state, context, coords, labels, mask_input)
        else:
            all_masks, scores_flat, logits_out = _infer_via_ultralytics(state, context)

    except Exception as error:
        _empty_cache(context)
        if context.oom_error and isinstance(error, context.oom_error):
            return InferenceResult(masks=None, best_idx=0, scores_str="", error="CUDA OOM — try a smaller image.")
        return InferenceResult(masks=None, best_idx=0, scores_str="", error=f"Inference error: {error}")
    finally:
        if client is None:
            _empty_cache(context)

    state.pending_logits = logits_out

    positive_points = [p for p in points if p.label == 1]
    all_masks = [_filter_connected_components(mask, positive_points) for mask in all_masks]

    best_index = int(np.argmax(scores_flat))
    scores_str = "  ".join(
        f"{MASK_LABELS[i]}: {scores_flat[i]:.3f}" for i in range(len(all_masks))
    )
    return InferenceResult(masks=all_masks, best_idx=best_index, scores_str=scores_str, error="")
