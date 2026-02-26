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
from datetime import UTC, datetime
from typing import TYPE_CHECKING

import cv2
import numpy as np

from src.constants import (
    DEVICE_CPU,
    DEVICE_CUDA,
    INFERENCE_DEFAULT_MASK_SCORE,
    INFERENCE_LOG_MAX_ENTRIES,
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


def _empty_cache(ctx: InferenceContext) -> None:
    """Free the CUDA memory cache when a CUDA-capable torch module is loaded.

    Args:
        ctx: Inference context holding the lazy-loaded ``torch_module``.
    """
    if ctx.torch_module is not None and ctx.torch_module.cuda.is_available():
        ctx.torch_module.cuda.empty_cache()


def _autocast_ctx(ctx: InferenceContext) -> contextlib.AbstractContextManager:
    """Return a ``torch.autocast`` context for CUDA, or a no-op on CPU.

    Args:
        ctx: Inference context holding the lazy-loaded ``torch_module``.

    Returns:
        Context manager suitable for use in a ``with`` block.
    """
    if ctx.torch_module is not None and ctx.torch_module.cuda.is_available():
        return ctx.torch_module.autocast(DEVICE_CUDA, dtype=ctx.torch_module.bfloat16)
    return contextlib.nullcontext()


def initialize_inference(client: ModelServerClient | None, ctx: InferenceContext) -> None:
    """Load SAM locally into *ctx* when no model server is reachable.

    Tries the native ``sam2`` package first (preferred), then falls back to the
    ``ultralytics`` SAM wrapper.  Does nothing when *client* is not ``None``
    (server mode is active) or when *ctx* already has a predictor loaded.

    The ML framework imports (``torch``, ``sam2``, ``ultralytics``) remain lazy
    inside this function so that importing ``src.inference`` does not trigger
    heavy library loading on the application side.

    Args:
        client: Active :class:`~src.sam_client.ModelServerClient`, or ``None``
                when running without a model server.
        ctx:    :class:`~src.models.InferenceContext` to populate in-place.
    """
    if client is not None:
        return

    # Lazy-import torch and capture the OOM error class for later detection.
    import torch
    ctx.torch_module = torch
    ctx.oom_error = torch.cuda.OutOfMemoryError
    device = DEVICE_CUDA if torch.cuda.is_available() else DEVICE_CPU

    local_ckpt = MODELS_DIR / SAM2_LOCAL_CHECKPOINT_FILENAME

    # Try the native sam2 package first; prefer a local checkpoint over HuggingFace download.
    try:
        from sam2.sam2_image_predictor import SAM2ImagePredictor  # type: ignore[import-untyped]
        if local_ckpt.exists():
            from sam2.build_sam import build_sam2  # type: ignore[import-untyped]
            _model = build_sam2(SAM2_DEFAULT_HIERA_CONFIG, str(local_ckpt), device=device)
            ctx.predictor = SAM2ImagePredictor(_model)
        else:
            ctx.predictor = SAM2ImagePredictor.from_pretrained(SAM2_DEFAULT_HF_REPO)
        ctx.use_native = True
    except Exception as e:
        logger.info("SAM2 native unavailable", extra={"_extra": {"err": str(e)}})
        # Ultralytics SAM is a lighter fallback that sacrifices multi-mask output.
        try:
            from ultralytics import SAM as _UltSAM  # type: ignore[import-untyped]
            ctx.predictor = _UltSAM(SAM2_LOCAL_CHECKPOINT_FILENAME)
            ctx.use_native = False
        except Exception as e2:
            logger.info("No SAM backend available", extra={"_extra": {"err": str(e2)}})


def _filter_cc(mask: np.ndarray, positive_pts: list[Point]) -> np.ndarray:
    """Keep only connected components that contain at least one positive click.

    This eliminates stray background blobs that SAM may include in the mask
    when the user has not placed points in those regions.

    Args:
        mask:          Boolean H×W mask array from SAM.
        positive_pts:  Points with ``label == 1`` (positive clicks).

    Returns:
        Filtered boolean H×W mask containing only components touched by a positive click.
        Returns *mask* unchanged when no components are selected (safety fallback).
    """
    if not mask.any():
        return mask

    _n_labels, label_map = cv2.connectedComponents(mask.astype(np.uint8))
    keep: set[int] = set()

    for pt in positive_pts:
        if pt.label != 1:
            continue
        px = max(0, min(pt.x, mask.shape[1] - 1))
        py = max(0, min(pt.y, mask.shape[0] - 1))
        lbl = label_map[py, px]
        if lbl != 0:
            keep.add(lbl)

    if not keep:
        return mask

    result = np.zeros_like(mask)
    for lbl in keep:
        result |= label_map == lbl
    return result


def _log_entry(state: AppState, msg: str) -> None:
    """Insert a timestamped entry at the front of ``state.log_entries``.

    Trims the list to :data:`~src.constants.INFERENCE_LOG_MAX_ENTRIES` entries.

    Args:
        state: Mutable session state to update.
        msg:   Message text to prepend.
    """
    ts = datetime.now(UTC).strftime("%H:%M:%S")
    state.log_entries.insert(0, f"[{ts}] {msg}")
    state.log_entries = state.log_entries[:INFERENCE_LOG_MAX_ENTRIES]


def run_sam_inference(
    state: AppState,
    client: ModelServerClient | None,
    ctx: InferenceContext,
) -> InferenceResult:
    """Run SAM on the current point buffer and return a structured result.

    Feeds ``state.pending_logits`` back as ``mask_input`` for iterative
    refinement when available.  Applies connected-component filtering to remove
    stray background blobs.

    Args:
        state:  Current :class:`~src.models.AppState` with image and point buffer.
        client: Active server client, or ``None`` for local inference.
        ctx:    Local inference context (predictor, torch, OOM class).

    Returns:
        :class:`~src.models.InferenceResult` with ``ok == True`` on success.
    """
    # Guard: require image, points, and at least one active backend.
    img = state.current_image
    if img is None:
        return InferenceResult(masks=None, best_idx=0, scores_str="", error="No image loaded.")
    if not state.point_buffer:
        return InferenceResult(masks=None, best_idx=0, scores_str="", error="No points in buffer.")
    if client is None and ctx.predictor is None:
        return InferenceResult(masks=None, best_idx=0, scores_str="", error="SAM model not loaded.")

    # Convert point buffer to arrays and carry over previous logits for iterative refinement.
    pts = state.point_buffer
    coords = np.array([[p.x, p.y] for p in pts], dtype=np.float32)
    labels = np.array([p.label for p in pts], dtype=np.int32)

    prev_logits = state.pending_logits
    mask_input = None
    if prev_logits is not None:
        raw = prev_logits[state.pending_mask_idx]
        mask_input = raw[None] if raw.ndim == 2 else raw

    logits_out = None
    all_masks: list[np.ndarray]
    scores_flat: np.ndarray

    try:
        if client is not None:
            # Server mode: delegate inference to the model server process.
            if not state.image_set:
                client.set_image(img)
                state.image_set = True
            all_masks, scores_list, logits_out = client.predict(
                coords, labels, mask_input=mask_input,
            )
            scores_flat = np.array(scores_list)

        elif ctx.use_native:
            # Native SAM2: pre-load the image on first call, then predict with autocast.
            if not state.image_set:
                with ctx.torch_module.inference_mode(), _autocast_ctx(ctx):
                    ctx.predictor.set_image(img)
                state.image_set = True
            with ctx.torch_module.inference_mode(), _autocast_ctx(ctx):
                masks, scores, logits_out = ctx.predictor.predict(
                    point_coords=coords,
                    point_labels=labels,
                    mask_input=mask_input,
                    multimask_output=True,
                )
            scores_flat = scores.flatten()
            all_masks = [masks[i].astype(bool) for i in range(len(masks))]

        else:
            # Ultralytics fallback: positive points only, single mask output.
            pos_pts = [[p.x, p.y] for p in pts if p.label == 1]
            pos_lbl = [1] * len(pos_pts)
            results = ctx.predictor(img, points=[pos_pts], labels=[pos_lbl])
            if not results or results[0].masks is None:
                return InferenceResult(masks=None, best_idx=0, scores_str="", error="SAM returned no mask.")
            all_masks = [results[0].masks.data[0].cpu().numpy().astype(bool)]
            scores_flat = np.array([INFERENCE_DEFAULT_MASK_SCORE])

    except Exception as e:
        if ctx.oom_error and isinstance(e, ctx.oom_error):
            _empty_cache(ctx)
            return InferenceResult(masks=None, best_idx=0, scores_str="", error="CUDA OOM — try a smaller image.")
        _empty_cache(ctx)
        return InferenceResult(masks=None, best_idx=0, scores_str="", error=f"Inference error: {e}")
    finally:
        if client is None:
            _empty_cache(ctx)

    # Store logits for iterative refinement on the next click.
    state.pending_logits = logits_out

    # Filter stray background blobs then select the highest-scoring mask.
    positive_pts = [p for p in pts if p.label == 1]
    all_masks = [_filter_cc(m, positive_pts) for m in all_masks]

    best_idx = int(np.argmax(scores_flat))
    scores_str = "  ".join(
        f"{MASK_LABELS[i]}: {scores_flat[i]:.3f}" for i in range(len(all_masks))
    )
    return InferenceResult(masks=all_masks, best_idx=best_idx, scores_str=scores_str, error="")
