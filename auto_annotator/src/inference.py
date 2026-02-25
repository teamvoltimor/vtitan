"""src.inference – SAM inference helpers (server or local fallback)."""

from __future__ import annotations

import contextlib

import numpy as np

from src.constants import MASK_LABELS
from src.models import AppState
from src.sam_client import ModelServerClient
from src.utils import get_logger

logger = get_logger("inference")

# ── Module-level model state (for local / fallback mode) ──────────────────────

_predictor = None       # SAM2ImagePredictor or equivalent
_use_native = False     # True → SAM2 native; False → Ultralytics fallback
_oom_error = None       # torch OOM error class (set when torch is imported)
_torch = None           # torch module (imported lazily)


def initialize_inference(client: ModelServerClient | None) -> None:
    """
    When *client* is None, attempt to load SAM2 directly (local mode).
    Called once from app.py after probing the model server.
    """
    global _predictor, _use_native, _oom_error, _torch

    if client is not None:
        return  # model server handles everything

    import torch  # noqa: PLC0415

    _torch = torch
    _oom_error = torch.cuda.OutOfMemoryError
    device = "cuda" if torch.cuda.is_available() else "cpu"

    import os  # noqa: PLC0415
    from pathlib import Path  # noqa: PLC0415

    models_dir = Path(os.environ.get("MODELS_DIR", Path(__file__).parent.parent / "models"))
    local_ckpt = models_dir / "sam2.1_l.pt"

    try:
        from sam2.sam2_image_predictor import SAM2ImagePredictor  # type: ignore  # noqa: PLC0415

        if local_ckpt.exists():
            from sam2.build_sam import build_sam2  # type: ignore  # noqa: PLC0415

            _cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"
            _model = build_sam2(_cfg, str(local_ckpt), device=device)
            _predictor = SAM2ImagePredictor(_model)
        else:
            _predictor = SAM2ImagePredictor.from_pretrained("facebook/sam2.1-hiera-large")
        _use_native = True
    except Exception as e:  # noqa: BLE001
        logger.info("SAM2 native unavailable; trying Ultralytics", extra={"_extra": {"err": str(e)}})
        try:
            from ultralytics import SAM as _UltSAM  # type: ignore  # noqa: PLC0415

            _predictor = _UltSAM("sam2.1_l.pt")
            _use_native = False
        except Exception as e2:  # noqa: BLE001
            logger.info("No SAM backend available", extra={"_extra": {"err": str(e2)}})


def _empty_cache() -> None:
    if _torch is not None and _torch.cuda.is_available():
        _torch.cuda.empty_cache()


def _autocast_ctx():
    if _torch is not None and _torch.cuda.is_available():
        return _torch.autocast("cuda", dtype=_torch.bfloat16)
    return contextlib.nullcontext()


def _filter_cc(mask: np.ndarray, positive_pts) -> np.ndarray:
    """Keep only connected components that contain at least one positive click."""
    import cv2  # noqa: PLC0415

    if not mask.any():
        return mask

    mask_u8 = mask.astype(np.uint8)
    n_labels, label_map = cv2.connectedComponents(mask_u8)

    keep: set[int] = set()
    for pt in positive_pts:
        if pt.label != 1:
            continue
        px = max(0, min(int(round(pt.x)), mask.shape[1] - 1))
        py = max(0, min(int(round(pt.y)), mask.shape[0] - 1))
        lbl = label_map[py, px]
        if lbl != 0:
            keep.add(lbl)

    if not keep:
        return mask

    result = np.zeros_like(mask)
    for lbl in keep:
        result |= label_map == lbl
    return result


def run_sam_inference(
    state: AppState,
    client: ModelServerClient | None,
) -> tuple[list | None, int, str, str]:
    """
    Run SAM on the current point buffer.

    Returns (all_masks, best_idx, scores_str, error_msg).
    error_msg is '' on success.
    """
    img = state.current_image
    if img is None:
        return None, 0, "", "No image loaded."
    if not state.point_buffer:
        return None, 0, "", "No points in buffer."
    if client is None and _predictor is None:
        return None, 0, "", "SAM model not loaded."

    pts = state.point_buffer
    coords = np.array([[p.x, p.y] for p in pts], dtype=np.float32)
    labels = np.array([p.label for p in pts], dtype=np.int32)

    prev_logits = state.pending_logits
    prev_idx = state.pending_mask_idx
    mask_input = None
    if prev_logits is not None:
        raw = prev_logits[prev_idx]
        mask_input = raw[None] if raw.ndim == 2 else raw

    logits_out = None
    all_masks: list
    scores_flat: np.ndarray

    try:
        if client is not None:
            if not state.image_set:
                client.set_image(img)
                state.image_set = True
            all_masks, scores_list, logits_out = client.predict(coords, labels, mask_input=mask_input)
            scores_flat = np.array(scores_list)

        elif _use_native:
            if not state.image_set:
                with _torch.inference_mode(), _autocast_ctx():
                    _predictor.set_image(img)
                state.image_set = True

            with _torch.inference_mode(), _autocast_ctx():
                masks, scores, logits_out = _predictor.predict(
                    point_coords=coords,
                    point_labels=labels,
                    mask_input=mask_input,
                    multimask_output=True,
                )
            scores_flat = scores.flatten()
            all_masks = [masks[i].astype(bool) for i in range(len(masks))]

        else:
            pos_pts = [[p.x, p.y] for p in pts if p.label == 1]
            pos_lbl = [1] * len(pos_pts)
            results = _predictor(img, points=[pos_pts], labels=[pos_lbl])
            if not results or results[0].masks is None:
                return None, 0, "", "SAM returned no mask."
            all_masks = [results[0].masks.data[0].cpu().numpy().astype(bool)]
            scores_flat = np.array([1.0])

    except Exception as e:  # noqa: BLE001
        if _oom_error and isinstance(e, _oom_error):
            _empty_cache()
            return None, 0, "", "CUDA OOM — try a smaller image."
        _empty_cache()
        return None, 0, "", f"Inference error: {e}"
    finally:
        if client is None:
            _empty_cache()

    state.pending_logits = logits_out

    positive_pts = [p for p in pts if p.label == 1]
    all_masks = [_filter_cc(m, positive_pts) for m in all_masks]

    best_idx = int(np.argmax(scores_flat))
    scores_str = "  ".join(
        f"{MASK_LABELS[i]}: {scores_flat[i]:.3f}" for i in range(len(all_masks))
    )
    return all_masks, best_idx, scores_str, ""
