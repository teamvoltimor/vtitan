"""src.server.dispatch – Model-management and inference commands on a ServerContext.

Each function below runs one command directly against a :class:`~src.server.context.ServerContext`
and returns a typed response model. These used to be routed by command string
from a pickled TCP request dict (see docs/internal/audits/2026-07-08-auto-annotator-ml-service.md,
finding M6); the TCP layer is gone, so callers now import and call the
function they need directly (see :class:`src.model_server.LocalModelClient`).
"""

from __future__ import annotations

import contextlib
from typing import TYPE_CHECKING

import numpy as np
import torch

from src.core.enums import ComputeDevice
from src.core.exceptions import ModelLoadError, ModelNotAvailable, ModelNotFound
from src.server.constants import PROJECT_ROOT
from src.server.loader import load_model
from src.server.registry import ModelRegistry
from src.server.responses import (
    ListModelsResponse,
    ModelDescriptor,
    PingResponse,
    PredictResponse,
    PredictTextResponse,
    SetImageResponse,
    SetModelResponse,
)

if TYPE_CHECKING:
    from src.server.context import ServerContext


def _autocast_ctx(device: str) -> contextlib.AbstractContextManager:
    """Return ``torch.autocast`` for CUDA or a no-op context for CPU.

    Args:
        device: Device string (ComputeDevice.CUDA or ComputeDevice.CPU).

    Returns:
        Context manager suitable for use in a ``with`` block.
    """
    if device == ComputeDevice.CUDA:
        return torch.autocast(ComputeDevice.CUDA, dtype=torch.bfloat16)
    return contextlib.nullcontext()


def _empty_cache() -> None:
    """Free the CUDA memory cache when a GPU is available."""
    if torch.cuda.is_available():
        torch.cuda.empty_cache()


def ping(ctx: ServerContext) -> PingResponse:
    """Report device and model status.

    Args:
        ctx: Current server context.

    Returns:
        :class:`~src.server.responses.PingResponse` with device and model info.
    """
    return PingResponse(
        device=ctx.device,
        model_loaded=ctx.predictor is not None,
        model_id=ctx.model_id,
    )


def list_models(ctx: ServerContext) -> ListModelsResponse:
    """Return descriptors for all configured models.

    Args:
        ctx: Current server context.

    Returns:
        :class:`~src.server.responses.ListModelsResponse` containing one
        :class:`~src.server.responses.ModelDescriptor` per configured model.
    """
    registry = ModelRegistry(ctx.models_config, PROJECT_ROOT)
    descriptors = [
        ModelDescriptor(
            id=cfg.id,
            label=cfg.label or cfg.id,
            model_type=cfg.model_type,
            available=cfg.id in registry.all_available(),
            active=cfg.id == ctx.model_id,
            supports_text=cfg.supports_text,
        )
        for cfg in ctx.models_config
    ]
    return ListModelsResponse(models=descriptors)


def set_model(ctx: ServerContext, model_id: str) -> SetModelResponse:
    """Load a different model by id.

    Args:
        model_id: Unique model identifier matching a config entry.
        ctx: Mutable server context.

    Returns:
        :class:`~src.server.responses.SetModelResponse` with ``ok=True`` on
        success or a non-empty ``error`` string on failure.
    """
    registry = ModelRegistry(ctx.models_config, PROJECT_ROOT)
    try:
        load_model(model_id, ctx, registry)
        return SetModelResponse(model_id=model_id, error="")
    except (ModelNotFound, ModelNotAvailable, ModelLoadError) as e:
        return SetModelResponse(model_id=model_id, error=str(e))


def set_image(ctx: ServerContext, image: np.ndarray) -> SetImageResponse:
    """Encode the provided image with SAM's image encoder.

    Args:
        image: RGB uint8 numpy array.
        ctx: Server context with an active predictor.

    Returns:
        :class:`~src.server.responses.SetImageResponse` (always ok if no exception).

    Raises:
        ModelNotAvailable: If no model has been loaded yet.
    """
    if ctx.predictor is None:
        msg = "No model loaded"
        raise ModelNotAvailable(msg)
    with torch.inference_mode(), _autocast_ctx(ctx.device):
        ctx.predictor.set_image(image)
    _empty_cache()
    return SetImageResponse()


def predict(
    ctx: ServerContext,
    coords: np.ndarray,
    labels: np.ndarray,
    mask_input: np.ndarray | None = None,
) -> PredictResponse:
    """Run point-prompted mask prediction on the current image.

    Args:
        coords:     Float32 point-coordinates array of shape ``(N, 2)``.
        labels:     Int32 point-labels array of shape ``(N,)``; 1=positive, 0=negative.
        mask_input: Optional logit mask from a previous call for iterative refinement.
        ctx: Server context with an active predictor and encoded image.

    Returns:
        :class:`~src.server.responses.PredictResponse` with masks, scores,
        and raw logits for iterative refinement.

    Raises:
        ModelNotAvailable: If no model has been loaded yet.
    """
    if ctx.predictor is None:
        msg = "No model loaded"
        raise ModelNotAvailable(msg)

    with torch.inference_mode(), _autocast_ctx(ctx.device):
        masks, scores, logits = ctx.predictor.predict(
            point_coords=coords,
            point_labels=labels,
            mask_input=mask_input,
            multimask_output=True,
        )
    _empty_cache()

    scores_flat = np.array(scores).flatten()
    return PredictResponse(
        masks=[masks[i].astype(bool) for i in range(len(masks))],
        scores=scores_flat.tolist(),
        logits=logits,
    )


def predict_text(ctx: ServerContext, image: np.ndarray, class_names: list[str]) -> PredictTextResponse:
    """Run text-prompted segmentation using the active text segmenter.

    Args:
        image:       RGB array to segment.
        class_names: List of class-name strings used as text prompts.
        ctx: Server context; ``text_seg`` must not be ``None``.

    Returns:
        :class:`~src.server.responses.PredictTextResponse` with per-class results,
        or an error response when no text segmenter is loaded.
    """
    if ctx.text_seg is None:
        return PredictTextResponse(error="Active model does not support text prompts")
    results = ctx.text_seg.segment_by_text(image, class_names)
    return PredictTextResponse(results=results)
