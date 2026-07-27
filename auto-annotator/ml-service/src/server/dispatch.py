"""src.server.dispatch – Request routing and per-command handlers.

Each public ``handle_*`` function processes one TCP command and returns a typed
response dataclass.  The :func:`dispatch` function calls ``.to_dict()`` on the
result before it is pickled and sent back over the TCP socket.  All key strings
come from :mod:`src.server.constants` so no magic strings appear here.
"""

from __future__ import annotations

import contextlib
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel

import numpy as np
import torch

from src.enums import ComputeDevice, ServerCommand
from src.exceptions import ModelLoadError, ModelNotAvailable, ModelNotFound
from src.server.constants import (
    MSG_KEY_CLASS_NAMES,
    MSG_KEY_CMD,
    MSG_KEY_COORDS,
    MSG_KEY_IMAGE,
    MSG_KEY_LABELS,
    MSG_KEY_MASK_INPUT,
    MSG_KEY_MODEL_ID,
    PROJECT_ROOT,
)
from src.server.loader import load_model
from src.server.registry import ModelRegistry
from src.server.responses import (
    ErrorResponse,
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


def handle_ping(_msg: dict, ctx: ServerContext) -> PingResponse:
    """Respond to a liveness probe with device and model status.

    Args:
        _msg: Incoming request dict (payload ignored).
        ctx: Current server context.

    Returns:
        :class:`~src.server.responses.PingResponse` with device and model info.
    """
    return PingResponse(
        device=ctx.device,
        model_loaded=ctx.predictor is not None,
        model_id=ctx.model_id,
    )


def handle_list_models(_msg: dict, ctx: ServerContext) -> ListModelsResponse:
    """Return descriptors for all configured models.

    Args:
        _msg: Incoming request dict (payload ignored).
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


def handle_set_model(msg: dict, ctx: ServerContext) -> SetModelResponse:
    """Load a different model by id.

    Args:
        msg: Request dict; must contain ``MSG_KEY_MODEL_ID``.
        ctx: Mutable server context.

    Returns:
        :class:`~src.server.responses.SetModelResponse` with ``ok=True`` on
        success or a non-empty ``error`` string on failure.
    """
    model_id = msg.get(MSG_KEY_MODEL_ID, "")
    registry = ModelRegistry(ctx.models_config, PROJECT_ROOT)
    try:
        load_model(model_id, ctx, registry)
        return SetModelResponse(model_id=model_id, error="")
    except (ModelNotFound, ModelNotAvailable, ModelLoadError) as e:
        return SetModelResponse(model_id=model_id, error=str(e))


def handle_set_image(msg: dict, ctx: ServerContext) -> SetImageResponse:
    """Encode the provided image with SAM's image encoder.

    Args:
        msg: Request dict; must contain ``MSG_KEY_IMAGE`` (RGB uint8 numpy array).
        ctx: Server context with an active predictor.

    Returns:
        :class:`~src.server.responses.SetImageResponse` (always ok if no exception).
    """
    img = msg[MSG_KEY_IMAGE]
    if ctx.predictor is None:
        # dispatch() only routes here via _MODEL_HANDLERS after confirming
        # ctx.predictor is not None; reaching this is an internal invariant
        # violation, not a normal "no model loaded" response.
        msg_text = "handle_set_image called with no predictor loaded"
        raise ModelNotAvailable(msg_text)
    with torch.inference_mode(), _autocast_ctx(ctx.device):
        ctx.predictor.set_image(img)
    _empty_cache()
    return SetImageResponse()


def handle_predict(msg: dict, ctx: ServerContext) -> PredictResponse:
    """Run point-prompted mask prediction on the current image.

    Args:
        msg: Request dict with ``coords``, ``labels``, and optional ``mask_input``.
        ctx: Server context with an active predictor and encoded image.

    Returns:
        :class:`~src.server.responses.PredictResponse` with masks, scores,
        and raw logits for iterative refinement.
    """
    coords = msg[MSG_KEY_COORDS]
    labels = msg[MSG_KEY_LABELS]
    mask_input = msg.get(MSG_KEY_MASK_INPUT)

    if ctx.predictor is None:
        # dispatch() only routes here via _MODEL_HANDLERS after confirming
        # ctx.predictor is not None; reaching this is an internal invariant
        # violation, not a normal "no model loaded" response.
        msg_text = "handle_predict called with no predictor loaded"
        raise ModelNotAvailable(msg_text)

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


def handle_predict_text(msg: dict, ctx: ServerContext) -> PredictTextResponse:
    """Run text-prompted segmentation using the active text segmenter.

    Args:
        msg: Request dict with ``image`` (RGB array) and ``class_names`` (list of str).
        ctx: Server context; ``text_seg`` must not be ``None``.

    Returns:
        :class:`~src.server.responses.PredictTextResponse` with per-class results,
        or an error response when no text segmenter is loaded.
    """
    if ctx.text_seg is None:
        return PredictTextResponse(error="Active model does not support text prompts")
    results = ctx.text_seg.segment_by_text(msg[MSG_KEY_IMAGE], msg[MSG_KEY_CLASS_NAMES])
    return PredictTextResponse(results=results)


# Command routing tables.
# Commands that do not require a loaded predictor.
_HANDLERS: dict[str, Any] = {
    ServerCommand.PING: handle_ping,
    ServerCommand.LIST_MODELS: handle_list_models,
    ServerCommand.SET_MODEL: handle_set_model,
}

# Commands that require an active predictor (model must be loaded first).
_MODEL_HANDLERS: dict[str, Any] = {
    ServerCommand.SET_IMAGE: handle_set_image,
    ServerCommand.PREDICT: handle_predict,
    ServerCommand.PREDICT_TEXT: handle_predict_text,
}


def _dump(resp: BaseModel) -> dict:
    """Serialise a response model to a wire-protocol dict.

    Uses ``.to_dict()`` when the model defines custom serialisation logic,
    otherwise falls back to ``.model_dump(by_alias=True)`` so that fields
    with ``Field(alias=...)`` are serialised under their wire-protocol key.
    """
    to_dict = getattr(resp, "to_dict", None)
    if to_dict is not None:
        return to_dict()
    return resp.model_dump(by_alias=True)


def dispatch(msg: dict, ctx: ServerContext) -> dict:
    """Route a decoded request dict to the appropriate command handler.

    Checks ``_HANDLERS`` first (no model required), then ``_MODEL_HANDLERS``
    (requires a loaded predictor).  Returns an error dict for unknown commands
    or when no model is loaded and a model-required command is received.

    Args:
        msg: Decoded request dict; must have at least a ``MSG_KEY_CMD`` field.
        ctx: Current server context.

    Returns:
        Wire-protocol response dict from the selected handler, or an error dict.
    """
    cmd = msg.get(MSG_KEY_CMD, "")

    if cmd in _HANDLERS:
        return _dump(_HANDLERS[cmd](msg, ctx))

    if ctx.predictor is None:
        return _dump(ErrorResponse(error="No model loaded"))

    if cmd in _MODEL_HANDLERS:
        return _dump(_MODEL_HANDLERS[cmd](msg, ctx))

    return _dump(ErrorResponse(error=f"Unknown command: {cmd!r}"))
