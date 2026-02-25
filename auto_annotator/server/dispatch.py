"""server.dispatch – Request/Response dataclasses and per-command handlers."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import torch

if TYPE_CHECKING:
    from server.context import ServerContext


@dataclass
class Request:
    cmd: str
    data: dict[str, Any]


@dataclass
class Response:
    ok: bool
    payload: dict[str, Any]


def _autocast_ctx(device: str):
    if device == "cuda":
        return torch.autocast("cuda", dtype=torch.bfloat16)
    import contextlib  # noqa: PLC0415

    return contextlib.nullcontext()


def _empty_cache() -> None:
    if torch.cuda.is_available():
        torch.cuda.empty_cache()


# ── Per-command handlers ───────────────────────────────────────────────────────


def handle_ping(msg: dict, ctx: "ServerContext") -> dict:
    return {
        "ok": True,
        "device": ctx.device,
        "model_loaded": ctx.predictor is not None,
        "model_id": ctx.model_id,
    }


def handle_list_models(msg: dict, ctx: "ServerContext") -> dict:
    from server.loader import is_available  # noqa: PLC0415

    base = Path(__file__).parent.parent
    return {
        "models": [
            {
                "id": cfg["id"],
                "label": cfg.get("label", cfg["id"]),
                "type": cfg.get("type", ""),
                "available": is_available(cfg, base),
                "active": cfg["id"] == ctx.model_id,
                "supports_text": cfg.get("supports_text", False),
            }
            for cfg in ctx.models_config
        ]
    }


def handle_set_model(msg: dict, ctx: "ServerContext") -> dict:
    from server.loader import load_model  # noqa: PLC0415

    model_id = msg.get("model_id", "")
    err = load_model(model_id, ctx)
    if err:
        return {"error": err}
    return {"ok": True, "model_id": model_id}


def handle_set_image(msg: dict, ctx: "ServerContext") -> dict:
    img = msg["image"]
    with torch.inference_mode(), _autocast_ctx(ctx.device):
        ctx.predictor.set_image(img)
    _empty_cache()
    return {"ok": True}


def handle_predict(msg: dict, ctx: "ServerContext") -> dict:
    coords = msg["coords"]
    labels = msg["labels"]
    mask_input = msg.get("mask_input")
    with torch.inference_mode(), _autocast_ctx(ctx.device):
        masks, scores, logits = ctx.predictor.predict(
            point_coords=coords,
            point_labels=labels,
            mask_input=mask_input,
            multimask_output=True,
        )
    _empty_cache()
    scores_flat = np.array(scores).flatten()
    return {
        "masks": [masks[i].astype(bool) for i in range(len(masks))],
        "scores": scores_flat.tolist(),
        "logits": logits,
    }


def handle_predict_text(msg: dict, ctx: "ServerContext") -> dict:
    if ctx.text_seg is None:
        return {"error": "Active model does not support text prompts"}
    results = ctx.text_seg.segment_by_text(msg["image"], msg["class_names"])
    return {"results": results}


# ── Dispatcher ─────────────────────────────────────────────────────────────────

_HANDLERS = {
    "ping": handle_ping,
    "list_models": handle_list_models,
    "set_model": handle_set_model,
}

_MODEL_HANDLERS = {
    "set_image": handle_set_image,
    "predict": handle_predict,
    "predict_text": handle_predict_text,
}


def dispatch(msg: dict, ctx: "ServerContext") -> dict:
    """Route a decoded message dict to the appropriate handler."""
    cmd = msg.get("cmd", "")

    if cmd in _HANDLERS:
        return _HANDLERS[cmd](msg, ctx)

    if ctx.predictor is None:
        return {"error": "No model loaded"}

    if cmd in _MODEL_HANDLERS:
        return _MODEL_HANDLERS[cmd](msg, ctx)

    return {"error": f"Unknown command: {cmd!r}"}
