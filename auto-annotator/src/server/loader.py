"""src.server.loader – Model availability checks and loading orchestration.

Determines which models from models.toml are loadable given the current
filesystem state, then delegates to the appropriate ``load_sam*`` function.
All config-dict key and model-type strings come from :mod:`src.server.constants`.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import torch

from src.server.constants import (
    CFG_KEY_CHECKPOINT,
    CFG_KEY_HF_REPO,
    CFG_KEY_ID,
    CFG_KEY_TYPE,
    MODEL_TYPE_GROUNDING_DINO,
    MODEL_TYPE_SAM1,
    MODEL_TYPE_SAM2,
    MODEL_TYPE_SAM3,
    MODEL_TYPE_YOLOE,
    PROJECT_ROOT,
)
from src.server.grounding_dino import load_grounding_dino
from src.server.sam1 import load_sam1
from src.server.sam2 import load_sam2
from src.server.sam3 import load_sam3
from src.server.yoloe import load_yoloe
from src.utils import get_logger

if TYPE_CHECKING:
    from collections.abc import Callable

    from src.server.context import ServerContext

logger = get_logger(__name__)


def _resolve(p: str | None, base: Path) -> Path | None:
    """Resolve a config path string relative to *base* when it is not absolute.

    Args:
        p:    Path string from the config dict, or ``None``.
        base: Base directory used when *p* is a relative path.

    Returns:
        Absolute :class:`Path` if *p* is not ``None``; otherwise ``None``.
    """
    if p is None:
        return None
    path = Path(p)
    return path if path.is_absolute() else base / path


def is_available(cfg: dict, base: Path) -> bool:
    """Return ``True`` when the model described by *cfg* can be loaded.

    Availability rules per model type:
      * ``sam1``: requires a resolvable checkpoint file.
      * ``sam2``: requires either a resolvable checkpoint *or* a non-empty hf_repo.
      * ``sam3``: requires a non-empty hf_repo.

    Args:
        cfg:  Model config dict from models.toml.
        base: Project root directory used to resolve relative checkpoint paths.

    Returns:
        Boolean availability flag.
    """
    mtype = cfg.get(CFG_KEY_TYPE, "")
    ckpt = _resolve(cfg.get(CFG_KEY_CHECKPOINT), base)
    hf = cfg.get(CFG_KEY_HF_REPO, "")

    if mtype == MODEL_TYPE_SAM1:
        return ckpt is not None and ckpt.exists()
    if mtype == MODEL_TYPE_SAM2:
        return bool(hf) or (ckpt is not None and ckpt.exists())
    if mtype == MODEL_TYPE_SAM3:
        return bool(hf)
    if mtype == MODEL_TYPE_YOLOE:
        return ckpt is not None and ckpt.exists()
    return mtype == MODEL_TYPE_GROUNDING_DINO


def load_model(model_id: str, ctx: ServerContext) -> str | None:
    """Load a model by *model_id* into *ctx*.

    Clears any previously loaded predictor, frees the GPU cache, then
    delegates to the appropriate ``load_sam*`` function.

    Args:
        model_id: Unique model identifier string (must match a config ``id``).
        ctx:      Mutable server context updated in-place on success.

    Returns:
        ``None`` on success, or a non-empty error string on failure.
    """
    cfg = next((c for c in ctx.models_config if c[CFG_KEY_ID] == model_id), None)
    if cfg is None:
        return f"Unknown model: {model_id!r}"

    if not is_available(cfg, PROJECT_ROOT):
        return f"Model {model_id!r} not available (checkpoint missing?)"

    ctx.predictor = None
    ctx.text_seg = None
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    loaders: dict[str, Callable[[dict, ServerContext], None]] = {
        MODEL_TYPE_SAM1: load_sam1,
        MODEL_TYPE_SAM2: load_sam2,
        MODEL_TYPE_SAM3: load_sam3,
        MODEL_TYPE_YOLOE: load_yoloe,
        MODEL_TYPE_GROUNDING_DINO: load_grounding_dino,
    }

    try:
        mtype = cfg.get(CFG_KEY_TYPE, "")
        loader = loaders.get(mtype)
        if loader is None:
            return f"Unknown model type: {mtype!r}"

        loader(cfg, ctx)

        ctx.model_id = model_id
    except Exception as e:
        return f"Failed to load {model_id}: {e}"
    else:
        return None


def initial_load(ctx: ServerContext, default_model: str = "") -> None:
    """Try to load the best available model on server startup.

    Tries *default_model* first (if specified), then iterates through all
    models in ``ctx.models_config`` in order.  Stops at the first successful
    load.  Prints a fatal warning to stderr when no model can be loaded.

    Args:
        ctx:           Mutable server context.
        default_model: Optional preferred model id to attempt first.
    """
    candidates: list[str] = []
    if default_model:
        candidates.append(default_model)
    candidates.extend(c[CFG_KEY_ID] for c in ctx.models_config)

    for model_id in dict.fromkeys(candidates):
        cfg = next((c for c in ctx.models_config if c[CFG_KEY_ID] == model_id), None)
        if cfg and is_available(cfg, PROJECT_ROOT):
            err = load_model(model_id, ctx)
            if err is None:
                return

    logger.critical("No models could be loaded")
