"""src.server.loader – Model loading orchestration via registry.

Uses ModelRegistry for parallel availability checks, then delegates to
appropriate loader function. All config-dict keys come from
:mod:`src.server.constants`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from src.exceptions import ModelLoadError, ModelNotAvailable, ModelNotFound
from src.server.constants import (
    CFG_KEY_TYPE,
    MODEL_TYPE_SAM1,
    MODEL_TYPE_SAM2,
    MODEL_TYPE_SAM3,
    MODEL_TYPE_YOLO11,
    MODEL_TYPE_YOLOE,
    PROJECT_ROOT,
)
from src.server.registry import ModelRegistry
from src.server.sam1 import load_sam1
from src.server.sam2 import load_sam2
from src.server.sam3 import load_sam3
from src.server.yolo11 import load_yolo11
from src.server.yoloe import load_yoloe
from src.utils import get_logger

if TYPE_CHECKING:
    from collections.abc import Callable

    from src.server.context import ServerContext

logger = get_logger(__name__)

_MODEL_LOADERS: dict[str, Callable[[dict, ServerContext], None]] = {
    MODEL_TYPE_SAM1: load_sam1,
    MODEL_TYPE_SAM2: load_sam2,
    MODEL_TYPE_SAM3: load_sam3,
    MODEL_TYPE_YOLOE: load_yoloe,
    MODEL_TYPE_YOLO11: load_yolo11,
}


def load_model(model_id: str, ctx: ServerContext, registry: ModelRegistry) -> None:
    """Load a model by *model_id* into *ctx* using registry.

    Clears any previously loaded predictor, frees GPU cache, then delegates
    to the appropriate loader function.

    Args:
        model_id: Unique model identifier (must be in registry and available).
        ctx:      Mutable server context updated in-place on success.
        registry: ModelRegistry with availability checks performed.

    Raises:
        ModelNotFound: If model_id is not in registry.
        ModelNotAvailable: If model does not meet availability rules.
        ModelLoadError: If loading fails.
    """
    cfg, capabilities = registry.get(model_id)

    ctx.predictor = None
    ctx.text_seg = None
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    mtype = cfg.get(CFG_KEY_TYPE, "")
    loader = _MODEL_LOADERS.get(mtype)
    if loader is None:
        err_msg = f"Unknown model type: {mtype!r}"
        raise ModelLoadError(err_msg)

    try:
        loader(cfg, ctx)
        ctx.model_id = model_id
        logger.info("Model loaded", extra={"_extra": {"model_id": model_id, "caps": str(capabilities)}})
    except (ModelNotFound, ModelNotAvailable, ModelLoadError):
        raise
    except Exception as e:
        err_msg = f"Failed to load {model_id}: {e}"
        raise ModelLoadError(err_msg) from e


def initial_load(ctx: ServerContext, configs: list[dict], default_model: str = "") -> None:
    """Try to load the best available model on server startup.

    Uses ModelRegistry to perform parallel availability checks, then tries
    default_model first (if specified), then iterates through all available
    models in order. Logs critical error if none can be loaded.

    Args:
        ctx:           Mutable server context.
        configs:       Model config list from models.toml.
        default_model: Optional preferred model id to attempt first.
    """
    registry = ModelRegistry(configs, PROJECT_ROOT)
    logger.info("Model registry initialized", extra={"_extra": {"registry": str(registry)}})

    candidates: list[str] = []
    if default_model:
        candidates.append(default_model)
    candidates.extend(registry.all_available())

    for model_id in dict.fromkeys(candidates):
        try:
            load_model(model_id, ctx, registry)
        except (ModelNotFound, ModelNotAvailable, ModelLoadError) as e:
            logger.warning("Could not load %s: %s", model_id, e)
        else:
            return

    logger.critical("No models could be loaded")
