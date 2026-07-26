"""src.server.sam1 – SAM 1 model loading.

Uses the ``segment_anything`` package (Meta's original SAM library).
All config-dict key strings come from :mod:`src.server.constants`.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from src.server.constants import SAM1_DEFAULT_VARIANT

if TYPE_CHECKING:
    from src.server.context import ServerContext
    from src.server.registry import ModelConfig


def load_sam1(cfg: ModelConfig, ctx: ServerContext) -> None:
    """Load a SAM 1 checkpoint into ``ctx.predictor``.

    Resolves the checkpoint path relative to the project root when the value
    stored in *cfg* is not absolute.

    Args:
        cfg: Model config dict from models.toml.  Must contain ``"checkpoint"``.
        ctx: Mutable server context; ``predictor`` and ``text_seg`` are updated in-place.

    Raises:
        ValueError: When the config does not specify a checkpoint path.
        ImportError: When ``segment_anything`` is not installed.
    """
    from segment_anything import SamPredictor, sam_model_registry  # type: ignore[import-untyped]

    if not cfg.checkpoint:
        msg = "SAM1 requires a 'checkpoint' path in config"
        raise ValueError(msg)

    ckpt = Path(cfg.checkpoint)
    if not ckpt.is_absolute():
        ckpt = Path(__file__).parent.parent.parent / ckpt

    variant = cfg.variant or SAM1_DEFAULT_VARIANT
    sam = sam_model_registry[variant](checkpoint=str(ckpt))
    sam.to(device=ctx.device)
    ctx.predictor = SamPredictor(sam)
    ctx.text_seg = None
