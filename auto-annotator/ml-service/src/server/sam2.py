"""src.server.sam2 – SAM 2 / 2.1 model loading.

Supports both local checkpoints and HuggingFace repositories.
All config-dict key strings come from :mod:`src.server.constants`.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from src.server.constants import SAM2_DEFAULT_HIERA_CONFIG

if TYPE_CHECKING:
    from src.server.context import ServerContext
    from src.server.registry import ModelConfig


def load_sam2(cfg: ModelConfig, ctx: ServerContext) -> None:
    """Load a SAM 2 / 2.1 model (local checkpoint or HuggingFace) into ``ctx.predictor``.

    Prefers a local checkpoint when ``cfg["checkpoint"]`` exists on disk; otherwise
    falls back to ``cfg["hf_repo"]`` for automatic download via HuggingFace.

    Args:
        cfg: Model config dict from models.toml.  Must contain either
             ``"checkpoint"`` (path to a local ``.pt`` file) or ``"hf_repo"``
             (HuggingFace repository ID).
        ctx: Mutable server context; ``predictor`` and ``text_seg`` are updated in-place.

    Raises:
        ValueError: When neither a valid checkpoint nor an hf_repo is specified.
        ImportError: When the ``sam2`` package is not installed.
    """
    from sam2.sam2_image_predictor import SAM2ImagePredictor  # type: ignore[import-untyped]

    hf = cfg.hf_repo or ""

    ckpt: Path | None = None
    if cfg.checkpoint:
        ckpt = Path(cfg.checkpoint)
        if not ckpt.is_absolute():
            ckpt = Path(__file__).parent.parent.parent / ckpt

    if ckpt and ckpt.exists():
        from sam2.build_sam import build_sam2  # type: ignore[import-untyped]

        hiera = cfg.hiera_config or SAM2_DEFAULT_HIERA_CONFIG
        model = build_sam2(hiera, str(ckpt), device=ctx.device)
        ctx.predictor = SAM2ImagePredictor(model)
    elif hf:
        ctx.predictor = SAM2ImagePredictor.from_pretrained(hf)
    else:
        msg = "SAM2 requires either 'checkpoint' or 'hf_repo' in config"
        raise ValueError(msg)

    ctx.text_seg = None
