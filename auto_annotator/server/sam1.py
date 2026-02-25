"""server.sam1 – SAM 1 model loading."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from server.context import ServerContext


def load_sam1(cfg: dict, ctx: "ServerContext") -> None:
    """Load a SAM 1 checkpoint into *ctx.predictor*."""
    from segment_anything import SamPredictor, sam_model_registry  # type: ignore

    ckpt_str = cfg.get("checkpoint")
    if not ckpt_str:
        msg = "SAM1 requires a 'checkpoint' path in config"
        raise ValueError(msg)

    ckpt = Path(ckpt_str)
    if not ckpt.is_absolute():
        ckpt = Path(__file__).parent.parent / ckpt

    variant = cfg.get("variant", "vit_h")
    sam = sam_model_registry[variant](checkpoint=str(ckpt))
    sam.to(device=ctx.device)
    ctx.predictor = SamPredictor(sam)
    ctx.text_seg = None
