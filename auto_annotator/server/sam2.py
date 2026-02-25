"""server.sam2 – SAM 2 / 2.1 model loading."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from server.context import ServerContext


def load_sam2(cfg: dict, ctx: "ServerContext") -> None:
    """Load a SAM 2 model (local checkpoint or HuggingFace) into *ctx.predictor*."""
    from sam2.sam2_image_predictor import SAM2ImagePredictor  # type: ignore

    ckpt_str = cfg.get("checkpoint")
    hf = cfg.get("hf_repo", "")

    ckpt: Path | None = None
    if ckpt_str:
        ckpt = Path(ckpt_str)
        if not ckpt.is_absolute():
            ckpt = Path(__file__).parent.parent / ckpt

    if ckpt and ckpt.exists():
        from sam2.build_sam import build_sam2  # type: ignore

        hiera = cfg.get("hiera_config", "configs/sam2.1/sam2.1_hiera_l.yaml")
        model = build_sam2(hiera, str(ckpt), device=ctx.device)
        ctx.predictor = SAM2ImagePredictor(model)
    elif hf:
        ctx.predictor = SAM2ImagePredictor.from_pretrained(hf)
    else:
        msg = "SAM2 requires either 'checkpoint' or 'hf_repo' in config"
        raise ValueError(msg)

    ctx.text_seg = None
