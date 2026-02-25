"""server.loader – Model availability checks and loading orchestration."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from server.context import ServerContext


def _resolve(p: str | None, base: Path) -> Path | None:
    if p is None:
        return None
    path = Path(p)
    return path if path.is_absolute() else base / path


def is_available(cfg: dict, base: Path) -> bool:
    """Return True when the model described by *cfg* can be loaded."""
    mtype = cfg.get("type", "")
    ckpt = _resolve(cfg.get("checkpoint"), base)
    hf = cfg.get("hf_repo", "")
    if mtype == "sam1":
        return ckpt is not None and ckpt.exists()
    if mtype == "sam2":
        return bool(hf) or (ckpt is not None and ckpt.exists())
    if mtype == "sam3":
        return bool(hf)
    return False


def load_model(model_id: str, ctx: "ServerContext") -> str | None:
    """Load a model by id.  Returns an error string on failure, None on success."""
    import torch  # noqa: PLC0415

    cfg = next((c for c in ctx.models_config if c["id"] == model_id), None)
    if cfg is None:
        return f"Unknown model: {model_id!r}"

    base = Path(__file__).parent.parent
    if not is_available(cfg, base):
        return f"Model {model_id!r} not available (checkpoint missing?)"

    # Unload current model
    ctx.predictor = None
    ctx.text_seg = None
    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    try:
        mtype = cfg.get("type", "")
        if mtype == "sam1":
            from server.sam1 import load_sam1  # noqa: PLC0415

            load_sam1(cfg, ctx)
        elif mtype == "sam2":
            from server.sam2 import load_sam2  # noqa: PLC0415

            load_sam2(cfg, ctx)
        elif mtype == "sam3":
            from server.sam3 import load_sam3  # noqa: PLC0415

            load_sam3(cfg, ctx)
        else:
            return f"Unknown model type: {mtype!r}"

        ctx.model_id = model_id
        return None
    except Exception as e:  # noqa: BLE001
        return f"Failed to load {model_id}: {e}"


def initial_load(ctx: "ServerContext", default_model: str = "") -> None:
    """Try to load the best available model on server startup."""
    candidates: list[str] = []
    if default_model:
        candidates.append(default_model)
    candidates.extend(c["id"] for c in ctx.models_config)

    base = Path(__file__).parent.parent
    for model_id in dict.fromkeys(candidates):
        cfg = next((c for c in ctx.models_config if c["id"] == model_id), None)
        if cfg and is_available(cfg, base):
            err = load_model(model_id, ctx)
            if err is None:
                return

    import sys  # noqa: PLC0415

    print("[server] FATAL: no models could be loaded.", file=sys.stderr)  # noqa: T201
