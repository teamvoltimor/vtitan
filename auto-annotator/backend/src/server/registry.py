"""src.server.registry – Model availability tracking and registry.

Performs parallel availability checks at initialization, caches results, and
allows dispatch to query capabilities without reloading. Replaces sequential
availability checks in loader.py with type-safe registry lookups.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import TYPE_CHECKING

from src.exceptions import ModelLoadError, ModelNotAvailable, ModelNotFound
from src.server.constants import (
    CFG_KEY_CHECKPOINT,
    CFG_KEY_HF_REPO,
    CFG_KEY_ID,
    CFG_KEY_TYPE,
    MODEL_TYPE_SAM1,
    MODEL_TYPE_SAM2,
    MODEL_TYPE_SAM3,
    MODEL_TYPE_YOLOE,
)
from src.utils import get_logger

if TYPE_CHECKING:
    from collections.abc import Callable

logger = get_logger(__name__)


class ModelCapabilities:
    """Describes what a loaded model can do.

    Attributes:
        supports_points: Model accepts point prompts.
        supports_text: Model accepts text prompts.
        supports_iterative: Model returns logits for iterative refinement.
    """

    def __init__(self, supports_points: bool = False, supports_text: bool = False, supports_iterative: bool = False):
        self.supports_points = supports_points
        self.supports_text = supports_text
        self.supports_iterative = supports_iterative

    def __repr__(self) -> str:
        parts = []
        if self.supports_points:
            parts.append("points")
        if self.supports_text:
            parts.append("text")
        if self.supports_iterative:
            parts.append("iterative")
        return f"ModelCapabilities({', '.join(parts)})"


_CAPABILITIES: dict[str, ModelCapabilities] = {
    MODEL_TYPE_SAM1: ModelCapabilities(supports_points=True, supports_iterative=True),
    MODEL_TYPE_SAM2: ModelCapabilities(supports_points=True, supports_text=True, supports_iterative=True),
    MODEL_TYPE_SAM3: ModelCapabilities(supports_points=True, supports_text=True, supports_iterative=True),
    MODEL_TYPE_YOLOE: ModelCapabilities(supports_text=True),
}


def _resolve(p: str | None, base: Path) -> Path | None:
    """Resolve a config path string relative to *base* when not absolute."""
    if p is None:
        return None
    path = Path(p)
    return path if path.is_absolute() else base / path


def _is_available(cfg: dict, base: Path) -> bool:
    """Check if a model config meets availability rules.

    Rules per type:
      * ``sam1``: requires a resolvable checkpoint file.
      * ``sam2``: requires either a checkpoint or a non-empty hf_repo.
      * ``sam3``: requires a non-empty hf_repo.
      * ``yoloe``: requires a resolvable checkpoint file.
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
    return False


class ModelRegistry:
    """Type-safe model registry with parallel availability checks.

    Attributes:
        models: Dict mapping model_id → (config, available, capabilities).
    """

    def __init__(self, configs: list[dict], base_dir: Path):
        """Initialize registry by checking all models in parallel.

        Args:
            configs: List of model config dicts from models.toml.
            base_dir: Base directory for resolving relative paths.
        """
        self.models: dict[str, tuple[dict, bool, ModelCapabilities]] = {}
        self._base_dir = base_dir

        with ThreadPoolExecutor(max_workers=8) as executor:
            futures = {executor.submit(_is_available, cfg, base_dir): cfg for cfg in configs}
            for future in futures:
                cfg = futures[future]
                model_id = cfg.get(CFG_KEY_ID, "unknown")
                available = future.result()
                mtype = cfg.get(CFG_KEY_TYPE, "")
                capabilities = _CAPABILITIES.get(mtype, ModelCapabilities())
                self.models[model_id] = (cfg, available, capabilities)

    def get(self, model_id: str) -> tuple[dict, ModelCapabilities]:
        """Get config and capabilities for a model.

        Raises:
            ModelNotFound: If model_id is not in registry.
            ModelNotAvailable: If model is not available on this system.
        """
        if model_id not in self.models:
            raise ModelNotFound(f"Model {model_id!r} not in registry")

        cfg, available, caps = self.models[model_id]
        if not available:
            raise ModelNotAvailable(f"Model {model_id!r} not available (checkpoint missing?)")

        return cfg, caps

    def all_available(self) -> list[str]:
        """Return list of all available model IDs."""
        return [model_id for model_id, (_cfg, available, _caps) in self.models.items() if available]

    def __repr__(self) -> str:
        available = self.all_available()
        return f"ModelRegistry({len(available)} available / {len(self.models)} total)"
