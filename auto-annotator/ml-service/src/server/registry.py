"""src.server.registry – Model availability tracking and registry.

Performs parallel availability checks at initialization, caches results, and
allows dispatch to query capabilities without reloading. Replaces sequential
availability checks in loader.py with type-safe registry lookups.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pydantic import BaseModel, ConfigDict, Field

from src.exceptions import ModelNotAvailable, ModelNotFound
from src.server.constants import (
    MODEL_TYPE_SAM1,
    MODEL_TYPE_SAM2,
    MODEL_TYPE_SAM3,
    MODEL_TYPE_YOLO11,
    MODEL_TYPE_YOLOE,
    resolve_checkpoint_path,
)
from src.utils import get_logger

logger = get_logger(__name__)


class ModelConfig(BaseModel):
    """Configuration for a single model entry loaded from ``models.toml``.

    Attributes:
        id:             Unique model identifier string used in API calls and the UI.
        label:          Human-readable model label displayed in the Settings dropdown.
        model_type:     Model family (``sam1``, ``sam2``, ``sam3``, ``yolo11``, ``yoloe``).
        checkpoint:     Relative or absolute path to a local model checkpoint file.
        hf_repo:        HuggingFace repository ID used when no local checkpoint is present.
        variant:        SAM 1 architecture variant (``vit_h``, ``vit_l``, …).
        hiera_config:   Relative path to the SAM 2 Hiera YAML configuration file.
        supports_text:  Whether the model supports text-prompted segmentation.
    """

    model_config = ConfigDict(populate_by_name=True)

    id: str
    label: str = ""
    model_type: str = Field(default="", alias="type")
    checkpoint: str | None = None
    hf_repo: str | None = None
    variant: str | None = None
    hiera_config: str | None = None
    supports_text: bool = False


@dataclass(slots=True)
class ModelCapabilities:
    """Describes what a loaded model can do.

    Attributes:
        supports_points: Model accepts point prompts.
        supports_text: Model accepts text prompts.
        supports_iterative: Model returns logits for iterative refinement.
    """

    supports_points: bool = False
    supports_text: bool = False
    supports_iterative: bool = False

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
    MODEL_TYPE_YOLO11: ModelCapabilities(supports_text=True),
}


def _is_available(cfg: ModelConfig, base: Path) -> bool:
    """Check if a model config meets availability rules.

    Rules per type:
      * ``sam1``: requires a resolvable checkpoint file.
      * ``sam2``: requires either a checkpoint or a non-empty hf_repo.
      * ``sam3``: requires a non-empty hf_repo.
      * ``yoloe``: requires a resolvable checkpoint file.
    """
    ckpt = resolve_checkpoint_path(cfg.checkpoint, base)
    hf = cfg.hf_repo or ""

    if cfg.model_type == MODEL_TYPE_SAM1:
        return ckpt is not None and ckpt.exists()
    if cfg.model_type == MODEL_TYPE_SAM2:
        return bool(hf) or (ckpt is not None and ckpt.exists())
    if cfg.model_type == MODEL_TYPE_SAM3:
        return bool(hf)
    if cfg.model_type == MODEL_TYPE_YOLOE:
        return ckpt is not None and ckpt.exists()
    if cfg.model_type == MODEL_TYPE_YOLO11:
        return ckpt is not None and ckpt.exists()
    return False


class ModelRegistry:
    """Type-safe model registry with parallel availability checks.

    Attributes:
        models: Dict mapping model_id → (config, available, capabilities).
    """

    def __init__(self, configs: list[ModelConfig], base_dir: Path):
        """Initialize registry by checking all models in parallel.

        Args:
            configs: List of model configs from models.toml.
            base_dir: Base directory for resolving relative paths.
        """
        self.models: dict[str, tuple[ModelConfig, bool, ModelCapabilities]] = {}
        self._base_dir = base_dir

        with ThreadPoolExecutor(max_workers=8) as executor:
            futures = {executor.submit(_is_available, cfg, base_dir): cfg for cfg in configs}
            for future, cfg in futures.items():
                model_id = cfg.id or "unknown"
                available = future.result()
                capabilities = _CAPABILITIES.get(cfg.model_type, ModelCapabilities())
                self.models[model_id] = (cfg, available, capabilities)

    def get(self, model_id: str) -> tuple[ModelConfig, ModelCapabilities]:
        """Get config and capabilities for a model.

        Raises:
            ModelNotFound: If model_id is not in registry.
            ModelNotAvailable: If model is not available on this system.
        """
        if model_id not in self.models:
            msg = f"Model {model_id!r} not in registry"
            raise ModelNotFound(msg)

        cfg, available, caps = self.models[model_id]
        if not available:
            msg = f"Model {model_id!r} not available (checkpoint missing?)"
            raise ModelNotAvailable(msg)

        return cfg, caps

    def all_available(self) -> list[str]:
        """Return list of all available model IDs."""
        return [model_id for model_id, (_cfg, available, _caps) in self.models.items() if available]

    def __repr__(self) -> str:
        available = self.all_available()
        return f"ModelRegistry({len(available)} available / {len(self.models)} total)"
