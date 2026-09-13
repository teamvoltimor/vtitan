"""src.server.constants – Constants used across server modules.

Centralises magic strings for model types, config dict keys, and shared path
constants so that server modules never scatter bare string literals in their code.
"""

from pathlib import Path

# Absolute path to the project root (three levels above src/server/).
# Used when resolving relative checkpoint paths from models.toml.
PROJECT_ROOT: Path = Path(__file__).parent.parent.parent
"""Absolute path to the project root directory."""

# Model config dictionary keys.
# Used when reading ``cfg`` dicts parsed from models.toml [[models]] entries.

CFG_KEY_ID: str = "id"
"""Config key: unique model identifier string used in API calls and the UI."""

CFG_KEY_LABEL: str = "label"
"""Config key: human-readable model label displayed in the Settings dropdown."""

CFG_KEY_TYPE: str = "type"
"""Config key: model family (``sam1`` / ``sam2`` / ``sam3``)."""

CFG_KEY_CHECKPOINT: str = "checkpoint"
"""Config key: relative or absolute path to a local model checkpoint file."""

CFG_KEY_HF_REPO: str = "hf_repo"
"""Config key: HuggingFace repository ID used when no local checkpoint is present."""

CFG_KEY_VARIANT: str = "variant"
"""Config key: SAM 1 architecture variant string (e.g. ``vit_h``, ``vit_l``)."""

CFG_KEY_HIERA_CONFIG: str = "hiera_config"
"""Config key: relative path to the SAM 2 Hiera YAML configuration file."""

CFG_KEY_SUPPORTS_TEXT: str = "supports_text"
"""Config key: boolean flag indicating whether the model supports text prompts."""

# Default values for optional config fields.

SAM1_DEFAULT_VARIANT: str = "vit_h"
"""Default SAM 1 architecture variant when ``variant`` is absent from the config."""

SAM2_DEFAULT_HIERA_CONFIG: str = "configs/sam2.1/sam2.1_hiera_l.yaml"
"""Default path to the Hiera YAML config used when ``hiera_config`` is absent."""

# Tensor shape constants used by src/server/sam3.py.

SAM3_MASK_TENSOR_NDIM: int = 4
"""Rank of a batched SAM 3 mask tensor (batch, channel, height, width)."""


# Validator error messages for model loader functions.

ERR_SAM1_NO_CHECKPOINT: str = "SAM1 requires a 'checkpoint' path in config"
ERR_SAM2_NO_CONFIG: str = "SAM2 requires either 'checkpoint' or 'hf_repo' in config"
ERR_SAM3_NO_HF_REPO: str = "SAM3 requires 'hf_repo' in config"
ERR_YOLO11_NO_CHECKPOINT: str = "YOLOv11 requires 'checkpoint' in config"
ERR_YOLOE_NO_CHECKPOINT: str = "YOLOE requires 'checkpoint' in config"


def resolve_checkpoint_path(checkpoint: str | None, base: Path = PROJECT_ROOT) -> Path | None:
    """Resolve a config path relative to *base* when not absolute.

    Args:
        checkpoint: Path string from a model config, or ``None``.
        base: Directory to resolve relative paths against (defaults to project root).

    Returns:
        Absolute ``Path`` when *checkpoint* is set, or ``None`` when it is ``None``.
    """
    if checkpoint is None:
        return None
    path = Path(checkpoint)
    return path if path.is_absolute() else base / path
