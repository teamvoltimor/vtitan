"""src.server.constants – Constants used across server modules.

Centralises all magic strings for model types, config dict keys, device names,
TCP command/response keys, payload field names, and shared path constants so
that server modules never scatter bare string literals in their code.
"""

from pathlib import Path

from src.enums import ComputeDevice, ModelType, ServerCommand

# Absolute path to the project root (three levels above src/server/).
# Used when resolving relative checkpoint paths from models.toml.
PROJECT_ROOT: Path = Path(__file__).parent.parent.parent
"""Absolute path to the project root directory."""

# Model type identifiers (see ModelType enum in src.enums).
# Match the value of the ``type`` field in each [[models]] entry of models.toml.

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

# Compute device names (see ComputeDevice enum in src.enums).

# TCP command strings (see ServerCommand enum in src.enums).

# Request message payload keys present in incoming request dicts.

MSG_KEY_CMD: str = "cmd"
"""Payload key for the command string in every request."""

MSG_KEY_IMAGE: str = "image"
"""Payload key for the numpy image array in ``set_image`` requests."""

MSG_KEY_COORDS: str = "coords"
"""Payload key for the float32 point-coordinates array in ``predict`` requests."""

MSG_KEY_LABELS: str = "labels"
"""Payload key for the int32 point-labels array (1=positive, 0=negative)."""

MSG_KEY_MASK_INPUT: str = "mask_input"
"""Payload key for the optional logit mask fed back from a previous prediction."""

MSG_KEY_CLASS_NAMES: str = "class_names"
"""Payload key for the list of class-name strings in ``predict_text`` requests."""

MSG_KEY_MODEL_ID: str = "model_id"
"""Payload key for the model-id string in ``set_model`` requests."""

# Response dict keys written into outgoing response dicts.

RESP_KEY_OK: str = "ok"
"""Response key indicating a successful operation (value is ``True``)."""

RESP_KEY_ERROR: str = "error"
"""Response key containing a human-readable error message string."""

RESP_KEY_MODELS: str = "models"
"""Response key containing the list of model descriptor dicts."""

RESP_KEY_MASKS: str = "masks"
"""Response key containing the list of predicted boolean mask arrays."""

RESP_KEY_SCORES: str = "scores"
"""Response key containing the list of mask confidence scores (floats)."""

RESP_KEY_LOGITS: str = "logits"
"""Response key containing the raw logit tensor for iterative refinement."""

RESP_KEY_RESULTS: str = "results"
"""Response key containing text-segmentation result dicts."""

RESP_KEY_DEVICE: str = "device"
"""Response key containing the server's active compute device string."""

RESP_KEY_MODEL_LOADED: str = "model_loaded"
"""Response key indicating whether a model is currently loaded (bool)."""

RESP_KEY_MODEL_ID: str = "model_id"
"""Response key containing the active model's identifier string."""

RESP_KEY_AVAILABLE: str = "available"
"""Model-list entry key: whether the model's checkpoint can be loaded."""

RESP_KEY_ACTIVE: str = "active"
"""Model-list entry key: whether this is the currently loaded model."""

RESP_KEY_SUPPORTS_TEXT: str = "supports_text"
"""Model-list entry key: whether the model supports text-prompted segmentation."""

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
