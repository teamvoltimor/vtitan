"""src.server.constants – Constants used across server modules.

Centralises all magic strings for model types, config dict keys, device names,
TCP command/response keys, payload field names, and shared path constants so
that server modules never scatter bare string literals in their code.
"""

from pathlib import Path

# Absolute path to the project root (three levels above src/server/).
# Used when resolving relative checkpoint paths from models.toml.
PROJECT_ROOT: Path = Path(__file__).parent.parent.parent
"""Absolute path to the project root directory."""

# Model type identifiers.
# Match the value of the ``type`` field in each [[models]] entry of models.toml.

MODEL_TYPE_SAM1: str = "sam1"
"""Model family identifier for Segment Anything Model 1 (Meta, ViT-based)."""

MODEL_TYPE_SAM2: str = "sam2"
"""Model family identifier for Segment Anything Model 2 / 2.1 (Meta, Hiera-based)."""

MODEL_TYPE_SAM3: str = "sam3"
"""Model family identifier for Segment Anything Model 3 (Meta, transformer-based)."""

MODEL_TYPE_YOLOE: str = "yoloe"
"""Model family identifier for YOLOE open-vocabulary detection + segmentation (Ultralytics)."""

MODEL_TYPE_GROUNDING_DINO: str = "grounding_dino"
"""Model family identifier for Grounding DINO + SAM auto-annotation (autodistill)."""

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

# Compute device names used when selecting CPU vs GPU backends.

DEVICE_CUDA: str = "cuda"
"""CUDA GPU device identifier passed to torch and model loading functions."""

DEVICE_CPU: str = "cpu"
"""CPU device identifier used as the fallback when no CUDA GPU is available."""

# TCP command strings matched against the ``cmd`` field in incoming requests.

CMD_PING: str = "ping"
"""Command: check server liveness; no model required."""

CMD_LIST_MODELS: str = "list_models"
"""Command: retrieve the list of all configured models and their availability."""

CMD_SET_MODEL: str = "set_model"
"""Command: load a different SAM model by id."""

CMD_SET_IMAGE: str = "set_image"
"""Command: set the current image for subsequent segmentation calls."""

CMD_PREDICT: str = "predict"
"""Command: run point-prompted mask prediction on the current image."""

CMD_PREDICT_TEXT: str = "predict_text"
"""Command: run text-prompted segmentation (SAM 3 only)."""

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
