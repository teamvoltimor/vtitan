"""src.server.responses – Typed response dataclasses for the dispatch layer.

Each handler in :mod:`src.server.dispatch` returns one of these dataclasses.
The :func:`src.server.dispatch.dispatch` function calls ``.to_dict()`` on the
result before pickling it and sending it back over the TCP socket.  This keeps
handler logic typed and testable while preserving the wire-protocol contract.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from src.server.constants import (
    CFG_KEY_ID,
    CFG_KEY_LABEL,
    CFG_KEY_TYPE,
    RESP_KEY_ACTIVE,
    RESP_KEY_AVAILABLE,
    RESP_KEY_DEVICE,
    RESP_KEY_ERROR,
    RESP_KEY_LOGITS,
    RESP_KEY_MASKS,
    RESP_KEY_MODEL_ID,
    RESP_KEY_MODEL_LOADED,
    RESP_KEY_MODELS,
    RESP_KEY_OK,
    RESP_KEY_RESULTS,
    RESP_KEY_SCORES,
    RESP_KEY_SUPPORTS_TEXT,
)


@dataclass
class PingResponse:
    """Response to the ``ping`` command.

    Attributes:
        device:       Active compute device string (``"cuda"`` or ``"cpu"``).
        model_loaded: Whether a predictor is currently loaded.
        model_id:     Identifier of the active model, or ``None`` if none is loaded.
    """

    device: str
    model_loaded: bool
    model_id: str | None

    def to_dict(self) -> dict:
        """Serialise to a wire-protocol response dict."""
        return {
            RESP_KEY_OK: True,
            RESP_KEY_DEVICE: self.device,
            RESP_KEY_MODEL_LOADED: self.model_loaded,
            RESP_KEY_MODEL_ID: self.model_id,
        }


@dataclass
class ModelDescriptor:
    """A single model entry in the list-models response.

    Attributes:
        id:            Unique model identifier.
        label:         Human-readable display label.
        model_type:    Model family string (``sam1``, ``sam2``, ``sam3``).
        available:     Whether the checkpoint can be loaded.
        active:        Whether this is the currently loaded model.
        supports_text: Whether the model supports text-prompted segmentation.
    """

    id: str
    label: str
    model_type: str
    available: bool
    active: bool
    supports_text: bool

    def to_dict(self) -> dict:
        """Serialise to the dict format expected by the client."""
        return {
            CFG_KEY_ID: self.id,
            CFG_KEY_LABEL: self.label,
            CFG_KEY_TYPE: self.model_type,
            RESP_KEY_AVAILABLE: self.available,
            RESP_KEY_ACTIVE: self.active,
            RESP_KEY_SUPPORTS_TEXT: self.supports_text,
        }


@dataclass
class ListModelsResponse:
    """Response to the ``list_models`` command.

    Attributes:
        models: List of :class:`ModelDescriptor` instances for each configured model.
    """

    models: list[ModelDescriptor] = field(default_factory=list)

    def to_dict(self) -> dict:
        """Serialise to a wire-protocol response dict."""
        return {RESP_KEY_MODELS: [m.to_dict() for m in self.models]}


@dataclass
class SetModelResponse:
    """Response to the ``set_model`` command.

    Attributes:
        model_id: Identifier of the newly loaded model.
        error:    Non-empty error string on failure; empty on success.
    """

    model_id: str
    error: str = ""

    @property
    def ok(self) -> bool:
        """Return ``True`` when the model was loaded successfully."""
        return not self.error

    def to_dict(self) -> dict:
        """Serialise to a wire-protocol response dict."""
        if self.error:
            return {RESP_KEY_ERROR: self.error}
        return {RESP_KEY_OK: True, RESP_KEY_MODEL_ID: self.model_id}


@dataclass
class SetImageResponse:
    """Response to the ``set_image`` command (always successful if no exception)."""

    def to_dict(self) -> dict:
        """Serialise to a wire-protocol response dict."""
        return {RESP_KEY_OK: True}


@dataclass
class PredictResponse:
    """Response to the ``predict`` command.

    Attributes:
        masks:  List of boolean H×W mask arrays, one per granularity level.
        scores: List of confidence scores, one per mask.
        logits: Raw SAM logit tensor for iterative refinement.
    """

    masks: list[Any]
    scores: list[float]
    logits: Any

    def to_dict(self) -> dict:
        """Serialise to a wire-protocol response dict."""
        return {
            RESP_KEY_MASKS: self.masks,
            RESP_KEY_SCORES: self.scores,
            RESP_KEY_LOGITS: self.logits,
        }


@dataclass
class PredictTextResponse:
    """Response to the ``predict_text`` command.

    Attributes:
        results: List of per-class segmentation result dicts.
        error:   Non-empty error string on failure; empty on success.
    """

    results: list[dict] = field(default_factory=list)
    error: str = ""

    @property
    def ok(self) -> bool:
        """Return ``True`` when text-segmentation succeeded."""
        return not self.error

    def to_dict(self) -> dict:
        """Serialise to a wire-protocol response dict."""
        if self.error:
            return {RESP_KEY_ERROR: self.error}
        return {RESP_KEY_RESULTS: self.results}


@dataclass
class ErrorResponse:
    """Generic error response returned for unknown commands or missing models.

    Attributes:
        error: Human-readable description of the failure.
    """

    error: str

    def to_dict(self) -> dict:
        """Serialise to a wire-protocol response dict."""
        return {RESP_KEY_ERROR: self.error}
