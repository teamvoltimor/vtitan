"""src.server.responses – Typed response Pydantic models for the dispatch layer.

Each handler in :mod:`src.server.dispatch` returns one of these models.
The :func:`src.server.dispatch.dispatch` function calls ``.model_dump()`` on the
result before pickling it and sending it back over the TCP socket.  This keeps
handler logic typed and testable while preserving the wire-protocol contract.
"""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field

from src.enums import ComputeDevice, ModelType
from src.server.constants import (
    RESP_KEY_ERROR,
    RESP_KEY_MODEL_ID,
    RESP_KEY_OK,
    RESP_KEY_RESULTS,
)
from src.server.context import TextSegmentationResult


class PingResponse(BaseModel):
    """Response to the ``ping`` command.

    Attributes:
        device:       Active compute device (ComputeDevice enum value).
        model_loaded: Whether a predictor is currently loaded.
        model_id:     Identifier of the active model, or ``None`` if none is loaded.
    """

    ok: bool = True
    device: ComputeDevice = ComputeDevice.CPU
    model_loaded: bool = False
    model_id: str | None = None


class ModelDescriptor(BaseModel):
    """A single model entry in the list-models response.

    Attributes:
        id:            Unique model identifier.
        label:         Human-readable display label.
        model_type:    Model family (ModelType enum value).
        available:     Whether the checkpoint can be loaded.
        active:        Whether this is the currently loaded model.
        supports_text: Whether the model supports text-prompted segmentation.
    """

    id: str
    label: str
    model_type: ModelType = Field(alias="type")
    available: bool = False
    active: bool = False
    supports_text: bool = False


class ListModelsResponse(BaseModel):
    """Response to the ``list_models`` command.

    Attributes:
        models: List of :class:`ModelDescriptor` instances for each configured model.
    """

    models: list[ModelDescriptor] = Field(default_factory=list)


class SetModelResponse(BaseModel):
    """Response to the ``set_model`` command.

    Attributes:
        model_id: Identifier of the newly loaded model.
        error:    Non-empty error string on failure; empty on success.
    """

    model_id: str = ""
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


class SetImageResponse(BaseModel):
    """Response to the ``set_image`` command (always successful if no exception)."""

    ok: bool = True


class PredictResponse(BaseModel):
    """Response to the ``predict`` command.

    Attributes:
        masks:  List of boolean H×W mask arrays, one per granularity level.
        scores: List of confidence scores, one per mask.
        logits: Raw SAM logit tensor for iterative refinement.
    """

    masks: list[Any] = Field(default_factory=list)
    scores: list[float] = Field(default_factory=list)
    logits: Any = None

    model_config = {"arbitrary_types_allowed": True}


class PredictTextResponse(BaseModel):
    """Response to the ``predict_text`` command.

    Attributes:
        results: List of per-class segmentation result models.
        error:   Non-empty error string on failure; empty on success.
    """

    results: list[TextSegmentationResult] = Field(default_factory=list)
    error: str = ""

    @property
    def ok(self) -> bool:
        """Return ``True`` when text-segmentation succeeded."""
        return not self.error

    def to_dict(self) -> dict:
        """Serialise to a wire-protocol response dict."""
        if self.error:
            return {RESP_KEY_ERROR: self.error}
        return {RESP_KEY_RESULTS: [r.model_dump() for r in self.results]}


class ErrorResponse(BaseModel):
    """Generic error response returned for unknown commands or missing models.

    Attributes:
        error: Human-readable description of the failure.
    """

    error: str
