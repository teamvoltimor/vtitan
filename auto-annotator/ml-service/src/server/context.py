"""src.server.context – Shared mutable server state (predictor, config)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Protocol

from pydantic import BaseModel, Field

if TYPE_CHECKING:
    import numpy as np

from src.server.registry import ModelConfig


class TextSegmentationResult(BaseModel):
    """Per-class result from a text-prompted segmentation call.

    Attributes:
        class_name: Class name string used as the text prompt.
        masks:      List of boolean H×W mask arrays, one per detected instance.
        scores:     List of confidence scores, one per mask.
        boxes:      List of ``[x1, y1, x2, y2]`` bounding boxes in pixels.
        error:      Optional error string if segmentation failed for this class.
    """

    class_name: str
    masks: list[Any] = Field(default_factory=list)
    scores: list[float] = Field(default_factory=list)
    boxes: list[list[float]] = Field(default_factory=list)
    error: str = ""

    model_config = {"arbitrary_types_allowed": True}


class PointPredictor(Protocol):
    """Structural interface for a loaded point-prompted predictor (SAM 1/2/3).

    Every ``ctx.predictor`` backend (``sam1.py``, ``sam2.py``, ``sam3.py``,
    and the point-prediction-unsupported ``_NoopPredictor`` in ``yolo11.py``/
    ``yoloe.py``) implements this pair, matching the wire commands dispatched
    in ``dispatch.py`` (``handle_set_image``/``handle_predict``).
    """

    def set_image(self, image: np.ndarray) -> None:
        """Encode *image* for subsequent :meth:`predict` calls."""
        ...

    def predict(
        self,
        point_coords: np.ndarray,
        point_labels: np.ndarray,
        mask_input: np.ndarray | None = None,
        multimask_output: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, Any]:
        """Run point-prompted mask prediction on the encoded image."""
        ...


class TextSegmenter(Protocol):
    """Structural interface for a loaded text-prompted segmenter (SAM3/YOLOE)."""

    def segment_by_text(self, image: np.ndarray, class_names: list[str]) -> list[TextSegmentationResult]:
        """Run text-prompted segmentation for each name in *class_names*."""
        ...


@dataclass
class ServerContext:
    """Holds all mutable state for the model server."""

    models_config: list[ModelConfig] = field(default_factory=list)
    predictor: PointPredictor | None = None
    text_seg: TextSegmenter | None = None
    model_id: str | None = None
    device: str = "cpu"
