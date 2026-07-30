"""src.models – Application dataclasses for request state and inference results.

All mutable per-request state, inference context, and structured return values
are represented as dataclasses so callers receive typed objects instead of raw
tuples or plain dicts.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Literal, Protocol, runtime_checkable

if TYPE_CHECKING:
    import numpy as np

    from src.coordinates import NormalizedPoint
    from src.enums import Status
    from src.types import ClassId, ImageId, ModelId


@runtime_checkable
class SAMClientProtocol(Protocol):
    """Structural interface for model-server clients (TCP or local).

    Both :class:`src.sam_client.ModelServerClient` and any local inference
    adapter must satisfy this Protocol so :class:`AppContext` can hold either
    without coupling to a concrete type.
    """

    def ping(self) -> bool:
        """Return ``True`` when the backend is reachable and responding."""
        ...

    def set_image(self, image: np.ndarray) -> None:
        """Encode *image* for subsequent predictions.

        Args:
            image: RGB uint8 numpy array.
        """
        ...

    def predict(
        self,
        coords: np.ndarray,
        labels: np.ndarray,
        mask_input: np.ndarray | None = None,
    ) -> tuple[list[np.ndarray], list[float], np.ndarray | None]:
        """Run point-prompted mask prediction.

        Args:
            coords:     Float32 array of shape ``(N, 2)``.
            labels:     Int32 array of shape ``(N,)``; 1=positive, 0=negative.
            mask_input: Optional logit mask for iterative refinement.

        Returns:
            Three-tuple ``(masks, scores, logits)`` where masks is a list of
            boolean H×W arrays, scores is a list of confidence floats, and
            logits is the raw SAM output for iterative refinement (or ``None``).
        """
        ...

    def list_models(self) -> list[dict[str, Any]]:
        """Return descriptors for all configured models.

        Returns:
            List of model descriptor dicts.
        """
        ...

    def set_model(self, model_id: ModelId) -> dict[str, Any]:
        """Load a different model by *model_id*.

        Args:
            model_id: Unique model identifier string matching a config entry.

        Returns:
            Response dict (contains ``"ok"`` or ``"error"``).
        """
        ...

    def predict_text(self, image: np.ndarray, class_names: list[str]) -> list[dict[str, Any]]:
        """Run text-prompted segmentation (SAM 3 only).

        Args:
            image:       RGB uint8 numpy array.
            class_names: Class-name strings used as text prompts.

        Returns:
            List of per-class result dicts.
        """
        ...


@runtime_checkable
class ImageRepoProtocol(Protocol):
    """Structural interface for the ``images`` sub-repository of a DB repository."""

    def get_by_id(self, image_id: int) -> ImageRecord:
        """Return the image row for *image_id*."""
        ...

    def register_augmented(self, record: AugmentedImage) -> int:
        """Persist *record* and return the new row's DB id."""
        ...


@runtime_checkable
class ImageRepositoryProtocol(Protocol):
    """Structural interface for the pre-Go-migration DB repository.

    Only satisfied by legacy call paths (:meth:`SegmentationService.segment`,
    :func:`src.augment.run_augmentation_job`) predating the Go API owning the
    database; the gRPC entrypoints resolve paths themselves and never
    construct a real implementation of this Protocol.
    """

    images: ImageRepoProtocol


@dataclass(frozen=True, slots=True)
class ClassInfo:
    """A single annotation class, mirroring a row from the ``classes`` DB table.

    Attributes:
        id:    Database primary key.
        name:  Unique class name (e.g. ``"red_prism"``).
        color: CSS hex colour string (e.g. ``"#ee2737"``).
    """

    id: ClassId
    name: str
    color: str


@dataclass(frozen=True, slots=True)
class Point:
    """A single click point added to the annotation buffer.

    Attributes:
        x:        Pixel x-coordinate on the displayed canvas.
        y:        Pixel y-coordinate on the displayed canvas.
        label:    ``1`` = positive (include), ``0`` = negative (exclude).
        class_id: DB id of the class active at click time.
    """

    x: int
    y: int
    label: Literal[0, 1]
    class_id: ClassId


@dataclass(frozen=True, slots=True)
class ClickPoint:
    """A click point from the external API (normalised coords, string identifiers).

    Attributes:
        x:           Normalised x-coordinate (``0.0``–``1.0``).
        y:           Normalised y-coordinate (``0.0``–``1.0``).
        point_type:  ``"positive"`` or ``"negative"``.
        class_name:  Class name string for this click.
    """

    x: float
    y: float
    point_type: str
    class_name: str


@dataclass(frozen=True, slots=True)
class ImageRecord:
    """A row from the ``images`` DB table.

    Attributes:
        id:          Database primary key.
        path:        Absolute path to the image file.
        status:      Integer status code (see :class:`src.enums.Status`).
        format_used: Export format used when the image was last saved, or ``None``.
        updated_at:  ISO-8601 timestamp of the last status change, or ``None``.
        parent_id:   FK to parent image id (None for originals), or ``None``.
    """

    id: ImageId
    path: str
    status: Status
    format_used: str | None
    updated_at: str | None
    parent_id: int | None = None


@dataclass(frozen=True, slots=True)
class InferenceResult:
    """Structured result returned by :func:`src.inference.run_sam_inference`.

    Attributes:
        masks:      Tuple of boolean H×W mask arrays (one per granularity level),
                    or ``None`` on error.
        best_idx:   Index into *masks* of the highest-confidence mask.
        scores_str: Human-readable score string, e.g. ``"Precise: 0.92  Object: 0.88"``.
        error:      Non-empty error message on failure; empty string on success.
    """

    masks: tuple[np.ndarray, ...] | None
    best_idx: int
    scores_str: str
    error: str

    @property
    def ok(self) -> bool:
        """Return ``True`` when inference succeeded (no error)."""
        return not self.error


@dataclass(frozen=True, slots=True)
class AugmentedImage:
    """A domain value object representing a newly generated augmented image.

    Used to pass generation results across the persistence port without
    leaking raw paths and scalar values.

    Attributes:
        path:        Absolute path to the newly saved augmented image.
        format_used: The export format inherited from the parent.
        parent_id:   The ID of the parent image this was generated from.
    """

    path: str
    format_used: str
    parent_id: int


@dataclass
class InferenceContext:
    """Mutable local-inference state that replaces module-level globals in src.inference.

    An instance is created once in :class:`AppContext` and passed through to
    :func:`src.inference.initialize_inference` and
    :func:`src.inference.run_sam_inference` so no module-level state is needed.

    Attributes:
        predictor:    SAM2ImagePredictor or ultralytics SAM instance, or ``None`` before load.
        use_native:   ``True`` when the native SAM2 package backend is active.
        oom_error:    ``torch.cuda.OutOfMemoryError`` class, or ``None`` on CPU.
        torch_module: The imported ``torch`` module (lazy-loaded), or ``None`` before init.
    """

    predictor: Any = None
    use_native: bool = False
    oom_error: type | None = None
    torch_module: Any = None


@dataclass
class AppContext:
    """Application-level context holding the server client and local inference state.

    Passed into FastAPI endpoint handlers instead of module-level global variables.

    Attributes:
        client:    Active :class:`~src.sam_client.ModelServerClient`, or ``None`` when
                   running without a model server.
        inference: Local inference context (predictor, torch, OOM class, etc.).
    """

    client: SAMClientProtocol | None = None
    inference: InferenceContext = field(default_factory=InferenceContext)


@dataclass
class InferenceRequest:
    """Minimal input for a single SAM inference call.

    Replaces the HTTP-layer AppState as the inference boundary, keeping
    run_sam_inference free of request-state coupling and mutation.

    Attributes:
        image:  RGB uint8 numpy array of the image to segment.
        points: Ordered list of click points (pixel coords, label 0/1).
    """

    image: np.ndarray
    points: list[Point]


@dataclass(frozen=True, slots=True)
class Shape:
    """A single annotation shape (polygon or bounding box).

    Attributes:
        id:         Shape identifier (e.g., "loaded-1-0" or "mask-2-1").
        class_name: Name of the class this shape belongs to.
        points:     List of normalized points defining the shape.
    """

    id: str
    class_name: str
    points: list[NormalizedPoint]
