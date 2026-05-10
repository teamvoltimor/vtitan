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

    def list_models(self) -> list[dict]:
        """Return descriptors for all configured models.

        Returns:
            List of model descriptor dicts.
        """
        ...

    def set_model(self, model_id: ModelId) -> dict:
        """Load a different model by *model_id*.

        Args:
            model_id: Unique model identifier string matching a config entry.

        Returns:
            Response dict (contains ``"ok"`` or ``"error"``).
        """
        ...

    def predict_text(self, image: np.ndarray, class_names: list[str]) -> list[dict]:
        """Run text-prompted segmentation (SAM 3 only).

        Args:
            image:       RGB uint8 numpy array.
            class_names: Class-name strings used as text prompts.

        Returns:
            List of per-class result dicts.
        """
        ...


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
class StatsResult:
    """Aggregate image status counts returned by :func:`src.db.get_stats`.

    Attributes:
        pending: Number of images awaiting annotation.
        done:    Number of successfully annotated images.
        skipped: Number of skipped images.
        total:   Total image count (pending + done + skipped).
        pct:     Percentage of images done (0.0 – 100.0, one decimal place).
    """

    pending: int
    done: int
    skipped: int
    total: int
    pct: float


@dataclass(frozen=True, slots=True)
class GroupedRow:
    """A parent image row with augmentation count for grouped views.

    Attributes:
        id:         Database primary key.
        filename:   Image file name (basename only).
        status:     Human-readable status string.
        format:     Export format string.
        updated_at: ISO-8601 timestamp of the last status change.
        path:       Absolute path to the source image file.
        aug_count:  Number of augmented copies of this image.
    """

    id: ImageId
    filename: str
    status: str
    format: str
    updated_at: str
    path: str
    aug_count: int


@dataclass(frozen=True, slots=True)
class BrowseRow:
    """A single row in the browse-view dataframe, as returned by :func:`src.db.get_all_images`.

    Attributes:
        id:         Database primary key.
        filename:   Image file name (basename only, not full path).
        status:     Human-readable status string: ``"pending"``, ``"done"``, or ``"skipped"``.
        format:     Export format string (``"seg"``, ``"det"``, or ``""`` if not yet saved).
        updated_at: ISO-8601 timestamp of the last status change, or ``""`` if never updated.
        path:       Absolute path to the source image file.
    """

    id: ImageId
    filename: str
    status: str
    format: str
    updated_at: str
    path: str


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
class AppState:
    """Per-request inference state passed through SAM inference helpers.

    Attributes:
        classes:             List of annotation classes loaded from the DB.
        current_image_id:    DB id of the image being segmented, or ``None``.
        current_image:       RGB numpy array of the current image, or ``None``.
        image_set:           Whether SAM has been initialised with the current image.
        point_buffer:        Ordered list of click points for the current inference call.
        pending_logits:      Raw SAM logits fed back as ``mask_input`` for iterative refinement.
        pending_mask_idx:    Index into ``pending_logits`` for the selected granularity level.
        pending_class_db_id: DB id of the class assigned to the in-progress annotation.
    """

    classes: list[ClassInfo] = field(default_factory=list)
    current_image_id: ImageId | None = None
    current_image: np.ndarray | None = None
    image_set: bool = False
    point_buffer: list[Point] = field(default_factory=list)
    pending_logits: np.ndarray | None = None
    pending_mask_idx: int = 0
    pending_class_db_id: ClassId | None = None
