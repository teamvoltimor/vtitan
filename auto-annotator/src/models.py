"""src.models – Application dataclasses stored in gr.State or returned from helpers.

All mutable session state, annotation data, inference context, and structured
return values are represented as dataclasses so callers receive typed objects
instead of raw tuples or plain dicts.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

if TYPE_CHECKING:
    import numpy as np


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
    ) -> tuple[list, list, object]:
        """Run point-prompted mask prediction.

        Args:
            coords:     Float32 array of shape ``(N, 2)``.
            labels:     Int32 array of shape ``(N,)``; 1=positive, 0=negative.
            mask_input: Optional logit mask for iterative refinement.

        Returns:
            Three-tuple ``(masks, scores, logits)``.
        """
        ...

    def list_models(self) -> list[dict]:
        """Return descriptors for all configured models.

        Returns:
            List of model descriptor dicts.
        """
        ...

    def set_model(self, model_id: str) -> dict:
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


@dataclass(frozen=True)
class ClassInfo:
    """A single annotation class, mirroring a row from the ``classes`` DB table.

    Attributes:
        id:    Database primary key.
        name:  Unique class name (e.g. ``"red_prism"``).
        color: CSS hex colour string (e.g. ``"#ee2737"``).
    """

    id: int
    name: str
    color: str


@dataclass(frozen=True)
class Point:
    """A single click point added to the annotation buffer.

    Attributes:
        x:        Pixel x-coordinate on the displayed canvas.
        y:        Pixel y-coordinate on the displayed canvas.
        label:    1 = positive (include), 0 = negative (exclude).
        class_id: DB id of the class active at click time.
    """

    x: int
    y: int
    label: int
    class_id: int


@dataclass
class Annotation:
    """One accepted annotation consisting of a mask, polygon, and class metadata.

    Attributes:
        class_db_id:   Database id of the associated class.
        class_name:    Human-readable class name.
        class_color:   CSS hex colour string for rendering.
        yolo_class_id: 0-based YOLO class index (derived from class order in DB).
        polygon:       Flat normalised YOLO polygon coords ``[x0, y0, x1, y1, ...]``.
        bbox:          Normalised bounding box ``[xc, yc, w, h]``.
        mask:          Boolean H×W numpy array of the accepted mask.
    """

    class_db_id: int
    class_name: str
    class_color: str
    yolo_class_id: int
    polygon: list[float]
    bbox: list[float]
    mask: np.ndarray


@dataclass(frozen=True)
class ImageRecord:
    """A row from the ``images`` DB table.

    Attributes:
        id:          Database primary key.
        path:        Absolute path to the image file.
        status:      Integer status code (see :class:`src.enums.Status`).
        format_used: Export format used when the image was last saved, or ``None``.
        updated_at:  ISO-8601 timestamp of the last status change, or ``None``.
    """

    id: int
    path: str
    status: int
    format_used: str | None
    updated_at: str | None


@dataclass
class InferenceResult:
    """Structured result returned by :func:`src.inference.run_sam_inference`.

    Attributes:
        masks:      List of boolean H×W mask arrays (one per granularity level),
                    or ``None`` on error.
        best_idx:   Index into *masks* of the highest-confidence mask.
        scores_str: Human-readable score string, e.g. ``"Precise: 0.92  Object: 0.88"``.
        error:      Non-empty error message on failure; empty string on success.
    """

    masks: list[np.ndarray] | None
    best_idx: int
    scores_str: str
    error: str

    @property
    def ok(self) -> bool:
        """Return ``True`` when inference succeeded (no error)."""
        return not self.error


@dataclass(frozen=True)
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


@dataclass(frozen=True)
class BrowseRow:
    """A single row in the browse-view dataframe, as returned by :func:`src.db.get_all_images`.

    Attributes:
        id:         Database primary key.
        filename:   Image file name (basename only, not full path).
        status:     Human-readable status string: ``"pending"``, ``"done"``, or ``"skipped"``.
        format:     Export format string (``"seg"``, ``"det"``, or ``""`` if not yet saved).
        updated_at: ISO-8601 timestamp of the last status change, or ``""`` if never updated.
    """

    id: int
    filename: str
    status: str
    format: str
    updated_at: str


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

    Passed through UI closures and handler functions instead of module-level
    global variables.  Replaces the ``_client_ref: list`` pattern.

    Attributes:
        client:    Active :class:`~src.sam_client.ModelServerClient`, or ``None`` when
                   running without a model server.
        inference: Local inference context (predictor, torch, OOM class, etc.).
    """

    client: SAMClientProtocol | None = None
    inference: InferenceContext = field(default_factory=InferenceContext)


@dataclass
class AppState:
    """All mutable per-session state stored in a Gradio ``gr.State``.

    Stored server-side via pickle; numpy arrays are fully supported.

    Attributes:
        classes:             List of annotation classes loaded from the DB.
        outline_color:       Current outline colour mode (see ``OUTLINE_MODES``).
        active_model_id:     ID of the currently loaded SAM model, or ``None``.
        model_supports_text: Whether the active model supports text-prompted segmentation.
        current_image_id:    DB id of the image currently displayed, or ``None``.
        current_image:       RGB numpy array of the current image, or ``None``.
        image_set:           Whether SAM has been initialised with the current image.
        point_buffer:        Ordered list of click points added since the last accept.
        pending_mask:        The mask currently proposed for acceptance, or ``None``.
        pending_masks:       All granularity masks from the last SAM call.
        pending_logits:      Raw SAM logits fed back as ``mask_input`` on the next call.
        pending_mask_idx:    Index of the selected mask in *pending_masks*.
        pending_class_db_id: DB id of the class assigned to the in-progress annotation.
        annotations:         List of all accepted annotations for the current image.
        log_entries:         Timestamped log lines shown in the status textbox (newest first).
    """

    classes: list[ClassInfo] = field(default_factory=list)
    outline_color: str = "Class color"
    active_model_id: str | None = None
    model_supports_text: bool = False
    current_image_id: int | None = None
    current_image: np.ndarray | None = None
    image_set: bool = False
    point_buffer: list[Point] = field(default_factory=list)
    pending_mask: np.ndarray | None = None
    pending_masks: list[np.ndarray] = field(default_factory=list)
    pending_logits: np.ndarray | None = None
    pending_mask_idx: int = 0
    pending_class_db_id: int | None = None
    annotations: list[Annotation] = field(default_factory=list)
    log_entries: list[str] = field(default_factory=list)
