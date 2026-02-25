"""src.models – Application dataclasses stored in gr.State."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class ClassInfo:
    """A single annotation class (mirrors the `classes` DB table row)."""

    id: int
    name: str
    color: str  # hex string e.g. "#ee2737"

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "ClassInfo":
        return cls(id=d["id"], name=d["name"], color=d["color"])


@dataclass
class Point:
    """A single click point in the annotation buffer."""

    x: int
    y: int
    label: int        # 1 = positive, 0 = negative
    class_id: int     # DB id of the class this point belongs to


@dataclass
class Annotation:
    """One accepted annotation (mask + polygon + class metadata)."""

    class_db_id: int
    class_name: str
    class_color: str
    yolo_class_id: int
    polygon: list[float]
    bbox: list[float]
    mask: np.ndarray   # bool H×W


@dataclass
class ImageRecord:
    """A row from the `images` DB table."""

    id: int
    path: str
    status: int
    format_used: str | None
    updated_at: str | None


@dataclass
class AppState:
    """All mutable per-session state stored in a Gradio gr.State."""

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
