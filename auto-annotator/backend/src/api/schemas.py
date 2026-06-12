"""API request and response schemas (Pydantic models).

All request/response DTOs are defined here, at the API boundary.
Domain objects (models.py) remain separate for internal use.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field
from pydantic.alias_generators import to_camel

_BASE = ConfigDict(alias_generator=to_camel, populate_by_name=True)
_REQUEST = ConfigDict(alias_generator=to_camel, populate_by_name=True, extra="forbid")


class AppBaseModel(BaseModel):
    """Base model for all API schemas — camelCase wire format, snake_case Python."""

    model_config = _BASE


class AppRequestModel(BaseModel):
    """Base model for request bodies — adds extra='forbid' to reject unknown fields."""

    model_config = _REQUEST


# Shared primitive


class NormalizedPoint(AppBaseModel):
    """A 2-D point with coordinates normalised to the range [0, 1]."""

    x: float
    y: float


class SegmentationShape(AppBaseModel):
    """A single mask or bounding-box shape from inference."""

    id: str
    class_name: str
    points: list[NormalizedPoint]


# Gallery & Image Schemas


class GalleryStats(AppBaseModel):
    """Aggregate image status counts for the gallery overview."""

    pending: int
    done: int
    skipped: int
    total: int
    pct: float


class GalleryItem(AppBaseModel):
    """A single image entry in the gallery listing."""

    id: int
    label: str
    src: str
    format: str
    status: str
    updated: str
    annotations: list[SegmentationShape]


class GalleryResponse(AppBaseModel):
    """Full gallery payload returned by ``GET /gallery``."""

    items: list[GalleryItem]
    stats: GalleryStats


class GroupedGalleryItem(AppBaseModel):
    """A class group in the grouped gallery view."""

    class_name: str
    count: int
    images: list[GalleryItem]


class GroupedGalleryResponse(AppBaseModel):
    """Gallery grouped by class returned by ``GET /gallery/grouped``."""

    groups: list[GroupedGalleryItem]
    stats: GalleryStats


class ParentImageItem(AppBaseModel):
    """A parent (original) image with augmentation count.

    ``updated_at`` and ``aug_count`` use explicit aliases to preserve the
    existing snake_case wire format consumed by the frontend.
    """

    id: int
    label: str
    src: str
    format: str
    status: str
    updated_at: str = Field(alias="updated_at", serialization_alias="updated_at")
    aug_count: int = Field(alias="aug_count", serialization_alias="aug_count")


# Segmentation Schemas


class SegmentationPoint(AppRequestModel):
    """A click point sent by the frontend for SAM inference."""

    x: float
    y: float
    point_type: Literal["positive", "negative"]
    class_name: str


class SegmentationRequest(AppRequestModel):
    """Payload for ``POST /segment``."""

    image_id: int
    points: list[SegmentationPoint]


class SegmentationResponse(AppBaseModel):
    """Response envelope for ``POST /segment``."""

    state: Literal["idle", "pending", "ready", "error"]
    message: str
    shapes: list[SegmentationShape]


# Annotation Schemas


class SaveAnnotationsRequest(AppRequestModel):
    """Payload for ``POST /save``."""

    image_id: int
    shapes: list[SegmentationShape]
    export_format: Literal["segmentation", "detection"]


class SkipRequest(AppRequestModel):
    """Payload for ``POST /skip``."""

    image_id: int


# Class Schemas


class ClassItem(AppBaseModel):
    """A single annotation class."""

    id: int
    name: str
    color: str


class UpsertClassRequest(AppRequestModel):
    """Payload for ``POST /classes``."""

    name: str
    color: str


# Model Schemas


class ModelItem(AppBaseModel):
    """A single SAM model available on the server."""

    id: str
    label: str
    model_type: str
    available: bool
    active: bool
    supports_text: bool


# Health Schemas


class HealthStatus(AppBaseModel):
    """System health status response."""

    ready: bool
    inference_available: bool
    database_accessible: bool
    message: str = ""


# Job Schemas


class JobStatusResponse(AppBaseModel):
    """Status response for long-running jobs (augmentation, training)."""

    running: bool
    message: str = ""


# Import Request Schemas


class DeleteImagesRequest(AppRequestModel):
    """Payload for ``DELETE /images``."""

    image_ids: list[int]


# Augmentation Schemas


class AugmentRequest(AppRequestModel):
    """Payload for ``POST /augment/start``."""

    image_ids: list[int]
    num_augmentations: int = 9


# Training Schemas


class TrainRequest(AppRequestModel):
    """Payload for ``POST /train/start``."""

    model_name: str = "yolo11s.pt"
    epochs: int = 50
    batch: int = 16
    imgsz: int = 640
