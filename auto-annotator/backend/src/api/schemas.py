"""API request and response schemas (Pydantic models).

All request/response DTOs are defined here, at the API boundary.
Domain objects (models.py) remain separate for internal use.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel

# Gallery & Image Schemas

class GalleryStats(BaseModel):
    """Aggregate image status counts for the gallery overview."""

    pending: int
    done: int
    skipped: int
    total: int
    pct: float


class SegmentationShape(BaseModel):
    """A single mask or bounding-box shape from inference."""

    id: str
    className: str
    points: list[NormalizedPoint]


class GalleryItem(BaseModel):
    """A single image entry in the gallery listing."""

    id: int
    label: str
    src: str
    format: str
    status: str
    updated: str
    annotations: list[SegmentationShape]


class GalleryResponse(BaseModel):
    """Full gallery payload returned by ``GET /gallery``."""

    items: list[GalleryItem]
    stats: GalleryStats


class GroupedGalleryItem(BaseModel):
    """A class group in the grouped gallery view."""

    className: str
    count: int
    images: list[GalleryItem]


class GroupedGalleryResponse(BaseModel):
    """Gallery grouped by class returned by ``GET /gallery/grouped``."""

    groups: list[GroupedGalleryItem]
    stats: GalleryStats


class ParentImageItem(BaseModel):
    """A parent (original) image with augmentation count."""

    id: int
    label: str
    src: str
    format: str
    status: str
    updated_at: str
    aug_count: int


# Segmentation Schemas

class NormalizedPoint(BaseModel):
    """A 2-D point with coordinates normalised to the range [0, 1]."""

    x: float
    y: float


class SegmentationPoint(BaseModel):
    """A click point sent by the frontend for SAM inference."""

    x: float
    y: float
    pointType: Literal["positive", "negative"]
    className: str


class SegmentationRequest(BaseModel):
    """Payload for ``POST /segment``."""

    imageId: int
    points: list[SegmentationPoint]


class SegmentationResponse(BaseModel):
    """Response envelope for ``POST /segment``."""

    state: Literal["idle", "pending", "ready", "error"]
    message: str
    shapes: list[SegmentationShape]


# Annotation Schemas

class SaveAnnotationsRequest(BaseModel):
    """Payload for ``POST /save``."""

    imageId: int
    shapes: list[SegmentationShape]
    exportFormat: Literal["segmentation", "detection"]


class SkipRequest(BaseModel):
    """Payload for ``POST /skip``."""

    imageId: int


# Class Schemas

class ClassItem(BaseModel):
    """A single annotation class."""

    id: int
    name: str
    color: str


class UpsertClassRequest(BaseModel):
    """Payload for ``POST /classes``."""

    name: str
    color: str


# Model Schemas

class ModelItem(BaseModel):
    """A single SAM model available on the server."""

    id: str
    label: str
    model_type: str
    available: bool
    active: bool
    supports_text: bool


# Job Schemas

class JobStatusResponse(BaseModel):
    """Status response for long-running jobs (augmentation, training)."""

    running: bool
    message: str = ""


# Import Request Schemas

class DeleteImagesRequest(BaseModel):
    """Payload for ``DELETE /images``."""

    imageIds: list[int]


# Augmentation Schemas

class AugmentRequest(BaseModel):
    """Payload for ``POST /augment/start``."""

    imageIds: list[int]
    numAugmentations: int = 9


# Training Schemas

class TrainRequest(BaseModel):
    """Payload for ``POST /train/start``."""

    modelName: str = "yolo11s.pt"
    epochs: int = 50
    batch: int = 16
    imgsz: int = 640
