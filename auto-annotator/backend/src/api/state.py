"""Strongly-typed application state container.

Replaces dynamic app.state with a typed dataclass that ensures
all services are properly initialized at startup. Type checker
verifies all required services are present.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.annotation_lifecycle import AnnotationLifecycle
    from src.db.repository import Repository
    from src.gallery_cache import AnnotationCache
    from src.job_manager import JobManager
    from src.label_store import LabelStore
    from src.models import AppContext
    from src.services import AnnotationService, GalleryService, SegmentationService
    from src.validation import Validator


@dataclass(frozen=True)
class AppState:
    """Immutable container for all application state.

    All services initialized in lifespan context manager.
    Type-safe access; no app.state.ctx and # type: ignore needed.
    """

    ctx: AppContext
    repository: Repository
    annotation_cache: AnnotationCache
    label_store: LabelStore
    annotation_service: AnnotationService
    gallery_service: GalleryService
    segmentation_service: SegmentationService
    lifecycle: AnnotationLifecycle
    job_manager: JobManager
    validator: Validator
