"""FastAPI dependency injection functions.

Provides strongly-typed access to application state (services, repositories, caches)
via the AppState container. No more app.state.field or # type: ignore.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Annotated

from fastapi import Depends, HTTPException, Request

from src.db.repository import Repository
from src.gallery_cache import AnnotationCache
from src.job_manager import JobManager
from src.models import AppContext, ImageRecord
from src.services import AnnotationService, GalleryService, SegmentationService
from src.validation import Validator

if TYPE_CHECKING:
    from src.api.state import AppState


def get_app_state(request: Request) -> AppState:
    """FastAPI dependency: return the typed application state container.

    Args:
        request: FastAPI request object.

    Returns:
        AppState with all services, repositories, and caches.
    """
    return request.app.state.app_state  # type: ignore[no-any-return]


def get_app_context(app_state: Annotated[AppState, Depends(get_app_state)]) -> AppContext:
    """FastAPI dependency: return the application-level inference context."""
    return app_state.ctx


def get_job_manager(app_state: Annotated[AppState, Depends(get_app_state)]) -> JobManager:
    """FastAPI dependency: return the job manager."""
    return app_state.job_manager


def get_repository(app_state: Annotated[AppState, Depends(get_app_state)]) -> Repository:
    """FastAPI dependency: return the repository."""
    return app_state.repository


def get_annotation_cache(app_state: Annotated[AppState, Depends(get_app_state)]) -> AnnotationCache:
    """FastAPI dependency: return the annotation cache."""
    return app_state.annotation_cache


def get_annotation_service(app_state: Annotated[AppState, Depends(get_app_state)]) -> AnnotationService:
    """FastAPI dependency: return the annotation service."""
    return app_state.annotation_service


def get_gallery_service(app_state: Annotated[AppState, Depends(get_app_state)]) -> GalleryService:
    """FastAPI dependency: return the gallery service."""
    return app_state.gallery_service


def get_segmentation_service(app_state: Annotated[AppState, Depends(get_app_state)]) -> SegmentationService:
    """FastAPI dependency: return the segmentation service."""
    return app_state.segmentation_service


def get_validator(app_state: Annotated[AppState, Depends(get_app_state)]) -> Validator:
    """FastAPI dependency: return the validator."""
    return app_state.validator


def validate_image_id(repository: Annotated[Repository, Depends(get_repository)], image_id: int) -> ImageRecord:
    """FastAPI dependency: retrieve image record by ID or raise 404."""
    record = repository.images.get_by_id(image_id)
    if record is None:
        raise HTTPException(status_code=404, detail="Image not found")
    return record


AppContextDep = Annotated[AppContext, Depends(get_app_context)]
RepositoryDep = Annotated[Repository, Depends(get_repository)]
AnnotationCacheDep = Annotated[AnnotationCache, Depends(get_annotation_cache)]
AnnotationServiceDep = Annotated[AnnotationService, Depends(get_annotation_service)]
GalleryServiceDep = Annotated[GalleryService, Depends(get_gallery_service)]
SegmentationServiceDep = Annotated[SegmentationService, Depends(get_segmentation_service)]
JobManagerDep = Annotated[JobManager, Depends(get_job_manager)]
ValidatorDep = Annotated[Validator, Depends(get_validator)]
ImageRecordDep = Annotated[ImageRecord, Depends(validate_image_id)]
