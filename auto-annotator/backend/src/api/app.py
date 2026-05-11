"""FastAPI application factory.

Initializes the application with middleware, dependency injection,
and includes domain-based routers under /api/v1/ prefix.
"""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from src import db
from src.api.routers import (
    annotations_router,
    augmentation_router,
    classes_router,
    gallery_router,
    health_router,
    images_router,
    models_router,
    segmentation_router,
    training_router,
)
from src.api.state import AppState
from src.db.repository import DefaultRepository
from src.gallery_cache import AnnotationCache
from src.inference import initialize_inference
from src.job_manager import JobManager
from src.model_server import connect_to_model_server
from src.models import AppContext
from src.services import AnnotationService, GalleryService, SegmentationService
from src.utils import get_logger
from src.validation import DefaultValidator

logger = get_logger(__name__)


@asynccontextmanager
async def _lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """Initialize shared resources on startup and clean up on shutdown."""
    # Application context (inference state)
    ctx = AppContext(client=connect_to_model_server())
    initialize_inference(ctx.client, ctx.inference)
    db.init_db()

    # Repository for data access
    repository = DefaultRepository()

    # Annotation cache (single instance, shared across all routers)
    annotation_cache = AnnotationCache()

    # Services (single instances, shared across all routers)
    annotation_service = AnnotationService(annotation_cache)
    gallery_service = GalleryService(repository, annotation_service)
    segmentation_service = SegmentationService(repository)

    # Job manager for async tasks
    job_manager = JobManager(asyncio.get_running_loop())

    # Validator (single instance)
    validator = DefaultValidator(repository)

    # Create strongly-typed app state container
    app.state.app_state = AppState(
        ctx=ctx,
        repository=repository,
        annotation_cache=annotation_cache,
        annotation_service=annotation_service,
        gallery_service=gallery_service,
        segmentation_service=segmentation_service,
        job_manager=job_manager,
        validator=validator,
    )

    yield


app = FastAPI(title="Auto-Annotator HTTP API", lifespan=_lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(health_router, prefix="", tags=["system"])
app.include_router(gallery_router, prefix="/api/v1/gallery", tags=["gallery"])
app.include_router(images_router, prefix="/api/v1/images", tags=["images"])
app.include_router(annotations_router, prefix="/api/v1/annotations", tags=["annotations"])
app.include_router(segmentation_router, prefix="/api/v1", tags=["segmentation"])
app.include_router(classes_router, prefix="/api/v1/classes", tags=["classes"])
app.include_router(models_router, prefix="/api/v1/models", tags=["models"])
app.include_router(augmentation_router, prefix="/api/v1/augment", tags=["augmentation"])
app.include_router(training_router, prefix="/api/v1/train", tags=["training"])
