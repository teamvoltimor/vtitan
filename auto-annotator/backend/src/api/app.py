"""FastAPI application factory.

Initializes the application with middleware, dependency injection,
and includes domain-based routers under /api/v1/ prefix.
"""

from __future__ import annotations

import asyncio
import os
from contextlib import asynccontextmanager
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import AsyncGenerator

from fastapi import FastAPI
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from starlette.exceptions import HTTPException as StarletteHTTPException

from src import db
from src.annotation_lifecycle import AnnotationLifecycle
from src.api.exception_handlers import http_exception_handler, validation_exception_handler
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
from src.config import AppConfig
from src.constants import API_V1_PREFIX
from src.db.repository import DefaultRepository
from src.gallery_cache import AnnotationCache
from src.inference import initialize_inference
from src.job_manager import JobManager
from src.label_store import LabelStore
from src.model_server import connect_to_model_server
from src.models import AppContext
from src.services import AnnotationService, GalleryService, SegmentationService
from src.utils import get_logger
from src.validation import DefaultValidator

logger = get_logger(__name__)


@asynccontextmanager
async def _lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """Initialize shared resources on startup and clean up on shutdown."""
    config = AppConfig.load()

    ctx = AppContext(client=connect_to_model_server())
    initialize_inference(ctx.client, ctx.inference)
    db.init_db(config.paths.db_path, config.paths.pending_dir, config.paths.labels_dir, config.paths.images_dir)

    repository = DefaultRepository(config.paths.db_path)

    # Annotation cache and label store (single instances, shared across all routers)
    annotation_cache = AnnotationCache()
    label_store = LabelStore(annotation_cache)

    # Services (single instances, shared across all routers)
    annotation_service = AnnotationService(label_store)
    gallery_service = GalleryService(repository, annotation_service)
    segmentation_service = SegmentationService(repository)

    # Job manager for async tasks
    job_manager = JobManager(asyncio.get_running_loop())

    # Validator (single instance)
    validator = DefaultValidator(repository)

    # Annotation lifecycle (owns PENDING → DONE/SKIPPED transitions)
    lifecycle = AnnotationLifecycle(
        repository=repository,
        cache=annotation_cache,
        images_dir=config.paths.images_dir,
        labels_dir=config.paths.labels_dir,
        data_yaml_path=config.paths.data_yaml_path,
    )

    # Create strongly-typed app state container
    app.state.app_state = AppState(
        ctx=ctx,
        repository=repository,
        annotation_cache=annotation_cache,
        label_store=label_store,
        annotation_service=annotation_service,
        gallery_service=gallery_service,
        segmentation_service=segmentation_service,
        lifecycle=lifecycle,
        job_manager=job_manager,
        validator=validator,
    )

    yield


app = FastAPI(title="Auto-Annotator HTTP API", lifespan=_lifespan)

_CORS_ORIGINS = [
    o.strip()
    for o in os.environ.get("CORS_ORIGINS", "http://localhost:5173,http://localhost:3000").split(",")
    if o.strip()
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=_CORS_ORIGINS,
    allow_methods=["*"],
    allow_headers=["*"],
)
app.add_exception_handler(StarletteHTTPException, http_exception_handler)
app.add_exception_handler(RequestValidationError, validation_exception_handler)

app.include_router(health_router, prefix="", tags=["system"])
app.include_router(gallery_router, prefix=f"{API_V1_PREFIX}/gallery", tags=["gallery"])
app.include_router(images_router, prefix=f"{API_V1_PREFIX}/images", tags=["images"])
app.include_router(annotations_router, prefix=f"{API_V1_PREFIX}/annotations", tags=["annotations"])
app.include_router(segmentation_router, prefix=API_V1_PREFIX, tags=["segmentation"])
app.include_router(classes_router, prefix=f"{API_V1_PREFIX}/classes", tags=["classes"])
app.include_router(models_router, prefix=f"{API_V1_PREFIX}/models", tags=["models"])
app.include_router(augmentation_router, prefix=f"{API_V1_PREFIX}/augment", tags=["augmentation"])
app.include_router(training_router, prefix=f"{API_V1_PREFIX}/train", tags=["training"])
