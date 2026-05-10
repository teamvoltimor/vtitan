"""API routers grouped by domain."""

from .annotations import router as annotations_router
from .augmentation import router as augmentation_router
from .classes import router as classes_router
from .gallery import router as gallery_router
from .health import router as health_router
from .images import router as images_router
from .models import router as models_router
from .segmentation import router as segmentation_router
from .training import router as training_router

__all__ = [
    "annotations_router",
    "augmentation_router",
    "classes_router",
    "gallery_router",
    "health_router",
    "images_router",
    "models_router",
    "segmentation_router",
    "training_router",
]
