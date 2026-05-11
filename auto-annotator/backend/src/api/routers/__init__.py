"""API routers grouped by domain."""

from src.api.routers.annotations import router as annotations_router
from src.api.routers.augmentation import router as augmentation_router
from src.api.routers.classes import router as classes_router
from src.api.routers.gallery import router as gallery_router
from src.api.routers.health import router as health_router
from src.api.routers.images import router as images_router
from src.api.routers.models import router as models_router
from src.api.routers.segmentation import router as segmentation_router
from src.api.routers.training import router as training_router

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
