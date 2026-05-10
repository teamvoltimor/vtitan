"""Image serving and management endpoints."""

from __future__ import annotations

import mimetypes
from pathlib import Path

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse

from src.api.dependencies import GalleryServiceDep, ImageRecordDep, RepositoryDep
from src.api.schemas import DeleteImagesRequest, GalleryResponse
from src.utils import get_logger

logger = get_logger(__name__)

router = APIRouter()


@router.get("/{image_id}")
def serve_image(image_record: ImageRecordDep) -> FileResponse:
    """Stream the raw image file for the given database id."""
    media_type, _ = mimetypes.guess_type(image_record.path)
    return FileResponse(image_record.path, media_type=media_type)


@router.post("/delete", response_model=GalleryResponse)
def delete_images(payload: DeleteImagesRequest, repository: RepositoryDep, gallery_service: GalleryServiceDep) -> GalleryResponse:
    """Delete images by id and return the updated gallery."""
    if not payload.imageIds:
        raise HTTPException(status_code=400, detail="No images to delete")

    for image_id in payload.imageIds:
        record = repository.images.get_by_id(image_id)
        if record is None:
            raise HTTPException(status_code=404, detail="Image not found")
        Path(record.path).unlink(missing_ok=True)
        repository.images.delete(image_id)

    logger.info("images_deleted", extra={"count": len(payload.imageIds)})
    return gallery_service.build_gallery_response()
