"""Gallery listing and import endpoints."""

from __future__ import annotations

import shutil
from pathlib import Path
from uuid import uuid4

from fastapi import APIRouter, File, HTTPException, UploadFile

from src.api.dependencies import GalleryServiceDep, RepositoryDep
from src.api.schemas import GalleryResponse, ParentImageItem
from src.constants import API_PUBLIC_URL, PENDING_DIR
from src.utils import get_logger

logger = get_logger(__name__)

router = APIRouter()


@router.get("/", response_model=GalleryResponse)
def read_gallery(gallery_service: GalleryServiceDep) -> GalleryResponse:
    """Return all images with status counts for the gallery view."""
    return gallery_service.build_gallery_response()


@router.post("/import", response_model=GalleryResponse)
async def upload_gallery_images(
    files: list[UploadFile] = File(...),
    gallery_service: GalleryServiceDep = ...,
    repository: RepositoryDep = ...,
) -> GalleryResponse:
    """Accept uploaded image files, save them to pending/, and return the updated gallery."""
    if not files:
        raise HTTPException(status_code=400, detail="No files provided")

    saved_paths: list[str] = []
    PENDING_DIR.mkdir(parents=True, exist_ok=True)

    for file in files:
        safe_name = Path(file.filename).name
        dest = PENDING_DIR / f"{uuid4().hex}_{safe_name}"
        with dest.open("wb") as buffer:
            shutil.copyfileobj(file.file, buffer)
        saved_paths.append(str(dest))

    if saved_paths:
        repository.images.add_from_paths(saved_paths)

    return gallery_service.build_gallery_response()


@router.get("/grouped", response_model=list[ParentImageItem])
def get_grouped_gallery(repository: RepositoryDep) -> list[ParentImageItem]:
    """Return parent (original) images with their augmentation counts."""
    rows = repository.images.get_grouped()
    return [
        ParentImageItem(
            id=row.id,
            label=row.filename,
            src=f"{API_PUBLIC_URL}/images/{row.id}",
            format=row.format,
            status=row.status,
            updated_at=row.updated_at,
            aug_count=row.aug_count,
        )
        for row in rows
    ]
