"""Annotation loading and saving endpoints."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from fastapi import APIRouter, HTTPException

from src.api.schemas import (
    GalleryResponse,
    SaveAnnotationsRequest,
    SegmentationShape,
    SkipRequest,
)
from src.models import Shape
from src.utils import get_logger

if TYPE_CHECKING:
    from src.api.dependencies import (
        AnnotationLifecycleDep,
        AnnotationServiceDep,
        GalleryServiceDep,
        ImageRecordDep,
        LabelStoreDep,
        RepositoryDep,
        ValidatorDep,
    )

logger = get_logger(__name__)

router = APIRouter()


@router.get("/{image_id}", response_model=list[SegmentationShape])
def get_annotations(
    image_record: ImageRecordDep, annotation_service: AnnotationServiceDep, repository: RepositoryDep,
) -> list[SegmentationShape]:
    """Return saved annotation shapes for a done image by reading its label file."""
    classes = repository.classes.get_all()
    return annotation_service.load_annotations(image_record, classes)


@router.post("/save", response_model=GalleryResponse, status_code=201)
def save_annotations(
    payload: SaveAnnotationsRequest,
    repository: RepositoryDep,
    lifecycle: AnnotationLifecycleDep,
    label_store: LabelStoreDep,
    gallery_service: GalleryServiceDep,
    validator: ValidatorDep,
) -> GalleryResponse:
    """Write label file and image copy under per-class subdirectories, then mark done."""
    record = repository.images.get_by_id(payload.image_id)
    if record is None:
        raise HTTPException(status_code=404, detail="Image not found")

    classes = repository.classes.get_all()
    try:
        validator.validate_save_annotations_request(payload, classes)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e

    src_path = Path(record.path)

    domain_shapes = [Shape(id=s.id, class_name=s.class_name, points=s.points) for s in payload.shapes]

    lifecycle.save_from_request(
        image_id=payload.image_id,
        shapes=domain_shapes,
        export_format_str=payload.export_format,
        src_path=src_path,
        label_store=label_store,
        classes=classes,
    )

    return gallery_service.build_gallery_response()


@router.post("/skip", response_model=GalleryResponse)
def skip_image(
    payload: SkipRequest,
    lifecycle: AnnotationLifecycleDep,
    gallery_service: GalleryServiceDep,
    repository: RepositoryDep,
) -> GalleryResponse:
    """Mark an image as skipped and return the updated gallery."""
    record = repository.images.get_by_id(payload.image_id)
    if record is None:
        raise HTTPException(status_code=404, detail="Image not found")
    lifecycle.mark_skipped(payload.image_id)
    return gallery_service.build_gallery_response()
