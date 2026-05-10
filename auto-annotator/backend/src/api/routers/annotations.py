"""Annotation loading and saving endpoints."""

from __future__ import annotations

import shutil
from pathlib import Path

from fastapi import APIRouter, HTTPException

from src.api.dependencies import (
    AnnotationCacheDep,
    AnnotationServiceDep,
    GalleryServiceDep,
    ImageRecordDep,
    RepositoryDep,
    ValidatorDep,
)
from src.api.schemas import (
    GalleryResponse,
    SaveAnnotationsRequest,
    SegmentationShape,
    SkipRequest,
)
from src.constants import DATA_YAML_PATH, IMAGES_DIR, LABELS_DIR
from src.coordinates import yolo_bbox_to_corners
from src.geometry import polygon_to_yolo_bbox
from src.models import ClassInfo
from src.utils import get_logger

logger = get_logger(__name__)

router = APIRouter()


def _write_data_yaml(classes: list[ClassInfo]) -> None:
    """Regenerate data.yaml from the supplied classes and existing image subdirectories."""
    if not classes:
        return

    existing = [cls for cls in classes if (IMAGES_DIR / cls.name).is_dir()]
    if not existing:
        return

    train_lines = "\n".join(f"  - images/{cls.name}" for cls in existing)
    names_lines = "\n".join(f"  - {cls.name}" for cls in classes)

    yaml_content = (
        f"path: {IMAGES_DIR.parent.resolve()}\n"
        f"train:\n{train_lines}\n\n"
        f"val:\n{train_lines}\n\n"
        f"nc: {len(classes)}\n"
        f"names:\n{names_lines}\n"
    )

    DATA_YAML_PATH.write_text(yaml_content, encoding="utf-8")
    logger.info("data_yaml_written", path=str(DATA_YAML_PATH))


@router.get("/{image_id}", response_model=list[SegmentationShape])
def get_annotations(image_record: ImageRecordDep, annotation_service: AnnotationServiceDep, repository: RepositoryDep) -> list[SegmentationShape]:
    """Return saved annotation shapes for a done image by reading its label file."""
    classes = repository.classes.get_all()
    return annotation_service.load_annotations(image_record, classes)


@router.post("/save", response_model=GalleryResponse)
def save_annotations(
    payload: SaveAnnotationsRequest,
    repository: RepositoryDep,
    annotation_cache: AnnotationCacheDep,
    gallery_service: GalleryServiceDep,
    validator: ValidatorDep,
) -> GalleryResponse:
    """Write label file and image copy under per-class subdirectories, then mark done."""
    record = repository.images.get_by_id(payload.imageId)
    if record is None:
        raise HTTPException(status_code=404, detail="Image not found")

    classes = repository.classes.get_all()
    # Validate request before processing
    try:
        validator.validate_save_annotations_request(payload, classes)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

    name_to_yolo: dict[str, int] = {cls.name: idx for idx, cls in enumerate(classes)}
    primary_class = payload.shapes[0].className

    img_class_dir = IMAGES_DIR / primary_class
    lbl_class_dir = LABELS_DIR / primary_class
    img_class_dir.mkdir(parents=True, exist_ok=True)
    lbl_class_dir.mkdir(parents=True, exist_ok=True)

    src_path = Path(record.path)
    shutil.copy2(src_path, img_class_dir / src_path.name)

    label_lines: list[str] = []
    for shape in payload.shapes:
        yolo_idx = name_to_yolo.get(shape.className)
        if yolo_idx is None:
            continue
        if payload.exportFormat == "segmentation":
            coords = " ".join(f"{p.x:.6f} {p.y:.6f}" for p in shape.points)
            label_lines.append(f"{yolo_idx} {coords}")
        else:
            xc, yc, w, h = polygon_to_yolo_bbox([(p.x, p.y) for p in shape.points])
            label_lines.append(f"{yolo_idx} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")

    (lbl_class_dir / (src_path.stem + ".txt")).write_text("\n".join(label_lines), encoding="utf-8")
    annotation_cache.invalidate(primary_class)

    _write_data_yaml(classes)
    repository.images.mark_done(payload.imageId, payload.exportFormat)
    logger.info("image_saved", image_id=payload.imageId, primary_class=primary_class, shapes=len(payload.shapes))

    return gallery_service.build_gallery_response()


@router.post("/skip", response_model=GalleryResponse)
def skip_image(
    payload: SkipRequest,
    repository: RepositoryDep,
    gallery_service: GalleryServiceDep,
) -> GalleryResponse:
    """Mark an image as skipped and return the updated gallery."""
    record = repository.images.get_by_id(payload.imageId)
    if record is None:
        raise HTTPException(status_code=404, detail="Image not found")
    repository.images.mark_skipped(payload.imageId)
    return gallery_service.build_gallery_response()
