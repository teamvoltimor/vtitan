"""HTTP API used by the React frontend.

Provides a gallery listing (with thumbnails), file upload, static image serving,
and a segmentation endpoint that delegates to the existing SAM inference stack.
"""

from __future__ import annotations

import mimetypes
import shutil
from contextlib import asynccontextmanager
from pathlib import Path
from typing import TYPE_CHECKING, Annotated, Literal
from uuid import uuid4


if TYPE_CHECKING:
    from collections.abc import AsyncGenerator

import numpy as np
from fastapi import Depends, FastAPI, File, HTTPException, Request, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from PIL import Image
from pydantic import BaseModel

from src import db
from src.constants import API_PUBLIC_URL, DATA_YAML_PATH, IMAGES_DIR, LABELS_DIR, PENDING_DIR
from src.geometry import mask_to_yolo_bbox, mask_to_yolo_polygon
from src.inference import initialize_inference, run_sam_inference
from src.model_server import connect_to_model_server
from src.models import AppContext, AppState, ImageRecord, Point
from src.utils import get_logger

logger = get_logger(__name__)


@asynccontextmanager
async def _lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """Initialise shared resources once on startup and clean up on shutdown."""
    ctx = AppContext(client=connect_to_model_server())
    initialize_inference(ctx.client, ctx.inference)
    db.init_db()
    app.state.ctx = ctx
    yield


app = FastAPI(title="Auto-Annotator HTTP API", lifespan=_lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


def _get_context(request: Request) -> AppContext:
    """FastAPI dependency: return the application-level inference context."""
    return request.app.state.ctx  # type: ignore[no-any-return]


AppContextDep = Annotated[AppContext, Depends(_get_context)]


class GalleryStats(BaseModel):
    """Aggregate image status counts for the gallery overview."""

    pending: int
    done: int
    skipped: int
    total: int
    pct: float


class GalleryItem(BaseModel):
    """A single image entry in the gallery listing."""

    id: int
    label: str
    src: str
    format: str
    status: str
    updated: str


class GalleryResponse(BaseModel):
    """Full gallery payload returned by ``GET /gallery``."""

    items: list[GalleryItem]
    stats: GalleryStats


class NormalizedPoint(BaseModel):
    """A 2-D point with coordinates normalised to the range [0, 1]."""

    x: float
    y: float


class SegmentationPoint(BaseModel):
    """A click point sent by the frontend for SAM inference."""

    x: float
    y: float
    pointType: Literal["positive", "negative"]
    className: str


class SegmentationRequest(BaseModel):
    """Payload for ``POST /segment``."""

    imageId: int
    points: list[SegmentationPoint]


class SegmentationShape(BaseModel):
    """A single mask or bounding-box shape returned after inference."""

    id: str
    className: str
    points: list[NormalizedPoint]


class SegmentationResponse(BaseModel):
    """Response envelope for ``POST /segment``."""

    state: Literal["idle", "pending", "ready", "error"]
    message: str
    shapes: list[SegmentationShape]


def _build_gallery_response() -> GalleryResponse:
    rows = db.get_all_images()
    stats = db.get_stats()

    items = [
        GalleryItem(
            id=row.id,
            label=row.filename,
            src=f"{API_PUBLIC_URL}/images/{row.id}",
            format=row.format or "",
            status=row.status,
            updated=row.updated_at or "",
        )
        for row in rows
    ]

    return GalleryResponse(
        items=items,
        stats=GalleryStats(
            pending=stats.pending,
            done=stats.done,
            skipped=stats.skipped,
            total=stats.total,
            pct=stats.pct,
        ),
    )


def _mask_to_shape(mask: np.ndarray, class_name: str, image_id: int, mask_idx: int) -> SegmentationShape | None:
    polygon = mask_to_yolo_polygon(mask)
    if polygon:
        points = [
            NormalizedPoint(x=max(0.0, min(1.0, float(polygon[i]))), y=max(0.0, min(1.0, float(polygon[i + 1]))))
            for i in range(0, len(polygon), 2)
        ]
        return SegmentationShape(id=f"mask-{image_id}-{mask_idx}", className=class_name, points=points)

    bbox = mask_to_yolo_bbox(mask)
    if bbox:
        xc, yc, w, h = bbox
        half_w = w / 2
        half_h = h / 2
        coords = [
            (xc - half_w, yc - half_h),
            (xc + half_w, yc - half_h),
            (xc + half_w, yc + half_h),
            (xc - half_w, yc + half_h),
        ]
        points = [NormalizedPoint(x=max(0.0, min(1.0, float(x))), y=max(0.0, min(1.0, float(y)))) for x, y in coords]
        return SegmentationShape(id=f"bbox-{image_id}-{mask_idx}", className=class_name, points=points)

    return None


def _validate_image_id(image_id: int) -> ImageRecord:
    record = db.get_by_id(image_id)
    if record is None:
        raise HTTPException(status_code=404, detail="Image not found")
    return record


@app.get("/gallery", response_model=GalleryResponse)
def read_gallery() -> GalleryResponse:
    """Return all images with status counts for the gallery view."""
    return _build_gallery_response()


@app.post("/gallery/import", response_model=GalleryResponse)
async def upload_gallery_images(files: list[UploadFile] = File(...)) -> GalleryResponse:
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
        db.add_images_from_paths(saved_paths)

    return _build_gallery_response()


@app.get("/images/{image_id}")
def serve_image(image_id: int) -> FileResponse:
    """Stream the raw image file for the given database id."""
    record = _validate_image_id(image_id)
    media_type, _ = mimetypes.guess_type(record.path)
    return FileResponse(record.path, media_type=media_type)


@app.post("/segment", response_model=SegmentationResponse)
def run_segmentation(payload: SegmentationRequest, app_context: AppContextDep) -> SegmentationResponse:
    """Run SAM inference for the provided click points and return the best mask as a polygon."""
    if not payload.points:
        raise HTTPException(status_code=400, detail="Add at least one point first")

    record = _validate_image_id(payload.imageId)
    image = Image.open(record.path).convert("RGB")
    image_np = np.array(image)
    width, height = image.width, image.height

    classes = db.get_classes()
    class_map = {cls.name: cls for cls in classes}

    annotated_points: list[Point] = []
    mask_class = None
    for click in payload.points:
        cls_info = class_map.get(click.className)
        if cls_info is None:
            raise HTTPException(status_code=400, detail=f"Unknown class '{click.className}'")
        if click.pointType == "positive" and mask_class is None:
            mask_class = cls_info

        x = int(min(max(click.x, 0.0), 1.0) * (width - 1))
        y = int(min(max(click.y, 0.0), 1.0) * (height - 1))
        label = 1 if click.pointType == "positive" else 0
        annotated_points.append(Point(x=x, y=y, label=label, class_id=cls_info.id))

    if mask_class is None:
        mask_class = class_map.get(payload.points[0].className)
        if mask_class is None:
            raise HTTPException(status_code=400, detail="No valid class found for points")

    state = AppState(
        current_image=image_np,
        current_image_id=record.id,
        classes=classes,
        point_buffer=annotated_points,
        pending_class_db_id=mask_class.id,
    )

    result = run_sam_inference(state, app_context.client, app_context.inference)
    if not result.ok or result.masks is None:
        raise HTTPException(status_code=500, detail=result.error or "Inference failed")

    best_mask = result.masks[result.best_idx]
    shape = _mask_to_shape(best_mask, mask_class.name, record.id, result.best_idx)
    if shape is None:
        return SegmentationResponse(state="error", message="Mask too small to render", shapes=[])

    return SegmentationResponse(state="ready", message="Model mask ready", shapes=[shape])


def _write_data_yaml() -> None:
    """Regenerate data.yaml from current DB classes and existing image subdirectories."""
    classes = db.get_classes()
    if not classes:
        return

    class_dirs = [IMAGES_DIR / cls.name for cls in classes if (IMAGES_DIR / cls.name).is_dir()]
    if not class_dirs:
        return

    IMAGES_DIR.parent.mkdir(parents=True, exist_ok=True)

    train_lines = "\n".join(f"  - images/{cls.name}" for cls in classes if (IMAGES_DIR / cls.name).is_dir())
    names_lines = "\n".join(f"  - {cls.name}" for cls in classes)

    yaml_content = (
        f"path: {IMAGES_DIR.parent.resolve()}\n"
        f"train:\n{train_lines}\n\n"
        f"nc: {len(classes)}\n"
        f"names:\n{names_lines}\n"
    )

    DATA_YAML_PATH.write_text(yaml_content, encoding="utf-8")
    logger.info("data_yaml_written", path=str(DATA_YAML_PATH))


class SaveAnnotationsRequest(BaseModel):
    """Payload for ``POST /save``."""

    imageId: int
    exportFormat: Literal["segmentation", "detection"]
    shapes: list[SegmentationShape]


class SkipRequest(BaseModel):
    """Payload for ``POST /skip``."""

    imageId: int


@app.post("/save", response_model=GalleryResponse)
def save_annotations(payload: SaveAnnotationsRequest) -> GalleryResponse:
    """Write label file and image copy under per-class subdirectories, then mark done."""
    if not payload.shapes:
        raise HTTPException(status_code=400, detail="No shapes to save")

    record = _validate_image_id(payload.imageId)

    classes = db.get_classes()
    name_to_yolo: dict[str, int] = {cls.name: idx for idx, cls in enumerate(classes)}

    primary_class = payload.shapes[0].className
    if primary_class not in name_to_yolo:
        raise HTTPException(status_code=400, detail=f"Unknown class '{primary_class}'")

    img_class_dir = IMAGES_DIR / primary_class
    lbl_class_dir = LABELS_DIR / primary_class
    img_class_dir.mkdir(parents=True, exist_ok=True)
    lbl_class_dir.mkdir(parents=True, exist_ok=True)

    src_path = Path(record.path)
    dest_image = img_class_dir / src_path.name
    shutil.copy2(src_path, dest_image)

    label_lines: list[str] = []
    for shape in payload.shapes:
        yolo_idx = name_to_yolo.get(shape.className)
        if yolo_idx is None:
            continue
        if payload.exportFormat == "segmentation":
            coords = " ".join(f"{p.x:.6f} {p.y:.6f}" for p in shape.points)
            label_lines.append(f"{yolo_idx} {coords}")
        else:
            xs = [p.x for p in shape.points]
            ys = [p.y for p in shape.points]
            xc = (min(xs) + max(xs)) / 2
            yc = (min(ys) + max(ys)) / 2
            w = max(xs) - min(xs)
            h = max(ys) - min(ys)
            label_lines.append(f"{yolo_idx} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")

    label_path = lbl_class_dir / (src_path.stem + ".txt")
    label_path.write_text("\n".join(label_lines), encoding="utf-8")

    _write_data_yaml()
    db.mark_done(payload.imageId, payload.exportFormat)
    logger.info("image_saved", image_id=payload.imageId, primary_class=primary_class, shapes=len(payload.shapes))

    return _build_gallery_response()


@app.post("/skip", response_model=GalleryResponse)
def skip_image(payload: SkipRequest) -> GalleryResponse:
    """Mark an image as skipped and return the updated gallery."""
    _validate_image_id(payload.imageId)
    db.mark_skipped(payload.imageId)
    return _build_gallery_response()
