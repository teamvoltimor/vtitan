"""src.augment – Image augmentation service for annotated images.

Adapts the legacy YOLO augmentation pipeline to work with the auto-annotator
DB and directory structure. Supports both bbox (det) and polygon (seg) formats.
"""

from __future__ import annotations

import collections
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import cv2
from albumentations import (
    BboxParams,
    Compose,
    HorizontalFlip,
    KeypointParams,
    RandomBrightnessContrast,
    RandomCrop,
    ShiftScaleRotate,
)

from src.core.constants import GEOMETRY_MINIMUM_POLYGON_POINTS, YOLO_BBOX_COORD_COUNT
from src.label_store import LabelRecord
from src.models.models import AugmentedImage
from src.utils import get_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator
    from typing import Any, Protocol

    import numpy as np

    from src.gallery_cache import AnnotationCache
    from src.label_store import LabelStore
    from src.models.models import ImageRecord, ImageRepositoryProtocol

    class ProgressReporter(Protocol):
        """Protocol for reporting augmentation progress."""

        def update(
            self,
            status: str,
            progress: float,
            message: str,
            details: dict[str, Any] | None = None,
        ) -> None:
            """Report progress update."""
            ...


@dataclass
class DataPaths:
    """Directories for image and label data."""

    images_dir: Path
    labels_dir: Path


def _resolve_paths(images_dir: Path | None, labels_dir: Path | None) -> DataPaths:
    """Return resolved images and labels directories.

    Falls back to AppConfig defaults when callers do not supply explicit paths.
    """
    if images_dir is not None and labels_dir is not None:
        return DataPaths(images_dir=images_dir, labels_dir=labels_dir)
    from src.core.config import AppConfig  # noqa: PLC0415
    cfg = AppConfig.load()
    return DataPaths(
        images_dir=images_dir if images_dir is not None else cfg.paths.images_dir,
        labels_dir=labels_dir if labels_dir is not None else cfg.paths.labels_dir,
    )

logger = get_logger(__name__)

# Augmentation transform parameters
SHIFT_LIMIT = 0.2
SCALE_LIMIT = 0.2
ROTATE_LIMIT = 25
AUGMENT_PROB_BRIGHTNESS = 0.5
AUGMENT_PROB_FLIP = 0.5
AUGMENT_PROB_SHIFT = 0.5
AUGMENT_PROB_CROP = 0.3
CROP_SCALE = 0.9


def _build_transforms(image_height: int, image_width: int) -> list[object]:
    return [
        RandomBrightnessContrast(p=AUGMENT_PROB_BRIGHTNESS),
        HorizontalFlip(p=AUGMENT_PROB_FLIP),
        ShiftScaleRotate(shift_limit=SHIFT_LIMIT, scale_limit=SCALE_LIMIT, rotate_limit=ROTATE_LIMIT, p=AUGMENT_PROB_SHIFT),
        RandomCrop(width=int(image_width * CROP_SCALE), height=int(image_height * CROP_SCALE), p=AUGMENT_PROB_CROP),
    ]


def _is_bbox(coords: list[float]) -> bool:
    return len(coords) == YOLO_BBOX_COORD_COUNT


def _augment_det(
    image_np: np.ndarray, class_ids: list[int], bboxes: list[list[float]], n: int,
) -> list[tuple[np.ndarray, list[int], list[list[float]]]]:
    """Augment image with detection (bbox) annotations."""
    h, w = image_np.shape[:2]
    transforms = _build_transforms(h, w)
    compose = Compose(
        transforms, bbox_params=BboxParams(format="yolo", label_fields=["class_labels"], min_visibility=0.3),
    )

    results = []
    for _ in range(n):
        out = compose(image=image_np, bboxes=bboxes, class_labels=class_ids)
        results.append((out["image"], list(out["class_labels"]), [list(b) for b in out["bboxes"]]))
    return results


def _augment_seg(
    image_np: np.ndarray, class_ids: list[int], polygons: list[list[float]], n: int,
) -> list[tuple[np.ndarray, list[int], list[list[float]]]]:
    """Augment image with segmentation (polygon) annotations.

    Polygon format: flat list [x1, y1, x2, y2, ...] in normalized coords.
    """
    h, w = image_np.shape[:2]
    transforms = _build_transforms(h, w)

    all_kpts: list[tuple[float, float]] = []
    poly_idx: list[int] = []
    for i, flat in enumerate(polygons):
        pts = [(flat[j] * w, flat[j + 1] * h) for j in range(0, len(flat), 2)]
        all_kpts.extend(pts)
        poly_idx.extend([i] * len(pts))

    compose = Compose(
        transforms, keypoint_params=KeypointParams(format="xy", label_fields=["poly_idx"], remove_invisible=True),
    )

    results = []
    for _ in range(n):
        out = compose(image=image_np, keypoints=all_kpts, poly_idx=poly_idx)
        new_h, new_w = out["image"].shape[:2]

        groups: dict[int, list[float]] = collections.defaultdict(list)
        # strict=False: albumentations may filter keypoints; length mismatch is expected
        for (kx, ky), pidx in zip(out["keypoints"], out["poly_idx"], strict=False):
            groups[pidx].extend([kx / new_w, ky / new_h])

        new_classes, new_polys = [], []
        for pidx in sorted(groups):
            flat = groups[pidx]
            if len(flat) >= GEOMETRY_MINIMUM_POLYGON_POINTS * 2:  # x,y pair per point
                new_classes.append(class_ids[pidx])
                new_polys.append(flat)

        results.append((out["image"], new_classes, new_polys))
    return results


def _augment_and_write(
    image_path: str,
    num_augmentations: int,
    label_store: LabelStore,
    images_dir: Path,
    labels_dir: Path,
) -> Iterator[tuple[str, str]]:
    """Augment one image, writing augmented image+label files to disk.

    Shared by :func:`augment_image` (DB-backed) and :func:`augment_image_files`
    (DB-free): both need identical path resolution, image/label loading, and
    per-copy file writing -- they differ only in how each augmented copy gets
    persisted (a registered DB row vs. a yielded DTO), which is left to the caller.

    Yields:
        ``(augmented_image_path, augmented_stem)`` for each copy written to disk.
    """
    img_path = Path(image_path)
    class_dir = img_path.parent.name
    label_path = labels_dir / class_dir / (img_path.stem + ".txt")

    records = label_store.load_raw(label_path)
    if not records:
        logger.info("label_not_found_or_empty", extra={"path": str(label_path)})
        return

    image_bgr = cv2.imread(str(img_path))
    if image_bgr is None:
        logger.info("image_not_readable", extra={"path": str(img_path)})
        return

    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    class_ids = [r.class_id for r in records]
    coords = [r.coords for r in records]

    if _is_bbox(coords[0]):
        aug_results = _augment_det(image_rgb, class_ids, coords, num_augmentations)
    else:
        aug_results = _augment_seg(image_rgb, class_ids, coords, num_augmentations)

    for i, (aug_img, new_classes, new_coords) in enumerate(aug_results):
        if not new_classes:
            continue

        aug_stem = f"{img_path.stem}_aug_{i}"
        aug_img_path = images_dir / class_dir / f"{aug_stem}.jpg"
        aug_lbl_path = labels_dir / class_dir / f"{aug_stem}.txt"

        aug_img_path.parent.mkdir(parents=True, exist_ok=True)
        aug_lbl_path.parent.mkdir(parents=True, exist_ok=True)

        cv2.imwrite(str(aug_img_path), cv2.cvtColor(aug_img, cv2.COLOR_RGB2BGR))
        # strict=False: augmentation may filter objects; length mismatch is expected
        label_store.save(aug_lbl_path, [LabelRecord(cid, c) for cid, c in zip(new_classes, new_coords, strict=False)])

        yield str(aug_img_path), aug_stem


def augment_image(
    record: ImageRecord,
    num_augmentations: int,
    repository: ImageRepositoryProtocol,
    label_store: LabelStore,
    progress: Callable[[dict], None] | None = None,
    cache: AnnotationCache | None = None,
    images_dir: Path | None = None,
    labels_dir: Path | None = None,
) -> list[int]:
    """Augment a single annotated image, write files and register in DB.

    Args:
        record:            ImageRecord for the original (done) image.
        num_augmentations: How many augmented versions to generate.
        repository:        Repository for database access.
        label_store:       LabelStore for reading and writing label files.
        progress:          Optional callback receiving ``{"step": str}`` events.
        cache:             Optional cache to invalidate after writing augmented files.
        images_dir:        Directory where annotated images are stored. Falls back to AppConfig.
        labels_dir:        Directory where label files are stored. Falls back to AppConfig.

    Returns:
        List of new image DB ids registered for the augmentations.
    """
    paths = _resolve_paths(images_dir, labels_dir)
    _images_dir, _labels_dir = paths.images_dir, paths.labels_dir
    img_path = Path(record.path)
    class_dir = img_path.parent.name
    fmt = record.format_used or "det"

    new_ids: list[int] = []
    for i, (aug_img_path, aug_stem) in enumerate(
        _augment_and_write(record.path, num_augmentations, label_store, _images_dir, _labels_dir),
    ):
        aug_record = AugmentedImage(path=aug_img_path, format_used=fmt, parent_id=record.id)
        new_id = repository.images.register_augmented(aug_record)
        new_ids.append(new_id)

        if cache:
            cache.invalidate(class_dir, aug_stem)

        if progress:
            progress({"step": f"{img_path.name} aug {i}"})

    return new_ids


def augment_image_files(
    image_path: str,
    format_used: str,
    parent_id: int,
    num_augmentations: int,
    label_store: LabelStore,
    images_dir: Path | None = None,
    labels_dir: Path | None = None,
) -> Iterator[AugmentedImage]:
    """Augment one image, writing augmented image+label files to disk.

    DB-free variant of :func:`augment_image` for the gRPC AugmentationService:
    the Go API owns the database, so this yields an :class:`AugmentedImage` per
    created copy for the caller to persist, instead of registering rows itself.

    Args:
        image_path:        Absolute path to the original (done) image.
        format_used:       Export format of the parent (``"seg"`` or ``"det"``).
        parent_id:         DB id of the original image (for the parent reference).
        num_augmentations: How many augmented versions to generate.
        label_store:       LabelStore for reading and writing label files.
        images_dir:        Directory where annotated images are stored. Falls back to AppConfig.
        labels_dir:        Directory where label files are stored. Falls back to AppConfig.

    Yields:
        An :class:`AugmentedImage` for each augmented copy written to disk.
    """
    paths = _resolve_paths(images_dir, labels_dir)
    _images_dir, _labels_dir = paths.images_dir, paths.labels_dir
    fmt = format_used or "det"
    for aug_img_path, _aug_stem in _augment_and_write(
        image_path, num_augmentations, label_store, _images_dir, _labels_dir,
    ):
        yield AugmentedImage(path=aug_img_path, format_used=fmt, parent_id=parent_id)


def run_augmentation_job(
    image_ids: list[int],
    num_augmentations: int,
    repository: ImageRepositoryProtocol,
    label_store: LabelStore,
    reporter: ProgressReporter | None = None,
    cache: AnnotationCache | None = None,
    images_dir: Path | None = None,
    labels_dir: Path | None = None,
) -> None:
    """Run augmentation for a list of image IDs.

    Args:
        image_ids:         List of DB ids of done original images to augment.
        num_augmentations: Augmented copies per image.
        repository:        Repository for database access.
        label_store:       LabelStore for reading and writing label files.
        reporter:          Optional ProgressReporter for progress updates.
        cache:             Optional cache to invalidate after augmentation.
        images_dir:        Directory where annotated images are stored. Falls back to AppConfig.
        labels_dir:        Directory where label files are stored. Falls back to AppConfig.
    """
    paths = _resolve_paths(images_dir, labels_dir)
    _images_dir, _labels_dir = paths.images_dir, paths.labels_dir
    total = len(image_ids) * num_augmentations
    done = 0

    for idx, img_id in enumerate(image_ids):
        record = repository.images.get_by_id(img_id)
        if record is None:
            continue

        def _step(event: dict) -> None:
            nonlocal done
            done += 1
            if reporter:
                reporter.update(
                    "augmenting", done / total, event.get("step", ""), details={"done": done, "total": total},
                )

        augment_image(
            record, num_augmentations, repository, label_store,
            progress=_step, cache=cache, images_dir=_images_dir, labels_dir=_labels_dir,
        )

        if reporter:
            reporter.update("augmenting", (idx + 1) / len(image_ids), f"Processed {idx + 1}/{len(image_ids)} images")

    if reporter:
        reporter.update("augmenting", 1.0, "Done", details={"done": total, "total": total, "finished": True})
    logger.info("augmentation_job_done", extra={"total": total})
