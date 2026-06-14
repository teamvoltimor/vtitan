"""src.augment – Image augmentation service for annotated images.

Adapts the legacy YOLO augmentation pipeline to work with the auto-annotator
DB and directory structure. Supports both bbox (det) and polygon (seg) formats.
"""

from __future__ import annotations

import collections
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

from src.label_store import LabelRecord
from src.models import AugmentedImage
from src.utils import get_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    import numpy as np

    from src.gallery_cache import AnnotationCache
    from src.label_store import LabelStore
    from src.models import ImageRecord


def _resolve_paths(images_dir: Path | None, labels_dir: Path | None) -> tuple[Path, Path]:
    """Return resolved images and labels directories.

    Falls back to AppConfig defaults when callers do not supply explicit paths.
    """
    if images_dir is not None and labels_dir is not None:
        return images_dir, labels_dir
    from src.config import AppConfig  # noqa: PLC0415
    cfg = AppConfig.load()
    return (
        images_dir if images_dir is not None else cfg.paths.images_dir,
        labels_dir if labels_dir is not None else cfg.paths.labels_dir,
    )

logger = get_logger(__name__)


def _build_transforms(image_height: int, image_width: int) -> list:
    return [
        RandomBrightnessContrast(p=0.5),
        HorizontalFlip(p=0.5),
        ShiftScaleRotate(shift_limit=0.2, scale_limit=0.2, rotate_limit=25, p=0.5),
        RandomCrop(width=int(image_width * 0.9), height=int(image_height * 0.9), p=0.3),
    ]


def _is_bbox(coords: list[float]) -> bool:
    return len(coords) == 4


def _augment_det(
    image_np: np.ndarray, class_ids: list[int], bboxes: list[list[float]], n: int,
) -> list[tuple[np.ndarray, list[int], list[list[float]]]]:
    """Augment image with detection (bbox) annotations."""
    H, W = image_np.shape[:2]
    transforms = _build_transforms(H, W)
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
    H, W = image_np.shape[:2]
    transforms = _build_transforms(H, W)

    all_kpts: list[tuple[float, float]] = []
    poly_idx: list[int] = []
    for i, flat in enumerate(polygons):
        pts = [(flat[j] * W, flat[j + 1] * H) for j in range(0, len(flat), 2)]
        all_kpts.extend(pts)
        poly_idx.extend([i] * len(pts))

    compose = Compose(
        transforms, keypoint_params=KeypointParams(format="xy", label_fields=["poly_idx"], remove_invisible=True),
    )

    results = []
    for _ in range(n):
        out = compose(image=image_np, keypoints=all_kpts, poly_idx=poly_idx)
        new_H, new_W = out["image"].shape[:2]

        groups: dict[int, list[float]] = collections.defaultdict(list)
        for (kx, ky), pidx in zip(out["keypoints"], out["poly_idx"], strict=False):
            groups[pidx].extend([kx / new_W, ky / new_H])

        new_classes, new_polys = [], []
        for pidx in sorted(groups):
            flat = groups[pidx]
            if len(flat) >= 6:  # at least 3 points
                new_classes.append(class_ids[pidx])
                new_polys.append(flat)

        results.append((out["image"], new_classes, new_polys))
    return results


def augment_image(
    record: ImageRecord,
    num_augmentations: int,
    repository: object,
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
    _images_dir, _labels_dir = _resolve_paths(images_dir, labels_dir)
    img_path = Path(record.path)
    class_dir = img_path.parent.name
    label_path = _labels_dir / class_dir / (img_path.stem + ".txt")

    records = label_store.load_raw(label_path)
    if not records:
        logger.info("label_not_found_or_empty", extra={"path": str(label_path)})
        return []

    image_bgr = cv2.imread(str(img_path))
    if image_bgr is None:
        logger.info("image_not_readable", extra={"path": str(img_path)})
        return []

    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    class_ids = [r.class_id for r in records]
    coords = [r.coords for r in records]

    fmt = record.format_used or "det"
    is_det = _is_bbox(coords[0])

    if is_det:
        aug_results = _augment_det(image_rgb, class_ids, coords, num_augmentations)
    else:
        aug_results = _augment_seg(image_rgb, class_ids, coords, num_augmentations)

    new_ids: list[int] = []
    for i, (aug_img, new_classes, new_coords) in enumerate(aug_results):
        if not new_classes:
            continue

        aug_stem = f"{img_path.stem}_aug_{i}"
        aug_img_path = _images_dir / class_dir / f"{aug_stem}.jpg"
        aug_lbl_path = _labels_dir / class_dir / f"{aug_stem}.txt"

        aug_img_path.parent.mkdir(parents=True, exist_ok=True)
        aug_lbl_path.parent.mkdir(parents=True, exist_ok=True)

        cv2.imwrite(str(aug_img_path), cv2.cvtColor(aug_img, cv2.COLOR_RGB2BGR))
        label_store.save(aug_lbl_path, [LabelRecord(cid, c) for cid, c in zip(new_classes, new_coords, strict=False)])

        aug_record = AugmentedImage(
            path=str(aug_img_path),
            format_used=fmt,
            parent_id=record.id,
        )
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
    _images_dir, _labels_dir = _resolve_paths(images_dir, labels_dir)
    img_path = Path(image_path)
    class_dir = img_path.parent.name
    label_path = _labels_dir / class_dir / (img_path.stem + ".txt")

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

    fmt = format_used or "det"
    if _is_bbox(coords[0]):
        aug_results = _augment_det(image_rgb, class_ids, coords, num_augmentations)
    else:
        aug_results = _augment_seg(image_rgb, class_ids, coords, num_augmentations)

    for i, (aug_img, new_classes, new_coords) in enumerate(aug_results):
        if not new_classes:
            continue

        aug_stem = f"{img_path.stem}_aug_{i}"
        aug_img_path = _images_dir / class_dir / f"{aug_stem}.jpg"
        aug_lbl_path = _labels_dir / class_dir / f"{aug_stem}.txt"

        aug_img_path.parent.mkdir(parents=True, exist_ok=True)
        aug_lbl_path.parent.mkdir(parents=True, exist_ok=True)

        cv2.imwrite(str(aug_img_path), cv2.cvtColor(aug_img, cv2.COLOR_RGB2BGR))
        label_store.save(
            aug_lbl_path, [LabelRecord(cid, c) for cid, c in zip(new_classes, new_coords, strict=False)],
        )

        yield AugmentedImage(path=str(aug_img_path), format_used=fmt, parent_id=parent_id)


def run_augmentation_job(
    image_ids: list[int],
    num_augmentations: int,
    repository: object,
    label_store: LabelStore,
    reporter: object | None = None,
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
    _images_dir, _labels_dir = _resolve_paths(images_dir, labels_dir)
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
