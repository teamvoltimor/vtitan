"""src.augment – Image augmentation service for annotated images.

Adapts the legacy YOLO augmentation pipeline to work with the auto-annotator
DB and directory structure. Supports both bbox (det) and polygon (seg) formats.
"""

from __future__ import annotations

import collections
import cv2
import numpy as np
from pathlib import Path
from typing import Callable

from albumentations import (
    BboxParams,
    Compose,
    HorizontalFlip,
    KeypointParams,
    RandomBrightnessContrast,
    RandomCrop,
    ShiftScaleRotate,
)

from src import db
from src.constants import IMAGES_DIR, LABELS_DIR
from src.models import ImageRecord
from src.utils import get_logger

logger = get_logger(__name__)

ProgressCallback = Callable[[dict], None]


def _build_transforms(image_height: int, image_width: int) -> list:
    return [
        RandomBrightnessContrast(p=0.5),
        HorizontalFlip(p=0.5),
        ShiftScaleRotate(shift_limit=0.2, scale_limit=0.2, rotate_limit=25, p=0.5),
        RandomCrop(width=int(image_width * 0.9), height=int(image_height * 0.9), p=0.3),
    ]


def _parse_label_file(label_path: Path) -> tuple[list[int], list[list[float]]]:
    """Parse YOLO label file. Returns (class_ids, coord_lists)."""
    class_ids, coords = [], []
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if not parts:
            continue
        class_ids.append(int(parts[0]))
        coords.append([float(v) for v in parts[1:]])
    return class_ids, coords


def _is_bbox(coords: list[float]) -> bool:
    return len(coords) == 4


def _augment_det(image_np: np.ndarray, class_ids: list[int], bboxes: list[list[float]], n: int) -> list[tuple[np.ndarray, list[int], list[list[float]]]]:
    """Augment image with detection (bbox) annotations."""
    H, W = image_np.shape[:2]
    transforms = _build_transforms(H, W)
    compose = Compose(transforms, bbox_params=BboxParams(format="yolo", label_fields=["class_labels"], min_visibility=0.3))

    results = []
    for _ in range(n):
        out = compose(image=image_np, bboxes=bboxes, class_labels=class_ids)
        results.append((out["image"], list(out["class_labels"]), [list(b) for b in out["bboxes"]]))
    return results


def _augment_seg(image_np: np.ndarray, class_ids: list[int], polygons: list[list[float]], n: int) -> list[tuple[np.ndarray, list[int], list[list[float]]]]:
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

    compose = Compose(transforms, keypoint_params=KeypointParams(format="xy", label_fields=["poly_idx"], remove_invisible=True))

    results = []
    for _ in range(n):
        out = compose(image=image_np, keypoints=all_kpts, poly_idx=poly_idx)
        new_H, new_W = out["image"].shape[:2]

        groups: dict[int, list[float]] = collections.defaultdict(list)
        for (kx, ky), pidx in zip(out["keypoints"], out["poly_idx"]):
            groups[pidx].extend([kx / new_W, ky / new_H])

        new_classes, new_polys = [], []
        for pidx in sorted(groups):
            flat = groups[pidx]
            if len(flat) >= 6:  # at least 3 points
                new_classes.append(class_ids[pidx])
                new_polys.append(flat)

        results.append((out["image"], new_classes, new_polys))
    return results


def _write_label(path: Path, class_ids: list[int], coords: list[list[float]]) -> None:
    lines = [
        f"{cid} " + " ".join(f"{v:.6f}" for v in coord)
        for cid, coord in zip(class_ids, coords)
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def augment_image(record: ImageRecord, num_augmentations: int, progress: ProgressCallback | None = None) -> list[int]:
    """Augment a single annotated image, write files and register in DB.

    Args:
        record:            ImageRecord for the original (done) image.
        num_augmentations: How many augmented versions to generate.
        progress:          Optional callback receiving ``{"step": str}`` events.

    Returns:
        List of new image DB ids registered for the augmentations.
    """
    img_path = Path(record.path)
    class_dir = img_path.parent.name
    label_path = LABELS_DIR / class_dir / (img_path.stem + ".txt")

    if not label_path.exists():
        logger.info("label_not_found", extra={"path": str(label_path)})
        return []

    image_bgr = cv2.imread(str(img_path))
    if image_bgr is None:
        logger.info("image_not_readable", extra={"path": str(img_path)})
        return []

    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    class_ids, coords = _parse_label_file(label_path)

    if not class_ids:
        return []

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
        aug_img_path = IMAGES_DIR / class_dir / f"{aug_stem}.jpg"
        aug_lbl_path = LABELS_DIR / class_dir / f"{aug_stem}.txt"

        aug_img_path.parent.mkdir(parents=True, exist_ok=True)
        aug_lbl_path.parent.mkdir(parents=True, exist_ok=True)

        cv2.imwrite(str(aug_img_path), cv2.cvtColor(aug_img, cv2.COLOR_RGB2BGR))
        _write_label(aug_lbl_path, new_classes, new_coords)

        new_id = db.register_augmented_image(str(aug_img_path), fmt, record.id)
        new_ids.append(new_id)

        if progress:
            progress({"step": f"{img_path.name} aug {i}"})

    return new_ids


def run_augmentation_job(
    image_ids: list[int],
    num_augmentations: int,
    on_progress: ProgressCallback,
) -> None:
    """Run augmentation for a list of image IDs. Calls on_progress for each step.

    Args:
        image_ids:         List of DB ids of done original images to augment.
        num_augmentations: Augmented copies per image.
        on_progress:       Callback called with progress event dicts.
    """
    total = len(image_ids) * num_augmentations
    done = 0

    for img_id in image_ids:
        record = db.get_by_id(img_id)
        if record is None:
            continue

        def _step(event: dict) -> None:
            nonlocal done
            done += 1
            on_progress({"done": done, "total": total, "step": event.get("step", "")})

        augment_image(record, num_augmentations, progress=_step)

    on_progress({"done": total, "total": total, "finished": True})
    logger.info("augmentation_job_done", extra={"total": total})
