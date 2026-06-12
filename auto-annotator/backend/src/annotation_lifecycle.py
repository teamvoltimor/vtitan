"""src.annotation_lifecycle – Owns the PENDING → DONE/SKIPPED state transition.

All side effects of completing an annotation — file copy, label write,
cache invalidation, data.yaml regeneration, DB status update — happen here
in a single call.  The router becomes a thin coordinator.
"""

from __future__ import annotations

import shutil
from typing import TYPE_CHECKING

from src.constants import VALID_EXTS
from src.enums import ExportFormat
from src.geometry import polygon_to_yolo_bbox
from src.label_store import LabelRecord
from src.utils import get_logger

if TYPE_CHECKING:
    from pathlib import Path

    from src.db.repository import Repository
    from src.gallery_cache import AnnotationCache
    from src.label_store import LabelStore
    from src.models import ClassInfo, Shape

logger = get_logger(__name__)

VAL_FRACTION: float = 0.2
"""Share of each class's images held out for validation in the generated data.yaml."""

TRAIN_LIST_NAME: str = "train.txt"
VAL_LIST_NAME: str = "val.txt"


class AnnotationLifecycle:
    """Coordinates the multi-step save and skip transitions for an image."""

    def __init__(
        self,
        repository: Repository,
        cache: AnnotationCache,
        images_dir: Path,
        labels_dir: Path,
        data_yaml_path: Path,
    ) -> None:
        self._repository = repository
        self._cache = cache
        self._images_dir = images_dir
        self._labels_dir = labels_dir
        self._data_yaml_path = data_yaml_path

    def save_from_request(
        self,
        image_id: int,
        shapes: list[Shape],
        export_format_str: str,
        src_path: Path,
        label_store: LabelStore,
        classes: list[ClassInfo],
    ) -> None:
        """Convert payload data to label records and mark done."""
        if not shapes:
            raise ValueError("Cannot save empty annotations")

        name_to_yolo: dict[str, int] = {cls.name: idx for idx, cls in enumerate(classes)}
        primary_class = shapes[0].class_name
        export_fmt = ExportFormat.SEG if export_format_str == "segmentation" else ExportFormat.DET

        label_records: list[LabelRecord] = []
        for shape in shapes:
            yolo_idx = name_to_yolo.get(shape.class_name)
            if yolo_idx is None:
                continue
            if export_fmt == ExportFormat.SEG:
                coords = [v for p in shape.points for v in (p.x, p.y)]
            else:
                xc, yc, w, h = polygon_to_yolo_bbox([(p.x, p.y) for p in shape.points])
                coords = [xc, yc, w, h]
            label_records.append(LabelRecord(class_id=yolo_idx, coords=coords))

        self.mark_done(
            image_id=image_id,
            src_path=src_path,
            label_records=label_records,
            label_store=label_store,
            primary_class=primary_class,
            export_format=export_fmt,
            classes=classes,
        )

    def mark_done(
        self,
        image_id: int,
        src_path: Path,
        label_records: list[LabelRecord],
        label_store: LabelStore,
        primary_class: str,
        export_format: ExportFormat,
        classes: list[ClassInfo],
    ) -> None:
        """Copy image, write label file, update DB, invalidate cache, rewrite data.yaml.

        DB update is the commit point: file operations happen first so a DB
        failure leaves files in place (orphaned but safe); a file failure
        raises before the DB is touched.
        """
        img_dest_dir = self._images_dir / primary_class
        lbl_dest_dir = self._labels_dir / primary_class
        img_dest_dir.mkdir(parents=True, exist_ok=True)
        lbl_dest_dir.mkdir(parents=True, exist_ok=True)

        shutil.copy2(src_path, img_dest_dir / src_path.name)
        label_store.save(lbl_dest_dir / (src_path.stem + ".txt"), label_records)

        self._repository.images.mark_done(image_id, export_format)
        self._cache.invalidate(primary_class)
        self._write_data_yaml(classes)

        logger.info("image_saved", extra={"image_id": image_id, "primary_class": primary_class})

    def mark_skipped(self, image_id: int) -> None:
        """Mark image as skipped in the DB."""
        self._repository.images.mark_skipped(image_id)

    def _write_data_yaml(self, classes: list[ClassInfo]) -> None:
        if not classes:
            return
        existing = [cls for cls in classes if (self._images_dir / cls.name).is_dir()]
        if not existing:
            return

        train_files, val_files = self._split_dataset(existing)
        if not train_files:
            return

        data_dir = self._images_dir.parent
        (data_dir / TRAIN_LIST_NAME).write_text("\n".join(train_files) + "\n", encoding="utf-8")
        (data_dir / VAL_LIST_NAME).write_text("\n".join(val_files) + "\n", encoding="utf-8")

        names_lines = "\n".join(f"  - {cls.name}" for cls in classes)
        # ``path`` is regenerated per environment (resolves correctly in Docker / any checkout);
        # ``train``/``val`` reference image-list files for a real (non-leaking) split.
        yaml_content = (
            f"path: {data_dir.resolve()}\n"
            f"train: {TRAIN_LIST_NAME}\n"
            f"val: {VAL_LIST_NAME}\n\n"
            f"nc: {len(classes)}\n"
            f"names:\n{names_lines}\n"
        )
        self._data_yaml_path.write_text(yaml_content, encoding="utf-8")
        logger.info(
            "data_yaml_written",
            extra={
                "path": str(self._data_yaml_path),
                "train": len(train_files),
                "val": len(val_files),
            },
        )

    def _split_dataset(self, classes: list[ClassInfo]) -> tuple[list[str], list[str]]:
        """Deterministic per-class train/val split returning data-dir-relative image paths.

        Files are sorted by name and the first ``VAL_FRACTION`` of each class is held out
        for validation, guaranteeing the val set never overlaps the train set. Classes with
        a single image contribute it to train only.
        """
        train: list[str] = []
        val: list[str] = []
        for cls in classes:
            files = sorted(
                p.name
                for p in (self._images_dir / cls.name).iterdir()
                if p.is_file() and p.suffix.lower() in VALID_EXTS
            )
            if not files:
                continue
            n_val = max(1, round(len(files) * VAL_FRACTION)) if len(files) >= 2 else 0
            val.extend(f"images/{cls.name}/{name}" for name in files[:n_val])
            train.extend(f"images/{cls.name}/{name}" for name in files[n_val:])

        # Avoid an empty val set (ultralytics errors on it) when every class had one image.
        if not val and len(train) >= 2:
            val.append(train.pop())
        return train, val
