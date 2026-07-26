"""Calibration data management: COCO download and image → .npy conversion."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from src.config import ConvertConfig, DownloadConfig  # noqa: TC001
from src.constants import NORMALIZE_FACTOR
from src.errors import CalibrationDataError, require_dep
from src.image import iter_images
from src.log import get_logger

log = get_logger(__name__)


def download(config: DownloadConfig) -> None:
    """Download COCO 2017 validation images via fiftyone.

    Args:
        config: Download parameters.

    Raises:
        HailoError: If ``fiftyone`` is not installed.
    """
    import_err = None
    try:
        from fiftyone.types import ImageDirectory  # noqa: PLC0415
        from fiftyone.zoo import load_zoo_dataset  # noqa: PLC0415
    except ImportError as exc:
        load_zoo_dataset = None
        import_err = exc

    require_dep(load_zoo_dataset, "fiftyone", cause=import_err)

    log.info(
        "Downloading %d COCO 2017 validation images → %s",
        config.samples,
        config.output,
    )
    dataset = load_zoo_dataset(
        "coco-2017",
        split="validation",
        max_samples=config.samples,
        shuffle=True,
    )
    dataset.export(export_dir=config.output, dataset_type=ImageDirectory)
    log.info("Download complete.")


def convert(config: ConvertConfig) -> None:
    """Convert raw calibration images to normalised float32 ``.npy`` arrays.

    Hailo DFC 3.10 expects calibration data in HWC layout, normalised to
    ``[0, 1]``. Each image is saved as a separate ``.npy`` file.

    Args:
        config: Conversion parameters.

    Raises:
        CalibrationDataError: If no images are found in ``config.input``.
    """
    output_path = Path(config.output)
    output_path.mkdir(parents=True, exist_ok=True)

    count = 0
    for fname, img_path in iter_images(config.input):
        img = cv2.imread(img_path)
        if img is None:
            log.warning("Could not read %s — skipping.", img_path)
            continue
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (config.size, config.size))
        img = img.astype(np.float32) / NORMALIZE_FACTOR  # HWC, [0, 1] — expected by Hailo DFC
        npy_path = output_path / (Path(fname).stem + ".npy")
        np.save(str(npy_path), img)
        count += 1

    if count == 0:
        msg = f"No images found in {config.input!r}. Run `hailo calib download` first."
        raise CalibrationDataError(msg)

    log.info("Converted %d images → %s", count, config.output)
