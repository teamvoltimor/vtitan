"""Calibration data management: COCO download and image → .npy conversion."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from src.common import (
    CalibrationDataError,
    HailoError,
    get_logger,
    iter_images,
)

try:
    from fiftyone.types import ImageDirectory
    from fiftyone.zoo import load_zoo_dataset
except ImportError:
    ImageDirectory = None  # type: ignore[assignment, misc]
    load_zoo_dataset = None  # type: ignore[assignment]

log = get_logger(__name__)


@dataclass(slots=True, frozen=True)
class DownloadConfig:
    """Parameters for the COCO calibration dataset download.

    Args:
        samples: Number of images to pull from the COCO 2017 validation split.
        output: Destination directory for the raw image files.
    """

    samples: int
    output: str


@dataclass(slots=True, frozen=True)
class ConvertConfig:
    """Parameters for converting raw images to float32 ``.npy`` arrays.

    Args:
        input: Directory containing ``.jpg``/``.png`` calibration images.
        output: Directory where ``.npy`` files will be written.
        size: Square resize target matching the model's input resolution.
    """

    input: str
    output: str
    size: int


def download(config: DownloadConfig) -> None:
    """Download COCO 2017 validation images via fiftyone.

    Args:
        config: Download parameters.

    Raises:
        HailoError: If ``fiftyone`` is not installed.
    """
    if load_zoo_dataset is None:
        msg = "fiftyone is not installed. Add it to pyproject.toml and run `uv sync`."
        raise HailoError(msg)

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
    Path(config.output).mkdir(parents=True, exist_ok=True)

    count = 0
    for fname, img_path in iter_images(config.input):
        img = cv2.imread(img_path)
        if img is None:
            log.warning("Could not read %s — skipping.", img_path)
            continue
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (config.size, config.size))
        img = img.astype(np.float32) / 255.0  # HWC, [0, 1] — expected by Hailo DFC
        npy_path = Path(config.output) / (Path(fname).stem + ".npy")
        np.save(str(npy_path), img)
        count += 1

    if count == 0:
        msg = f"No images found in {config.input!r}. Run `hailo calib download` first."
        raise CalibrationDataError(msg)

    log.info("Converted %d images → %s", count, config.output)
