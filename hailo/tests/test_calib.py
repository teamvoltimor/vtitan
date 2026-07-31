"""Unit tests for src.calib's image → .npy conversion."""

from __future__ import annotations

from typing import TYPE_CHECKING

import cv2
import numpy as np
import pytest

from src.calib import convert
from src.config import ConvertConfig
from src.errors import CalibrationDataError

if TYPE_CHECKING:
    from pathlib import Path


def _write_image(path: Path, size: int = 16) -> None:
    img = np.zeros((size, size, 3), dtype=np.uint8)
    cv2.imwrite(str(path), img)


def test_convert_writes_normalised_npy_per_image(tmp_path: Path) -> None:
    input_dir = tmp_path / "in"
    output_dir = tmp_path / "out"
    input_dir.mkdir()
    _write_image(input_dir / "a.jpg")
    _write_image(input_dir / "b.png")

    convert(ConvertConfig(input=str(input_dir), output=str(output_dir), size=8))

    npy_files = sorted(output_dir.glob("*.npy"))
    assert [f.stem for f in npy_files] == ["a", "b"]

    arr = np.load(npy_files[0])
    assert arr.shape == (8, 8, 3)
    assert arr.dtype == np.float32
    assert arr.min() >= 0.0
    assert arr.max() <= 1.0


def test_convert_raises_when_no_images_found(tmp_path: Path) -> None:
    input_dir = tmp_path / "empty"
    input_dir.mkdir()
    output_dir = tmp_path / "out"

    with pytest.raises(CalibrationDataError, match="No images found"):
        convert(ConvertConfig(input=str(input_dir), output=str(output_dir), size=8))


def test_convert_skips_unreadable_images(tmp_path: Path) -> None:
    input_dir = tmp_path / "in"
    output_dir = tmp_path / "out"
    input_dir.mkdir()
    (input_dir / "corrupt.jpg").write_bytes(b"not an image")
    _write_image(input_dir / "good.png")

    convert(ConvertConfig(input=str(input_dir), output=str(output_dir), size=8))

    npy_files = list(output_dir.glob("*.npy"))
    assert [f.stem for f in npy_files] == ["good"]


def test_convert_creates_output_directory(tmp_path: Path) -> None:
    input_dir = tmp_path / "in"
    output_dir = tmp_path / "nested" / "out"
    input_dir.mkdir()
    _write_image(input_dir / "a.jpg")

    convert(ConvertConfig(input=str(input_dir), output=str(output_dir), size=8))

    assert output_dir.is_dir()
