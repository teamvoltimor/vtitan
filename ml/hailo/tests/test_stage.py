"""Unit tests for the flattening that stage() applies to nested datasets."""

from __future__ import annotations

from typing import TYPE_CHECKING

from src.hailomz import IMAGE_SUFFIXES, LABEL_SUFFIXES, _stage_flat

if TYPE_CHECKING:
    from pathlib import Path


def _dataset(root: Path) -> None:
    for class_name in ("green_prism", "red_prism"):
        (root / "images" / class_name).mkdir(parents=True)
        (root / "labels" / class_name).mkdir(parents=True)
        (root / "images" / class_name / "shot.jpg").write_bytes(b"x")
        (root / "labels" / class_name / "shot.txt").write_text("0 0.5 0.5 0.1 0.1\n")


def test_nested_classes_are_flattened_with_a_parent_prefix(tmp_path: Path) -> None:
    _dataset(tmp_path)
    dest = tmp_path / "out"
    assert _stage_flat(tmp_path / "images", dest, IMAGE_SUFFIXES) == 2
    assert {p.name for p in dest.iterdir()} == {"green_prism_shot.jpg", "red_prism_shot.jpg"}


def test_images_and_labels_flatten_to_matching_stems(tmp_path: Path) -> None:
    # The evaluators pair an image with its label by stem, so the same
    # prefixing rule has to apply to both sides.
    _dataset(tmp_path)
    images, labels = tmp_path / "out_img", tmp_path / "out_lbl"
    _stage_flat(tmp_path / "images", images, IMAGE_SUFFIXES)
    _stage_flat(tmp_path / "labels", labels, LABEL_SUFFIXES)
    assert {p.stem for p in images.iterdir()} == {p.stem for p in labels.iterdir()}


def test_suffix_filter_keeps_the_two_sides_apart(tmp_path: Path) -> None:
    _dataset(tmp_path)
    dest = tmp_path / "out"
    # Labels live under their own root; staging images must not pick up .txt.
    _stage_flat(tmp_path / "images", dest, IMAGE_SUFFIXES)
    assert all(p.suffix == ".jpg" for p in dest.iterdir())


def test_flat_source_needs_no_prefix(tmp_path: Path) -> None:
    source = tmp_path / "flat"
    source.mkdir()
    (source / "a.png").write_bytes(b"x")
    dest = tmp_path / "out"
    assert _stage_flat(source, dest, IMAGE_SUFFIXES) == 1
    assert {p.name for p in dest.iterdir()} == {"a.png"}
