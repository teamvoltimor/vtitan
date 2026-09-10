"""Unit tests for eval.compare_hars' pure decoding/CLI logic.

``compare_hars.py`` imports ``hailo_sdk_client`` at module scope, which only
exists inside the Hailo AI Software Suite container. A minimal stub is
installed into ``sys.modules`` before import so the module's pure-logic
functions (``decode``, ``parse_args``) can be unit-tested on the host.
"""

from __future__ import annotations

import sys
import types
from typing import Any

import numpy as np
import pytest


def _install_hailo_sdk_client_stub() -> None:
    stub = types.ModuleType("hailo_sdk_client")
    stub.ClientRunner = object  # type: ignore[attr-defined]
    stub.InferenceContext = object  # type: ignore[attr-defined]
    sys.modules["hailo_sdk_client"] = stub


@pytest.fixture()
def compare_hars() -> Any:
    _install_hailo_sdk_client_stub()
    from eval import compare_hars as module  # noqa: PLC0415

    return module


def test_decode_filters_by_min_score(compare_hars: Any) -> None:
    # One class, one detection above the floor and one below.
    per_image = np.array([[[0.1, 0.2], [0.1, 0.2], [0.9, 0.8], [0.9, 0.8], [0.5, 0.01]]])

    rows = compare_hars.decode(per_image, min_score=0.1)

    assert len(rows) == 1
    class_id, score, _ = rows[0]
    assert class_id == 0
    assert score == 0.5


def test_decode_sorts_by_descending_score(compare_hars: Any) -> None:
    per_image = np.array([[[0.1, 0.2], [0.1, 0.2], [0.9, 0.8], [0.9, 0.8], [0.3, 0.7]]])

    rows = compare_hars.decode(per_image, min_score=0.0)

    assert [round(score, 2) for _, score, _ in rows] == [0.7, 0.3]


def test_decode_scales_boxes_by_image_size(compare_hars: Any) -> None:
    per_image = np.array([[[0.5], [0.25], [1.0], [0.75], [0.9]]])

    rows = compare_hars.decode(per_image, min_score=0.0)

    _, _, box = rows[0]
    np.testing.assert_allclose(box, np.array([0.25, 0.5, 0.75, 1.0]) * compare_hars.IMAGE_SIZE)


def test_parse_args_defaults(compare_hars: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sys, "argv", ["compare_hars.py"])
    args = compare_hars.parse_args()

    assert args.limit == 300
    assert args.conf == 0.01
    assert args.har is None
    assert str(compare_hars.SHARED / "calib_data_gmr") == args.images


def test_parse_args_overrides(compare_hars: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        ["compare_hars.py", "--limit", "50", "--conf", "0.2", "--har", "a=x.har", "--har", "b=y.har"],
    )
    args = compare_hars.parse_args()

    assert args.limit == 50
    assert args.conf == 0.2
    assert args.har == ["a=x.har", "b=y.har"]
