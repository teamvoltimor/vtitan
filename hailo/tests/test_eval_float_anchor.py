"""Unit tests for eval.float_anchor's CLI argument parsing.

``float_anchor.py`` imports ``ultralytics`` at module scope; unlike
``hailo_sdk_client`` this is a normal host dependency, so the module imports
cleanly without a stub. ``main()`` itself needs a real ``.pt`` checkpoint and
is out of scope here — see the audit's "Hard-to-test modules" note.
"""

from __future__ import annotations

import sys
from typing import TYPE_CHECKING

from eval import float_anchor

if TYPE_CHECKING:
    import pytest


def test_parse_args_defaults(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(sys, "argv", ["float_anchor.py"])
    args = float_anchor.parse_args()

    assert args.limit == 300
    assert args.conf == 0.01
    assert args.checkpoint == float_anchor.GMR_CHECKPOINT_PATH
    assert args.images == "shared_with_docker/calib_data_gmr"
    assert args.labels == "shared_with_docker/calib_labels_gmr"


def test_parse_args_overrides(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        ["float_anchor.py", "--limit", "10", "--conf", "0.5", "--checkpoint", "custom.pt"],
    )
    args = float_anchor.parse_args()

    assert args.limit == 10
    assert args.conf == 0.5
    assert args.checkpoint == "custom.pt"


def test_gmr_checkpoint_path_points_under_auto_annotator() -> None:
    assert "auto-annotator" in float_anchor.GMR_CHECKPOINT_PATH
    assert float_anchor.GMR_CHECKPOINT_PATH.endswith(".pt")
