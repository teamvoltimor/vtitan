"""Integration test for eval.float_anchor's main(), against real local fixtures.

Skipped unless the real GMR checkpoint and calibration images/labels are
present -- all three are gitignored, host-only artifacts (see
``hailo/.gitignore`` and ``apps/auto-annotator/ml-service/.gitignore``), so this
test cannot run in a fresh clone or CI. It exists to give ``main()`` genuine
coverage on machines that do have the fixtures, complementing the
fixture-free CLI-parsing tests in ``test_eval_float_anchor.py``.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from eval import float_anchor

_HAILO_ROOT = Path(__file__).resolve().parent.parent
CHECKPOINT = _HAILO_ROOT.parent / "apps/auto-annotator" / "ml-service" / "models" / "gmr" / "best.pt"
IMAGES = _HAILO_ROOT / "shared_with_docker" / "calib_data_gmr"
LABELS = _HAILO_ROOT / "shared_with_docker" / "calib_labels_gmr"

pytestmark = pytest.mark.skipif(
    not (CHECKPOINT.exists() and IMAGES.exists() and LABELS.exists()),
    reason="real GMR checkpoint/calibration fixtures not present locally (gitignored, host-only)",
)


def test_main_runs_end_to_end_against_the_real_checkpoint(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "float_anchor.py",
            "--limit",
            "4",
            "--images",
            str(IMAGES),
            "--labels",
            str(LABELS),
            "--checkpoint",
            str(CHECKPOINT),
        ],
    )

    float_anchor.main()

    out = capsys.readouterr().out
    assert "SUMMARY" in out
    assert "float" in out
