# ruff: noqa: S101
# Asserts are the assertion mechanism in a test file.
"""Permanent checks for the sweep tooling (adr:0087-test-methodology).

The sweeps under ``src/python/scripts/sim/sweeps/`` used to write their results
to ``/tmp`` and keep the coordinates in the runner's head. The results store
(``src/tools/sweep_results.py``) replaced that. This is the cheap part of the
harness to keep honest without a corpus run: it round-trips the store on a
temporary database and parses every sweep script with ``bash -n``. Stdlib only,
so the ``config`` CI job runs it with no pixi.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import pytest

if TYPE_CHECKING:
    import types

REPO_ROOT = Path(__file__).resolve().parents[3]
SWEEPS_DIR = REPO_ROOT / "src" / "python" / "scripts" / "sim" / "sweeps"

sys.path.insert(0, str(REPO_ROOT / "src" / "tools"))

import sweep_results  # noqa: E402

CORPUS = "tests/unit/test_obstacles_challenge_sim.py"

RAW_TWO_FAILED = (
    "FAILED tests/unit/test_a.py::test_one - AssertionError\n"
    "FAILED tests/unit/test_b.py::test_two - AssertionError\n"
    "=========================== 2 failed, 30 passed in 1.23s ===========================\n"
)
RAW_ONE_FAILED = (
    "FAILED tests/unit/test_b.py::test_two - AssertionError\n"
    "=========================== 1 failed, 31 passed in 1.20s ===========================\n"
)


def test_parse_pytest_reads_the_failure_set_and_counts() -> None:
    failures, failed, passed = sweep_results.parse_pytest(RAW_TWO_FAILED)
    assert failures == ["tests/unit/test_a.py::test_one", "tests/unit/test_b.py::test_two"]
    assert (failed, passed) == (2, 30)


def test_parse_pytest_refuses_an_unfinished_run() -> None:
    with pytest.raises(ValueError, match="summary"):
        sweep_results.parse_pytest("collected 3 items\n\nno summary line\n")


@pytest.fixture()
def store(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> types.ModuleType:
    """Point the store at a throwaway database and stub the git coordinates."""
    monkeypatch.setenv("VTITAN_RESULTS_DB", str(tmp_path / "sweeps.sqlite"))
    monkeypatch.setattr(sweep_results, "_git", lambda *args: "cafebabe\n" if args[0] == "rev-parse" else "diff\n")
    return sweep_results


def _record(store: types.ModuleType, tmp_path: Path, arm: str, raw: str) -> None:
    raw_path = tmp_path / f"{arm}.out"
    raw_path.write_text(raw)
    store.record(argparse.Namespace(sweep="demo", arm=arm, overrides="", corpus=CORPUS, raw=str(raw_path)))


def test_record_then_diff_reports_fixed_and_broke(
    store: types.ModuleType, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    _record(store, tmp_path, "A", RAW_TWO_FAILED)
    _record(store, tmp_path, "B", RAW_ONE_FAILED)

    assert store.diff(argparse.Namespace(a=1, b=2, force=False)) == 0
    out = capsys.readouterr().out
    assert "fixed=1 broke=0" in out
    assert "FIXED tests/unit/test_a.py::test_one" in out


def test_diff_refuses_runs_that_do_not_share_coordinates(
    store: types.ModuleType,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    _record(store, tmp_path, "A", RAW_TWO_FAILED)
    monkeypatch.setattr(sweep_results, "_git", lambda *args: "0thersha\n" if args[0] == "rev-parse" else "diff\n")
    _record(store, tmp_path, "B", RAW_ONE_FAILED)

    assert store.diff(argparse.Namespace(a=1, b=2, force=False)) == 1
    assert "--force" in capsys.readouterr().out


def test_every_sweep_script_parses() -> None:
    scripts = sorted(SWEEPS_DIR.glob("sweep_*.sh"))
    assert scripts, f"no sweep scripts under {SWEEPS_DIR}"
    for script in scripts:
        subprocess.run(["bash", "-n", str(script)], check=True)


def test_a_sweep_that_records_sources_the_store() -> None:
    for script in sorted(SWEEPS_DIR.glob("sweep_*.sh")):
        text = script.read_text(encoding="utf-8")
        if "record_arm" in text:
            assert "_results.sh" in text, f"{script.name} calls record_arm but does not source _results.sh"
