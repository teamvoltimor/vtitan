"""Tests for the periodic dataset-frame capture.

Pins the two cadence rules: Open Challenge saves unconditionally every
interval_s, Obstacles Challenge stays on that same cadence but then waits
frame over frame for a detection before saving -- and that reset() clears
per-run state (RACING start edge) rather than letting a prior run's timer
leak into the next.
"""

from __future__ import annotations

import numpy as np

from src.vision.dataset_capture import DatasetFrameCapture


def _frame(width: int = 32, height: int = 24) -> np.ndarray:
    return np.zeros((height, width, 3), dtype=np.uint8)


def test_no_run_path_never_saves() -> None:
    capture = DatasetFrameCapture(interval_s=10.0, subdir="captures")

    capture.maybe_capture(0.0, _frame(), run_path=None, require_detection=False, has_detection=False)

    assert capture._count == 0


def test_open_challenge_saves_unconditionally_once_interval_elapses(tmp_path) -> None:
    capture = DatasetFrameCapture(interval_s=10.0, subdir="captures")
    run_dir = tmp_path / "run_1"
    run_dir.mkdir()

    capture.maybe_capture(0.0, _frame(), run_path=str(run_dir), require_detection=False, has_detection=False)
    assert capture._count == 1
    assert (run_dir / "captures" / "capture_0000.jpg").exists()

    # Too soon -- interval hasn't elapsed yet.
    capture.maybe_capture(5.0, _frame(), run_path=str(run_dir), require_detection=False, has_detection=False)
    assert capture._count == 1

    capture.maybe_capture(10.0, _frame(), run_path=str(run_dir), require_detection=False, has_detection=False)
    assert capture._count == 2
    assert (run_dir / "captures" / "capture_0001.jpg").exists()


def test_obstacles_challenge_waits_for_a_detection_past_the_interval(tmp_path) -> None:
    capture = DatasetFrameCapture(interval_s=10.0, subdir="captures")
    run_dir = tmp_path / "run_1"
    run_dir.mkdir()

    # Interval elapsed, but nothing detected yet -- must not save.
    capture.maybe_capture(10.0, _frame(), run_path=str(run_dir), require_detection=True, has_detection=False)
    capture.maybe_capture(11.0, _frame(), run_path=str(run_dir), require_detection=True, has_detection=False)
    assert capture._count == 0

    # A detection finally shows up -- saves now, even though more than
    # interval_s has passed since the last (nonexistent) save.
    capture.maybe_capture(12.0, _frame(), run_path=str(run_dir), require_detection=True, has_detection=True)
    assert capture._count == 1

    # Timer resets from the successful save -- next detection right away
    # still must not save again immediately.
    capture.maybe_capture(12.5, _frame(), run_path=str(run_dir), require_detection=True, has_detection=True)
    assert capture._count == 1


def test_missing_run_directory_skips_without_error(tmp_path) -> None:
    capture = DatasetFrameCapture(interval_s=10.0, subdir="captures")
    missing_dir = tmp_path / "not_created_yet"

    capture.maybe_capture(0.0, _frame(), run_path=str(missing_dir), require_detection=False, has_detection=False)

    assert capture._count == 0


def test_reset_clears_cadence_and_filename_counter(tmp_path) -> None:
    capture = DatasetFrameCapture(interval_s=10.0, subdir="captures")
    run_dir = tmp_path / "run_1"
    run_dir.mkdir()
    capture.maybe_capture(0.0, _frame(), run_path=str(run_dir), require_detection=False, has_detection=False)
    assert capture._count == 1

    capture.reset()

    # A "new run" at monotonic time 0.0 again (a fresh run's clock) must be
    # able to save immediately rather than being blocked by the old timer,
    # and must restart filenames from capture_0000 for the new run's folder.
    other_run_dir = tmp_path / "run_2"
    other_run_dir.mkdir()
    capture.maybe_capture(0.0, _frame(), run_path=str(other_run_dir), require_detection=False, has_detection=False)
    assert capture._count == 1
    assert (other_run_dir / "captures" / "capture_0000.jpg").exists()
