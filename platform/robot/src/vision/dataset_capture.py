"""Periodic un-annotated frame capture for later dataset accumulation / fine-tuning.

Writes raw camera frames (no detection boxes, no HUD) to disk at a bounded
cadence during a race, so hardware runs accumulate real-world training data
for free alongside the mcap bag and debug video. See VisionNode._process,
which calls this right after detect() with the same rgb frame -- before
overlay.annotate() ever touches it.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import TYPE_CHECKING

import cv2

if TYPE_CHECKING:
    import numpy as np

logger = logging.getLogger(__name__)


class DatasetFrameCapture:
    """Saves raw frames to ``<run_dir>/<subdir>/`` at most once every ``interval_s``.

    Open Challenge has no obstacles to wait for, so every ``interval_s`` tick
    saves unconditionally. Obstacles Challenge stays on the same cadence but,
    once ``interval_s`` has elapsed, keeps waiting frame over frame until a
    detection is actually present -- so every frame this saves during
    Obstacles has a sign/obstacle in it, instead of capturing empty track by
    coincidence.
    """

    def __init__(self, interval_s: float, subdir: str) -> None:
        self._interval_s = interval_s
        self._subdir = subdir
        self._last_capture_time = float("-inf")
        self._count = 0

    def reset(self) -> None:
        """Clear per-run state. Call this on each RACING start edge."""
        self._last_capture_time = float("-inf")
        self._count = 0

    def maybe_capture(
        self,
        now: float,
        rgb: np.ndarray,
        *,
        run_path: str | None,
        require_detection: bool,
        has_detection: bool,
    ) -> None:
        """Save ``rgb`` if the cadence and (for Obstacles) detection gate both allow it."""
        if run_path is None:
            return
        if now - self._last_capture_time < self._interval_s:
            return
        if require_detection and not has_detection:
            return

        run_dir = Path(run_path)
        if not run_dir.is_dir():
            # Same rule as VisionNode's video recording: bag_recorder_node
            # owns creating the run directory, this must only poll for it.
            return

        capture_dir = run_dir / self._subdir
        capture_dir.mkdir(exist_ok=True)
        filename = f"capture_{self._count:04d}.jpg"
        # rgb -> bgr, cv2's expected channel order (same conversion
        # VideoRecorder._run does before writer.write()).
        if cv2.imwrite(str(capture_dir / filename), rgb[:, :, ::-1]):
            self._count += 1
            self._last_capture_time = now
        else:
            logger.warning("Failed to write dataset capture frame to %s", capture_dir / filename)
