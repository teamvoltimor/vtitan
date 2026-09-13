"""Periodic un-annotated frame capture for later dataset accumulation / fine-tuning.

Writes raw camera frames (no detection boxes, no HUD) to disk at a bounded
cadence during a race, so hardware runs accumulate real-world training data
for free alongside the mcap bag and debug video. See VisionNode._process,
which calls this right after detect() with the same rgb frame -- before
overlay.annotate() ever touches it.

The JPEG encode runs on a dedicated writer thread, so ``cv2.imwrite`` (which
blocks on disk) never lands on the capture/inference tick -- the same hazard
``VideoRecorder`` exists to avoid. ``flush()`` waits for the queue to drain and
``close()`` stops the thread; both exist so a run can finalize its captures.
"""

from __future__ import annotations

import logging
import queue
import threading
from pathlib import Path

import cv2
import numpy as np

logger = logging.getLogger(__name__)

_QUEUE_MAXSIZE = 8
"""Bounded so a stalled disk cannot buffer frames without limit; older frames
are dropped, which for a periodic training set is preferable to unbounded RAM."""

_CLOSE_TIMEOUT_S = 5.0
"""Bound on joining the writer thread at shutdown."""


class DatasetFrameCapture:
    """Saves raw frames to ``<run_dir>/<subdir>/`` at most once every ``interval_s``.

    Open Challenge has no obstacles to wait for, so every ``interval_s`` tick
    saves unconditionally. Obstacles Challenge stays on the same cadence but,
    once ``interval_s`` has elapsed, keeps waiting frame over frame until a
    detection is actually present -- so every frame this saves during
    Obstacles has a sign/obstacle in it, instead of capturing empty track by
    coincidence.

    Encoding is asynchronous: ``maybe_capture`` accepts a frame (cadence and
    detection gates still apply synchronously) and hands it to a writer
    thread. Call :meth:`flush` before reading the files back, or
    :meth:`close` at shutdown.
    """

    def __init__(self, interval_s: float, subdir: str) -> None:
        self._interval_s = interval_s
        self._subdir = subdir
        self._last_capture_time = float("-inf")
        self._count = 0
        self._queue: queue.Queue[tuple[Path, np.ndarray] | None] = queue.Queue(maxsize=_QUEUE_MAXSIZE)
        self._thread: threading.Thread | None = None

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
        # VideoRecorder._run does before writer.write()). Copied to a
        # contiguous array because the write now outlives this call.
        frame = np.ascontiguousarray(rgb[:, :, ::-1])
        self._ensure_thread()
        try:
            self._queue.put_nowait((capture_dir / filename, frame))
        except queue.Full:
            logger.warning("Dataset capture queue full, dropping frame %s", filename)
            return
        self._count += 1
        self._last_capture_time = now

    def flush(self) -> None:
        """Block until every accepted frame has been written."""
        self._queue.join()

    def close(self) -> None:
        """Stop the writer thread, draining queued frames first."""
        thread = self._thread
        if thread is None:
            return
        try:
            self._queue.put(None, timeout=_CLOSE_TIMEOUT_S)
        except queue.Full:
            logger.exception("Dataset capture queue full at shutdown; some frames may be dropped")
        thread.join(timeout=_CLOSE_TIMEOUT_S)
        if thread.is_alive():
            logger.error("Dataset capture thread did not finish within %.0fs", _CLOSE_TIMEOUT_S)
        self._thread = None

    def _ensure_thread(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._write_loop, name="dataset-capture", daemon=True)
            self._thread.start()

    def _write_loop(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                path, frame = item
                try:
                    if not cv2.imwrite(str(path), frame):
                        logger.warning("Failed to write dataset capture frame to %s", path)
                except Exception:
                    logger.exception("Error writing dataset capture frame to %s", path)
            finally:
                self._queue.task_done()
