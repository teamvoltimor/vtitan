"""Threaded, drop-on-backpressure video writer for per-run debug recordings.

Runs the actual encode on a dedicated thread so a slow or stalled encoder can
never stall whatever feeds it frames -- for VisionNode, that's the same tick
Hailo inference runs on. See
docs/internal/plans/2026-08-11-run-video-recording-colocated-with-mcap.md.
"""

from __future__ import annotations

import logging
import queue
import threading
from typing import TYPE_CHECKING

import cv2

if TYPE_CHECKING:
    from pathlib import Path

    import numpy as np

logger = logging.getLogger(__name__)

_QUEUE_MAXSIZE = 3
"""Small on purpose: a full queue means the encoder is genuinely behind, and
the point is to notice and start dropping quickly, not to buffer minutes of
frames in memory hoping the encoder catches up."""

_JOIN_TIMEOUT_SEC = 5.0


class VideoRecorder:
    """Encodes RGB frames to an mp4 file on a dedicated thread.

    ``submit()`` never blocks the caller: if the encoder thread is behind, the
    frame is dropped rather than stalling whatever called submit(). Frames are
    resized to ``video_width`` (height derived from the first submitted
    frame's own aspect ratio, not hardcoded) on the encoder thread, so the
    resize cost never lands on the caller either.
    """

    def __init__(self, video_width: int, fps: float) -> None:
        self._video_width = video_width
        self._fps = fps
        self._queue: queue.Queue[np.ndarray | None] = queue.Queue(maxsize=_QUEUE_MAXSIZE)
        self._thread: threading.Thread | None = None
        self._dropped = 0

    @property
    def is_recording(self) -> bool:
        """Whether the encoder thread is currently running."""
        return self._thread is not None

    def start(self, path: Path) -> None:
        """Start the encoder thread. A no-op if already recording."""
        if self._thread is not None:
            return
        self._dropped = 0
        self._thread = threading.Thread(target=self._run, args=(path,), name="video-recorder", daemon=True)
        self._thread.start()

    def submit(self, frame_rgb: np.ndarray) -> None:
        """Hand a frame to the encoder thread. Drops it if the queue is full.

        Never blocks -- a queue.Full means the encoder is behind, and the
        caller (the capture/inference tick) must never wait on it.
        """
        if self._thread is None:
            return
        try:
            self._queue.put_nowait(frame_rgb)
        except queue.Full:
            self._dropped += 1
            if self._dropped % 30 == 1:
                logger.warning("Video recorder queue full, dropped %d frame(s) so far", self._dropped)

    def stop(self) -> None:
        """Signal the encoder thread to finish, drain, and release the writer.

        A no-op if not currently recording. Blocks briefly (bounded by
        ``_JOIN_TIMEOUT_SEC``) for the thread to drain its queue and finalize
        the file -- an unfinalized mp4 container can be unplayable.
        """
        if self._thread is None:
            return
        self._queue.put(None)  # sentinel; a blocking put is fine here, this is not the hot path
        self._thread.join(timeout=_JOIN_TIMEOUT_SEC)
        self._thread = None

    def _run(self, path: Path) -> None:
        writer: cv2.VideoWriter | None = None
        try:
            while True:
                frame = self._queue.get()
                if frame is None:
                    break
                if writer is None:
                    height, width = frame.shape[:2]
                    out_height = round(self._video_width * height / width)
                    writer = cv2.VideoWriter(
                        str(path),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        self._fps,
                        (self._video_width, out_height),
                    )
                resized = cv2.resize(frame, (self._video_width, out_height))
                writer.write(resized[:, :, ::-1])  # RGB -> BGR, OpenCV's expected order
        finally:
            if writer is not None:
                writer.release()
