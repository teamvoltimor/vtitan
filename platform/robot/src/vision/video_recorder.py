"""Threaded, drop-on-backpressure video writer for per-run debug recordings.

Runs the actual encode on a dedicated thread so a slow or stalled encoder can
never stall whatever feeds it frames -- for VisionNode, that's the same tick
Hailo inference runs on. See
docs/internal/plans/2026-08-11-run-video-recording-colocated-with-mcap.md and
docs/internal/plans/2026-08-11-navigation-hud-overlay-and-open-challenge-recording.md.
"""

from __future__ import annotations

import logging
import queue
import threading
from dataclasses import dataclass
from typing import TYPE_CHECKING

import cv2

from src.vision.hud import HudConfig, draw_logo, draw_radar, draw_stats

if TYPE_CHECKING:
    from collections.abc import Sequence
    from pathlib import Path

    import numpy as np

logger = logging.getLogger(__name__)

_QUEUE_MAXSIZE = 3
"""Small on purpose: a full queue means the encoder is genuinely behind, and
the point is to notice and start dropping quickly, not to buffer minutes of
frames in memory hoping the encoder catches up."""


@dataclass(frozen=True, slots=True)
class FrameSnapshot:
    """One frame plus whatever telemetry was cached at submit() time.

    Built on the caller's thread (VisionNode's capture/inference tick) from
    the already-annotated (detection boxes drawn) frame and the latest
    values VisionNode's own /nav_debug and /scan subscriptions have cached --
    both may still be None early in a race, before the first message on
    either topic arrives. Immutable and self-contained so the encoder thread
    never reaches back into VisionNode's mutable state.
    """

    frame: np.ndarray
    nav_debug: dict | None = None
    scan_ranges: Sequence[float] | None = None
    scan_angles: Sequence[float] | None = None
    active_challenge: str | None = None


class VideoRecorder:
    """Encodes annotated frames, with the navigation HUD composited on top, to an mp4 file.

    ``submit()`` never blocks the caller: if the encoder thread is behind, the
    frame is dropped rather than stalling whatever called submit(). Both the
    resize (to ``video_width``, height derived from the first submitted
    frame's own aspect ratio) and the HUD compositing happen on the encoder
    thread, so neither cost lands on the caller.
    """

    def __init__(self, video_width: int, fps: float, hud_config: HudConfig) -> None:
        self._video_width = video_width
        self._fps = fps
        self._hud_config = hud_config
        self._queue: queue.Queue[FrameSnapshot | None] = queue.Queue(maxsize=_QUEUE_MAXSIZE)
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

    def submit(self, snapshot: FrameSnapshot) -> None:
        """Hand a frame snapshot to the encoder thread. Drops it if the queue is full.

        Never blocks -- a queue.Full means the encoder is behind, and the
        caller (the capture/inference tick) must never wait on it.
        """
        if self._thread is None:
            return
        try:
            self._queue.put_nowait(snapshot)
        except queue.Full:
            self._dropped += 1
            if self._dropped % 30 == 1:
                logger.warning("Video recorder queue full, dropped %d frame(s) so far", self._dropped)

    def stop(self) -> None:
        """Signal the encoder thread to finish, drain, and release the writer.

        A no-op if not currently recording. Blocks briefly (bounded by
        ``join_timeout_sec``) for the thread to drain its queue and finalize
        the file -- an unfinalized mp4 container can be unplayable.
        """
        if self._thread is None:
            return
        join_timeout_sec = self._hud_config.join_timeout_sec
        self._queue.put(None)  # sentinel; a blocking put is fine here, this is not the hot path
        self._thread.join(timeout=join_timeout_sec)
        if self._thread.is_alive():
            logger.error(
                "Video recorder thread did not finish finalizing within %.0fs -- "
                "the output file may be missing or unplayable",
                join_timeout_sec,
            )
        self._thread = None

    def _run(self, path: Path) -> None:
        writer: cv2.VideoWriter | None = None
        try:
            while True:
                snapshot = self._queue.get()
                if snapshot is None:
                    break
                frame = snapshot.frame
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
                # HUD drawn post-resize, unlike the detection boxes already on
                # `frame` -- its geometry is independent of detection
                # coordinates, and drawing it on the small output frame keeps
                # text a fixed, readable size regardless of capture resolution.
                hud_frame = draw_stats(resized, snapshot.nav_debug, snapshot.active_challenge, config=self._hud_config)
                hud_frame = draw_radar(
                    hud_frame,
                    snapshot.scan_ranges,
                    snapshot.scan_angles,
                    config=self._hud_config,
                )
                hud_frame = draw_logo(hud_frame, config=self._hud_config)
                writer.write(hud_frame[:, :, ::-1])  # RGB -> BGR, OpenCV's expected order
        finally:
            if writer is not None:
                writer.release()
