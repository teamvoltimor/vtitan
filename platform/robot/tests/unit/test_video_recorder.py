"""Tests for the threaded, drop-on-backpressure video writer.

The point of VideoRecorder is that a slow encoder must never stall its
caller (VisionNode's capture/inference tick). These pin that contract --
non-blocking submit, drop-on-full, and correct thread lifecycle -- using a
real cv2.VideoWriter written to a temp file rather than mocking cv2, so the
resize/fourcc/release calls are exercised for real.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from src.vision.video_recorder import VideoRecorder


def _frame(width: int = 320, height: int = 180) -> np.ndarray:
    return np.zeros((height, width, 3), dtype=np.uint8)


def test_not_recording_before_start() -> None:
    recorder = VideoRecorder(video_width=160, fps=10.0)
    assert recorder.is_recording is False


def test_start_marks_recording_and_stop_finalizes_the_file(tmp_path) -> None:
    recorder = VideoRecorder(video_width=160, fps=10.0)
    path = tmp_path / "video.mp4"

    recorder.start(path)
    assert recorder.is_recording is True
    recorder.submit(_frame())
    recorder.stop()

    assert recorder.is_recording is False
    assert path.exists()
    assert path.stat().st_size > 0


def test_starting_twice_is_a_no_op(tmp_path) -> None:
    recorder = VideoRecorder(video_width=160, fps=10.0)
    path = tmp_path / "video.mp4"
    recorder.start(path)
    first_thread = recorder._thread

    recorder.start(tmp_path / "other.mp4")

    assert recorder._thread is first_thread
    recorder.stop()


def test_submit_before_start_is_a_no_op_not_an_error() -> None:
    recorder = VideoRecorder(video_width=160, fps=10.0)
    recorder.submit(_frame())  # must not raise


def test_stop_before_start_is_a_no_op() -> None:
    recorder = VideoRecorder(video_width=160, fps=10.0)
    recorder.stop()  # must not raise


def test_recorded_video_is_downscaled_to_video_width_preserving_aspect_ratio(tmp_path) -> None:
    """Height is derived from the input frame's own aspect ratio, not hardcoded."""
    recorder = VideoRecorder(video_width=100, fps=10.0)
    path = tmp_path / "video.mp4"

    recorder.start(path)
    recorder.submit(_frame(width=400, height=300))  # 4:3
    recorder.stop()

    import cv2

    cap = cv2.VideoCapture(str(path))
    try:
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    finally:
        cap.release()
    assert width == 100
    # 100 * 300 / 400 == 75; allow the codec's own even-dimension rounding
    # rather than asserting the pre-codec value exactly.
    assert height == pytest.approx(75, abs=1)


def test_submit_never_blocks_when_the_encoder_falls_behind(tmp_path) -> None:
    """The actual regression test for 'recording must not limit the NPU framerate'.

    A stalled encoder thread (blocked on the queue's own get(), never draining)
    must never make submit() itself block -- the queue fills, then every
    further submit() drops the frame and returns immediately.
    """
    recorder = VideoRecorder(video_width=160, fps=10.0)
    path = tmp_path / "video.mp4"
    recorder.start(path)

    # The queue's maxsize is small (3) and nothing is draining it in this test
    # (start() launches the real encoder thread, but it's fast enough that we
    # flood it deliberately -- submit a lot of frames back-to-back and assert
    # every call returns promptly regardless of how many are dropped).
    started = time.monotonic()
    for _ in range(200):
        recorder.submit(_frame())
    elapsed = time.monotonic() - started

    assert elapsed < 2.0, "submit() must never block the caller waiting on the encoder"
    recorder.stop()


def test_dropped_frame_counter_increments_when_queue_is_full() -> None:
    """White-box check that Full is actually handled, not just that submit() returns."""
    recorder = VideoRecorder(video_width=160, fps=10.0)
    # Fill the internal queue directly without starting the encoder thread, so
    # nothing drains it and the next submit() is guaranteed to hit queue.Full.
    recorder._thread = object()
    for _ in range(recorder._queue.maxsize):
        recorder._queue.put_nowait(_frame())

    recorder.submit(_frame())

    assert recorder._dropped == 1
    recorder._thread = None


@pytest.mark.slow()
def test_stop_drains_queued_frames_before_releasing(tmp_path) -> None:
    """stop() must not truncate frames that were already queued when it was called."""
    recorder = VideoRecorder(video_width=160, fps=10.0)
    path = tmp_path / "video.mp4"
    recorder.start(path)
    for _ in range(3):
        recorder.submit(_frame())
    recorder.stop()

    import cv2

    cap = cv2.VideoCapture(str(path))
    try:
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    finally:
        cap.release()
    assert frame_count == 3
