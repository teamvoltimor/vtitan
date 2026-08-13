"""Bounded drop-oldest queue driven by a background producer thread."""

from __future__ import annotations

import logging
import threading
import time
from contextlib import suppress
from queue import Empty, Queue
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Callable


class FrameStreamer[T]:
    """Runs `produce` in a background thread, pushing results into a bounded queue that drops the oldest item once full.

    `produce` returning None means "nothing ready this tick": the loop sleeps
    `idle_sleep` and retries rather than treating it as an error or enqueuing
    a placeholder.
    """

    def __init__(
        self,
        produce: Callable[[], T | None],
        *,
        maxsize: int = 2,
        idle_sleep: float = 0.0,
        error_sleep: float = 0.1,
        error_message: str = "Streaming error",
        logger: logging.Logger | None = None,
    ) -> None:
        self._produce = produce
        self._queue: Queue[T] = Queue(maxsize=maxsize)
        self._idle_sleep = idle_sleep
        self._error_sleep = error_sleep
        self._error_message = error_message
        self._logger = logger or logging.getLogger(__name__)
        self._running = False
        self._thread: threading.Thread | None = None

    def _loop(self) -> None:
        while self._running:
            try:
                item = self._produce()
            except Exception:
                self._logger.exception(self._error_message)
                time.sleep(self._error_sleep)
                continue

            if item is None:
                if self._idle_sleep:
                    time.sleep(self._idle_sleep)
                continue

            if self._queue.full():
                with suppress(Empty):
                    self._queue.get_nowait()
            self._queue.put(item)

    @property
    def running(self) -> bool:
        """Whether the background thread is (meant to be) active."""
        return self._running

    def start(self) -> None:
        """Start the background producer thread. No-op if already running."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        """Stop the background thread and wait up to `timeout` seconds for it to exit."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=timeout)

    def get_nowait(self) -> T | None:
        """Return the next queued item, or None if the queue is currently empty."""
        try:
            return self._queue.get_nowait()
        except Empty:
            return None

    def get(self, timeout: float | None = None) -> T:
        """Block until an item is available, or `timeout` seconds elapse.

        Raises `queue.Empty` on timeout, matching `Queue.get`.
        """
        return self._queue.get(timeout=timeout)
