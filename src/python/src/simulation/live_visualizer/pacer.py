"""Wall-clock pacing and rclpy init for the live visualizer scripts."""

from __future__ import annotations

import time

import rclpy


class RealTimePacer:
    """Sleeps between ticks so a headless-speed loop plays back at wall-clock rate."""

    def __init__(self, dt: float, rate: float = 1.0) -> None:
        self._tick_budget = dt / rate if rate > 0 else 0.0
        self._next_tick: float | None = None

    def wait(self) -> None:
        """Block until the next tick's wall-clock deadline (no-op if unthrottled)."""
        if self._tick_budget <= 0.0:
            return
        now = time.monotonic()
        if self._next_tick is None:
            self._next_tick = now
        self._next_tick += self._tick_budget
        delay = self._next_tick - now
        if delay > 0.0:
            time.sleep(delay)
        else:
            self._next_tick = now


def init_rclpy_once() -> None:
    """Initialize rclpy if it hasn't been already (idempotent for script reuse)."""
    if not rclpy.ok():
        rclpy.init()
