"""Simulated quadrature encoder (no GPIO).

``SimulatedEncoder`` is pure Python and is the reference implementation of
``EncoderSensor`` -- it integrates a settable target RPM over a pluggable
clock to produce believable counts, so tests can exercise the encoder path on
any machine without hardware or a paired drive.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING

from src.hardware.motors.base import DriveOdometry, EncoderSensor
from src.hardware.motors.encoder.control import counts_to_distance, counts_to_revolutions

if TYPE_CHECKING:
    from collections.abc import Callable

logger = logging.getLogger(__name__)


class SimulatedEncoder(EncoderSensor):
    """No-hardware ``EncoderSensor`` that fakes counts from a settable target RPM."""

    def __init__(
        self,
        counts_per_rev: float,
        wheel_diameter_m: float,
        invert: bool = False,
        time_source: Callable[[], float] = time.monotonic,
    ) -> None:
        self._counts_per_rev = counts_per_rev
        self._wheel_diameter_m = wheel_diameter_m
        self._sign = -1.0 if invert else 1.0
        self._now = time_source
        self._target_rpm = 0.0
        self._counts = 0.0
        self._last_rpm = 0.0
        self._last_t = time_source()

    def _advance(self) -> None:
        """Integrate counts for the time elapsed since the last update."""
        now = self._now()
        dt = now - self._last_t
        self._last_t = now
        if dt > 0:
            self._counts += self._target_rpm / 60.0 * dt * self._counts_per_rev

    def set_target_rpm(self, rpm: float) -> None:
        """Latch a new (already-signed) target RPM after settling outstanding counts.

        Called by whatever's driving the simulation (a test, or a paired
        ``DriveDriver`` fake) -- this encoder has no notion of a drive command
        itself, only of the shaft speed it's currently reading.
        """
        self._advance()
        self._target_rpm = self._sign * rpm

    def connect(self) -> None:
        """Reset the integration clock; there is no hardware to open."""
        self._last_t = self._now()
        logger.info("SimulatedEncoder connected (counts/rev=%.1f)", self._counts_per_rev)

    def disconnect(self) -> None:
        """No-op; there is no hardware to release."""

    def reset(self) -> None:
        """Zero the simulated encoder counts."""
        self._advance()
        self._counts = 0.0

    def get_counts(self) -> int:
        """Integrated quadrature counts since the last reset."""
        self._advance()
        return int(self._counts)

    def get_rpm(self) -> float:
        """Current (ideal) output-shaft RPM."""
        self._last_rpm = self._target_rpm
        return self._last_rpm

    def get_last_rpm(self) -> float:
        """The most recent value ``get_rpm()`` computed, without resampling."""
        return self._last_rpm

    def get_odometry(self) -> DriveOdometry:
        """Full odometry sample (counts, revolutions, RPM, distance)."""
        counts = self.get_counts()
        return DriveOdometry(
            counts=counts,
            revolutions=counts_to_revolutions(counts, self._counts_per_rev),
            rpm=self._last_rpm,
            distance_m=counts_to_distance(counts, self._counts_per_rev, self._wheel_diameter_m),
        )
