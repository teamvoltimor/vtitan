"""Simulated DC-encoder drive driver (no GPIO).

``SimulatedEncoderDriver`` is pure Python and is the reference implementation
of ``EncodedDriveDriver`` -- it integrates commanded RPM over a pluggable
clock to produce believable counts, so the navigation/ROS2 stack and tests can
exercise the encoder path on any machine.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING

from src.hardware.motors.base import DriveOdometry, EncodedDriveDriver
from src.hardware.motors.dc_encoder.calibration import (
    DEFAULT_COUNTS_PER_REV,
    DEFAULT_MAX_RPM,
    DEFAULT_WHEEL_DIAMETER_M,
)
from src.hardware.motors.dc_encoder.control import counts_to_distance, counts_to_revolutions
from src.navigation.utils import clamp

if TYPE_CHECKING:
    from collections.abc import Callable

logger = logging.getLogger(__name__)


class SimulatedEncoderDriver(EncodedDriveDriver):
    """No-hardware ``EncodedDriveDriver`` that fakes counts from commanded RPM."""

    def __init__(
        self,
        counts_per_rev: float = DEFAULT_COUNTS_PER_REV,
        wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M,
        max_rpm: float = DEFAULT_MAX_RPM,
        invert: bool = False,
        time_source: Callable[[], float] = time.monotonic,
    ) -> None:
        self._counts_per_rev = counts_per_rev
        self._wheel_diameter_m = wheel_diameter_m
        self._max_rpm = max_rpm
        self._sign = -1.0 if invert else 1.0
        self._now = time_source
        self._target_rpm = 0.0
        self._counts = 0.0
        self._last_t = time_source()

    def _advance(self) -> None:
        """Integrate counts for the time elapsed since the last update."""
        now = self._now()
        dt = now - self._last_t
        self._last_t = now
        if dt > 0:
            self._counts += self._target_rpm / 60.0 * dt * self._counts_per_rev

    def _set_rpm(self, rpm: float) -> None:
        """Latch a new target RPM after settling outstanding counts."""
        self._advance()
        self._target_rpm = self._sign * clamp(rpm, -self._max_rpm, self._max_rpm)

    def connect(self) -> None:
        """Reset the integration clock; there is no hardware to open."""
        self._last_t = self._now()
        logger.info("SimulatedEncoderDriver connected (counts/rev=%.1f)", self._counts_per_rev)

    def run_drive_forward(self, speed: int | None = None) -> None:
        """Open-loop forward at ``speed`` percent of max RPM (default 50%)."""
        pct = 50 if speed is None else speed
        self._set_rpm(self._max_rpm * pct / 100.0)

    def run_drive_reverse(self, speed: int | None = None) -> None:
        """Open-loop reverse at ``speed`` percent of max RPM (default 50%)."""
        pct = 50 if speed is None else speed
        self._set_rpm(-self._max_rpm * pct / 100.0)

    def run_drive_at_rpm(self, rpm: float) -> None:
        """Set the simulated closed-loop target output-shaft RPM."""
        self._set_rpm(rpm)

    def stop_drive(self) -> None:
        """Stop the drive (target RPM = 0)."""
        self._advance()
        self._target_rpm = 0.0

    def reset_drive_encoder(self) -> None:
        """Zero the simulated encoder counts."""
        self._advance()
        self._counts = 0.0

    def get_drive_counts(self) -> int:
        """Integrated quadrature counts since the last reset."""
        self._advance()
        return int(self._counts)

    def get_drive_rpm(self) -> float:
        """Current (ideal) output-shaft RPM."""
        return self._target_rpm

    def get_drive_odometry(self) -> DriveOdometry:
        """Full odometry sample (counts, revolutions, RPM, distance)."""
        counts = self.get_drive_counts()
        return DriveOdometry(
            counts=counts,
            revolutions=counts_to_revolutions(counts, self._counts_per_rev),
            rpm=self._target_rpm,
            distance_m=counts_to_distance(counts, self._counts_per_rev, self._wheel_diameter_m),
        )

    def get_drive_position(self) -> float:
        """Output-shaft angle in degrees."""
        return counts_to_revolutions(self.get_drive_counts(), self._counts_per_rev) * 360.0

    def get_drive_speed(self) -> float:
        """Output-shaft speed in degrees/s."""
        return self._target_rpm / 60.0 * 360.0
