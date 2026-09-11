"""Hardware quadrature-encoder driver (bolted to a motor shaft, on Raspberry Pi).

``QuadratureEncoder`` is a plain sensor, independent of whatever H-bridge (if
any) is turning the shaft it reads -- see ``base.py``'s ``EncoderSensor`` for
why this is a separate component from the drive driver rather than bundled
into it. ``gpiozero`` is imported lazily inside ``connect`` so this module
imports cleanly on dev machines without ``lgpio``/``gpiozero``.
"""

from __future__ import annotations

import logging
import time

from src.hardware.exceptions import MotorConnectionError
from src.hardware.motors.base import DriveOdometry, EncoderSensor
from src.hardware.motors.encoder.control import (
    SpeedEstimator,
    counts_to_distance,
    counts_to_revolutions,
)

logger = logging.getLogger(__name__)

_NOMINAL_DT_S = 0.02
"""Assumed step on the first call, before a real interval can be measured."""

_MIN_DT_S = 0.001
_MAX_DT_S = 0.5
"""Bounds on a measured step, so a duplicate call or a stall can't blow up the loop."""


class QuadratureEncoder(EncoderSensor):
    """A/B quadrature encoder on Raspberry Pi (lgpio/gpiozero's ``RotaryEncoder``)."""

    def __init__(
        self,
        pin_a: int,
        pin_b: int,
        counts_per_rev: float,
        wheel_diameter_m: float,
        invert: bool = False,
    ) -> None:
        self._pins = (pin_a, pin_b)
        self._counts_per_rev = counts_per_rev
        self._wheel_diameter_m = wheel_diameter_m
        # Independent of a paired drive's own `invert`: the motor leads and
        # the encoder's A/B channels are separate connections, so swapping one
        # does not swap the other.
        self._sign = -1 if invert else 1
        self._estimator = SpeedEstimator(counts_per_rev)
        self._encoder = None
        self._last_rpm = 0.0
        self._last_rpm_time: float | None = None

    def _fail(self, reason: str) -> MotorConnectionError:
        return MotorConnectionError([str(p) for p in self._pins], reason)

    def connect(self) -> None:
        """Open the encoder's A/B GPIO lines."""
        try:
            from gpiozero import RotaryEncoder
        except ImportError as err:
            raise MotorConnectionError(
                [str(p) for p in self._pins],
                "gpiozero/lgpio not available (Pi 5 hardware only)",
            ) from err

        pin_a, pin_b = self._pins
        try:
            self._encoder = RotaryEncoder(pin_a, pin_b, max_steps=0)
        except Exception as err:  # gpiozero raises GPIOZeroError/OSError families
            raise MotorConnectionError(
                [str(p) for p in self._pins],
                f"GPIO init failed: {type(err).__name__}",
            ) from err

        logger.info("Quadrature encoder connected on pins %s", self._pins)

    def disconnect(self) -> None:
        """Release the encoder GPIO lines."""
        if self._encoder is not None:
            self._encoder.close()
        self._encoder = None

    def reset(self) -> None:
        """Zero the hardware encoder counter and speed estimator."""
        if self._encoder is not None:
            self._encoder.steps = 0
        self._estimator.reset()
        self._last_rpm = 0.0

    def get_counts(self) -> int:
        """Quadrature counts in the COMMAND frame (0 until connected).

        Sign-corrected here rather than at each call site so every derived
        quantity -- revolutions, RPM, distance, speed -- inherits it
        consistently and cannot disagree with the others.
        """
        return 0 if self._encoder is None else self._sign * int(self._encoder.steps)

    def get_rpm(self) -> float:
        """Sample and return smoothed wheel RPM. See ``EncoderSensor.get_rpm``."""
        self._last_rpm = self._estimator.update(self.get_counts(), dt=self._elapsed())
        return self._last_rpm

    def get_last_rpm(self) -> float:
        """The most recent value ``get_rpm()`` computed, without resampling."""
        return self._last_rpm

    def _elapsed(self) -> float:
        """Seconds since the last ``get_rpm()`` call, seeding it on first use."""
        now = time.monotonic()
        previous = self._last_rpm_time
        self._last_rpm_time = now
        if previous is None:
            return _NOMINAL_DT_S
        # Guard against a zero/absurd dt (two calls in the same instant, or a
        # long stall) turning into a divide-by-zero or a huge derivative kick.
        return min(max(now - previous, _MIN_DT_S), _MAX_DT_S)

    def get_odometry(self) -> DriveOdometry:
        """Full odometry sample (counts, revolutions, RPM, distance), from the cached RPM."""
        counts = self.get_counts()
        return DriveOdometry(
            counts=counts,
            revolutions=counts_to_revolutions(counts, self._counts_per_rev),
            rpm=self.get_last_rpm(),
            distance_m=counts_to_distance(counts, self._counts_per_rev, self._wheel_diameter_m),
        )
