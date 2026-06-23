"""RC servo steering driver (single servo, parallel/crab steering).

Implements the ``SteeringDriver`` port for a hobby RC servo driven by a 50 Hz
PWM signal. gpiozero is imported lazily inside :meth:`connect`, so this module
imports cleanly on machines without GPIO libraries, mirroring the
``dc_encoder`` backend. The servo has no position feedback: reported speed is
always 0 and reported position is the last commanded value.
"""

from __future__ import annotations

import logging
from typing import override

from src.hardware.exceptions import MotorConnectionError
from src.hardware.motors.base import (
    DEFAULT_STEERING_SPEED,
    STEERING_CENTER_DEG,
    SteeringDriver,
)
from src.hardware.motors.servo.config import PWM_FREQUENCY_HZ, US_PER_SECOND, ServoConfig

logger = logging.getLogger(__name__)

_FRAME_WIDTH_US = US_PER_SECOND / PWM_FREQUENCY_HZ
"""Length of one PWM frame in microseconds (pulse-width -> duty cycle)."""


class Driver(SteeringDriver):
    """Single-servo steering via 50 Hz PWM (gpiozero, lazy GPIO import)."""

    def __init__(self, config: ServoConfig | None = None) -> None:
        """Build the driver without touching hardware (see :meth:`connect`).

        Args:
            config: Servo configuration; defaults to env-derived ``ServoConfig``.
        """
        self._config = config or ServoConfig()
        self._position = STEERING_CENTER_DEG
        self._pwm = None

    @override
    def connect(self) -> None:
        """Open the servo PWM output and centre the wheels.

        gpiozero/lgpio are imported here so the module stays importable on dev
        machines without GPIO libraries.

        Raises:
            MotorConnectionError: gpiozero/lgpio missing or GPIO init failed.
        """
        try:
            from gpiozero import PWMOutputDevice  # noqa: PLC0415 - lazy: keep importable without GPIO libs
        except ImportError as err:
            raise MotorConnectionError(
                str(self._config.gpio_pin),
                "gpiozero/lgpio not available (GPIO hardware only)",
            ) from err

        try:
            self._pwm = PWMOutputDevice(self._config.gpio_pin, frequency=PWM_FREQUENCY_HZ)
        except Exception as err:  # gpiozero raises GPIOZeroError/OSError families
            raise MotorConnectionError(
                str(self._config.gpio_pin),
                f"GPIO init failed: {type(err).__name__}",
            ) from err

        self.center_steering()
        logger.info("Servo steering connected on GPIO %d", self._config.gpio_pin)

    @override
    def disconnect(self) -> None:
        """Release the servo PWM output."""
        if self._pwm is not None:
            self._pwm.close()
            self._pwm = None

    def _position_to_pulse_us(self, position_deg: float) -> float:
        """Map an absolute steering angle (deg, 0 = centre) to a pulse width (us)."""
        cfg = self._config
        signed = -position_deg if cfg.reversed else position_deg
        span_us = cfg.max_pulse_us - cfg.min_pulse_us
        pulse_us = cfg.center_pulse_us + (signed / cfg.range_deg) * span_us
        return max(cfg.min_pulse_us, min(cfg.max_pulse_us, pulse_us))

    @override
    def move_steering_to(self, position: float, speed: int = DEFAULT_STEERING_SPEED) -> None:
        """Move steering to an absolute angle in degrees.

        ``speed`` is accepted for interface parity but ignored: an RC servo
        self-paces to its commanded position with no host-side rate control.

        Raises:
            MotorConnectionError: if called before :meth:`connect`.
        """
        del speed  # servo self-paces; no host-side rate control
        if self._pwm is None:
            raise MotorConnectionError(str(self._config.gpio_pin), "driver not connected")
        self._pwm.value = self._position_to_pulse_us(position) / _FRAME_WIDTH_US
        self._position = position

    @override
    def get_steering_position(self) -> float:
        """Last commanded steering angle (servo has no position feedback)."""
        return self._position
