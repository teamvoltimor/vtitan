"""DC-encoder drive drivers: a simulated reference and a hardware adapter.

``SimulatedEncoderDriver`` is pure Python (no GPIO) and is the reference
implementation of ``EncodedDriveDriver`` — it integrates commanded RPM over a
pluggable clock to produce believable counts, so the navigation/ROS2 stack and
tests can exercise the encoder path on any machine.

``Driver`` targets a TB6612FNG- or L298N-class H-bridge + quadrature encoder on
a Raspberry Pi. GPIO libraries are imported lazily inside ``connect`` so this
module imports cleanly on dev machines without ``lgpio``/``gpiozero``.
Hardware bring-up (gear-ratio confirmation, PID tuning, odometry validation)
is still pending — see docs/internal/2026-06-11-jgb37-dc-encoder-motor.md.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING

from shared.config.constants import RobotSpecs

from src.hardware.exceptions import MotorConnectionError
from src.hardware.motors.base import DriveOdometry, EncodedDriveDriver
from src.hardware.motors.dc_encoder.control import (
    PIDController,
    SpeedEstimator,
    counts_to_distance,
    counts_to_revolutions,
)

if TYPE_CHECKING:
    from collections.abc import Callable

logger = logging.getLogger(__name__)

# JGB37-520 1590 RPM variant defaults — confirm the printed gear ratio per unit.
_DEFAULT_COUNTS_PER_REV = 194.0  # 11 PPR x4 quadrature x ~4.4 gear ratio
# Previously an independent hardcoded 0.056m, drifted from RobotSpecs.WHEEL_RADIUS (a
# placeholder pending hardware bring-up, per the module docstring). Derived from the same
# measured wheel radius the rest of the stack uses instead of a second independent guess.
_DEFAULT_WHEEL_DIAMETER_M = RobotSpecs.WHEEL_RADIUS * 2
_DEFAULT_MAX_RPM = 1590.0


class SimulatedEncoderDriver(EncodedDriveDriver):
    """No-hardware ``EncodedDriveDriver`` that fakes counts from commanded RPM."""

    def __init__(
        self,
        counts_per_rev: float = _DEFAULT_COUNTS_PER_REV,
        wheel_diameter_m: float = _DEFAULT_WHEEL_DIAMETER_M,
        max_rpm: float = _DEFAULT_MAX_RPM,
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
        self._target_rpm = self._sign * max(-self._max_rpm, min(self._max_rpm, rpm))

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


class Driver(EncodedDriveDriver):
    """H-bridge + quadrature encoder drive on Raspberry Pi (lgpio/gpiozero).

    GPIO libraries are imported lazily in :meth:`connect` so the module imports
    on machines without them. Closed-loop RPM runs a background control thread
    driving the PID against encoder feedback.

    ``standby_pin`` wires the chip-enable line a TB6612FNG exposes (STBY); pass
    ``None`` for an L298N, which has no standby line — its per-channel enable
    (ENA/ENB) is the PWM pin, so disabling output is simply ``pwm.value = 0``.
    """

    def __init__(
        self,
        pwm_pin: int,
        dir_a_pin: int,
        dir_b_pin: int,
        encoder_a_pin: int,
        encoder_b_pin: int,
        standby_pin: int | None = None,
        counts_per_rev: float = _DEFAULT_COUNTS_PER_REV,
        wheel_diameter_m: float = _DEFAULT_WHEEL_DIAMETER_M,
        max_rpm: float = _DEFAULT_MAX_RPM,
        pid: PIDController | None = None,
        invert: bool = False,
    ) -> None:
        self._pins = (pwm_pin, dir_a_pin, dir_b_pin, encoder_a_pin, encoder_b_pin)
        self._standby_pin = standby_pin
        self._counts_per_rev = counts_per_rev
        self._wheel_diameter_m = wheel_diameter_m
        self._max_rpm = max_rpm
        self._sign = -1.0 if invert else 1.0
        self._pid = pid or PIDController(kp=0.002, ki=0.004, kd=0.0, feedforward=1.0 / max_rpm)
        self._estimator = SpeedEstimator(counts_per_rev)
        self._encoder = None
        self._pwm = None
        self._ain1 = None
        self._ain2 = None
        self._standby = None

    def connect(self) -> None:
        """Open the H-bridge and encoder via gpiozero (lazy import; Pi 5 only)."""
        try:
            from gpiozero import (  # noqa: PLC0415 - lazy: keep module importable without GPIO libs
                DigitalOutputDevice,
                PWMOutputDevice,
                RotaryEncoder,
            )
        except ImportError as err:
            raise MotorConnectionError(
                [str(p) for p in self._pins],
                "gpiozero/lgpio not available (Pi 5 hardware only)",
            ) from err

        pwm, ain1, ain2, enc_a, enc_b = self._pins
        try:
            self._encoder = RotaryEncoder(enc_a, enc_b, max_steps=0)
            self._pwm = PWMOutputDevice(pwm)
            self._ain1 = DigitalOutputDevice(ain1)
            self._ain2 = DigitalOutputDevice(ain2)
            if self._standby_pin is not None:  # TB6612 STBY; L298N has none
                standby = DigitalOutputDevice(self._standby_pin)
                self._standby = standby
                standby.on()
        except Exception as err:  # gpiozero raises GPIOZeroError/OSError families
            raise MotorConnectionError(
                [str(p) for p in self._pins],
                f"GPIO init failed: {type(err).__name__}",
            ) from err
        logger.info("DC encoder driver connected on pins %s", self._pins)

    def _set_output(self, duty: float) -> None:
        """Drive the H-bridge from a signed duty in [-1, 1]."""
        signed = self._sign * max(-1.0, min(1.0, duty))
        if self._ain1 is None or self._ain2 is None or self._pwm is None:
            raise MotorConnectionError([str(p) for p in self._pins], "driver not connected")
        if signed >= 0:
            self._ain1.on()
            self._ain2.off()
        else:
            self._ain1.off()
            self._ain2.on()
        self._pwm.value = abs(signed)

    def run_drive_forward(self, speed: int | None = None) -> None:
        """Open-loop forward at ``speed`` percent duty (default 50%)."""
        self._set_output((50 if speed is None else speed) / 100.0)

    def run_drive_reverse(self, speed: int | None = None) -> None:
        """Open-loop reverse at ``speed`` percent duty (default 50%)."""
        self._set_output(-(50 if speed is None else speed) / 100.0)

    def run_drive_at_rpm(self, rpm: float) -> None:
        """One closed-loop step: PID the duty toward ``rpm`` from encoder feedback.

        A node/timer is expected to call this at a fixed rate against fresh
        encoder readings.
        """
        measured = self.get_drive_rpm()
        duty = self._pid.update(rpm, measured, dt=0.02)
        self._set_output(duty)

    def stop_drive(self) -> None:
        """Stop the drive and clear the PID state."""
        self._pid.reset()
        if self._pwm is not None:
            self._pwm.value = 0.0

    def reset_drive_encoder(self) -> None:
        """Zero the hardware encoder counter and speed estimator."""
        if self._encoder is not None:
            self._encoder.steps = 0
        self._estimator.reset()

    def get_drive_counts(self) -> int:
        """Raw quadrature counts (0 until connected)."""
        return 0 if self._encoder is None else int(self._encoder.steps)

    def get_drive_rpm(self) -> float:
        """Smoothed output-shaft RPM from the encoder."""
        return self._estimator.update(self.get_drive_counts(), dt=0.02)

    def get_drive_odometry(self) -> DriveOdometry:
        """Full odometry sample (counts, revolutions, RPM, distance)."""
        counts = self.get_drive_counts()
        return DriveOdometry(
            counts=counts,
            revolutions=counts_to_revolutions(counts, self._counts_per_rev),
            rpm=self.get_drive_rpm(),
            distance_m=counts_to_distance(counts, self._counts_per_rev, self._wheel_diameter_m),
        )

    def get_drive_position(self) -> float:
        """Output-shaft angle in degrees."""
        return counts_to_revolutions(self.get_drive_counts(), self._counts_per_rev) * 360.0

    def get_drive_speed(self) -> float:
        """Output-shaft speed in degrees/s."""
        return self.get_drive_rpm() / 60.0 * 360.0
