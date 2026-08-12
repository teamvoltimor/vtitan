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

The H-bridge's PWM enable line is driven via the kernel's **hardware** PWM
peripheral (``/sys/class/pwm``), not ``gpiozero.PWMOutputDevice``: under
``LGPIOFactory`` (what the Pi Zero uses) gpiozero's software PWM runs a
continuous background thread that both burns CPU on an already-overloaded
board and lands scheduling jitter on the duty cycle -- the same defect that
made the servo visibly twitch (see ``servo/driver.py``), just showing up here
as measured-speed noise the PID has to fight rather than a visible twitch.
Direction pins and the quadrature encoder stay on gpiozero: those are plain
digital I/O with no PWM jitter to inherit.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING

from shared.config.constants import RobotSpecs

from src.hardware.exceptions import MotorConnectionError
from src.hardware.motors.base import DriveOdometry, EncodedDriveDriver
from src.hardware.motors.dc_encoder.config import NS_PER_S, DcMotorPwmConfig
from src.hardware.motors.dc_encoder.control import (
    PIDController,
    SpeedEstimator,
    counts_to_distance,
    counts_to_revolutions,
)
from src.hardware.motors.pwm_sysfs import EXPORT_TIMEOUT_S, SYSFS_PWM_ROOT, wait_for_pwm_channel_writable
from src.navigation.utils import clamp

if TYPE_CHECKING:
    from collections.abc import Callable
    from pathlib import Path

logger = logging.getLogger(__name__)

# JGB37-520 1590 RPM variant defaults — confirm the printed gear ratio per unit.
# MEASURED on hardware 2026-07-25, not derived from the datasheet: the previous
# 194.0 (11 PPR x4 quadrature x an assumed ~4.4 gear ratio) made
# counts_to_distance() over-report by ~3.5x. Calibrated from raw quadrature
# counts against tape-measured travel, at two duty levels:
#     1650 counts / 54 cm  -> 672 counts per wheel revolution
#     2447 counts / 79 cm  -> 681 counts per wheel revolution
# The two agree to 1.3%, which is the point: wheel slip only ever inflates the
# count for a given distance, so agreement across speeds means slip is
# negligible and this is the true geometric ratio. Back-predicts both runs to
# within 1%. Implies ~15.4:1 gearing (676/44 counts per motor revolution).
#
# Measure with scripts/hardware/calibrate_encoder.py if the drivetrain changes. Do NOT
# derive it by integrating /motor/drive_speed -- that feedback is exponentially
# smoothed and rate-derived, and doing so gave answers ~2x wrong.
_DEFAULT_COUNTS_PER_REV = 676.0
# Previously an independent hardcoded 0.056m, drifted from RobotSpecs.WHEEL_RADIUS (a
# placeholder pending hardware bring-up, per the module docstring). Derived from the same
# measured wheel radius the rest of the stack uses instead of a second independent guess.
_DEFAULT_WHEEL_DIAMETER_M = RobotSpecs.WHEEL_RADIUS * 2
_NOMINAL_DT_S = 0.02
"""Assumed step on the first call, before a real interval can be measured."""

_MIN_DT_S = 0.001
_MAX_DT_S = 0.5
"""Bounds on a measured step, so a duplicate call or a stall can't blow up the loop."""

_DEFAULT_MAX_RPM = 42.5
"""Maximum achievable WHEEL rpm, measured 2026-07-25 (2447 counts / 5.11 s).

Was 1590.0 -- the motor's free-running rpm from the datasheet, which is the
wrong quantity twice over: it is the motor shaft rather than the wheel (~15.4:1
apart), and it is the unloaded figure. Since counts_per_rev counts WHEEL
revolutions, get_drive_rpm() reports wheel rpm, so the PID's feedforward term
(1/max_rpm) was scaled ~37x too small -- it would contribute ~3% duty where
~70% is needed to overcome stiction, leaving the integrator to crawl there
alone.

Corresponds to ~0.156 m/s. Note the achievable maximum sags with battery
charge (0.129 m/s measured on a tired pack), so commanded speeds should stay
below this for the loop to have headroom to correct.
"""


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


class Driver(EncodedDriveDriver):
    """H-bridge + quadrature encoder drive on Raspberry Pi (lgpio/gpiozero).

    GPIO libraries are imported lazily in :meth:`connect` so the module imports
    on machines without them. Closed-loop RPM runs a background control thread
    driving the PID against encoder feedback.

    ``standby_pin`` wires the chip-enable line a TB6612FNG exposes (STBY); pass
    ``None`` for an L298N, which has no standby line — its per-channel enable
    (ENA/ENB) is the PWM pin, so disabling output is simply a duty write of 0.
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
        invert_encoder: bool = False,
        pwm_config: DcMotorPwmConfig | None = None,
    ) -> None:
        self._pins = (pwm_pin, dir_a_pin, dir_b_pin, encoder_a_pin, encoder_b_pin)
        self._standby_pin = standby_pin
        self._pwm_config = pwm_config or DcMotorPwmConfig()
        self._period_ns = int(NS_PER_S / self._pwm_config.frequency_hz)
        self._channel_dir: Path | None = None
        self._counts_per_rev = counts_per_rev
        self._wheel_diameter_m = wheel_diameter_m
        self._max_rpm = max_rpm
        self._sign = -1.0 if invert else 1.0
        # Deliberately independent of ``invert``: the motor leads and the
        # encoder's A/B channels are separate connections, so swapping one does
        # not swap the other. ``invert`` also negates the command in software
        # rather than physically, which leaves the encoder reporting true
        # physical rotation while the command frame is flipped -- exactly the
        # case on this robot, where driving forward reads negative counts.
        # Anything integrating this feedback (odometry, a closed speed loop)
        # needs it in the command frame or it accumulates backwards.
        self._encoder_sign = -1 if invert_encoder else 1
        self._last_rpm = 0.0
        # Tuned on hardware 2026-07-25 against the corrected wheel-rpm units.
        # The original kp=0.002/ki=0.004 were set when max_rpm was the motor's
        # 1590 free-running figure; at true wheel rpm they left the integrator
        # closing the gap at ~0.11 duty/s, so a 0.10 m/s step took ~3.5 s to
        # settle. Feedforward (1/max_rpm) now lands near the stiction duty on
        # its own, and these gains close the remainder without overshoot.
        self._pid = pid or PIDController(kp=0.010, ki=0.020, kd=0.0, feedforward=1.0 / max_rpm)
        self._estimator = SpeedEstimator(counts_per_rev)
        self._encoder = None
        self._ain1 = None
        self._ain2 = None
        self._standby = None

    def _fail(self, reason: str) -> MotorConnectionError:
        return MotorConnectionError([str(p) for p in self._pins], reason)

    @property
    def _chip_dir(self) -> Path:
        return SYSFS_PWM_ROOT / f"pwmchip{self._pwm_config.pwmchip}"

    def _export_channel(self) -> Path:
        """Export the PWM channel and wait for it to become writable."""
        chip = self._chip_dir
        if not chip.is_dir():
            msg = (
                f"{chip} not present -- hardware PWM overlay missing. Add "
                "'dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4' to "
                "/boot/firmware/config.txt and reboot"
            )
            raise self._fail(
                msg,
            )

        channel_dir = chip / f"pwm{self._pwm_config.pwm_channel}"
        if not channel_dir.is_dir():
            try:
                (chip / "export").write_text(str(self._pwm_config.pwm_channel))
            except OSError as err:
                # EBUSY means someone already exported it, which is fine.
                if not channel_dir.is_dir():
                    msg = f"cannot export PWM channel: {err}"
                    raise self._fail(msg) from err

        if wait_for_pwm_channel_writable(channel_dir):
            return channel_dir

        msg = (
            f"{channel_dir} did not become writable within {EXPORT_TIMEOUT_S}s "
            "(is the service user in the 'gpio' group?)"
        )
        raise self._fail(
            msg,
        )

    def connect(self) -> None:
        """Open the H-bridge (hardware PWM + gpiozero direction pins) and encoder."""
        try:
            from gpiozero import (  # noqa: PLC0415 - lazy: keep module importable without GPIO libs
                DigitalOutputDevice,
                RotaryEncoder,
            )
        except ImportError as err:
            raise MotorConnectionError(
                [str(p) for p in self._pins],
                "gpiozero/lgpio not available (Pi 5 hardware only)",
            ) from err

        _, ain1, ain2, enc_a, enc_b = self._pins
        try:
            self._encoder = RotaryEncoder(enc_a, enc_b, max_steps=0)
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

        channel_dir = self._export_channel()
        try:
            # Order matters: duty_cycle may never exceed period, so a stale
            # larger duty from a previous run would make the period write
            # fail. Zero the duty first, then set the frame, then enable.
            (channel_dir / "duty_cycle").write_text("0")
            (channel_dir / "period").write_text(str(self._period_ns))
            (channel_dir / "enable").write_text("1")
        except OSError as err:
            msg = f"PWM init failed: {err}"
            raise self._fail(msg) from err
        self._channel_dir = channel_dir

        logger.info("DC encoder driver connected on pins %s (PWM %s)", self._pins, channel_dir)

    def disconnect(self) -> None:
        """Stop the drive, release the PWM channel, and close the GPIO lines.

        Without closing ``_encoder``/``_ain1``/``_ain2``/``_standby``, gpiozero
        keeps them reserved against its pin factory -- a later ``connect()``
        call (same process, e.g. a SYSTEM_RESET re-arm) would then fail to
        claim the same pins instead of getting a clean handle.
        """
        self.stop_drive()
        if self._channel_dir is not None:
            try:
                (self._channel_dir / "enable").write_text("0")
            except OSError:
                logger.warning("Failed to disable drive PWM on disconnect", exc_info=True)
            self._channel_dir = None
        for device in (self._encoder, self._ain1, self._ain2, self._standby):
            if device is not None:
                device.close()
        self._encoder = None
        self._ain1 = None
        self._ain2 = None
        self._standby = None

    def _set_output(self, duty: float) -> None:
        """Drive the H-bridge from a signed duty in [-1, 1]."""
        signed = self._sign * clamp(duty, -1.0, 1.0)
        if self._ain1 is None or self._ain2 is None or self._channel_dir is None:
            raise MotorConnectionError([str(p) for p in self._pins], "driver not connected")
        if signed >= 0:
            self._ain1.on()
            self._ain2.off()
        else:
            self._ain1.off()
            self._ain2.on()
        try:
            (self._channel_dir / "duty_cycle").write_text(str(int(abs(signed) * self._period_ns)))
        except OSError as err:
            msg = f"PWM duty_cycle write failed: {err}"
            raise self._fail(msg) from err

    def run_drive_forward(self, speed: int | None = None) -> None:
        """Open-loop forward at ``speed`` percent duty (default 50%)."""
        self._set_output((50 if speed is None else speed) / 100.0)

    def run_drive_reverse(self, speed: int | None = None) -> None:
        """Open-loop reverse at ``speed`` percent duty (default 50%)."""
        self._set_output(-(50 if speed is None else speed) / 100.0)

    def run_drive_at_rpm(self, rpm: float) -> None:
        """One closed-loop step: PID the duty toward ``rpm`` from encoder feedback.

        Uses real elapsed time rather than assuming a fixed call rate, so the
        loop stays correct if the timer jitters or the rate changes.
        """
        measured = self.get_drive_rpm()
        duty = self._pid.update(rpm, measured, dt=self._elapsed("_last_pid_time"))
        self._set_output(duty)

    def stop_drive(self) -> None:
        """Stop the drive and clear the PID state."""
        self._pid.reset()
        if self._channel_dir is not None:
            try:
                (self._channel_dir / "duty_cycle").write_text("0")
            except OSError:
                logger.warning("Failed to zero drive PWM duty on stop", exc_info=True)

    def reset_drive_encoder(self) -> None:
        """Zero the hardware encoder counter and speed estimator."""
        if self._encoder is not None:
            self._encoder.steps = 0
        self._estimator.reset()

    def get_drive_counts(self) -> int:
        """Quadrature counts in the COMMAND frame (0 until connected).

        Sign-corrected here rather than at each call site so every derived
        quantity -- revolutions, RPM, distance, speed -- inherits it
        consistently and cannot disagree with the others.
        """
        return 0 if self._encoder is None else self._encoder_sign * int(self._encoder.steps)

    def get_drive_rpm(self) -> float:
        """Smoothed wheel RPM from the encoder.

        NOTE this MUTATES the speed estimator -- it consumes the counts accrued
        since the previous call. It is called from more than one place (the
        closed loop and the feedback publisher, at different rates), so the
        elapsed time must be measured rather than assumed: with a hardcoded
        dt=0.02 each caller divided a partial count window by a full period and
        under-read the speed by roughly the number of callers, which made the
        closed loop settle ~40% below its setpoint.
        """
        self._last_rpm = self._estimator.update(self.get_drive_counts(), dt=self._elapsed("_last_rpm_time"))
        return self._last_rpm

    def _elapsed(self, attr: str) -> float:
        """Seconds since this named checkpoint, seeding it on first use."""
        now = time.monotonic()
        previous: float | None = getattr(self, attr, None)
        setattr(self, attr, now)
        if previous is None:
            return _NOMINAL_DT_S
        # Guard against a zero/absurd dt (two calls in the same instant, or a
        # long stall) turning into a divide-by-zero or a huge derivative kick.
        return min(max(now - previous, _MIN_DT_S), _MAX_DT_S)

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
        """Wheel speed in degrees/s, from the last sampled estimate.

        Deliberately does NOT re-sample: ``get_drive_rpm()`` consumes the counts
        accrued since its previous call, so a second consumer calling it at a
        different rate steals part of each window from the first. When the
        feedback publisher (100 Hz) and the control loop (50 Hz) both did that,
        the loop's speed estimate was mis-scaled and it settled ~40% off its
        setpoint -- in either direction depending on how the rates interleaved.
        Only the control loop samples now; everyone else reads this cache.
        """
        return self._last_rpm / 60.0 * 360.0
