"""Hardware L298N (or TB6612FNG-class) H-bridge drive driver.

``Driver`` targets an L298N- or TB6612FNG-class H-bridge on a Raspberry Pi:
one hardware-PWM enable line + two digital direction pins. GPIO libraries are
imported lazily inside ``connect`` so this module imports cleanly on dev
machines without ``lgpio``/``gpiozero``.

This driver has no encoder feedback of its own -- a quadrature encoder is a
separate physical part bolted to the motor shaft, not something the H-bridge
chip provides. See ``src/hardware/motors/encoder/`` and ``base.py``'s
``ClosedLoopDrive`` for how the node composes this ``DriveDriver`` with an
``EncoderSensor`` for closed-loop RPM control.

The H-bridge's PWM enable line is driven via the kernel's **hardware** PWM
peripheral (``/sys/class/pwm``), not ``gpiozero.PWMOutputDevice``: under
``LGPIOFactory`` (what the Pi Zero uses) gpiozero's software PWM runs a
continuous background thread that both burns CPU on an already-overloaded
board and lands scheduling jitter on the duty cycle -- the same defect that
made the servo visibly twitch (see ``servo/driver.py``), just showing up here
as measured-speed noise a closed loop has to fight rather than a visible
twitch. Direction pins stay on gpiozero: those are plain digital I/O with no
PWM jitter to inherit.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from src.hardware.exceptions import MotorConnectionError
from src.hardware.motors import constants as motor_const
from src.hardware.motors.base import DriveDriver
from src.hardware.motors.l298n.config import L298nPwmConfig
from src.hardware.motors.pwm_sysfs import EXPORT_TIMEOUT_S, SYSFS_PWM_ROOT, wait_for_pwm_channel_writable
from src.navigation.utils import clamp

if TYPE_CHECKING:
    from pathlib import Path

logger = logging.getLogger(__name__)


class Driver(DriveDriver):
    """H-bridge-only drive on Raspberry Pi (lgpio/gpiozero).

    GPIO libraries are imported lazily in :meth:`connect` so the module imports
    on machines without them.

    All wiring facts (PWM chip/channel/carrier, and pin numbers) come from
    ``pwm_config`` (``L298nPwmConfig``) rather than separate constructor
    args, so they are TOML/env configurable the same way ``ServoConfig``
    already is -- see ``config/hardware/motors/l298n.toml``.

    ``pwm_config.standby_pin`` wires the chip-enable line a TB6612FNG
    exposes (STBY); leave it ``None`` for an L298N, which has no standby
    line — its per-channel enable (ENA/ENB) is the PWM pin, so disabling
    output is simply a duty write of 0.
    """

    def __init__(
        self,
        invert: bool = False,
        pwm_config: L298nPwmConfig | None = None,
    ) -> None:
        self._pwm_config = pwm_config or L298nPwmConfig()
        self._pins = (self._pwm_config.pwm_pin, self._pwm_config.dir_a_pin, self._pwm_config.dir_b_pin)
        self._period_ns = int(motor_const.NS_PER_S / self._pwm_config.frequency_hz)
        self._channel_dir: Path | None = None
        self._sign = -1.0 if invert else 1.0
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
        """Open the H-bridge (hardware PWM + gpiozero direction pins)."""
        try:
            from gpiozero import DigitalOutputDevice
        except ImportError as err:
            raise MotorConnectionError(
                [str(p) for p in self._pins],
                "gpiozero/lgpio not available (Pi 5 hardware only)",
            ) from err

        _, ain1, ain2 = self._pins
        try:
            self._ain1 = DigitalOutputDevice(ain1)
            self._ain2 = DigitalOutputDevice(ain2)
            if self._pwm_config.standby_pin is not None:  # TB6612 STBY; L298N has none
                standby = DigitalOutputDevice(self._pwm_config.standby_pin)
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

        logger.info("L298N driver connected on pins %s (PWM %s)", self._pins, channel_dir)

    def disconnect(self) -> None:
        """Stop the drive, release the PWM channel, and close the GPIO lines.

        Without closing ``_ain1``/``_ain2``/``_standby``, gpiozero keeps them
        reserved against its pin factory -- a later ``connect()`` call (same
        process, e.g. a SYSTEM_RESET re-arm) would then fail to claim the same
        pins instead of getting a clean handle.
        """
        self.stop_drive()
        if self._channel_dir is not None:
            try:
                (self._channel_dir / "enable").write_text("0")
            except OSError:
                logger.warning("Failed to disable drive PWM on disconnect", exc_info=True)
            self._channel_dir = None
        for device in (self._ain1, self._ain2, self._standby):
            if device is not None:
                device.close()
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

    def run_drive_forward(self, speed: int | float | None = None) -> None:
        """Open-loop forward at ``speed`` percent duty (default 50%)."""
        self._set_output((50 if speed is None else speed) / 100.0)

    def run_drive_reverse(self, speed: int | float | None = None) -> None:
        """Open-loop reverse at ``speed`` percent duty (default 50%)."""
        self._set_output(-(50 if speed is None else speed) / 100.0)

    def stop_drive(self) -> None:
        """Stop the drive."""
        if self._channel_dir is not None:
            try:
                (self._channel_dir / "duty_cycle").write_text("0")
            except OSError:
                logger.warning("Failed to zero drive PWM duty on stop", exc_info=True)

    def get_drive_position(self) -> float:
        """No feedback of its own -- see ``ClosedLoopDrive`` for encoder-backed position."""
        return 0.0

    def get_drive_speed(self) -> float:
        """No feedback of its own -- see ``ClosedLoopDrive`` for encoder-backed speed."""
        return 0.0
