"""Hardware BTS7960 (IBT-2 module) H-bridge drive driver.

``Driver`` targets a BTS7960/IBT-2 module the way every reference wiring for
this chip does: genuinely independent ``RPWM``/``LPWM`` signals, with
``R_EN``/``L_EN`` held permanently HIGH and direction selected entirely by
which PWM channel carries a nonzero duty (the other held at 0). See
``docs/bts7960-ibt2-wiring.md``.

An earlier revision of this driver shared ONE PWM signal into both
``RPWM``/``LPWM`` and toggled ``R_EN``/``L_EN`` to pick a side, on the
(wrong) assumption that a disabled side's half-bridge would ignore its PWM
input entirely. It doesn't: per the chip's own truth table, RPWM=LPWM=HIGH
is "Fast Brake" and RPWM=LPWM=LOW is "Coast" -- our shared-signal wiring
alternated between exactly those two braking states every PWM cycle,
regardless of R_EN/L_EN, and never actually drove the motor. The module
itself was very likely fine the whole time.

``forward_pwm_pin`` rides the Pi's one free hardware-PWM engine (RPWM,
forward -- the overwhelmingly more frequent, performance-critical
direction). ``reverse_pwm_pin`` rides software PWM via gpiozero (LPWM,
reverse -- used only for parking/recovery, tolerant of the mild jitter
software PWM carries on this board). GPIO libraries are imported lazily
inside ``connect`` so this module imports cleanly on dev machines without
``lgpio``/``gpiozero``.

This driver has no encoder feedback of its own -- see ``l298n/driver.py``'s
docstring and ``base.py``'s ``ClosedLoopDrive`` for why that's a separate
component.
"""

from __future__ import annotations

import logging
import subprocess
from typing import TYPE_CHECKING

from src.hardware.exceptions import MotorConnectionError
from src.hardware.motors import constants as motor_const
from src.hardware.motors.base import DriveDriver
from src.hardware.motors.bts7960.config import Bts7960PwmConfig
from src.hardware.motors.pwm_sysfs import EXPORT_TIMEOUT_S, SYSFS_PWM_ROOT, wait_for_pwm_channel_writable
from src.navigation.utils import clamp

if TYPE_CHECKING:
    from pathlib import Path

logger = logging.getLogger(__name__)


class Driver(DriveDriver):
    """H-bridge-only drive on Raspberry Pi (lgpio/gpiozero), independent RPWM/LPWM.

    All wiring facts (PWM chip/channel/carrier, and pin numbers) come from
    ``pwm_config`` (``Bts7960PwmConfig``) rather than separate constructor
    args, so they are TOML/env configurable the same way ``ServoConfig``
    already is -- see ``config/hardware/motors/bts7960.toml``.

    ``r_en_pin``/``l_en_pin`` are set HIGH once at :meth:`connect` and never
    touched again -- they gate the module's overcurrent/thermal protection,
    not direction.
    """

    def __init__(
        self,
        invert: bool = False,
        pwm_config: Bts7960PwmConfig | None = None,
    ) -> None:
        self._pwm_config = pwm_config or Bts7960PwmConfig()
        self._pins = (
            self._pwm_config.forward_pwm_pin,
            self._pwm_config.reverse_pwm_pin,
            self._pwm_config.r_en_pin,
            self._pwm_config.l_en_pin,
        )
        self._period_ns = int(motor_const.NS_PER_S / self._pwm_config.frequency_hz)
        self._channel_dir: Path | None = None
        self._sign = -1.0 if invert else 1.0
        self._reverse_pwm = None
        self._r_en = None
        self._l_en = None

    def _fail(self, reason: str) -> MotorConnectionError:
        return MotorConnectionError([str(p) for p in self._pins], reason)

    @property
    def _chip_dir(self) -> Path:
        return SYSFS_PWM_ROOT / f"pwmchip{self._pwm_config.pwmchip}"

    def _export_channel(self) -> Path:
        """Export the forward (RPWM) hardware-PWM channel and wait for it to become writable."""
        chip = self._chip_dir
        if not chip.is_dir():
            msg = (
                f"{chip} not present -- hardware PWM overlay missing. Add "
                "'dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4' to "
                "/boot/firmware/config.txt and reboot"
            )
            raise self._fail(msg)

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
        raise self._fail(msg)

    def connect(self) -> None:
        """Open the H-bridge: RPWM hardware PWM, LPWM software PWM, EN pins latched HIGH."""
        try:
            from gpiozero import DigitalOutputDevice, PWMOutputDevice
        except ImportError as err:
            raise MotorConnectionError(
                [str(p) for p in self._pins],
                "gpiozero/lgpio not available (Pi 5 hardware only)",
            ) from err

        _, reverse_pwm_pin, r_en_pin, l_en_pin = self._pins
        try:
            # HIGH for the driver's lifetime -- these gate the module's
            # protection circuitry, not direction (see module docstring).
            self._r_en = DigitalOutputDevice(r_en_pin, initial_value=True)
            self._l_en = DigitalOutputDevice(l_en_pin, initial_value=True)
            self._reverse_pwm = PWMOutputDevice(
                reverse_pwm_pin,
                frequency=self._pwm_config.frequency_hz,
                initial_value=0,
            )
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

        logger.info(
            "BTS7960 driver connected: RPWM hw-PWM %s, LPWM sw-PWM GPIO%d, EN pins %s",
            channel_dir,
            reverse_pwm_pin,
            (r_en_pin, l_en_pin),
        )

    def disconnect(self) -> None:
        """Stop the drive, release both PWM channels, and close the EN GPIO lines."""
        self.stop_drive()
        if self._channel_dir is not None:
            try:
                (self._channel_dir / "enable").write_text("0")
            except OSError:
                logger.warning("Failed to disable forward PWM on disconnect", exc_info=True)
            self._channel_dir = None
        if self._reverse_pwm is not None:
            self._reverse_pwm.close()
            self._reverse_pwm = None
        for device in (self._r_en, self._l_en):
            if device is not None:
                device.close()
        self._r_en = None
        self._l_en = None
        self._force_gpio_low()

    def _force_gpio_low(self) -> None:
        """Re-drive LPWM/R_EN/L_EN low after gpiozero releases them.

        Released, these float, and the level shifter's onboard pull-up
        drives them HIGH -- full-speed reverse on the BTS7960, confirmed on
        hardware. gpiozero's own close() above cannot prevent this: closing
        a device un-claims the GPIO line entirely rather than leaving it
        driven at its last value. ``pinctrl`` programs the line's output
        state directly in the SoC's GPIO controller (not through a
        held-open handle), so -- like the boot-time ``config.txt``
        ``gpio=...,dl`` line and the systemd unit's ``ExecStopPost`` this
        mirrors -- it persists after this process exits, closing the
        floating window regardless of which caller (this driver, a bench
        script, a test) triggered the disconnect.
        """
        pins = f"{self._pwm_config.l_en_pin},{self._pwm_config.r_en_pin},{self._pwm_config.reverse_pwm_pin}"
        try:
            subprocess.run(["pinctrl", "set", pins, "op", "dl"], check=False)  # noqa: S607 - fixed, no user input
        except OSError:
            logger.warning("Failed to force GPIO low via pinctrl on disconnect", exc_info=True)

    def _set_output(self, duty: float) -> None:
        """Drive the H-bridge from a signed duty in [-1, 1].

        Positive -> RPWM (forward, hardware PWM) carries the duty, LPWM held
        at 0. Negative -> LPWM (reverse, software PWM) carries the duty,
        RPWM held at 0. Never both nonzero at once -- that's Fast Brake, not
        drive (see module docstring).
        """
        signed = self._sign * clamp(duty, -1.0, 1.0)
        if self._reverse_pwm is None or self._channel_dir is None:
            raise MotorConnectionError([str(p) for p in self._pins], "driver not connected")
        try:
            if signed >= 0:
                self._reverse_pwm.value = 0
                (self._channel_dir / "duty_cycle").write_text(str(int(signed * self._period_ns)))
            else:
                (self._channel_dir / "duty_cycle").write_text("0")
                self._reverse_pwm.value = -signed
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
        """Stop the drive (both channels to 0 duty)."""
        if self._channel_dir is not None:
            try:
                (self._channel_dir / "duty_cycle").write_text("0")
            except OSError:
                logger.warning("Failed to zero forward PWM duty on stop", exc_info=True)
        if self._reverse_pwm is not None:
            self._reverse_pwm.value = 0

    def get_drive_position(self) -> float:
        """No feedback of its own -- see ``ClosedLoopDrive`` for encoder-backed position."""
        return 0.0

    def get_drive_speed(self) -> float:
        """No feedback of its own -- see ``ClosedLoopDrive`` for encoder-backed speed."""
        return 0.0
