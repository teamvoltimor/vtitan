"""Hardware BTS7960 (IBT-2 module) H-bridge drive driver.

``Driver`` targets a BTS7960/IBT-2 module through an external 2:1 demux
(``RPWM = PWM AND dir``, ``LPWM = PWM AND NOT dir``; see
``docs/bts7960-ibt2-wiring.md``) rather than driving ``RPWM``/``LPWM``
directly -- the Pi Zero 2 W's SoC has exactly 2 hardware-PWM engines, both
already claimed by the servo and this driver's own shared channel, so there
is no 3rd channel available for a second, independent PWM input. GPIO
libraries are imported lazily inside ``connect`` so this module imports
cleanly on dev machines without ``lgpio``/``gpiozero``.

This driver has no encoder feedback of its own -- see ``l298n/driver.py``'s
docstring and ``base.py``'s ``ClosedLoopDrive`` for why that's a separate
component. Same reasoning for driving the shared PWM channel through the
kernel's hardware PWM peripheral rather than gpiozero's software PWM.
"""

from __future__ import annotations

import logging
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
    """H-bridge-only drive on Raspberry Pi (lgpio/gpiozero), through the demux.

    ``dir_select_pin`` feeds the demux's direction input (routes the shared
    PWM to ``RPWM`` when high, ``LPWM`` when low). ``r_en_pin``/``l_en_pin``
    are the module's own per-direction enables, wired directly (not through
    the demux) and simply held ``HIGH`` for the driver's lifetime -- the
    demux, not these pins, is what gates which direction actually receives
    PWM at any moment.
    """

    def __init__(
        self,
        pwm_pin: int,
        dir_select_pin: int,
        r_en_pin: int,
        l_en_pin: int,
        invert: bool = False,
        pwm_config: Bts7960PwmConfig | None = None,
    ) -> None:
        self._pins = (pwm_pin, dir_select_pin, r_en_pin, l_en_pin)
        self._pwm_config = pwm_config or Bts7960PwmConfig()
        self._period_ns = int(motor_const.NS_PER_S / self._pwm_config.frequency_hz)
        self._channel_dir: Path | None = None
        self._sign = -1.0 if invert else 1.0
        self._dir_select = None
        self._r_en = None
        self._l_en = None

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
        """Open the H-bridge (hardware PWM + gpiozero direction-select/enable pins)."""
        try:
            from gpiozero import DigitalOutputDevice
        except ImportError as err:
            raise MotorConnectionError(
                [str(p) for p in self._pins],
                "gpiozero/lgpio not available (Pi 5 hardware only)",
            ) from err

        _, dir_select_pin, r_en_pin, l_en_pin = self._pins
        try:
            self._dir_select = DigitalOutputDevice(dir_select_pin)
            self._r_en = DigitalOutputDevice(r_en_pin)
            self._l_en = DigitalOutputDevice(l_en_pin)
            self._r_en.on()
            self._l_en.on()
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

        logger.info("BTS7960 driver connected on pins %s (PWM %s)", self._pins, channel_dir)

    def disconnect(self) -> None:
        """Stop the drive, release the PWM channel, and close the GPIO lines."""
        self.stop_drive()
        if self._channel_dir is not None:
            try:
                (self._channel_dir / "enable").write_text("0")
            except OSError:
                logger.warning("Failed to disable drive PWM on disconnect", exc_info=True)
            self._channel_dir = None
        for device in (self._dir_select, self._r_en, self._l_en):
            if device is not None:
                device.close()
        self._dir_select = None
        self._r_en = None
        self._l_en = None

    def _set_output(self, duty: float) -> None:
        """Drive the H-bridge (through the demux) from a signed duty in [-1, 1]."""
        signed = self._sign * clamp(duty, -1.0, 1.0)
        if self._dir_select is None or self._channel_dir is None:
            raise MotorConnectionError([str(p) for p in self._pins], "driver not connected")
        # dir_select HIGH -> demux routes PWM to RPWM (forward); LOW -> LPWM.
        if signed >= 0:
            self._dir_select.on()
        else:
            self._dir_select.off()
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
