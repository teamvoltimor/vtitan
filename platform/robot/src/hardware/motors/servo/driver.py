"""RC servo steering driver (single servo, counter-phase four-wheel steering).

One servo drives both axles through the linkage: the rear wheels turn by the
same amount as the front but in the OPPOSITE direction (confirmed on hardware
2026-07-25 -- this was previously documented as "parallel/crab steering",
which is the opposite geometry). The consequence for anything reasoning about
motion is that the chassis pivots about its centre, not the rear axle, and
yaws twice as fast as a front-steer car at the same angle -- see
src/simulation/kinematics.py.

Implements the ``SteeringDriver`` port for a hobby RC servo driven by a 50 Hz
PWM signal, using the kernel's **hardware** PWM peripheral via
``/sys/class/pwm``. The servo has no position feedback: reported speed is
always 0 and reported position is the last commanded value.

Why sysfs and not gpiozero: gpiozero's ``PWMOutputDevice`` under the pin
factory the Pi Zero actually uses (``LGPIOFactory``) generates the pulse train
in *software*, so kernel scheduling jitter lands directly on the servo's pulse
width and the servo visibly twitches even while holding a fixed angle.
Measured on hardware 2026-07-25: the twitching persisted with a single command
and no PWM rewrites at all, and on a fresh battery, which ruled out both
command traffic and supply sag. The hardware peripheral is immune to CPU load.

This requires the PWM overlay to be mapped onto the servo's pin. On this
robot (servo on GPIO 12) that is::

    dtoverlay=pwm,pin=12,func=4      # func=4 is ALT0, GPIO 12's PWM function

in ``/boot/firmware/config.txt`` (applied by ``scripts/provisioning/bootstrap-fresh-zero.sh``),
followed by a reboot. Without it there is no ``pwmchip`` to open and
:meth:`connect` fails with a message saying so -- deliberately, rather than
silently falling back to jittery software PWM.

Access is group-based, not root-only: Raspberry Pi OS ships a udev rule
(``99-com.rules``) that chgrp's ``/sys/class/pwm`` to ``gpio``, and the service
user is in that group.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, override

from src.hardware.exceptions import MotorConnectionError
from src.hardware.motors.base import (
    DEFAULT_STEERING_SPEED,
    STEERING_CENTER_DEG,
    SteeringDriver,
)
from src.hardware.motors.pwm_sysfs import (
    EXPORT_TIMEOUT_S,
    SYSFS_PWM_ROOT,
    wait_for_pwm_channel_writable,
)
from src.hardware.motors.servo.config import (
    NS_PER_US,
    PWM_FREQUENCY_HZ,
    US_PER_SECOND,
    ServoConfig,
)

if TYPE_CHECKING:
    from pathlib import Path

logger = logging.getLogger(__name__)

_PERIOD_NS = int(US_PER_SECOND / PWM_FREQUENCY_HZ) * NS_PER_US
"""One PWM frame in nanoseconds (20 ms at 50 Hz)."""

_PULSE_EPSILON_US = 1.0
"""Smallest pulse-width change worth writing.

``move_steering_to`` is called once per incoming ``/ackermann_cmd``, and a
controller holding a steady heading re-sends the same angle indefinitely.
Skipping identical writes avoids pointless syscalls. 1 us is well under a
hobby servo's own deadband (typically 2-10 us), so this cannot swallow a
movement the servo would have acted on.
"""


class Driver(SteeringDriver):
    """Single-servo steering via the kernel's hardware PWM (50 Hz, sysfs)."""

    def __init__(self, config: ServoConfig | None = None) -> None:
        """Build the driver without touching hardware (see :meth:`connect`).

        Args:
            config: Servo configuration; defaults to env-derived ``ServoConfig``.
        """
        self._config = config or ServoConfig()
        self._position = STEERING_CENTER_DEG
        self._channel_dir: Path | None = None
        self._pulse_us: float | None = None

    @property
    def _chip_dir(self) -> Path:
        return SYSFS_PWM_ROOT / f"pwmchip{self._config.pwmchip}"

    def _fail(self, reason: str) -> MotorConnectionError:
        return MotorConnectionError(str(self._config.gpio_pin), reason)

    def _export_channel(self) -> Path:
        """Export the PWM channel and wait for it to become writable."""
        chip = self._chip_dir
        if not chip.is_dir():
            msg = (
                f"{chip} not present -- hardware PWM overlay missing. Add "
                f"'dtoverlay=pwm,pin={self._config.gpio_pin},func=4' to "
                "/boot/firmware/config.txt and reboot"
            )
            raise self._fail(
                msg,
            )

        channel_dir = chip / f"pwm{self._config.pwm_channel}"
        if not channel_dir.is_dir():
            try:
                (chip / "export").write_text(str(self._config.pwm_channel))
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

    @override
    def connect(self) -> None:
        """Export the PWM channel, start the 50 Hz carrier, and centre the wheels.

        Raises:
            MotorConnectionError: overlay missing, or the channel never became
                writable.
        """
        channel_dir = self._export_channel()

        try:
            # Order matters: duty_cycle may never exceed period, so a stale
            # larger duty from a previous run would make the period write
            # fail. Zero the duty first, then set the frame, then enable.
            (channel_dir / "duty_cycle").write_text("0")
            (channel_dir / "period").write_text(str(_PERIOD_NS))
            (channel_dir / "enable").write_text("1")
        except OSError as err:
            msg = f"PWM init failed: {err}"
            raise self._fail(msg) from err

        self._channel_dir = channel_dir
        self.center_steering()
        logger.info(
            "Servo steering connected on hardware PWM %s (GPIO %d)",
            channel_dir,
            self._config.gpio_pin,
        )

    @override
    def disconnect(self) -> None:
        """Stop the PWM carrier and release the channel."""
        if self._channel_dir is not None:
            try:
                (self._channel_dir / "enable").write_text("0")
            except OSError:
                logger.warning("Failed to disable servo PWM on disconnect", exc_info=True)
            self._channel_dir = None
        # Forget the cached pulse so the first write after reconnecting is not
        # skipped as a no-op (the carrier was disabled in between).
        self._pulse_us = None

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
            MotorConnectionError: if called before :meth:`connect`, or the
                write fails.
        """
        del speed  # servo self-paces; no host-side rate control
        if self._channel_dir is None:
            msg = "driver not connected"
            raise self._fail(msg)

        pulse_us = self._position_to_pulse_us(position)
        if self._pulse_us is None or abs(pulse_us - self._pulse_us) >= _PULSE_EPSILON_US:
            try:
                (self._channel_dir / "duty_cycle").write_text(str(int(pulse_us * NS_PER_US)))
            except OSError as err:
                msg = f"PWM duty_cycle write failed: {err}"
                raise self._fail(msg) from err
            self._pulse_us = pulse_us
        self._position = position

    @override
    def get_steering_position(self) -> float:
        """Last commanded steering angle (servo has no position feedback)."""
        return self._position
