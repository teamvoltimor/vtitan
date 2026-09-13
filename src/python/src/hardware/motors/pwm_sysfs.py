"""Shared kernel hardware-PWM (/sys/class/pwm) export helpers.

Used by servo/driver.py, l298n/driver.py and bts7960/driver.py -- all drive their PWM
channel through the kernel's **hardware** PWM peripheral via sysfs rather
than gpiozero's software PWM (see either module's docstring for why the
hardware peripheral is required). Both used to declare an identical
``_SYSFS_PWM_ROOT``/``_EXPORT_TIMEOUT_S`` pair and duplicate the export-wait
polling loop verbatim; extracted here so the two drivers cannot silently
diverge on either.
"""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Callable

SYSFS_PWM_ROOT = Path("/sys/class/pwm")

EXPORT_TIMEOUT_S = 2.0
"""How long to wait for the kernel + udev to create and chgrp the channel dir.

Exporting a channel is asynchronous: the ``pwmN`` directory appears slightly
after the write returns, and the udev rule that makes it group-writable runs
later still. Writing immediately races both and fails with ENOENT or EACCES.
"""


def wait_for_pwm_channel_writable(channel_dir: Path, timeout_s: float = EXPORT_TIMEOUT_S) -> bool:
    """Poll ``channel_dir/duty_cycle`` until it exists and is writable, or timeout."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        duty = channel_dir / "duty_cycle"
        if duty.exists() and os.access(duty, os.W_OK):
            return True
        time.sleep(0.05)
    return False


def export_pwm_channel(
    chip: Path,
    channel: int,
    *,
    overlay_hint: str,
    fail: Callable[[str], Exception],
) -> Path:
    """Export ``channel`` on ``chip`` and wait for it to become writable.

    The servo, L298N and BTS7960 drivers each carried a near-identical copy of
    this; the only differences were the config attribute holding the channel,
    the ``dtoverlay`` hint and the driver's own error factory, so those are
    parameters and the control flow lives in one place.

    ``overlay_hint`` is the full ``dtoverlay=...`` line (quotes included).
    ``fail`` builds the driver-specific exception type from a message.
    """
    if not chip.is_dir():
        msg = f"{chip} not present -- hardware PWM overlay missing. Add {overlay_hint} to /boot/firmware/config.txt and reboot"
        raise fail(msg)

    channel_dir = chip / f"pwm{channel}"
    if not channel_dir.is_dir():
        try:
            (chip / "export").write_text(str(channel))
        except OSError as err:
            # EBUSY means someone already exported it, which is fine.
            if not channel_dir.is_dir():
                msg = f"cannot export PWM channel: {err}"
                raise fail(msg) from err

    if wait_for_pwm_channel_writable(channel_dir):
        return channel_dir

    msg = (
        f"{channel_dir} did not become writable within {EXPORT_TIMEOUT_S}s "
        "(is the service user in the 'gpio' group?)"
    )
    raise fail(msg)
