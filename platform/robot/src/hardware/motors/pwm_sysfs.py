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
