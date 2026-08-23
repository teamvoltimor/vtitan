"""Shared motor/driver constants for the vTitan Ackermann hardware.

Centralises the magic numbers that were duplicated across the DC-encoder and
servo driver configs (PWM chip/channel indices, time-base unit conversions,
carrier frequencies) so the two drivers cannot drift apart. Each driver's
``config.py`` re-exports the names it already published, so existing importers
are unaffected.
"""

from __future__ import annotations

# --- sysfs PWM controller addressing --------------------------------------
# The two-channel overlay (``dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4``)
# maps GPIO 12 (servo) to channel 0 and GPIO 13 (DC-encoder H-bridge) to channel 1.
DEFAULT_PWMCHIP = 0
"""sysfs PWM controller index (``/sys/class/pwm/pwmchip<N>``)."""

DEFAULT_DC_PWM_CHANNEL = 1
"""Channel for the DC-encoder H-bridge (GPIO 13 on the 2-chan overlay)."""

DEFAULT_SERVO_PWM_CHANNEL = 0
"""Channel for the servo (GPIO 12 on the 2-chan overlay)."""

# --- time-base unit conversions (sysfs PWM interface works in ns) ---------
NS_PER_S = 1_000_000_000
"""Nanoseconds per second."""

US_PER_SECOND = 1_000_000
"""Microseconds per second, for pulse-width to duty-cycle conversion."""

NS_PER_US = 1_000
"""Nanoseconds per microsecond."""

# --- carrier frequencies --------------------------------------------------
DC_PWM_FREQUENCY_HZ = 1000
"""H-bridge PWM carrier frequency (plain motor drive, above audible range)."""

SERVO_PWM_FREQUENCY_HZ = 50
"""Servo PWM carrier frequency (50 Hz position signalling)."""
