"""Shared motor/driver constants for the vTitan Ackermann hardware.

Holds only the sysfs PWM time-base unit conversions that are pure physical
facts of the interface (not tunable per deployment). The config-shaped values
-- PWM chip/channel indices and carrier frequencies -- live on the
``ServoConfig`` / ``DcMotorPwmConfig`` pydantic models (TOML + env backed), so
they are not duplicated here.
"""

from __future__ import annotations

NS_PER_S = 1_000_000_000
"""Nanoseconds per second -- the sysfs PWM interface works in nanoseconds."""

US_PER_SECOND = 1_000_000
"""Microseconds per second, for pulse-width to duty-cycle conversion."""

NS_PER_US = 1_000
"""Nanoseconds per microsecond."""
