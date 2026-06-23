"""Servo steering constants and configuration.

All servo magic numbers live here as named constants and feed the
``ServoConfig`` settings object, so the driver body stays literal-free. Every
field is overridable via a ``SERVO_*`` environment variable (e.g.
``SERVO_GPIO_PIN``), matching the ``Config`` pattern in ``motors/config.py``.
"""

from pydantic_settings import BaseSettings, SettingsConfigDict

# Hardware defaults
DEFAULT_SERVO_GPIO_PIN = 12
"""BCM pin driving the servo signal (Pi Zero physical 32 / PWM1)."""

DEFAULT_MIN_PULSE_US = 500.0
"""Pulse width (us) at full-left travel."""

DEFAULT_MAX_PULSE_US = 2500.0
"""Pulse width (us) at full-right travel."""

DEFAULT_RANGE_DEG = 180.0
"""Total mechanical travel (deg) spanned between min and max pulse."""

DEFAULT_CENTER_PULSE_US = 1500.0
"""Pulse width (us) for wheels-straight."""

# Control constants
PWM_FREQUENCY_HZ = 50
"""Servo PWM carrier frequency."""

US_PER_SECOND = 1_000_000
"""Microseconds per second, for pulse-width to duty-cycle conversion."""


class ServoConfig(BaseSettings):
    """Servo steering configuration (env-overridable, ``SERVO_`` prefix)."""

    model_config = SettingsConfigDict(env_prefix="servo_")

    gpio_pin: int = DEFAULT_SERVO_GPIO_PIN
    """BCM pin driving the servo signal."""

    min_pulse_us: float = DEFAULT_MIN_PULSE_US
    """Pulse width (us) at full-left travel."""

    max_pulse_us: float = DEFAULT_MAX_PULSE_US
    """Pulse width (us) at full-right travel."""

    range_deg: float = DEFAULT_RANGE_DEG
    """Total mechanical travel (deg) spanned between min and max pulse."""

    center_pulse_us: float = DEFAULT_CENTER_PULSE_US
    """Pulse width (us) for wheels-straight."""

    reversed: bool = False
    """Invert steering direction if the servo is mounted so left commands turn right."""
