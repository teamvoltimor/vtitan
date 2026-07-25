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

DEFAULT_PWMCHIP = 0
"""sysfs PWM controller index (``/sys/class/pwm/pwmchip<N>``)."""

DEFAULT_PWM_CHANNEL = 0
"""Channel within the PWM controller.

With ``dtoverlay=pwm,pin=12,func=4`` the overlay exposes a single channel, so
GPIO 12 is channel 0. A two-channel overlay (``pwm-2chan``) would map its
second pin to channel 1.
"""

# Control constants
PWM_FREQUENCY_HZ = 50
"""Servo PWM carrier frequency."""

US_PER_SECOND = 1_000_000
"""Microseconds per second, for pulse-width to duty-cycle conversion."""

NS_PER_US = 1_000
"""Nanoseconds per microsecond -- the sysfs PWM interface works in ns."""


class ServoConfig(BaseSettings):
    """Servo steering configuration (env-overridable, ``SERVO_`` prefix)."""

    model_config = SettingsConfigDict(env_prefix="servo_")

    gpio_pin: int = DEFAULT_SERVO_GPIO_PIN
    """BCM pin driving the servo signal.

    Informational only: which pin the PWM peripheral actually drives is fixed
    by the ``dtoverlay=pwm,pin=...`` line in ``/boot/firmware/config.txt``, not
    by this value. Kept for logging and to document the wiring.
    """

    pwmchip: int = DEFAULT_PWMCHIP
    """sysfs PWM controller index."""

    pwm_channel: int = DEFAULT_PWM_CHANNEL
    """Channel within the PWM controller."""

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
