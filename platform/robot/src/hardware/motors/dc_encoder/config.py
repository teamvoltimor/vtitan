"""DC-encoder drive H-bridge PWM configuration.

All PWM magic numbers live here as named constants and feed the
``DcMotorPwmConfig`` settings object, matching the pattern in
``motors/servo/config.py``. Every field is overridable via a
``DC_MOTOR_PWM_*`` environment variable.
"""

from pydantic_settings import SettingsConfigDict

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings

DEFAULT_PWMCHIP = 0
"""sysfs PWM controller index (``/sys/class/pwm/pwmchip<N>``)."""

DEFAULT_PWM_CHANNEL = 1
"""Channel within the PWM controller.

The two-channel overlay (``dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4``)
maps GPIO 12 (the servo, see ``servo/config.py``) to channel 0 and GPIO 13
(this driver's PWM pin, ``_DcEncoderPins.pwm_pin`` in ``ackermann_motor_node.py``)
to channel 1.
"""

DEFAULT_PWM_FREQUENCY_HZ = 1000
"""H-bridge PWM carrier frequency.

Unlike the servo's 50 Hz pulse-width-encoded position, this is a plain motor
drive carrier: an L298N/TB6612 switches cleanly well above audible range, so
1 kHz was picked for headroom rather than measured against a spec limit.
"""

NS_PER_S = 1_000_000_000
"""Nanoseconds per second -- the sysfs PWM interface works in ns."""


class DcMotorPwmConfig(HardwareBaseSettings):
    """DC-encoder drive PWM configuration (env-overridable, ``DC_MOTOR_PWM_`` prefix)."""

    model_config = SettingsConfigDict(
        env_prefix="dc_motor_pwm_",
        toml_file=CONFIG_DIR / "motors" / "dc_encoder.toml",
    )

    pwmchip: int = DEFAULT_PWMCHIP
    """sysfs PWM controller index."""

    pwm_channel: int = DEFAULT_PWM_CHANNEL
    """Channel within the PWM controller."""

    frequency_hz: int = DEFAULT_PWM_FREQUENCY_HZ
    """H-bridge PWM carrier frequency."""
