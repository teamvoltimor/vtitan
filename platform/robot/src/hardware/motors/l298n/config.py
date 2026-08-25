"""L298N drive H-bridge PWM configuration.

The PWM addressing/carrier defaults live on the ``L298nPwmConfig`` pydantic
model (env-overridable via ``L298N_PWM_*`` and the ``l298n.toml`` file under
``config/hardware/motors/``), so no loose module-level constants remain.
"""

from pydantic_settings import SettingsConfigDict

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class L298nPwmConfig(HardwareBaseSettings):
    """L298N drive PWM configuration (env-overridable, ``L298N_PWM_`` prefix)."""

    model_config = SettingsConfigDict(
        env_prefix="l298n_pwm_",
        toml_file=CONFIG_DIR / "motors" / "l298n.toml",
    )

    pwmchip: int = 0
    """sysfs PWM controller index (``/sys/class/pwm/pwmchip<N>``)."""

    pwm_channel: int = 1
    """Channel within the PWM controller.

    The two-channel overlay (``dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4``)
    maps GPIO 12 (the servo, see ``servo/config.py``) to channel 0 and GPIO 13
    (this driver's PWM pin, ``_L298nPins.pwm_pin`` in ``ackermann_motor_node.py``)
    to channel 1.
    """

    frequency_hz: int = 1000
    """H-bridge PWM carrier frequency.

    Unlike the servo's 50 Hz pulse-width-encoded position, this is a plain motor
    drive carrier: an L298N/TB6612 switches cleanly well above audible range, so
    1 kHz was picked for headroom rather than measured against a spec limit.
    """
