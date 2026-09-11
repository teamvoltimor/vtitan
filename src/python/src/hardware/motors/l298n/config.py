"""L298N drive H-bridge PWM configuration.

The PWM addressing/carrier defaults live on the ``L298nPwmConfig`` pydantic
model (env-overridable via ``L298N_PWM_*`` and the ``l298n.toml`` file under
``src/config/hardware/motors/``), so no loose module-level constants remain.
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

    pwm_pin: int = 13
    """BCM pin driving the H-bridge's PWM enable line (L298N's ENA).

    Informational only, like ``ServoConfig.gpio_pin``: which pin the PWM
    peripheral actually drives is fixed by the ``dtoverlay=pwm-2chan,...``
    line in ``/boot/firmware/config.txt``, not by this value.
    """

    dir_a_pin: int = 5
    """BCM pin for the H-bridge's first direction input (L298N's IN3)."""

    dir_b_pin: int = 6
    """BCM pin for the H-bridge's second direction input (L298N's IN4)."""

    standby_pin: int | None = None
    """BCM pin for a TB6612FNG's STBY (chip-enable) line, or ``None`` for an
    L298N, which has no standby line -- its per-channel enable (ENA/ENB) is
    the PWM pin, so disabling output is simply a duty write of 0."""
