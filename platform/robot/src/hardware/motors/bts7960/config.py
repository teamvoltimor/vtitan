"""BTS7960 (IBT-2 module) drive H-bridge PWM configuration.

Same shape as ``l298n/config.py``: a single hardware-PWM channel + carrier
frequency. Unlike the L298N, BTS7960 natively wants two independent PWM
inputs (``RPWM``/``LPWM``) -- but only one direction is ever active at a
time, so this driver shares ONE hardware-PWM channel through an external
2:1 demux (``RPWM = PWM AND dir``, ``LPWM = PWM AND NOT dir``; see
``docs/bts7960-ibt2-wiring.md``) rather than needing a second hardware-PWM
channel the Pi Zero 2 W's SoC doesn't have (it has exactly 2 PWM engines,
both already spoken for by the servo + this channel).
"""

from pydantic_settings import SettingsConfigDict

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class Bts7960PwmConfig(HardwareBaseSettings):
    """BTS7960 drive PWM configuration (env-overridable, ``BTS7960_PWM_`` prefix)."""

    model_config = SettingsConfigDict(
        env_prefix="bts7960_pwm_",
        toml_file=CONFIG_DIR / "motors" / "bts7960.toml",
    )

    pwmchip: int = 0
    """sysfs PWM controller index (``/sys/class/pwm/pwmchip<N>``)."""

    pwm_channel: int = 1
    """Channel within the PWM controller -- same channel the L298N's ``ENA``
    used (GPIO 13 under the ``pwm-2chan`` overlay), now feeding the external
    demux instead of the H-bridge directly."""

    frequency_hz: int = 1000
    """H-bridge PWM carrier frequency. Same reasoning as ``l298n/config.py``."""
