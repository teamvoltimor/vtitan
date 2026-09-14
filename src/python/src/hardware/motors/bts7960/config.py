"""BTS7960 (IBT-2 module) drive H-bridge PWM configuration.

Genuinely independent RPWM/LPWM, matching every documented reference
wiring for this chip (see docs/bts7960-ibt2-wiring.md for why a shared
single PWM signal doesn't work: with RPWM/LPWM tied together, the chip sees
either both-high or both-low every cycle, which per its own truth table
means continuous Fast Brake / Coast, never actual drive, regardless of
R_EN/L_EN). R_EN/L_EN are held permanently HIGH -- direction is selected
entirely by which PWM channel carries a nonzero duty.

Forward (RPWM) stays on the Pi's one free hardware-PWM engine, since it's
the performance-critical, overwhelmingly-more-frequent direction. Reverse
(LPWM) rides software PWM (gpiozero, ``lgpio`` pin factory) on a second
GPIO -- confirmed tolerable jitter for a spinning motor with encoder feedback
and mechanical inertia (unlike the steering servo, which needs hardware PWM
because jitter there directly shows up as visible twitching at a fixed
position -- see servo/driver.py's docstring).
"""

from pydantic_settings import SettingsConfigDict
from shared.config.generated.hardware.motors.bts7960_schema import HardwareMotorsBts7960

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class Bts7960PwmConfig(HardwareBaseSettings, HardwareMotorsBts7960):
    """BTS7960 drive PWM configuration (env-overridable, ``BTS7960_PWM_`` prefix).

    Subclasses the generated DTO purely to attach the TOML/env settings
    wiring; the field declarations (and their descriptions) are generated.
    """

    model_config = SettingsConfigDict(
        env_prefix="bts7960_pwm_",
        toml_file=CONFIG_DIR / "motors" / "bts7960.toml",
    )
