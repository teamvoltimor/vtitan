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
    used (GPIO 13 under the ``pwm-2chan`` overlay), now feeding RPWM only."""

    frequency_hz: int = 1000
    """PWM carrier frequency, shared by both the hardware and software channel."""

    forward_pwm_pin: int = 13
    """BCM pin driving RPWM (forward) via the Pi's hardware PWM engine.

    Same physical pin the L298N's ``ENA`` used. Informational only, like
    ``ServoConfig.gpio_pin`` -- fixed by the ``dtoverlay=pwm-2chan,...`` line,
    not by this value.
    """

    reverse_pwm_pin: int = 26
    """BCM pin driving LPWM (reverse) via software PWM (gpiozero).

    Previously wired for the abandoned demux ``dir_select`` line; repurposed
    here since the wire was already run to the module.
    """

    r_en_pin: int = 6
    """BCM pin wired to the module's ``R_EN``. Held permanently HIGH -- does
    not select direction (see module docstring)."""

    l_en_pin: int = 5
    """BCM pin wired to the module's ``L_EN``. Held permanently HIGH -- does
    not select direction (see module docstring)."""
