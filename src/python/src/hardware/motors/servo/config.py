"""Servo steering configuration.

Every servo parameter lives as a field on the ``ServoConfig`` pydantic model,
env-overridable via ``SERVO_*`` and the ``servo.toml`` file under
``src/config/hardware/motors/``. No loose module-level constants remain -- the
driver body reads everything from ``self._config``.
"""

from pydantic_settings import SettingsConfigDict

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class ServoConfig(HardwareBaseSettings):
    """Servo steering configuration (env-overridable, ``SERVO_`` prefix)."""

    model_config = SettingsConfigDict(env_prefix="servo_", toml_file=CONFIG_DIR / "motors" / "servo.toml")

    gpio_pin: int = 12
    """BCM pin driving the servo signal (Pi Zero physical 32 / PWM1).

    Informational only: which pin the PWM peripheral actually drives is fixed
    by the ``dtoverlay=pwm,pin=...`` line in ``/boot/firmware/config.txt``, not
    by this value. Kept for logging and to document the wiring.
    """

    pwmchip: int = 0
    """sysfs PWM controller index (``/sys/class/pwm/pwmchip<N>``)."""

    pwm_channel: int = 0
    """Channel within the PWM controller.

    With ``dtoverlay=pwm,pin=12,func=4`` the overlay exposes a single channel, so
    GPIO 12 is channel 0. A two-channel overlay (``pwm-2chan``) would map its
    second pin to channel 1.
    """

    min_pulse_us: float = 500.0
    """Pulse width (us) at full-left travel."""

    max_pulse_us: float = 2500.0
    """Pulse width (us) at full-right travel."""

    range_deg: float = 180.0
    """Total mechanical travel (deg) spanned between min and max pulse."""

    center_pulse_us: float = 1500.0
    """Pulse width (us) for wheels-straight."""

    reversed: bool = False
    """Invert steering direction if the servo is mounted so left commands turn right."""

    pwm_frequency_hz: int = 50
    """Servo PWM carrier frequency (Hz). The servo's frame period derives from
    this -- the sole servo timing constant that previously lived only as a
    module literal. ``move_steering_to`` maps a pulse-width in us to a duty
    cycle against this frame, so it must match the servo's expected 50 Hz
    signalling."""
