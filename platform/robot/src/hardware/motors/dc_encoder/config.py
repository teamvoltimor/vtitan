"""DC-encoder drive H-bridge PWM configuration.

All PWM magic numbers live here as named constants and feed the
``DcMotorPwmConfig`` settings object, matching the pattern in
``motors/servo/config.py``. Every field is overridable via a
``DC_MOTOR_PWM_*`` environment variable.
"""

from pydantic_settings import SettingsConfigDict

from src.hardware.motors import constants as motor_const
from src.hardware.motors.dc_encoder.calibration import (
    DEFAULT_COUNTS_PER_REV,
    DEFAULT_MAX_RPM,
)
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings

DEFAULT_PWMCHIP = motor_const.DEFAULT_PWMCHIP
"""sysfs PWM controller index (``/sys/class/pwm/pwmchip<N>``)."""

DEFAULT_PWM_CHANNEL = motor_const.DEFAULT_DC_PWM_CHANNEL
"""Channel within the PWM controller.

The two-channel overlay (``dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4``)
maps GPIO 12 (the servo, see ``servo/config.py``) to channel 0 and GPIO 13
(this driver's PWM pin, ``_DcEncoderPins.pwm_pin`` in ``ackermann_motor_node.py``)
to channel 1.
"""

DEFAULT_PWM_FREQUENCY_HZ = motor_const.DC_PWM_FREQUENCY_HZ
"""H-bridge PWM carrier frequency.

Unlike the servo's 50 Hz pulse-width-encoded position, this is a plain motor
drive carrier: an L298N/TB6612 switches cleanly well above audible range, so
1 kHz was picked for headroom rather than measured against a spec limit.
"""


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

    counts_per_rev: float = DEFAULT_COUNTS_PER_REV
    """Quadrature counts per wheel revolution -- bench-calibrated odometry
    calibration (measured 2026-07-25), see calibration.py for the derivation.
    Used to turn raw counts into revolutions/distance."""

    max_rpm: float = DEFAULT_MAX_RPM
    """Maximum achievable wheel rpm (measured 2026-07-25). A hard physical
    ceiling, not a tuning knob: the speed PID's feedforward (1/max_rpm) and the
    rpm clamp both scale from it, so it must track a re-measurement rather than
    a datasheet figure."""

    pid_kp: float = 0.010
    """Closed-loop speed PID proportional gain, tuned on hardware 2026-07-25."""

    pid_ki: float = 0.020
    """Closed-loop speed PID integral gain, tuned on hardware 2026-07-25."""

    pid_kd: float = 0.0
    """Closed-loop speed PID derivative gain (unused)."""
