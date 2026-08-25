"""Quadrature-encoder + closed-loop PID configuration.

Independent of which H-bridge (``l298n``/``bts7960``/...) is turning the
shaft this encoder reads -- env-overridable via ``ENCODER_*`` and the
``encoder.toml`` file under ``config/hardware/motors/``.
"""

from pydantic_settings import SettingsConfigDict

from src.hardware.motors.encoder.calibration import (
    DEFAULT_COUNTS_PER_REV,
    DEFAULT_MAX_RPM,
)
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings

# wheel_diameter_m is deliberately NOT a field here: it derives from
# RobotSpecs.WHEEL_RADIUS (calibration.py's DEFAULT_WHEEL_DIAMETER_M), and a
# second independent literal in this TOML could drift from that single
# source. Callers building QuadratureEncoder take the calibration default
# directly, or pass an explicit override in code.


class EncoderConfig(HardwareBaseSettings):
    """Quadrature-encoder configuration (env-overridable, ``ENCODER_`` prefix)."""

    model_config = SettingsConfigDict(
        env_prefix="encoder_",
        toml_file=CONFIG_DIR / "motors" / "encoder.toml",
    )

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
