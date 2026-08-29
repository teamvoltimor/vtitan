"""Quadrature-encoder + closed-loop PID configuration.

Independent of which H-bridge (``l298n``/``bts7960``/...) is turning the
shaft this encoder reads -- env-overridable via ``ENCODER_*`` and the
``encoder.toml`` file under ``config/hardware/motors/``.
"""

from pydantic_settings import SettingsConfigDict

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

    pin_a: int = 16
    """BCM pin for the encoder's A channel."""

    pin_b: int = 20
    """BCM pin for the encoder's B channel."""

    counts_per_rev: float
    """Quadrature counts per wheel revolution -- bench-calibrated odometry
    calibration, see calibration.py for the derivation of the currently
    shipped value in encoder.toml. REQUIRED, no default: this is per-motor
    physical data, and a stale default would silently misconfigure a
    different motor's encoder rather than erroring at startup. Re-measure
    with scripts/hardware/calibrate_encoder.py whenever the motor changes."""

    max_rpm: float
    """Maximum achievable wheel rpm at 100% duty. A hard physical ceiling, not
    a tuning knob: the rpm clamp and the speed PID's feedforward slope both
    scale from it. REQUIRED, no default -- same reasoning as counts_per_rev.

    The feedforward slope is ``(1 - feedforward_deadband_duty) / max_rpm``, NOT
    ``1 / max_rpm``. Duty is affine in rpm, not proportional -- see
    feedforward_deadband_duty."""

    feedforward_deadband_duty: float = 0.0
    """Duty below which the motor does not turn at all, as a fraction (0-1).

    The speed PID's feedforward is ``offset + slope * setpoint_rpm``, and this
    is the offset. Without it the feedforward is a line through the origin
    while the real drivetrain is not, so it under-commands at every speed and
    the integrator absorbs the difference.

    Measured 2026-08-29 on this chassis, loaded, at counts_per_rev=60:

        rpm = 434.6 * duty - 86.7      (R^2 0.9999, 3 points 0.5/0.75/0.9)
        -> deadband at duty 0.200, and max_rpm 348 at duty 1.0

    Defaults 0.0 so an unconfigured motor keeps the old proportional-only
    behaviour rather than silently gaining an offset it was never tuned with.

    Not applied to a zero setpoint -- see PIDController._feed_forward_for, or
    a commanded stop would hold the deadband duty and the chassis would creep."""

    pid_kp: float = 0.010
    """Closed-loop speed PID proportional gain, tuned on hardware 2026-07-25."""

    pid_ki: float = 0.020
    """Closed-loop speed PID integral gain, tuned on hardware 2026-07-25."""

    pid_kd: float = 0.0
    """Closed-loop speed PID derivative gain (unused)."""

    max_duty: float = 1.0
    """Ceiling on the closed-loop PID's output magnitude (fraction, 0-1), fed
    into ``PIDController(output_min=-max_duty, output_max=max_duty)``. Default
    1.0 is full duty (no extra cap beyond the PID's own [-1, 1] range).
    Lowering this caps the actual PWM duty the drive can ever be commanded to,
    independent of ``counts_per_rev``/``max_rpm``/the drivetrain's
    ``max_speed_mps`` -- useful as an immediate hardware safety cap after a
    motor swap, before the encoder is recalibrated for the new motor."""
