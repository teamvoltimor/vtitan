"""L298N drive H-bridge PWM configuration.

The PWM addressing/carrier defaults live on the ``L298nPwmConfig`` pydantic
model (env-overridable via ``L298N_PWM_*`` and the ``l298n.toml`` file under
``src/config/hardware/motors/``), so no loose module-level constants remain.
"""

from pydantic_settings import SettingsConfigDict
from shared.config.generated.hardware.motors.l298n_schema import HardwareMotorsL298n

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class L298nPwmConfig(HardwareBaseSettings, HardwareMotorsL298n):
    """L298N drive PWM configuration (env-overridable, ``L298N_PWM_`` prefix).

    Subclasses the generated DTO for the PWM/direction pins; ``standby_pin`` is
    declared here because the shared ``l298n.toml`` schema omits it (the
    TB6612FNG-only chip-enable line, ``None`` on this board's L298N).
    """

    model_config = SettingsConfigDict(
        env_prefix="l298n_pwm_",
        toml_file=CONFIG_DIR / "motors" / "l298n.toml",
    )

    standby_pin: int | None = None
    """BCM pin for a TB6612FNG's STBY (chip-enable) line, or ``None`` for an
    L298N, which has no standby line -- its per-channel enable (ENA/ENB) is
    the PWM pin, so disabling output is simply a duty write of 0."""
