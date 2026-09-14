"""Servo steering configuration.

Every servo parameter lives as a field on the ``ServoConfig`` pydantic model,
env-overridable via ``SERVO_*`` and the ``servo.toml`` file under
``src/config/hardware/motors/``. No loose module-level constants remain -- the
driver body reads everything from ``self._config``.
"""

from pydantic_settings import SettingsConfigDict
from shared.config.generated.hardware.motors.servo_schema import HardwareMotorsServo

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class ServoConfig(HardwareBaseSettings, HardwareMotorsServo):
    """Servo steering configuration (env-overridable, ``SERVO_`` prefix).

    Subclasses the generated DTO purely to attach the TOML/env settings
    wiring; the field declarations (and their descriptions) are generated.
    """

    model_config = SettingsConfigDict(env_prefix="servo_", toml_file=CONFIG_DIR / "motors" / "servo.toml")
