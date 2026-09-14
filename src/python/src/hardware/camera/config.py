from pydantic_settings import SettingsConfigDict
from shared.config.generated.hardware.camera.config_schema import HardwareCameraConfig

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class Config(HardwareBaseSettings, HardwareCameraConfig):
    """Camera streaming configuration.

    Subclasses the generated DTO purely to attach the TOML/env settings
    wiring; the field declarations (and their descriptions) are generated.
    """

    model_config = SettingsConfigDict(
        env_prefix="camera_",
        toml_file=CONFIG_DIR / "camera" / "config.toml",
    )
