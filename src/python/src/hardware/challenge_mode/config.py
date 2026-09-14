from pydantic_settings import SettingsConfigDict
from shared.config.generated.hardware.challenge_mode_schema import HardwareChallengeMode

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class Config(HardwareBaseSettings, HardwareChallengeMode):
    """Configuration for the challenge-mode jumper GPIO driver.

    Subclasses the generated DTO purely to attach the TOML/env settings
    wiring; the field declarations (and their descriptions) are generated.
    """

    model_config = SettingsConfigDict(env_prefix="", toml_file=CONFIG_DIR / "challenge_mode.toml")
