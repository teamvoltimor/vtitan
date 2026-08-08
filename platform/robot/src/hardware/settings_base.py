"""Shared base for every hardware driver's ``Config``.

Reads from its own TOML file under ``platform/robot/config/hardware/`` instead
of ``.env``.

An env var of the same name still overrides the toml value (env_settings
runs before the toml source below), so systemd EnvironmentFile= and local
.env overrides for quick tweaks keep working without editing the committed
config tree.
"""

from pathlib import Path

from pydantic_settings import BaseSettings, PydanticBaseSettingsSource, TomlConfigSettingsSource

# settings_base.py -> hardware -> src -> robot
ROBOT_ROOT: Path = Path(__file__).resolve().parents[2]
CONFIG_DIR: Path = ROBOT_ROOT / "config" / "hardware"


class HardwareBaseSettings(BaseSettings):
    """Base class for hardware driver configs sourced from ``config/hardware/*.toml``.

    Subclasses set ``model_config``'s ``toml_file`` to their own file under
    :data:`CONFIG_DIR`.
    """

    @classmethod
    def settings_customise_sources(
        cls,
        settings_cls: type[BaseSettings],
        init_settings: PydanticBaseSettingsSource,
        env_settings: PydanticBaseSettingsSource,
        dotenv_settings: PydanticBaseSettingsSource,
        file_secret_settings: PydanticBaseSettingsSource,
    ) -> tuple[PydanticBaseSettingsSource, ...]:
        """Prefer init/env, then fall back to the driver's TOML config file."""
        return (
            init_settings,
            env_settings,
            TomlConfigSettingsSource(settings_cls),
            dotenv_settings,
            file_secret_settings,
        )
