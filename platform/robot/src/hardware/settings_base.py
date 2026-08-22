"""Shared base for every hardware driver's ``Config``.

Reads from its own TOML file under ``platform/robot/config/hardware/`` instead
of ``.env``.

An env var of the same name still overrides the toml value (env_settings
runs before the toml source below), so systemd EnvironmentFile= and local
.env overrides for quick tweaks keep working without editing the committed
config tree.

A hardware profile (``VTITAN_HARDWARE_PROFILE``, see
``shared.config.hardware_profile``) additionally layers an optional
``<toml's dir>/profiles/<name>/<toml's filename>`` on top of the base file,
deep-merged, so an overlay only needs to declare the keys it changes. Only
drivers whose config actually varies by profile (currently just the servo,
for an alternate-travel servo) need an overlay file to exist -- every other
driver's TOML tree is untouched and behaves exactly as before.
"""

from pathlib import Path
from typing import Annotated

from pydantic import BeforeValidator
from pydantic_settings import BaseSettings, PydanticBaseSettingsSource, TomlConfigSettingsSource
from shared.config.hardware_profile import active_profiles

# settings_base.py -> hardware -> src -> robot
ROBOT_ROOT: Path = Path(__file__).resolve().parents[2]
CONFIG_DIR: Path = ROBOT_ROOT / "config" / "hardware"
SAFE_SHUTDOWN_BOTH_SCRIPT: Path = ROBOT_ROOT / "scripts" / "provisioning" / "safe-shutdown-both.sh"


def _parse_hex_int(value: object) -> object:
    """Coerce hex/octal/decimal strings (e.g. "0x04D8") to int; pass through ints."""
    if isinstance(value, str):
        return int(value, 0)
    return value


HexInt = Annotated[int, BeforeValidator(_parse_hex_int)]
"""An int field that also accepts hex/octal/decimal string literals (e.g. an I2C address as "0x3C")."""


def _profile_overlay_paths(base_toml: Path) -> list[Path]:
    """``<base_toml's dir>/profiles/<name>/<base_toml's filename>`` for each active profile.

    A missing overlay file is not an error here -- ``TomlConfigSettingsSource``
    silently skips any path in its list that doesn't exist, the same way a
    missing ``<group>.toml`` falls back to defaults in
    ``NavigationTuning.load_from_toml_dirs``.
    """
    return [base_toml.parent / "profiles" / name / base_toml.name for name in active_profiles()]


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
        """Prefer init/env, then fall back to the driver's TOML config file (+ profile overlay)."""
        base_toml = settings_cls.model_config.get("toml_file")
        if isinstance(base_toml, (str, Path)):
            base_toml = Path(base_toml)
            toml_files = [base_toml, *_profile_overlay_paths(base_toml)]
        else:
            toml_files = base_toml
        return (
            init_settings,
            env_settings,
            TomlConfigSettingsSource(settings_cls, toml_file=toml_files, deep_merge=True),
            dotenv_settings,
            file_secret_settings,
        )
