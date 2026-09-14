"""Shared base for every hardware driver's ``Config``.

Sourced from a TOML file under ``src/config/hardware/`` instead of ``.env``.
The TOML/env/profile precedence and the shared parse cache come from
:class:`shared.config.settings_base.TomlSettings`, which any non-hardware
consumer can use too.
"""

from pathlib import Path
from typing import Annotated

from pydantic import BeforeValidator
from shared.config.settings_base import TomlSettings

# settings_base.py -> hardware -> src -> robot (src/python)
ROBOT_ROOT: Path = Path(__file__).resolve().parents[2]
# hardware/ is shared with src/go (motor/button/display/imu/lidar drivers
# read the same TOML), so it lives in src/config, not src/python/config.
CONFIG_DIR: Path = ROBOT_ROOT.parent / "config" / "hardware"
SAFE_SHUTDOWN_BOTH_SCRIPT: Path = ROBOT_ROOT / "scripts" / "provisioning" / "safe-shutdown-both.sh"


def parse_hex_int(value: str) -> int:
    """Parse a hex/octal/decimal string (e.g. "0x04D8") into an int."""
    return int(value, 0)


def _parse_hex_int(value: object) -> object:
    """Coerce hex/octal/decimal strings (e.g. "0x04D8") to int; pass through non-strings."""
    if isinstance(value, str):
        return parse_hex_int(value)
    return value


HexInt = Annotated[int, BeforeValidator(_parse_hex_int)]
"""An int field that also accepts hex/octal/decimal string literals (e.g. an I2C address as "0x3C")."""


class HardwareBaseSettings(TomlSettings):
    """Base class for hardware driver configs sourced from ``src/config/hardware/*.toml``.

    Subclasses set ``model_config``'s ``toml_file`` to their own file under
    :data:`CONFIG_DIR`.
    """
