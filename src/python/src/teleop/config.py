from pydantic_settings import SettingsConfigDict
from shared.config.generated.hardware.teleop_schema import HardwareTeleop

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings


class Config(HardwareBaseSettings, HardwareTeleop):
    """Joystick teleop configuration.

    Bench-testing tool: maps `sensor_msgs/Joy` (published by the stock
    `joy` package's `joy_node`) to `ackermann_msgs/AckermannDriveStamped`
    commands on `/ackermann_cmd`, so `ackermann_motor_node` needs no changes.

    Subclasses the generated DTO purely to attach the TOML/env settings
    wiring; the field declarations (and their descriptions) are generated.
    The axis/button index meanings are documented in ``teleop.toml``.
    """

    model_config = SettingsConfigDict(env_prefix="joy_teleop_", toml_file=CONFIG_DIR / "teleop.toml")
