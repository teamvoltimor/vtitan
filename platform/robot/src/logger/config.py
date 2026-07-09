"""Logging configuration constants."""

from pathlib import Path

from dotenv import load_dotenv
from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

# Load the robot-level .env (platform/robot/.env) into the process environment
# on import, so every pydantic-settings config in the codebase -- which all
# read from os.environ -- picks up local overrides. No-op when the file is
# absent (e.g. CI/dev without a .env, or systemd's own EnvironmentFile=
# already populated the environment), so it's safe. This module is imported
# by essentially every hardware driver via src.logger.configure_json_logging,
# making it the earliest common import point for the dev/pixi-run path.
load_dotenv(Path(__file__).resolve().parents[2] / ".env")


class _LoggerSettings(BaseSettings):
    """Logging configuration read from the environment."""

    model_config = SettingsConfigDict(env_prefix="")

    log_level: str = Field(default="INFO", validation_alias="LOG_LEVEL")
    """Logging level for the application. Supported levels are: DEBUG, INFO, WARNING, ERROR, CRITICAL."""

    competition_mode: bool = Field(default=False, validation_alias="COMPETITION_MODE")
    """Whether to enable competition mode. When enabled, the logger will use a more compact format
    suitable for competition environments."""


_settings = _LoggerSettings()

LOG_LEVEL: str = _settings.log_level
COMPETITION_MODE: bool = _settings.competition_mode


def is_competition_mode() -> bool:
    """Check if competition mode is enabled."""
    return COMPETITION_MODE
