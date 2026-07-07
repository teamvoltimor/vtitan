"""Environment variable handling utilities."""

import logging
import os
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import TypeVar, cast, override

from dotenv import load_dotenv

logger = logging.getLogger(__name__)

# Load the robot-level .env (robot/.env, sibling of src/) into the process
# environment on import, so both EnvVar (below) and the pydantic-settings
# hardware configs — all of which read os.environ — pick up local overrides.
# No-op when the file is absent (e.g. CI/dev without a .env), so it's safe.
load_dotenv(Path(__file__).resolve().parent.parent / ".env")

T = TypeVar("T")


@dataclass
class EnvVar[T]:
    """A dataclass representing an environment variable with a key and default value."""

    key: str
    default: T
    cast: Callable[[str], T] | None = None
    raw_value: str | None = field(init=False, repr=False)
    value: T = field(init=False)

    def __post_init__(self):
        self.raw_value = os.getenv(self.key)

        if self.raw_value is None:
            self.value = self.default
            logger.debug("env loaded", extra={"details": {"key": self.key, "value": self.value, "source": "default"}})
            return

        if self.raw_value == "":
            self.value = self.default
            logger.warning(
                "Environment variable '%s' is set to an empty string. Using default '%s'.", self.key, self.default,
            )
            logger.debug(
                "env loaded",
                extra={
                    "details": {
                        "key": self.key,
                        "value": self.value,
                        "source": "default",
                        "reason": "empty string ignored",
                    },
                },
            )
            return

        if self.cast is None:
            self.value = cast("T", self.raw_value)
            logger.debug(
                "env loaded", extra={"details": {"key": self.key, "value": self.value, "source": "environment"}},
            )
            return

        try:
            self.value = self.cast(self.raw_value)
            logger.debug(
                "env loaded", extra={"details": {"key": self.key, "value": self.value, "source": "environment"}},
            )
        except (ValueError, TypeError) as e:
            logger.warning(
                "Environment variable '%s' has invalid value '%s': %s. Using default '%s'.",
                self.key,
                self.raw_value,
                e,
                self.default,
            )
            self.value = self.default
            logger.debug(
                "env loaded",
                extra={"details": {"key": self.key, "value": self.value, "source": "default", "reason": "cast failed"}},
            )

    def __fspath__(self) -> str:
        return str(self.value)

    def __truediv__(self, other: str | Path) -> Path:
        return Path(str(self.value)) / other

    def __rtruediv__(self, other: str | Path) -> Path:
        return other / Path(str(self.value))

    @override
    def __str__(self) -> str:
        return str(self.value)

    @override
    def __repr__(self) -> str:
        return f"EnvVar(key={self.key}, value={self.value})"


def bool_cast(value: str) -> bool:
    """Cast string to boolean."""
    return value.lower() in ("true", "1", "yes")
