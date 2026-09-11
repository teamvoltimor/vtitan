"""Competition match rules for the WRO Future Engineers challenge.

Lap counts and round timing live in ``src/config/competition_specs.toml``
(the single source of truth), loaded at import -- previously these were
hand-maintained literals here. The lighting/Z-layering constants that used to
sit in this module (``LightingSpec(s)``, ``LightingScenarios``, ``ZLayers``)
were never consumed anywhere and have been removed.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Final

from pydantic import BaseModel, ConfigDict

from shared.config.paths import SHARED_CONFIG_ROOT, load_toml_model

if TYPE_CHECKING:
    from pathlib import Path

DEFAULT_CONFIG_PATH: Path = SHARED_CONFIG_ROOT / "competition_specs.toml"


class _CompetitionSpecsModel(BaseModel):
    """Match rules loaded from competition_specs.toml."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    round_time_limit_s: float
    open_challenge_laps: int
    obstacle_challenge_laps: int


def _load() -> _CompetitionSpecsModel:
    return load_toml_model(_CompetitionSpecsModel, DEFAULT_CONFIG_PATH)


_specs = _load()


class CompetitionSpecs:
    """Official WRO Future Engineers match rules (round timing, lap counts)."""

    ROUND_TIME_LIMIT_S: Final[float] = _specs.round_time_limit_s  # Official round duration: 3 minutes
    OPEN_CHALLENGE_LAPS: Final[int] = _specs.open_challenge_laps  # Laps required per Open Challenge run
    OBSTACLE_CHALLENGE_LAPS: Final[int] = _specs.obstacle_challenge_laps  # Laps required per Obstacle Challenge run
