"""Shared configuration, constants, and enums for vtitan-platform.

This module provides a single source of truth for domain constants and
enumerations. Currently consumed only by ``src`` (Python); the Go
backend and TypeScript frontend do not import it.

Exports:
    - Constants: Physical specifications, track dimensions, lighting configs
    - Enums: Domain types (Section, Direction, RobotState, etc.)
    - Navigation Tuning: Runtime-configurable navigation parameters
"""

from shared.config.constants import (
    CorridorDimensions,
    DictKeys,
    ModelNames,
    RobotSpecs,
    TrackDimensions,
    TrafficSignSpecs,
    WallSpecs,
)
from shared.config.navigation_tuning import (
    ClearanceZones,
    EscapeManeuverParams,
    HeadingErrorZones,
    NavigationTuning,
    PurePursuitParams,
    SpeedControlParams,
)
from shared.domain.enums import (
    Direction,
    LightingScenario,
    NodeHealth,
    RiskLevel,
    RobotState,
    ScenarioType,
    Section,
)

__all__ = [
    "ClearanceZones",
    "CorridorDimensions",
    "DictKeys",
    "Direction",
    "EscapeManeuverParams",
    "HeadingErrorZones",
    "LightingScenario",
    "ModelNames",
    "NavigationTuning",
    "NodeHealth",
    "PurePursuitParams",
    "RiskLevel",
    "RobotSpecs",
    "RobotState",
    "ScenarioType",
    "Section",
    "SpeedControlParams",
    "TrackDimensions",
    "TrafficSignSpecs",
    "WallSpecs",
]
