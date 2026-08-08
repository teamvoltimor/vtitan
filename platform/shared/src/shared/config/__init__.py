"""Shared configuration, constants, and enums for vtitan-platform.

This module provides a single source of truth for domain constants and
enumerations. Currently consumed only by ``platform/robot`` (Python); the Go
backend and TypeScript frontend do not import it.

Exports:
    - Constants: Physical specifications, track dimensions, lighting configs
    - Enums: Domain types (Section, Direction, RobotState, etc.)
    - Navigation Tuning: Runtime-configurable navigation parameters
    - Coordinate Transform: Utilities for coordinate space conversions
"""

from shared.config.constants import (
    CorridorDimensions,
    DictKeys,
    LightingScenarios,
    ModelNames,
    RobotSpecs,
    TrackDimensions,
    TrafficSignSpecs,
    WallSpecs,
    ZLayers,
)
from shared.config.coordinate_transform import CoordinateTransform
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
    "CoordinateTransform",
    "CorridorDimensions",
    "DictKeys",
    "Direction",
    "EscapeManeuverParams",
    "HeadingErrorZones",
    "LightingScenario",
    "LightingScenarios",
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
    "ZLayers",
]
