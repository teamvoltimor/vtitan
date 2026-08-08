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
    ColorNames,
    CorridorDimensions,
    DictKeys,
    LightingScenarios,
    ModelNames,
    RandomizationRanges,
    RobotSpecs,
    TrackDimensions,
    TrafficSignSpecs,
    WallSpecs,
    ZLayers,
)
from shared.config.coordinate_transform import CoordinateTransform
from shared.config.enums import (
    Direction,
    LightingScenario,
    NodeHealth,
    RiskLevel,
    RobotState,
    ScenarioType,
    Section,
)
from shared.config.navigation_tuning import (
    ClearanceZones,
    EscapeManeuverParams,
    HeadingErrorZones,
    NavigationTuning,
    PurePursuitParams,
    SpeedControlParams,
)

__all__ = [
    # Constants
    "ColorNames",
    "CorridorDimensions",
    "DictKeys",
    "LightingScenarios",
    "ModelNames",
    "RandomizationRanges",
    "RobotSpecs",
    "TrackDimensions",
    "TrafficSignSpecs",
    "WallSpecs",
    "ZLayers",
    # Enums
    "Direction",
    "LightingScenario",
    "NodeHealth",
    "RiskLevel",
    "RobotState",
    "ScenarioType",
    "Section",
    # Navigation Tuning
    "ClearanceZones",
    "EscapeManeuverParams",
    "HeadingErrorZones",
    "NavigationTuning",
    "PurePursuitParams",
    "SpeedControlParams",
    # Utilities
    "CoordinateTransform",
]
