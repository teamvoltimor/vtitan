"""Shared configuration, constants, types, and enums for voldemorbot-platform.

This module provides a single source of truth for domain constants, type definitions,
and enumerations shared across all modules (backend, robot, simulation, frontend).

Exports:
    - Constants: Physical specifications, track dimensions, lighting configs
    - Enums: Domain types (Section, Direction, RobotState, etc.)
    - Types: TypedDict definitions for structured data
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
    WidthTypes,
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
from shared.config.types import (
    CameraParamsDict,
    ConfigResponseDict,
    CorridorWidths,
    DetectionDict,
    HealthCheckResponseDict,
    LightingConfig,
    MetricsDict,
    ParkingLotConfig,
    ParamPatchDict,
    RaceMetricsDict,
    RobotSnapshotDict,
    StartingConditions,
    StartingPosition,
    SystemStatusDict,
    TopicUpdateDict,
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
    "WidthTypes",
    "ZLayers",
    # Enums
    "Direction",
    "LightingScenario",
    "NodeHealth",
    "RiskLevel",
    "RobotState",
    "ScenarioType",
    "Section",
    # Types
    "CameraParamsDict",
    "ConfigResponseDict",
    "CorridorWidths",
    "DetectionDict",
    "HealthCheckResponseDict",
    "LightingConfig",
    "MetricsDict",
    "ParkingLotConfig",
    "ParamPatchDict",
    "RaceMetricsDict",
    "RobotSnapshotDict",
    "StartingConditions",
    "StartingPosition",
    "SystemStatusDict",
    "TopicUpdateDict",
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
