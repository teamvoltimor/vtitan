"""Competition rules, lighting randomization, and Z-layering constants.

Hand-maintained — none of these come from track.toml/robot.toml.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

from shared.domain.enums import LightingScenario


class CompetitionSpecs:
    """Official WRO Future Engineers match rules (round timing, lap counts)."""

    ROUND_TIME_LIMIT_S: Final[float] = 180.0  # Official round duration: 3 minutes
    OPEN_CHALLENGE_LAPS: Final[int] = 3  # Laps required per Open Challenge run
    OBSTACLE_CHALLENGE_LAPS: Final[int] = 3  # Laps required per Obstacle Challenge run


class LightingSpecs:
    """Simulation lighting parameters."""

    # Sun intensity range
    SUN_INTENSITY_MIN = 0.5
    SUN_INTENSITY_MAX = 1.0  # Clamped to 1.0 for valid SDF

    # Ambient light range
    AMBIENT_INTENSITY_MIN = 0.3
    AMBIENT_INTENSITY_MAX = 0.8

    # Direction variance (radians)
    DIRECTION_VARIANCE = 0.3


@dataclass(frozen=True, slots=True)
class LightingSpec:
    """Randomization ranges and shadow behavior for one lighting scenario.

    Attributes:
        intensity:     (min, max) sun intensity range [0.0, 1.0].
        ambient:       (min, max) ambient intensity range [0.0, 1.0].
        direction:     ((x_min, x_max), (y_min, y_max), z) sun direction, z fixed.
        cast_shadows:  Whether the sun casts shadows in this scenario.
    """

    intensity: tuple[float, float]
    ambient: tuple[float, float]
    direction: tuple[tuple[float, float], tuple[float, float], float]
    cast_shadows: bool


class LightingScenarios:
    """Table-driven lighting scenario specifications.

    Each scenario defines the ranges for intensity, ambient intensity,
    direction, and shadow casting behavior. This table-driven approach
    eliminates 50+ lines of duplicated branching code.
    """

    SPECS: Final[dict[LightingScenario, LightingSpec]] = {
        LightingScenario.DIRECT_SUNLIGHT: LightingSpec(
            intensity=(0.9, 1.0),
            ambient=(0.3, 0.4),
            direction=((-0.7, -0.3), (-0.7, -0.3), -1.0),
            cast_shadows=True,
        ),
        LightingScenario.CLOUDY: LightingSpec(
            intensity=(0.6, 0.75),
            ambient=(0.5, 0.6),
            direction=((-0.5, -0.5), (-0.5, -0.5), -1.0),
            cast_shadows=True,
        ),
        LightingScenario.INDOOR_BRIGHT: LightingSpec(
            intensity=(0.7, 0.85),
            ambient=(0.6, 0.7),
            direction=((0.0, 0.0), (0.0, 0.0), -1.0),
            cast_shadows=False,
        ),
        LightingScenario.INDOOR_DIM: LightingSpec(
            intensity=(0.5, 0.65),
            ambient=(0.4, 0.5),
            direction=((0.0, 0.0), (0.0, 0.0), -1.0),
            cast_shadows=False,
        ),
        LightingScenario.EVENING: LightingSpec(
            intensity=(0.6, 0.8),
            ambient=(0.3, 0.4),
            direction=((-0.9, -0.7), (-0.5, 0.5), -0.3),
            cast_shadows=True,
        ),
        LightingScenario.MIXED: LightingSpec(
            intensity=(0.7, 0.9),
            ambient=(0.5, 0.65),
            direction=((-0.6, -0.4), (-0.6, -0.4), -1.0),
            cast_shadows=True,
        ),
    }


class ZLayers:
    """Z-axis positioning for visual layering and collision.

    Centralizes all Z-position constants to prevent scattered magic
    numbers throughout the codebase. These values ensure proper layering
    and prevent rendering artifacts.
    """

    TRACK_FLOOR = 0.00001  # Lowest level: track surface
    GRID_LINES = 0.0001  # Grid lines on track
    STARTING_ZONE_BASE = 0.0002  # Starting zone visual marker
    DIRECTION_INDICATOR = 0.004  # Direction indicator on starting zone
    TRAFFIC_SIGN = 0.05  # Traffic signs (half their height)
    PARKING_BLOCK = 0.05  # Parking blocks (half their height)
    COLLISION_SURFACE = 0.05  # Collision detection surface
    ROBOT_BASE = None  # Computed from RobotSpecs.WHEEL_RADIUS (dynamic)
