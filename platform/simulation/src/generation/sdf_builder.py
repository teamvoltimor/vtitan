"""SDF XML construction helpers for WRO 2026 Gazebo worlds.

SDFBuilder coordinates modular builders to assemble world elements:
- PluginBuilder: System plugins
- LightingBuilder: Sun and ambient light
- TrackBuilder: Walls and zones
- ObjectBuilder: Signs, parking, robot

The old SDFBuilder god class has been refactored to use composition,
improving testability, maintainability, and single responsibility.
"""

from __future__ import annotations

from typing import Any
from xml.etree import ElementTree as ET

from shared.config.constants import DictKeys
from shared.config.enums import ScenarioType, Section

from src.generation.builders import (
    LightingBuilder,
    ObjectBuilder,
    PluginBuilder,
    TrackBuilder,
)

# Legacy robot building module (kept for backward compatibility during refactoring)
from . import sdf_robot_builder


class SDFBuilder:
    """Coordinates modular builders for Gazebo SDF world construction.

    This refactored version uses composition instead of monolithic design,
    delegating responsibilities to specialized builders:
    - PluginBuilder: Sensors and physics plugins
    - LightingBuilder: Lighting configuration
    - TrackBuilder: Walls and zones
    - ObjectBuilder: Signs, parking, robot

    Args:
        challenge_type: One of ScenarioType.OPEN or ScenarioType.OBSTACLES.
    """

    def __init__(self, challenge_type: ScenarioType) -> None:
        self._challenge_type = challenge_type
        self._plugins = PluginBuilder()
        self._lighting = LightingBuilder()
        self._track = TrackBuilder()
        self._objects = ObjectBuilder()

    def build(
        self,
        world: ET.Element,
        scenario_data: dict[str, Any],
        base_world_path: str = "",
    ) -> None:
        """Build complete world from scenario data.

        Args:
            world: The <world> ET element to populate.
            scenario_data: Complete scenario data dict containing:
                - lighting: Lighting config
                - corridor_widths: Per-section widths
                - starting_conditions: Robot starting position/direction
                - sign_positions: Traffic sign locations
                - sign_colors: Traffic sign colors
                - parking_lot: Parking config (obstacles only)
            base_world_path: Path to base world sdf
        """
        # Stage 1: Inject system plugins
        self._plugins.add_system_plugins(world)

        # Stage 2: Configure lighting
        self._lighting.apply_lighting(world, scenario_data[DictKeys.LIGHTING])

        # Stage 3: Build track elements (walls, zones)
        self._track.add_interior_walls(world, scenario_data[DictKeys.CORRIDOR_WIDTHS])
        self._track.add_starting_zone(
            world,
            scenario_data[DictKeys.STARTING_CONDITIONS],
            scenario_data[DictKeys.CORRIDOR_WIDTHS],
            scenario_data.get(DictKeys.PARKING_LOT),
        )

        # Stage 4: Place objects
        self._objects.add_traffic_signs(
            world,
            scenario_data[DictKeys.SIGN_POSITIONS],
            scenario_data.get("sign_colors", []),
        )

        if DictKeys.PARKING_LOT in scenario_data:
            self._objects.add_parking_lot(
                world,
                scenario_data[DictKeys.PARKING_LOT],
            )

        # Stage 5: Add robot with full sensor/drivetrain configuration
        # NOTE: Legacy function still handles complex robot model
        starting_conditions = scenario_data[DictKeys.STARTING_CONDITIONS]
        sdf_robot_builder.add_robot_model(world, starting_conditions)

    # ========================================================================
    # Legacy public methods for backward compatibility during gradual migration
    # ========================================================================

    def add_system_plugins(self, world: ET.Element) -> None:
        """Backward compatibility wrapper. Use PluginBuilder directly."""
        self._plugins.add_system_plugins(world)

    def apply_lighting(
        self,
        world: ET.Element,
        lighting: dict[str, Any],
    ) -> None:
        """Backward compatibility wrapper. Use LightingBuilder directly."""
        self._lighting.apply_lighting(world, lighting)

    def add_interior_walls(
        self,
        world: ET.Element,
        corridor_widths: dict[Section, dict[str, Any]],
    ) -> None:
        """Backward compatibility wrapper. Use TrackBuilder directly."""
        self._track.add_interior_walls(world, corridor_widths)

    def add_traffic_signs(
        self,
        world: ET.Element,
        sign_positions: list[tuple[float, float]],
        sign_colors: list[tuple[str, list[float]]],
    ) -> None:
        """Backward compatibility wrapper. Use ObjectBuilder directly."""
        self._objects.add_traffic_signs(world, sign_positions, sign_colors)

    def add_parking_lot(
        self,
        world: ET.Element,
        parking_config: dict[str, Any],
    ) -> None:
        """Backward compatibility wrapper. Use ObjectBuilder directly."""
        self._objects.add_parking_lot(world, parking_config)

    def add_starting_zone(
        self,
        world: ET.Element,
        starting_conditions: dict[str, Any],
        corridor_widths: dict[Section, dict[str, Any]],
        parking_config: dict[str, Any] | None,
        base_world_path: str = "",
    ) -> None:
        """Backward compatibility wrapper. Use TrackBuilder directly."""
        self._track.add_starting_zone(
            world,
            starting_conditions,
            corridor_widths,
            parking_config,
        )

    def add_robot_model(
        self,
        world: ET.Element,
        starting_conditions: dict[str, Any],
    ) -> None:
        """Backward compatibility wrapper. Use sdf_robot_builder directly."""
        sdf_robot_builder.add_robot_model(world, starting_conditions)
