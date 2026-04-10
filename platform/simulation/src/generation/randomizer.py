"""Scenario randomization logic for WRO 2026 simulation.

Handles all stochastic elements: corridor widths, lighting, starting
conditions, parking lot positions, and traffic sign placement.
"""

from __future__ import annotations

import math
import random
from typing import Any

import numpy as np
from shared.config.constants import (
    CorridorDimensions,
    DictKeys,
    GridSections,
    LightingScenarios,
    ParkingLotSpecs,
    RobotSpecs,
    StartingZoneSpecs,
    TrackDimensions,
    TrafficSignSpecs,
    WidthTypes,
)
from shared.config.enums import Direction, LightingScenario, Section

from src.generation.scenarios import apply_scenario_to_section


class ScenarioRandomizer:
    """Generates all randomized parameters for a WRO 2026 scenario.

    Args:
        randomization_config: Dict with 'colors' sub-dict holding
            color mean/std arrays keyed by ColorNames constants.
    """

    CORRIDOR_WIDTH_MAP: dict[str, float] = {
        WidthTypes.NARROW: CorridorDimensions.NARROW,
        WidthTypes.WIDE: CorridorDimensions.WIDE,
    }

    def __init__(self, randomization_config: dict[str, Any]) -> None:
        self._randomization = randomization_config
        self._sections: tuple[Section, ...] = GridSections.SECTIONS
        self._validate_config()

    def _validate_config(self) -> None:
        """Validate randomization config structure on initialization.

        Raises:
            ValueError: If required config keys or structure is invalid.
        """
        if DictKeys.COLORS not in self._randomization:
            raise ValueError(f"Randomization config missing required key '{DictKeys.COLORS}'")

        colors_config = self._randomization[DictKeys.COLORS]
        for color_name in ["red", "green"]:
            if color_name not in colors_config:
                raise ValueError(
                    f"Colors config missing '{color_name}'. "
                    f"Available: {list(colors_config.keys())}"
                )
            if DictKeys.MEAN not in colors_config[color_name]:
                raise ValueError(f"Color '{color_name}' config missing '{DictKeys.MEAN}' key")
            if DictKeys.STD not in colors_config[color_name]:
                raise ValueError(f"Color '{color_name}' config missing '{DictKeys.STD}' key")

    def randomize_color(self, color_name: str) -> list[float]:
        """Sample a traffic sign color with Gaussian noise around the WRO mean.

        Args:
            color_name: One of ColorNames.RED or ColorNames.GREEN.

        Returns:
            Normalized RGB list clamped to [0.0, 1.0].

        Raises:
            ValueError: If color_name is not recognized.
        """
        if color_name not in self._randomization[DictKeys.COLORS]:
            valid_colors = tuple(self._randomization[DictKeys.COLORS].keys())
            raise ValueError(f"Unknown color '{color_name}'. Valid: {valid_colors}")

        params = self._randomization[DictKeys.COLORS][color_name]
        color = np.random.normal(params[DictKeys.MEAN], params[DictKeys.STD])
        return np.clip(color, 0.0, 1.0).tolist()

    def randomize_lighting(self) -> dict[str, Any]:
        """Generate lighting parameters for one of six realistic scenarios.

        Returns:
            Dict with keys: intensity, direction, ambient_intensity,
            cast_shadows, scenario.
        """
        scenario = random.choice(list(LightingScenario))
        return _build_lighting_config(scenario)

    def randomize_corridor_widths(self) -> dict[Section, dict[str, Any]]:
        """Assign random narrow/wide width to each corridor (Open challenge).

        Returns:
            Dict mapping Section to {type: str, width: float}.
        """
        return {
            section: _build_width_entry(random.choice([WidthTypes.NARROW, WidthTypes.WIDE]))
            for section in self._sections
        }

    def randomize_starting_conditions(
        self,
        corridor_widths: dict[Section, dict[str, Any]],
    ) -> dict[str, Any]:
        """Choose a random starting section, direction, and position.

        Args:
            corridor_widths: Per-section width info (type + width in meters).

        Returns:
            Dict with direction, section, section_name, position (tuple), yaw.

        Raises:
            ValueError: If corridor_widths is invalid or incomplete.
        """
        self._validate_corridor_widths(corridor_widths)

        direction = random.choice(list(Direction))
        starting_section = random.choice(self._sections)
        corridor_width = corridor_widths[starting_section][DictKeys.WIDTH]
        starting_position = _pick_start_position(starting_section, corridor_width)
        starting_yaw = _compute_starting_yaw(starting_section, direction)

        return {
            DictKeys.DIRECTION: direction,
            DictKeys.SECTION: starting_section,
            DictKeys.SECTION_NAME: starting_section.capitalized,
            DictKeys.POSITION: starting_position,
            DictKeys.YAW: starting_yaw,
        }

    def _validate_corridor_widths(self, corridor_widths: dict[Section, dict[str, Any]]) -> None:
        """Validate corridor_widths dict structure and values.

        Args:
            corridor_widths: Dict to validate.

        Raises:
            ValueError: If structure is invalid or values are out of range.
        """
        if not isinstance(corridor_widths, dict):
            raise ValueError(
                f"corridor_widths must be a dict, got {type(corridor_widths).__name__}"
            )

        for section in self._sections:
            if section not in corridor_widths:
                raise ValueError(
                    f"Missing corridor width for section {section}. "
                    f"Available: {list(corridor_widths.keys())}"
                )

            width_config = corridor_widths[section]
            if DictKeys.WIDTH not in width_config:
                raise ValueError(
                    f"Section {section} missing '{DictKeys.WIDTH}' key. "
                    f"Available keys: {list(width_config.keys())}"
                )

            width_value = width_config[DictKeys.WIDTH]
            if not isinstance(width_value, (int, float)):
                raise ValueError(
                    f"Section {section} width must be numeric, "
                    f"got {type(width_value).__name__}: {width_value}"
                )

            # Validate width is within reasonable bounds (from shared constants)
            min_width = CorridorDimensions.MIN_WIDTH
            max_width = CorridorDimensions.MAX_WIDTH
            if not (min_width <= width_value <= max_width):
                raise ValueError(
                    f"Section {section} width {width_value}m is out of valid range "
                    f"[{min_width}, {max_width}]m"
                )

    def generate_parking_lot_positions(
        self,
        starting_section: Section,
    ) -> dict[str, Any]:
        """Compute parking block positions in the starting section's corner.

        Args:
            starting_section: The corridor section where parking is placed.

        Returns:
            Dict with block1_pos, block2_pos, block1_yaw, block2_yaw, depth.
        """
        depth_choices = [
            TrafficSignSpecs.GRID_DEPTH_NEAR,
            TrafficSignSpecs.GRID_DEPTH_MIDDLE,
            TrafficSignSpecs.GRID_DEPTH_FAR,
        ]
        depth = random.choice(depth_choices)
        spacing = ParkingLotSpecs.BLOCK_SPACING_FACTOR * RobotSpecs.WIDTH
        depth2 = _compute_second_block_depth(depth, spacing)
        wall_offset = ParkingLotSpecs.WALL_OFFSET
        block1_pos, block2_pos, yaw = _parking_positions_for_section(
            starting_section,
            depth,
            depth2,
            wall_offset,
        )
        return {
            DictKeys.BLOCK1_POS: block1_pos,
            DictKeys.BLOCK2_POS: block2_pos,
            DictKeys.BLOCK1_YAW: yaw,
            DictKeys.BLOCK2_YAW: yaw,
            DictKeys.DEPTH: depth,
        }

    def generate_starting_zone(
        self,
        starting_section: Section,
        corridor_width: float,
        parking_config: dict[str, Any] | None,
    ) -> dict[str, Any]:
        """Compute the starting zone rectangle dimensions and position."""
        track_max = TrackDimensions.MAX_COORD
        zone_length = StartingZoneSpecs.DEFAULT_LENGTH

        is_obstacles_with_parking = parking_config is not None
        if is_obstacles_with_parking:
            zone_length, zone_x, zone_y = _zone_from_parking(
                starting_section,
                parking_config,
                zone_length,
                track_max,
            )
            return {"length": zone_length, "x": zone_x, "y": zone_y}

        # Open challenge: random position within corridor
        width_sections = [0.2, 0.5, 0.8] if corridor_width >= 1.0 else [0.2, 0.5]
        width_offset = random.choice(width_sections)
        length_offset = random.choice([1.25, 1.75])

        is_ns = starting_section in (Section.NORTH, Section.SOUTH)
        if is_ns:
            zone_x = length_offset
            zone_y = (
                width_offset if starting_section is Section.SOUTH else track_max - width_offset
            )
        else:
            zone_y = length_offset
            zone_x = width_offset if starting_section is Section.WEST else track_max - width_offset

        return {"length": zone_length, "x": zone_x, "y": zone_y}

    def generate_sign_positions(
        self,
        _corridor_widths: dict[Section, dict[str, Any]],
        exclude_section: Section | None = None,
    ) -> tuple[list[tuple[float, float]], list[tuple[str, list[float]]]]:
        """Place traffic signs using the WRO 36-scenario system.

        Args:
            _corridor_widths: Per-section width data. Reserved for future
                narrow-corridor sign-exclusion logic; not yet consumed.
            exclude_section: Section to skip (always the starting section for
                the obstacles challenge — parking lot occupies that corner).

        Returns:
            Tuple (positions, colors). positions is [(x, y), ...].
            colors is [(color_name, color_rgb), ...].
        """
        all_signs: list[dict[str, Any]] = []
        for section in self._sections:
            if section == exclude_section:
                continue
            scenario_id = random.randint(1, 36)
            for color, x, y in apply_scenario_to_section(scenario_id, section):
                all_signs.append({DictKeys.X: x, DictKeys.Y: y, DictKeys.COLOR: color})

        positions = [(s[DictKeys.X], s[DictKeys.Y]) for s in all_signs]
        colors = [
            (
                s[DictKeys.COLOR],
                self._randomization[DictKeys.COLORS][s[DictKeys.COLOR]][DictKeys.MEAN],
            )
            for s in all_signs
        ]
        return positions, colors


# Private pure helpers
def _build_width_entry(width_type: str) -> dict[str, Any]:
    if width_type not in ScenarioRandomizer.CORRIDOR_WIDTH_MAP:
        valid_types = tuple(ScenarioRandomizer.CORRIDOR_WIDTH_MAP.keys())
        raise ValueError(f"Unknown width type '{width_type}'. Valid types: {valid_types}")
    width = ScenarioRandomizer.CORRIDOR_WIDTH_MAP[width_type]
    return {DictKeys.TYPE: width_type, DictKeys.WIDTH: width}


def _build_lighting_config(scenario: LightingScenario) -> dict[str, Any]:
    """Build lighting config from table-driven scenario specifications.

    Args:
        scenario: LightingScenario enum value specifying the scenario.

    Returns:
        Dict with intensity, ambient_intensity, direction, cast_shadows, and scenario keys.

    Raises:
        KeyError: If scenario is not in LightingScenarios.SPECS.
    """
    scenario_str = str(scenario)
    if scenario_str not in LightingScenarios.SPECS:
        valid_scenarios = tuple(LightingScenarios.SPECS.keys())
        raise ValueError(
            f"Unknown lighting scenario: {scenario_str}. Valid scenarios: {valid_scenarios}"
        )

    spec = LightingScenarios.SPECS[scenario_str]

    return {
        DictKeys.INTENSITY: random.uniform(*spec["intensity"]),
        DictKeys.AMBIENT_INTENSITY: random.uniform(*spec["ambient"]),
        DictKeys.DIRECTION: [
            random.uniform(*spec["direction"][0]),
            random.uniform(*spec["direction"][1]),
            spec["direction"][2],
        ],
        DictKeys.CAST_SHADOWS: spec["cast_shadows"],
        DictKeys.SCENARIO: scenario_str,
    }


def _pick_start_position(
    section: Section,
    corridor_width: float,
) -> tuple[float, float]:
    """Pick a random starting position within the given corridor."""
    track_max = TrackDimensions.MAX_COORD
    track_center = TrackDimensions.CENTER_COORD
    offsets = [-0.5, 0.0, 0.5]

    if section is Section.NORTH:
        y_pos = track_max - corridor_width / 2
        return random.choice([(track_center + d, y_pos) for d in offsets])
    if section is Section.SOUTH:
        y_pos = corridor_width / 2
        return random.choice([(track_center + d, y_pos) for d in offsets])
    if section is Section.EAST:
        x_pos = track_max - corridor_width / 2
        return random.choice([(x_pos, track_center + d) for d in offsets])
    # Section.WEST
    x_pos = corridor_width / 2
    return random.choice([(x_pos, track_center + d) for d in offsets])


def _compute_starting_yaw(section: Section, direction: Direction) -> float:
    """Return the robot's initial heading angle (radians) for a given corridor/direction."""
    half_pi = math.pi / 2
    yaw_map: dict[Section, dict[Direction, float]] = {
        Section.SOUTH: {Direction.CLOCKWISE: math.pi, Direction.COUNTERCLOCKWISE: 0.0},
        Section.NORTH: {Direction.CLOCKWISE: 0.0, Direction.COUNTERCLOCKWISE: math.pi},
        Section.EAST: {Direction.CLOCKWISE: -half_pi, Direction.COUNTERCLOCKWISE: half_pi},
        Section.WEST: {Direction.CLOCKWISE: half_pi, Direction.COUNTERCLOCKWISE: -half_pi},
    }
    return yaw_map[section][direction]


def _compute_second_block_depth(depth: float, spacing: float) -> float:
    """Return depth of the second parking block, respecting corner bounds."""
    near = TrafficSignSpecs.GRID_DEPTH_NEAR
    far = TrafficSignSpecs.GRID_DEPTH_FAR

    if depth == near:
        return depth + spacing
    if depth == far:
        return depth - spacing
    # Middle: randomly choose inward or outward
    return depth + (spacing if random.random() < 0.5 else -spacing)


def _parking_positions_for_section(
    section: Section,
    depth: float,
    depth2: float,
    wall_offset: float,
) -> tuple[tuple[float, float], tuple[float, float], float]:
    """Return ((block1_x, block1_y), (block2_x, block2_y), yaw) for a section."""
    track_max = TrackDimensions.MAX_COORD

    half_pi = math.pi / 2
    if section is Section.SOUTH:
        return (depth, wall_offset), (depth2, wall_offset), half_pi
    if section is Section.NORTH:
        y = track_max - wall_offset
        return (depth, y), (depth2, y), half_pi
    if section is Section.EAST:
        x = track_max - wall_offset
        return (x, depth), (x, depth2), 0.0
    # Section.WEST
    return (wall_offset, depth), (wall_offset, depth2), 0.0


def _zone_from_parking(
    section: Section,
    parking_config: dict[str, Any],
    default_length: float,
    track_max: float,
) -> tuple[float, float, float]:
    """Compute zone placement centered between the two parking blocks."""
    b1 = parking_config[DictKeys.BLOCK1_POS]
    b2 = parking_config[DictKeys.BLOCK2_POS]

    is_ns = section in (Section.NORTH, Section.SOUTH)
    if is_ns:
        spacing = abs(b2[0] - b1[0])
        zone_x = (b1[0] + b2[0]) / 2
        zone_y = b1[1]
    else:
        spacing = abs(b2[1] - b1[1])
        zone_y = (b1[1] + b2[1]) / 2
        zone_x = b1[0]

    available_gap = spacing - ParkingLotSpecs.WIDTH
    zone_length = (
        min(default_length, available_gap * StartingZoneSpecs.OBSTACLES_SIZE_FACTOR)
        if available_gap < default_length
        else default_length
    )

    return zone_length, zone_x, zone_y
