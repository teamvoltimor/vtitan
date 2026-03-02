"""Scenario randomization logic for WRO 2026 simulation.

Handles all stochastic elements: corridor widths, lighting, starting
conditions, parking lot positions, and traffic sign placement.
"""

from __future__ import annotations

import math
import random
from typing import Any

import numpy as np

from src.config.constants import (
    CorridorDimensions,
    DictKeys,
    GridSections,
    ParkingLotSpecs,
    RobotSpecs,
    TrackDimensions,
    TrafficSignSpecs,
    WidthTypes,
)
from src.config.enums import Direction, Section
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

    def randomize_color(self, color_name: str) -> list[float]:
        """Sample a traffic sign color with Gaussian noise around the WRO mean.

        Args:
            color_name: One of ColorNames.RED or ColorNames.GREEN.

        Returns:
            Normalized RGB list clamped to [0.0, 1.0].
        """
        params = self._randomization[DictKeys.COLORS][color_name]
        color = np.random.normal(params[DictKeys.MEAN], params[DictKeys.STD])
        return np.clip(color, 0.0, 1.0).tolist()

    def randomize_lighting(self) -> dict[str, Any]:
        """Generate lighting parameters for one of six realistic scenarios.

        Returns:
            Dict with keys: intensity, direction, ambient_intensity,
            cast_shadows, scenario.
        """
        scenario = random.choice([
            "direct_sunlight", "cloudy", "indoor_bright",
            "indoor_dim", "evening", "mixed",
        ])
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
        """
        direction = random.choice(list(Direction))
        starting_section = random.choice(self._sections)
        corridor_width = corridor_widths.get(
            starting_section, {DictKeys.WIDTH: 0.65},
        )[DictKeys.WIDTH]
        starting_position = _pick_start_position(starting_section, corridor_width)
        starting_yaw = _compute_starting_yaw(starting_section, direction)

        return {
            DictKeys.DIRECTION: direction,
            DictKeys.SECTION: starting_section,
            DictKeys.SECTION_NAME: starting_section.capitalized,
            DictKeys.POSITION: starting_position,
            DictKeys.YAW: starting_yaw,
        }

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
            starting_section, depth, depth2, wall_offset,
        )
        return {
            DictKeys.BLOCK1_POS: block1_pos,
            DictKeys.BLOCK2_POS: block2_pos,
            DictKeys.BLOCK1_YAW: yaw,
            DictKeys.BLOCK2_YAW: yaw,
            DictKeys.DEPTH: depth,
        }

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


# ── Private pure helpers ──────────────────────────────────────────────────────

def _build_width_entry(width_type: str) -> dict[str, Any]:
    width = ScenarioRandomizer.CORRIDOR_WIDTH_MAP[width_type]
    return {DictKeys.TYPE: width_type, DictKeys.WIDTH: width}


def _build_lighting_config(scenario: str) -> dict[str, Any]:
    """Return lighting parameters dict for the named scenario.

    Each branch calls ``random.uniform`` only for the selected scenario,
    avoiding wasteful sampling of the five branches that are discarded.
    """
    if scenario == "direct_sunlight":
        config: dict[str, Any] = {
            DictKeys.INTENSITY: random.uniform(0.9, 1.0),
            DictKeys.AMBIENT_INTENSITY: random.uniform(0.3, 0.4),
            DictKeys.DIRECTION: [random.uniform(-0.7, -0.3), random.uniform(-0.7, -0.3), -1.0],
            "cast_shadows": True,
        }
    elif scenario == "cloudy":
        config = {
            DictKeys.INTENSITY: random.uniform(0.6, 0.75),
            DictKeys.AMBIENT_INTENSITY: random.uniform(0.5, 0.6),
            DictKeys.DIRECTION: [-0.5, -0.5, -1.0],
            "cast_shadows": True,
        }
    elif scenario == "indoor_bright":
        config = {
            DictKeys.INTENSITY: random.uniform(0.7, 0.85),
            DictKeys.AMBIENT_INTENSITY: random.uniform(0.6, 0.7),
            DictKeys.DIRECTION: [0.0, 0.0, -1.0],
            "cast_shadows": False,
        }
    elif scenario == "indoor_dim":
        config = {
            DictKeys.INTENSITY: random.uniform(0.5, 0.65),
            DictKeys.AMBIENT_INTENSITY: random.uniform(0.4, 0.5),
            DictKeys.DIRECTION: [0.0, 0.0, -1.0],
            "cast_shadows": False,
        }
    elif scenario == "evening":
        config = {
            DictKeys.INTENSITY: random.uniform(0.6, 0.8),
            DictKeys.AMBIENT_INTENSITY: random.uniform(0.3, 0.4),
            DictKeys.DIRECTION: [random.uniform(-0.9, -0.7), random.uniform(-0.5, 0.5), -0.3],
            "cast_shadows": True,
        }
    else:  # "mixed"
        config = {
            DictKeys.INTENSITY: random.uniform(0.7, 0.9),
            DictKeys.AMBIENT_INTENSITY: random.uniform(0.5, 0.65),
            DictKeys.DIRECTION: [random.uniform(-0.6, -0.4), random.uniform(-0.6, -0.4), -1.0],
            "cast_shadows": True,
        }
    config["scenario"] = scenario
    return config


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
        Section.SOUTH: {Direction.CLOCKWISE: math.pi,  Direction.COUNTERCLOCKWISE: 0.0},
        Section.NORTH: {Direction.CLOCKWISE: 0.0,       Direction.COUNTERCLOCKWISE: math.pi},
        Section.EAST:  {Direction.CLOCKWISE: -half_pi,  Direction.COUNTERCLOCKWISE: half_pi},
        Section.WEST:  {Direction.CLOCKWISE: half_pi,   Direction.COUNTERCLOCKWISE: -half_pi},
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
