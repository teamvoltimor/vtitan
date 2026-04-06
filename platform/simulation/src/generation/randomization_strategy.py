"""Randomization strategies for scenario generation (Strategy Pattern).

This module decouples scenario generation from randomization implementation,
allowing different randomization strategies to be swapped without changing
the generator logic.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from typing import Any

from shared.config.constants import (
    CorridorDimensions,
    DictKeys,
    GridSections,
    WidthTypes,
)
from shared.config.enums import Direction, Section

from src.generation.randomizer import ScenarioRandomizer


class RandomizationStrategy(ABC):
    """Abstract base class for randomization strategies.

    Defines the interface that all randomization strategies must implement.
    Different strategies can vary how they randomize corridor widths,
    lighting, and starting conditions.
    """

    @abstractmethod
    def randomize_corridor_widths(self) -> dict[Section, dict[str, Any]]:
        """Generate corridor width configuration.

        Returns:
            Dict mapping Section to {type, width}.
        """
        pass

    @abstractmethod
    def randomize_lighting(self) -> dict[str, Any]:
        """Generate lighting configuration.

        Returns:
            Dict with intensity, direction, ambient_intensity, etc.
        """
        pass

    @abstractmethod
    def randomize_starting_conditions(
        self,
        corridor_widths: dict[Section, dict[str, Any]],
    ) -> dict[str, Any]:
        """Generate starting position and orientation.

        Args:
            corridor_widths: Corridor width config.

        Returns:
            Dict with direction, section, position, yaw.
        """
        pass


class FullRandomization(RandomizationStrategy):
    """Strategy: Randomize all parameters (lighting, widths, starting position).

    Used for generating diverse training datasets where every scenario
    is different and explores the full parameter space.
    """

    def __init__(self, randomizer: ScenarioRandomizer) -> None:
        self._randomizer = randomizer

    def randomize_corridor_widths(self) -> dict[Section, dict[str, Any]]:
        """Generate random narrow/wide widths for each corridor."""
        return self._randomizer.randomize_corridor_widths()

    def randomize_lighting(self) -> dict[str, Any]:
        """Generate random lighting scenario."""
        return self._randomizer.randomize_lighting()

    def randomize_starting_conditions(
        self,
        corridor_widths: dict[Section, dict[str, Any]],
    ) -> dict[str, Any]:
        """Generate random starting position and orientation."""
        return self._randomizer.randomize_starting_conditions(corridor_widths)


class DeterministicDefaults(RandomizationStrategy):
    """Strategy: Use deterministic default values (no randomization).

    Used for reproducible testing and debugging where consistent scenarios
    make it easier to isolate issues.
    """

    def randomize_corridor_widths(self) -> dict[Section, dict[str, Any]]:
        """Return default wide corridors for all sections."""
        return {
            s: {
                DictKeys.TYPE: WidthTypes.WIDE,
                DictKeys.WIDTH: CorridorDimensions.WIDE,
            }
            for s in GridSections.SECTIONS
        }

    def randomize_lighting(self) -> dict[str, Any]:
        """Return default outdoor lighting (direct sunlight)."""
        return {
            DictKeys.INTENSITY: 0.95,
            DictKeys.AMBIENT_INTENSITY: 0.35,
            DictKeys.DIRECTION: [-0.5, -0.5, -1.0],
            DictKeys.CAST_SHADOWS: True,
            DictKeys.SCENARIO: "direct_sunlight",
        }

    def randomize_starting_conditions(
        self,
        corridor_widths: dict[Section, dict[str, Any]],
    ) -> dict[str, Any]:
        """Return fixed starting condition (South section, clockwise)."""
        return {
            DictKeys.DIRECTION: Direction.CLOCKWISE,
            DictKeys.SECTION: Section.SOUTH,
            DictKeys.SECTION_NAME: "South",
            DictKeys.POSITION: (1.5, 0.4),
            DictKeys.YAW: math.pi,
        }


class PartialRandomization(RandomizationStrategy):
    """Strategy: Randomize some parameters, keep others fixed.

    Useful for generating scenarios with controlled variation, e.g.,
    varying corridor widths but keeping lighting constant.
    """

    def __init__(
        self,
        randomizer: ScenarioRandomizer,
        randomize_widths: bool = True,
        randomize_lighting: bool = False,
        randomize_starting: bool = True,
    ) -> None:
        self._randomizer = randomizer
        self._randomize_widths = randomize_widths
        self._randomize_lighting = randomize_lighting
        self._randomize_starting = randomize_starting
        self._defaults = DeterministicDefaults()

    def randomize_corridor_widths(self) -> dict[Section, dict[str, Any]]:
        """Conditionally randomize corridor widths."""
        if self._randomize_widths:
            return self._randomizer.randomize_corridor_widths()
        return self._defaults.randomize_corridor_widths()

    def randomize_lighting(self) -> dict[str, Any]:
        """Conditionally randomize lighting."""
        if self._randomize_lighting:
            return self._randomizer.randomize_lighting()
        return self._defaults.randomize_lighting()

    def randomize_starting_conditions(
        self,
        corridor_widths: dict[Section, dict[str, Any]],
    ) -> dict[str, Any]:
        """Conditionally randomize starting position."""
        if self._randomize_starting:
            return self._randomizer.randomize_starting_conditions(corridor_widths)
        return self._defaults.randomize_starting_conditions(corridor_widths)
