"""Manages lighting elements (sun and ambient light) in Gazebo worlds."""

from __future__ import annotations

from typing import Any
from xml.etree import ElementTree as ET

from shared.config.constants import DictKeys, ModelNames


class LightingBuilder:
    """Handles sun and ambient light configuration.

    Responsibilities:
    - Sun/directional light intensity and direction
    - Ambient light intensity
    - Shadow casting control
    """

    @staticmethod
    def apply_lighting(
        world: ET.Element,
        lighting: dict[str, Any],
    ) -> None:
        """Update sun and ambient light elements from a lighting config dict.

        Args:
            world: The <world> ET element to modify in-place.
            lighting: Dict with intensity, direction, ambient_intensity,
                     cast_shadows, and scenario keys.
        """
        sun = world.find(f".//light[@name='{ModelNames.SUN_LIGHT}']")
        if sun is not None:
            intensity = max(0.0, min(1.0, lighting[DictKeys.INTENSITY]))
            sun.find("diffuse").text = f"{intensity} {intensity} {intensity} 1"
            d = lighting[DictKeys.DIRECTION]
            sun.find("direction").text = f"{d[0]} {d[1]} {d[2]}"
            cast_node = sun.find("cast_shadows")
            if cast_node is not None:
                cast_shadows = lighting.get(DictKeys.CAST_SHADOWS, True)
                cast_node.text = "true" if cast_shadows else "false"

        ambient = world.find(f".//light[@name='{ModelNames.AMBIENT_LIGHT}']")
        if ambient is not None:
            amb = max(0.0, min(1.0, lighting[DictKeys.AMBIENT_INTENSITY]))
            ambient.find("diffuse").text = f"{amb} {amb} {amb} 1"
