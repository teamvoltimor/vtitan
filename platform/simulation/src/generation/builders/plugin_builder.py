"""Manages Gazebo system plugins for physics and sensor simulation."""

from __future__ import annotations

from xml.etree import ElementTree as ET


class PluginBuilder:
    """Handles injection of Gazebo system plugins into world elements.

    Responsibilities:
    - Physics system plugin
    - Sensors system plugin
    - Rendering engine configuration
    """

    @staticmethod
    def add_system_plugins(world: ET.Element) -> None:
        """Inject Sensors and Physics system plugins if not already present.

        Args:
            world: The <world> ET element to modify in-place.
        """
        if world.find(".//plugin[@name='gz::sim::systems::Sensors']") is None:
            sensors = ET.Element(
                "plugin",
                filename="gz-sim-sensors-system",
                name="gz::sim::systems::Sensors",
            )
            ET.SubElement(sensors, "render_engine").text = "ogre2"
            world.insert(0, sensors)

        if world.find(".//plugin[@name='gz::sim::systems::Physics']") is None:
            physics = ET.Element(
                "plugin",
                filename="gz-sim-physics-system",
                name="gz::sim::systems::Physics",
            )
            world.insert(1, physics)
