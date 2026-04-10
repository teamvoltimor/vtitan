"""Builds dynamic scene objects (traffic signs, parking blocks, robot)."""

from __future__ import annotations

from typing import Any
from xml.etree import ElementTree as ET

from shared.config.constants import (
    DictKeys,
    ModelNames,
    ParkingLotSpecs,
    RobotSpecs,
    TrafficSignSpecs,
)
from shared.config.enums import ScenarioType

from .xml_helpers import add_box_collision, add_box_visual


class ObjectBuilder:
    """Handles dynamic objects: traffic signs, parking blocks, and robot model.

    Responsibilities:
    - Traffic sign placement and coloring
    - Parking block positioning
    - Robot model with sensors (LIDAR, IMU, camera)
    """

    @staticmethod
    def add_traffic_signs(
        world: ET.Element,
        sign_positions: list[tuple[float, float]],
        sign_colors: list[tuple[str, list[float]]],
    ) -> None:
        """Append traffic sign box models to the world.

        Args:
            world: The <world> ET element to append to.
            sign_positions: List of (x, y) world coordinates.
            sign_colors: Parallel list of (color_name, rgb_list) tuples.
        """
        for index, ((x, y), (color_name, color_rgb)) in enumerate(
            zip(sign_positions, sign_colors, strict=True),
        ):
            prefix = (
                ModelNames.RED_SIGN_PREFIX if color_name == "red" else ModelNames.GREEN_SIGN_PREFIX
            )
            model = ET.Element("model", name=f"{prefix}{index}")
            ET.SubElement(model, "static").text = "true"
            ET.SubElement(model, "pose").text = f"{x} {y} {TrafficSignSpecs.Z_POSITION} 0 0 0"
            link = ET.SubElement(model, "link", name="link")
            add_box_visual(
                link,
                TrafficSignSpecs.WIDTH,
                TrafficSignSpecs.DEPTH,
                TrafficSignSpecs.HEIGHT,
                color_rgb,
            )
            add_box_collision(
                link,
                TrafficSignSpecs.WIDTH,
                TrafficSignSpecs.DEPTH,
                TrafficSignSpecs.HEIGHT,
            )
            world.append(model)

    @staticmethod
    def add_parking_lot(
        world: ET.Element,
        parking_config: dict[str, Any],
    ) -> None:
        """Append the two parking limitation blocks to the world.

        Args:
            world: The <world> ET element to append to.
            parking_config: Dict from ScenarioRandomizer.generate_parking_lot_positions().
        """
        parking_color = list(ParkingLotSpecs.COLOR)
        block_dims = (ParkingLotSpecs.LENGTH, ParkingLotSpecs.WIDTH, ParkingLotSpecs.HEIGHT)

        for name_key, pos_key, yaw_key in [
            (
                ModelNames.PARKING_LIMITATION_1,
                DictKeys.BLOCK1_POS,
                DictKeys.BLOCK1_YAW,
            ),
            (
                ModelNames.PARKING_LIMITATION_2,
                DictKeys.BLOCK2_POS,
                DictKeys.BLOCK2_YAW,
            ),
        ]:
            bx, by = parking_config[pos_key]
            yaw = parking_config[yaw_key]
            model = ET.Element("model", name=name_key)
            ET.SubElement(model, "static").text = "true"
            ET.SubElement(model, "pose").text = f"{bx} {by} {ParkingLotSpecs.Z_POSITION} 0 0 {yaw}"
            link = ET.SubElement(model, "link", name="link")
            add_box_visual(link, *block_dims, parking_color)
            add_box_collision(link, *block_dims)
            world.append(model)

    @staticmethod
    def add_robot_model(
        world: ET.Element,
        starting_conditions: dict[str, Any],
        challenge_type: ScenarioType,
    ) -> None:
        """Append the robot model with LIDAR, IMU, and camera sensors.

        Args:
            world: The <world> ET element to append to.
            starting_conditions: Dict from ScenarioRandomizer.randomize_starting_conditions().
            challenge_type: ScenarioType.OPEN or ScenarioType.OBSTACLES.
        """
        px, py = starting_conditions[DictKeys.POSITION]
        yaw = starting_conditions[DictKeys.YAW]

        model = ET.Element("model", name="robot")
        ET.SubElement(model, "pose").text = f"{px} {py} 0 0 0 {yaw}"
        ET.SubElement(model, "self_collide").text = "false"

        # Chassis link
        chassis_link = ET.SubElement(model, "link", name="chassis")

        # Inertial
        inertial = ET.SubElement(chassis_link, "inertial")
        ET.SubElement(inertial, "mass").text = str(RobotSpecs.CHASSIS_MASS)
        ET.SubElement(ET.SubElement(inertial, "inertia"), "ixx").text = "0.01"  # Placeholder

        # Collision (as bounding box)
        collision = ET.SubElement(chassis_link, "collision", name="collision")
        ET.SubElement(collision, "pose").text = "0 0 0.05 0 0 0"
        geom = ET.SubElement(ET.SubElement(collision, "geometry"), "box")
        ET.SubElement(
            geom, "size"
        ).text = f"{RobotSpecs.LENGTH} {RobotSpecs.WIDTH} {RobotSpecs.HEIGHT}"

        # Visual
        visual = ET.SubElement(chassis_link, "visual", name="visual")
        geom_vis = ET.SubElement(ET.SubElement(visual, "geometry"), "box")
        ET.SubElement(
            geom_vis, "size"
        ).text = f"{RobotSpecs.LENGTH} {RobotSpecs.WIDTH} {RobotSpecs.HEIGHT}"
        mat = ET.SubElement(visual, "material")
        ET.SubElement(mat, "ambient").text = "0.3 0.3 0.3 1"
        ET.SubElement(mat, "diffuse").text = "0.5 0.5 0.5 1"

        # LIDAR sensor
        lidar_sensor = ET.SubElement(chassis_link, "sensor", name="lidar", type="lidar")
        ET.SubElement(lidar_sensor, "pose").text = "0.08 0 0.05 0 0 0"
        ET.SubElement(lidar_sensor, "update_rate").text = f"{RobotSpecs.LIDAR_UPDATE_RATE}"
        lidar_topic = ET.SubElement(ET.SubElement(lidar_sensor, "topic"), "name")
        lidar_topic.text = "/scan"
        lidar = ET.SubElement(lidar_sensor, "lidar")
        ET.SubElement(lidar, "scan").tag = "scan"
        ET.SubElement(ET.SubElement(lidar, "scan"), "horizontal").tag = "samples"
        ET.SubElement(
            ET.SubElement(ET.SubElement(lidar, "scan"), "horizontal"), "samples"
        ).text = str(RobotSpecs.LIDAR_SAMPLES)

        # Camera sensor
        camera_sensor = ET.SubElement(chassis_link, "sensor", name="camera", type="camera")
        ET.SubElement(camera_sensor, "pose").text = "0.12 0 0.08 0 0 0"
        ET.SubElement(camera_sensor, "update_rate").text = f"{RobotSpecs.CAMERA_UPDATE_RATE}"
        camera_topic = ET.SubElement(ET.SubElement(camera_sensor, "topic"), "name")
        camera_topic.text = "/camera"

        # IMU sensor
        imu_sensor = ET.SubElement(chassis_link, "sensor", name="imu", type="imu")
        ET.SubElement(imu_sensor, "pose").text = "-0.05 0 0.05 0 0 0"
        ET.SubElement(imu_sensor, "update_rate").text = f"{RobotSpecs.IMU_UPDATE_RATE}"
        imu_topic = ET.SubElement(ET.SubElement(imu_sensor, "topic"), "name")
        imu_topic.text = "/imu"

        world.append(model)
