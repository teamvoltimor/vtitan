"""SDF XML construction helpers for WRO 2026 Gazebo worlds.

SDFBuilder assembles all model elements (walls, signs, parking, robot,
sensors) into an ElementTree world node. It is a pure builder — it
never reads files or randomizes values.
"""

from __future__ import annotations

import math
from typing import Any
from xml.etree import ElementTree as ET

from shared.config.constants import (
    DictKeys,
    ModelNames,
    ParkingLotSpecs,
    RobotSpecs,
    StartingZoneSpecs,
    TrackDimensions,
    TrafficSignSpecs,
    WallSpecs,
)
from shared.config.enums import Direction, ScenarioType, Section

# Cylinder geometry requires a 90° roll to align the cylinder axis with Y (wheel roll axis).
_WHEEL_ROLL_POSE = f"0 0 0 {math.pi / 2:.6f} 0 0"


class SDFBuilder:
    """Builds Gazebo SDF model elements into an ElementTree world node.

    Args:
        challenge_type: One of ScenarioType.OPEN or ScenarioType.OBSTACLES.
    """

    def __init__(self, challenge_type: ScenarioType) -> None:
        self._challenge_type = challenge_type

    # System plugins
    def add_system_plugins(self, world: ET.Element) -> None:
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

    # Lighting
    def apply_lighting(
        self,
        world: ET.Element,
        lighting: dict[str, Any],
    ) -> None:
        """Update sun and ambient light elements from a lighting config dict.

        Args:
            world: The <world> ET element to modify in-place.
            lighting: Dict produced by ScenarioRandomizer.randomize_lighting().
        """
        sun = world.find(f".//light[@name='{ModelNames.SUN_LIGHT}']")
        if sun is not None:
            intensity = max(0.0, min(1.0, lighting[DictKeys.INTENSITY]))
            sun.find("diffuse").text = f"{intensity} {intensity} {intensity} 1"
            d = lighting[DictKeys.DIRECTION]
            sun.find("direction").text = f"{d[0]} {d[1]} {d[2]}"
            cast_node = sun.find("cast_shadows")
            if cast_node is not None:
                cast_node.text = "true" if lighting.get("cast_shadows", True) else "false"

        ambient = world.find(f".//light[@name='{ModelNames.AMBIENT_LIGHT}']")
        if ambient is not None:
            amb = max(0.0, min(1.0, lighting[DictKeys.AMBIENT_INTENSITY]))
            ambient.find("diffuse").text = f"{amb} {amb} {amb} 1"

    # Interior walls
    def add_interior_walls(
        self,
        world: ET.Element,
        corridor_widths: dict[Section, dict[str, Any]],
    ) -> None:
        """Append the four interior walls computed from corridor widths.

        Args:
            world: The <world> ET element to append to.
            corridor_widths: Per-section width info from ScenarioRandomizer.
        """
        track_max = TrackDimensions.MAX_COORD

        north_y = track_max - corridor_widths[Section.NORTH][DictKeys.WIDTH]
        south_y = corridor_widths[Section.SOUTH][DictKeys.WIDTH]
        east_x = track_max - corridor_widths[Section.EAST][DictKeys.WIDTH]
        west_x = corridor_widths[Section.WEST][DictKeys.WIDTH]

        walls: list[tuple[str, float, float, float, float, bool]] = [
            # (name, cx, cy, length_x, length_y, is_horizontal)
            (
                ModelNames.INTERIOR_WALL_NORTH,
                (east_x + west_x) / 2,
                north_y - WallSpecs.INTERIOR_OFFSET,
                east_x - west_x,
                WallSpecs.THICKNESS,
                True,
            ),
            (
                ModelNames.INTERIOR_WALL_SOUTH,
                (east_x + west_x) / 2,
                south_y + WallSpecs.INTERIOR_OFFSET,
                east_x - west_x,
                WallSpecs.THICKNESS,
                True,
            ),
            (
                ModelNames.INTERIOR_WALL_EAST,
                east_x - WallSpecs.INTERIOR_OFFSET,
                (north_y + south_y) / 2,
                WallSpecs.THICKNESS,
                north_y - south_y,
                False,
            ),
            (
                ModelNames.INTERIOR_WALL_WEST,
                west_x + WallSpecs.INTERIOR_OFFSET,
                (north_y + south_y) / 2,
                WallSpecs.THICKNESS,
                north_y - south_y,
                False,
            ),
        ]

        for name, cx, cy, vis_x, vis_y, horizontal in walls:
            col_x = WallSpecs.COLLISION_THICKNESS if not horizontal else vis_x
            col_y = WallSpecs.COLLISION_THICKNESS if horizontal else vis_y
            world.append(
                _build_wall_model(name, cx, cy, vis_x, vis_y, col_x, col_y),
            )

    # Traffic signs
    def add_traffic_signs(
        self,
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
            _add_box_visual(
                link,
                TrafficSignSpecs.WIDTH,
                TrafficSignSpecs.DEPTH,
                TrafficSignSpecs.HEIGHT,
                color_rgb,
            )
            _add_box_collision(
                link,
                TrafficSignSpecs.WIDTH,
                TrafficSignSpecs.DEPTH,
                TrafficSignSpecs.HEIGHT,
            )
            world.append(model)

    # Parking lot
    def add_parking_lot(
        self,
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
            (ModelNames.PARKING_LIMITATION_1, DictKeys.BLOCK1_POS, DictKeys.BLOCK1_YAW),
            (ModelNames.PARKING_LIMITATION_2, DictKeys.BLOCK2_POS, DictKeys.BLOCK2_YAW),
        ]:
            bx, by = parking_config[pos_key]
            yaw = parking_config[yaw_key]
            model = ET.Element("model", name=name_key)
            ET.SubElement(model, "static").text = "true"
            ET.SubElement(model, "pose").text = f"{bx} {by} {ParkingLotSpecs.Z_POSITION} 0 0 {yaw}"
            link = ET.SubElement(model, "link", name="link")
            _add_box_visual(link, *block_dims, parking_color)
            _add_box_collision(link, *block_dims)
            world.append(model)

    # Starting zone
    def add_starting_zone(
        self,
        world: ET.Element,
        starting_conditions: dict[str, Any],
        corridor_widths: dict[Section, dict[str, Any]],
        parking_config: dict[str, Any] | None,
        base_world_path: str,
    ) -> None:
        """Add the starting zone rectangle and direction indicator.

        Removes any pre-existing starting zone from the base world, then
        adds the dynamically positioned zone with a colored direction circle.

        Args:
            world: The <world> ET element.
            starting_conditions: Dict from ScenarioRandomizer.randomize_starting_conditions().
            corridor_widths: Per-section width info.
            parking_config: Parking lot positions (obstacles challenge) or None.
            base_world_path: Path to the base world SDF (used to locate icon files).
        """
        starting_section: Section = starting_conditions[DictKeys.SECTION]

        sz = starting_conditions["starting_zone"]
        zone_length, zone_x, zone_y = sz["length"], sz["x"], sz["y"]

        # Remove existing static zone from base template
        for existing in world.findall(".//model[@name='starting_zone_south']"):
            world.remove(existing)

        direction: Direction = starting_conditions[DictKeys.DIRECTION]
        indicator_rgb = (
            StartingZoneSpecs.CLOCKWISE_COLOR
            if direction is Direction.CLOCKWISE
            else StartingZoneSpecs.COUNTERCLOCKWISE_COLOR
        )

        is_ns_corridor = starting_section in (Section.NORTH, Section.SOUTH)
        if is_ns_corridor:
            zone_size = f"{zone_length} {StartingZoneSpecs.WIDTH} {StartingZoneSpecs.THICKNESS}"
        else:
            zone_size = f"{StartingZoneSpecs.WIDTH} {zone_length} {StartingZoneSpecs.THICKNESS}"

        zone_name = f"{ModelNames.STARTING_ZONE_PREFIX}{starting_section}"
        zone_model = ET.Element("model", name=zone_name)
        ET.SubElement(zone_model, "static").text = "true"
        ET.SubElement(zone_model, "pose").text = f"{zone_x} {zone_y} 0.0002 0 0 0"

        link = ET.SubElement(zone_model, "link", name="link")

        # Base grey rectangle
        vis_base = ET.SubElement(link, "visual", name="visual_base")
        geom_b = ET.SubElement(ET.SubElement(vis_base, "geometry"), "box")
        ET.SubElement(geom_b, "size").text = zone_size
        mat_b = ET.SubElement(vis_base, "material")
        c = StartingZoneSpecs.COLOR
        ET.SubElement(mat_b, "ambient").text = f"{c[0]} {c[1]} {c[2]} 1"
        ET.SubElement(mat_b, "diffuse").text = f"{c[0]} {c[1]} {c[2]} 1"

        # Colored direction indicator circle
        vis_ind = ET.SubElement(link, "visual", name="visual_direction_indicator")
        ET.SubElement(vis_ind, "pose").text = "0 0 0.004 0 0 0"
        cyl = ET.SubElement(ET.SubElement(vis_ind, "geometry"), "cylinder")
        ET.SubElement(cyl, "radius").text = str(StartingZoneSpecs.INDICATOR_RADIUS)
        ET.SubElement(cyl, "length").text = str(StartingZoneSpecs.THICKNESS)
        mat_ind = ET.SubElement(vis_ind, "material")
        ic = indicator_rgb
        color_str = f"{ic[0]} {ic[1]} {ic[2]} 1"
        ET.SubElement(mat_ind, "ambient").text = color_str
        ET.SubElement(mat_ind, "diffuse").text = color_str
        ET.SubElement(mat_ind, "emissive").text = color_str

        world.append(zone_model)

        # Sync starting position with actual spawn point
        starting_conditions[DictKeys.POSITION] = (zone_x, zone_y)

    # Robot model
    def add_robot_model(
        self,
        world: ET.Element,
        starting_conditions: dict[str, Any],
    ) -> None:
        """Append the full WRO robot model with sensors and Ackermann plugin.

        Args:
            world: The <world> ET element.
            starting_conditions: Must contain updated POSITION and YAW keys.
        """
        robot_x, robot_y = starting_conditions[DictKeys.POSITION]
        start_yaw = starting_conditions[DictKeys.YAW]
        robot_z = RobotSpecs.WHEEL_RADIUS

        robot_model = ET.Element("model", name="wro_robot")
        ET.SubElement(robot_model, "pose").text = f"{robot_x} {robot_y} {robot_z} 0 0 {start_yaw}"

        _build_chassis(robot_model)
        _build_ackermann_plugin(robot_model)
        _build_wheels(robot_model)
        _build_front_steering(robot_model)
        _build_camera_link(robot_model)
        _build_lidar_link(robot_model)
        _build_imu_link(robot_model)
        _build_debug_overhead_camera(world)

        world.append(robot_model)


# Private XML helpers
def _build_wall_model(
    name: str,
    cx: float,
    cy: float,
    vis_x: float,
    vis_y: float,
    col_x: float,
    col_y: float,
) -> ET.Element:
    """Return a static wall model element with visual and collision geometry."""
    model = ET.Element("model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{cx} {cy} 0.05 0 0 0"
    link = ET.SubElement(model, "link", name="link")

    vis = ET.SubElement(link, "visual", name="visual")
    _set_box_size(ET.SubElement(vis, "geometry"), vis_x, vis_y, 0.1)
    mat = ET.SubElement(vis, "material")
    ET.SubElement(mat, "ambient").text = "0.0 0.0 0.0 1"
    ET.SubElement(mat, "diffuse").text = "0.0 0.0 0.0 1"
    ET.SubElement(mat, "specular").text = "0.0 0.0 0.0 1"

    col = ET.SubElement(link, "collision", name="collision")
    _set_box_size(ET.SubElement(col, "geometry"), col_x, col_y, 0.1)
    _add_contact_surface(col)

    return model


def _add_box_visual(
    link: ET.Element,
    size_x: float,
    size_y: float,
    size_z: float,
    rgb: list[float],
) -> None:
    vis = ET.SubElement(link, "visual", name="visual")
    _set_box_size(ET.SubElement(vis, "geometry"), size_x, size_y, size_z)
    mat = ET.SubElement(vis, "material")
    color_str = f"{rgb[0]} {rgb[1]} {rgb[2]} 1"
    ET.SubElement(mat, "ambient").text = color_str
    ET.SubElement(mat, "diffuse").text = color_str


def _add_box_collision(
    link: ET.Element,
    size_x: float,
    size_y: float,
    size_z: float,
) -> None:
    col = ET.SubElement(link, "collision", name="collision")
    _set_box_size(ET.SubElement(col, "geometry"), size_x, size_y, size_z)


def _set_box_size(
    geom: ET.Element,
    size_x: float,
    size_y: float,
    size_z: float,
) -> None:
    ET.SubElement(ET.SubElement(geom, "box"), "size").text = f"{size_x} {size_y} {size_z}"


def _add_contact_surface(col: ET.Element) -> None:
    surf = ET.SubElement(col, "surface")
    ode = ET.SubElement(ET.SubElement(surf, "contact"), "ode")
    ET.SubElement(ode, "kp").text = "1e8"
    ET.SubElement(ode, "kd").text = "1000"
    ET.SubElement(ode, "max_vel").text = "0.0"
    ET.SubElement(ode, "min_depth").text = "0.0"


def _build_chassis(robot_model: ET.Element) -> None:
    half_h = RobotSpecs.HEIGHT / 2
    cm = RobotSpecs.CHASSIS_MASS
    ixx = (1 / 12) * cm * (RobotSpecs.WIDTH**2 + RobotSpecs.HEIGHT**2)
    iyy = (1 / 12) * cm * (RobotSpecs.LENGTH**2 + RobotSpecs.HEIGHT**2)
    izz = (1 / 12) * cm * (RobotSpecs.LENGTH**2 + RobotSpecs.WIDTH**2)

    base = ET.SubElement(robot_model, "link", name="base_link")

    vis = ET.SubElement(base, "visual", name="visual")
    ET.SubElement(vis, "pose").text = f"0 0 {half_h} 0 0 0"
    _set_box_size(
        ET.SubElement(vis, "geometry"),
        RobotSpecs.LENGTH,
        RobotSpecs.WIDTH,
        RobotSpecs.HEIGHT,
    )
    mat = ET.SubElement(vis, "material")
    ET.SubElement(mat, "ambient").text = "0 0 0.8 1"
    ET.SubElement(mat, "diffuse").text = "0 0 0.8 1"

    # Red front indicator (above LIDAR scan plane)
    front = ET.SubElement(base, "visual", name="front_indicator")
    ET.SubElement(
        front, "pose"
    ).text = f"{RobotSpecs.LENGTH / 2 - 0.02} 0 {RobotSpecs.HEIGHT + 0.003} 0 0 0"
    _set_box_size(ET.SubElement(front, "geometry"), 0.04, 0.04, 0.005)
    fm = ET.SubElement(front, "material")
    ET.SubElement(fm, "ambient").text = "1 0 0 1"
    ET.SubElement(fm, "diffuse").text = "1 0 0 1"

    col = ET.SubElement(base, "collision", name="collision")
    ET.SubElement(col, "pose").text = f"0 0 {half_h} 0 0 0"
    _set_box_size(
        ET.SubElement(col, "geometry"),
        RobotSpecs.LENGTH,
        RobotSpecs.WIDTH,
        RobotSpecs.HEIGHT,
    )

    inertial = ET.SubElement(base, "inertial")
    ET.SubElement(inertial, "mass").text = str(cm)
    ix = ET.SubElement(inertial, "inertia")
    ET.SubElement(ix, "ixx").text = f"{ixx:.6f}"
    ET.SubElement(ix, "iyy").text = f"{iyy:.6f}"
    ET.SubElement(ix, "izz").text = f"{izz:.6f}"


def _build_ackermann_plugin(robot_model: ET.Element) -> None:
    plugin = ET.SubElement(
        robot_model,
        "plugin",
        filename="gz-sim-ackermann-steering-system",
        name="gz::sim::systems::AckermannSteering",
    )
    params: list[tuple[str, str]] = [
        ("left_joint", "rear_left_wheel_joint"),
        ("right_joint", "rear_right_wheel_joint"),
        ("left_steering_joint", "front_left_steering_joint"),
        ("right_steering_joint", "front_right_steering_joint"),
        ("wheel_separation", str(RobotSpecs.TRACK_WIDTH)),
        ("kingpin_width", str(RobotSpecs.TRACK_WIDTH)),
        ("wheel_base", str(RobotSpecs.WHEELBASE)),
        ("wheel_radius", str(RobotSpecs.WHEEL_RADIUS)),
        ("min_steering_angle", f"-{RobotSpecs.MAX_STEERING_ANGLE}"),
        ("max_steering_angle", str(RobotSpecs.MAX_STEERING_ANGLE)),
        ("topic", "/wro_robot/cmd_vel"),
        ("odom_topic", "/wro_robot/odom"),
        ("odom_publish_frequency", "50"),
        ("frame_id", "odom"),
        ("child_frame_id", "base_link"),
    ]
    for tag, text in params:
        ET.SubElement(plugin, tag).text = text


def _build_wheel_link(
    parent: ET.Element,
    name: str,
    pose_text: str,
    wheel_r: float,
    wheel_w: float,
    stripe_offset: float,
    stripe_dims: tuple[float, float, float],
    wm: float,
    wheel_ixx: float,
    wheel_iyy: float,
) -> None:
    link = ET.SubElement(parent, "link", name=name)
    ET.SubElement(link, "pose", relative_to="base_link").text = pose_text

    # Main dark-grey cylinder
    vis = ET.SubElement(link, "visual", name="visual")
    ET.SubElement(vis, "pose").text = _WHEEL_ROLL_POSE
    cyl = ET.SubElement(ET.SubElement(vis, "geometry"), "cylinder")
    ET.SubElement(cyl, "radius").text = str(wheel_r)
    ET.SubElement(cyl, "length").text = str(wheel_w)
    mat = ET.SubElement(vis, "material")
    ET.SubElement(mat, "ambient").text = "0.1 0.1 0.1 1"
    ET.SubElement(mat, "diffuse").text = "0.1 0.1 0.1 1"

    # Yellow rotation stripe
    sv = ET.SubElement(link, "visual", name="stripe")
    ET.SubElement(sv, "pose").text = f"{stripe_offset} 0 0 {math.pi / 2:.6f} 0 0"
    sb = ET.SubElement(ET.SubElement(sv, "geometry"), "box")
    ET.SubElement(sb, "size").text = f"{stripe_dims[0]} {stripe_dims[1]} {stripe_dims[2]}"
    sm = ET.SubElement(sv, "material")
    ET.SubElement(sm, "ambient").text = "1.0 1.0 0.0 1"
    ET.SubElement(sm, "diffuse").text = "1.0 1.0 0.0 1"

    # Collision cylinder
    col = ET.SubElement(link, "collision", name="collision")
    ET.SubElement(col, "pose").text = _WHEEL_ROLL_POSE
    cc = ET.SubElement(ET.SubElement(col, "geometry"), "cylinder")
    ET.SubElement(cc, "radius").text = str(wheel_r)
    ET.SubElement(cc, "length").text = str(wheel_w)
    surf = ET.SubElement(col, "surface")
    ode_f = ET.SubElement(ET.SubElement(surf, "friction"), "ode")
    ET.SubElement(ode_f, "mu").text = "1.0"
    ET.SubElement(ode_f, "mu2").text = "1.0"
    ode_c = ET.SubElement(ET.SubElement(surf, "contact"), "ode")
    ET.SubElement(ode_c, "kp").text = "1e7"
    ET.SubElement(ode_c, "kd").text = "500"
    ET.SubElement(ode_c, "max_vel").text = "0.01"
    ET.SubElement(ode_c, "min_depth").text = "0.001"

    # Inertial
    iner = ET.SubElement(link, "inertial")
    ET.SubElement(iner, "mass").text = str(wm)
    ix = ET.SubElement(iner, "inertia")
    ET.SubElement(ix, "ixx").text = f"{wheel_ixx:.8f}"
    ET.SubElement(ix, "iyy").text = f"{wheel_iyy:.8f}"
    ET.SubElement(ix, "izz").text = f"{wheel_ixx:.8f}"


def _build_wheels(robot_model: ET.Element) -> None:
    wheel_r = RobotSpecs.WHEEL_RADIUS
    wheel_w = RobotSpecs.WHEEL_WIDTH
    wm = RobotSpecs.WHEEL_MASS
    half_wb = RobotSpecs.WHEELBASE / 2
    half_track = RobotSpecs.TRACK_WIDTH / 2

    wheel_ixx = (1 / 12) * wm * (3 * wheel_r**2 + wheel_w**2)
    wheel_iyy = 0.5 * wm * wheel_r**2
    stripe_offset = round(wheel_r * 0.314, 4)
    stripe_dims = (
        round(0.004 * wheel_r / 0.035, 4),
        round(0.050 * wheel_r / 0.035, 4),
        round(0.008 * wheel_r / 0.035, 4),
    )

    for name, y_sign in [("rear_left_wheel", 1), ("rear_right_wheel", -1)]:
        _build_wheel_link(
            robot_model,
            name,
            f"{-half_wb} {y_sign * half_track} 0 0 0 0",
            wheel_r,
            wheel_w,
            stripe_offset,
            stripe_dims,
            wm,
            wheel_ixx,
            wheel_iyy,
        )

    for wheel_name, joint_name, _y_sign in [
        ("rear_left_wheel", "rear_left_wheel_joint", 1),
        ("rear_right_wheel", "rear_right_wheel_joint", -1),
    ]:
        jt = ET.SubElement(robot_model, "joint", name=joint_name, type="revolute")
        ET.SubElement(jt, "parent").text = "base_link"
        ET.SubElement(jt, "child").text = wheel_name
        ax = ET.SubElement(jt, "axis")
        ET.SubElement(ax, "xyz").text = "0 1 0"
        lim = ET.SubElement(ax, "limit")
        ET.SubElement(lim, "lower").text = "-1e16"
        ET.SubElement(lim, "upper").text = "1e16"
        ET.SubElement(lim, "effort").text = "10.0"
        ET.SubElement(lim, "velocity").text = "100.0"
        dyn = ET.SubElement(ax, "dynamics")
        ET.SubElement(dyn, "friction").text = "0.01"
        ET.SubElement(dyn, "damping").text = "0.01"


def _build_front_steering(robot_model: ET.Element) -> None:
    wheel_r = RobotSpecs.WHEEL_RADIUS
    wheel_w = RobotSpecs.WHEEL_WIDTH
    wm = RobotSpecs.WHEEL_MASS
    half_wb = RobotSpecs.WHEELBASE / 2
    half_track = RobotSpecs.TRACK_WIDTH / 2

    wheel_ixx = (1 / 12) * wm * (3 * wheel_r**2 + wheel_w**2)
    wheel_iyy = 0.5 * wm * wheel_r**2
    stripe_offset = round(wheel_r * 0.314, 4)
    stripe_dims = (
        round(0.004 * wheel_r / 0.035, 4),
        round(0.050 * wheel_r / 0.035, 4),
        round(0.008 * wheel_r / 0.035, 4),
    )

    for side, y_sign in [("left", 1), ("right", -1)]:
        y_pos = y_sign * half_track
        steer_name = f"front_{side}_steering"

        # Steering hinge link (near-zero inertia)
        steer_link = ET.SubElement(robot_model, "link", name=steer_name)
        ET.SubElement(
            steer_link, "pose", relative_to="base_link"
        ).text = f"{half_wb} {y_pos} 0 0 0 0"
        si = ET.SubElement(steer_link, "inertial")
        ET.SubElement(si, "mass").text = "0.001"
        six = ET.SubElement(si, "inertia")
        for tag in ("ixx", "iyy", "izz"):
            ET.SubElement(six, tag).text = "0.00001"
        for tag in ("ixy", "ixz", "iyz"):
            ET.SubElement(six, tag).text = "0"

        # Steering joint (Z-axis, ±max_steering_angle)
        sj = ET.SubElement(
            robot_model,
            "joint",
            name=f"front_{side}_steering_joint",
            type="revolute",
        )
        ET.SubElement(sj, "parent").text = "base_link"
        ET.SubElement(sj, "child").text = steer_name
        sax = ET.SubElement(sj, "axis")
        ET.SubElement(sax, "xyz").text = "0 0 1"
        slim = ET.SubElement(sax, "limit")
        ET.SubElement(slim, "lower").text = f"-{RobotSpecs.MAX_STEERING_ANGLE}"
        ET.SubElement(slim, "upper").text = str(RobotSpecs.MAX_STEERING_ANGLE)
        ET.SubElement(slim, "effort").text = "5.0"
        ET.SubElement(slim, "velocity").text = "10.0"

        # Wheel link attached to steering hinge
        wheel_name = f"front_{side}_wheel"
        _build_wheel_link(
            robot_model,
            wheel_name,
            f"{half_wb} {y_pos} 0 0 0 0",
            wheel_r,
            wheel_w,
            stripe_offset,
            stripe_dims,
            wm,
            wheel_ixx,
            wheel_iyy,
        )

        # Wheel roll joint
        wj = ET.SubElement(
            robot_model,
            "joint",
            name=f"front_{side}_wheel_joint",
            type="revolute",
        )
        ET.SubElement(wj, "parent").text = steer_name
        ET.SubElement(wj, "child").text = wheel_name
        wax = ET.SubElement(wj, "axis")
        ET.SubElement(wax, "xyz").text = "0 1 0"
        wlim = ET.SubElement(wax, "limit")
        ET.SubElement(wlim, "lower").text = "-1e16"
        ET.SubElement(wlim, "upper").text = "1e16"
        ET.SubElement(wlim, "effort").text = "0.0"
        ET.SubElement(wlim, "velocity").text = "100.0"


def _build_camera_link(robot_model: ET.Element) -> None:
    cam_link = ET.SubElement(robot_model, "link", name="camera_link")
    ET.SubElement(
        cam_link, "pose", relative_to="base_link"
    ).text = f"{RobotSpecs.LENGTH / 2} 0 {RobotSpecs.HEIGHT} 0 0 0"
    ci = ET.SubElement(cam_link, "inertial")
    ET.SubElement(ci, "mass").text = "0.01"
    cix = ET.SubElement(ci, "inertia")
    for tag in ("ixx", "iyy", "izz"):
        ET.SubElement(cix, tag).text = "0.00001"

    sensor = ET.SubElement(cam_link, "sensor", name="camera", type="camera")
    ET.SubElement(sensor, "update_rate").text = str(RobotSpecs.CAMERA_UPDATE_RATE)
    ET.SubElement(sensor, "visualize").text = "false"
    ET.SubElement(sensor, "topic").text = "/robot/camera"
    ET.SubElement(sensor, "always_on").text = "true"
    cam = ET.SubElement(sensor, "camera")
    ET.SubElement(cam, "horizontal_fov").text = str(RobotSpecs.CAMERA_HFOV)
    img = ET.SubElement(cam, "image")
    ET.SubElement(img, "width").text = str(RobotSpecs.CAMERA_WIDTH)
    ET.SubElement(img, "height").text = str(RobotSpecs.CAMERA_HEIGHT)
    ET.SubElement(img, "format").text = "R8G8B8"
    clip = ET.SubElement(cam, "clip")
    ET.SubElement(clip, "near").text = str(RobotSpecs.CAMERA_NEAR_CLIP)
    ET.SubElement(clip, "far").text = str(RobotSpecs.CAMERA_FAR_CLIP)

    jt = ET.SubElement(robot_model, "joint", name="camera_joint", type="fixed")
    ET.SubElement(jt, "parent").text = "base_link"
    ET.SubElement(jt, "child").text = "camera_link"


def _build_lidar_link(robot_model: ET.Element) -> None:
    lidar_link = ET.SubElement(robot_model, "link", name="lidar_link")
    ET.SubElement(
        lidar_link, "pose", relative_to="base_link"
    ).text = f"0 0 {RobotSpecs.HEIGHT + 0.02} 0 0 0"
    li = ET.SubElement(lidar_link, "inertial")
    ET.SubElement(li, "mass").text = "0.05"
    lix = ET.SubElement(li, "inertia")
    for tag in ("ixx", "iyy", "izz"):
        ET.SubElement(lix, tag).text = "0.00001"

    sensor = ET.SubElement(lidar_link, "sensor", name="lidar", type="gpu_lidar")
    ET.SubElement(sensor, "update_rate").text = str(RobotSpecs.LIDAR_UPDATE_RATE)
    ET.SubElement(sensor, "visualize").text = "true"
    ET.SubElement(sensor, "topic").text = "lidar"
    ET.SubElement(sensor, "always_on").text = "true"
    lidar = ET.SubElement(sensor, "lidar")
    horizontal = ET.SubElement(ET.SubElement(lidar, "scan"), "horizontal")
    ET.SubElement(horizontal, "samples").text = str(RobotSpecs.LIDAR_SAMPLES)
    ET.SubElement(horizontal, "resolution").text = "1.0"
    ET.SubElement(horizontal, "min_angle").text = str(-math.pi)
    ET.SubElement(horizontal, "max_angle").text = str(math.pi)
    rng = ET.SubElement(lidar, "range")
    ET.SubElement(rng, "min").text = str(RobotSpecs.LIDAR_SIM_MIN_RANGE)
    ET.SubElement(rng, "max").text = str(RobotSpecs.LIDAR_MAX_RANGE)
    ET.SubElement(rng, "resolution").text = "0.01"
    noise = ET.SubElement(lidar, "noise")
    ET.SubElement(noise, "type").text = "gaussian"
    ET.SubElement(noise, "mean").text = "0.0"
    ET.SubElement(noise, "stddev").text = str(RobotSpecs.LIDAR_NOISE_STDDEV)

    jt = ET.SubElement(robot_model, "joint", name="lidar_joint", type="fixed")
    ET.SubElement(jt, "parent").text = "base_link"
    ET.SubElement(jt, "child").text = "lidar_link"


def _build_imu_link(robot_model: ET.Element) -> None:
    imu_link = ET.SubElement(robot_model, "link", name="imu_link")
    ET.SubElement(imu_link, "pose", relative_to="base_link").text = "0 0 0.01 0 0 0"
    ii = ET.SubElement(imu_link, "inertial")
    ET.SubElement(ii, "mass").text = str(RobotSpecs.IMU_MASS)
    iix = ET.SubElement(ii, "inertia")
    for tag in ("ixx", "iyy", "izz"):
        ET.SubElement(iix, tag).text = "0.00001"
    for tag in ("ixy", "ixz", "iyz"):
        ET.SubElement(iix, tag).text = "0"

    vis = ET.SubElement(imu_link, "visual", name="visual")
    sz = RobotSpecs.IMU_SIZE
    _set_box_size(ET.SubElement(vis, "geometry"), sz[0], sz[1], sz[2])
    mat = ET.SubElement(vis, "material")
    ET.SubElement(mat, "ambient").text = "0.0 0.4 0.0 1"
    ET.SubElement(mat, "diffuse").text = "0.0 0.4 0.0 1"

    sensor = ET.SubElement(imu_link, "sensor", name="imu", type="imu")
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = str(RobotSpecs.IMU_UPDATE_RATE)
    ET.SubElement(sensor, "topic").text = "imu"
    imu = ET.SubElement(sensor, "imu")
    for axis_group_tag, noise_std in [
        ("angular_velocity", RobotSpecs.IMU_GYRO_NOISE),
        ("linear_acceleration", RobotSpecs.IMU_ACCEL_NOISE),
    ]:
        ag = ET.SubElement(imu, axis_group_tag)
        for axis in ("x", "y", "z"):
            n = ET.SubElement(ET.SubElement(ag, axis), "noise", type="gaussian")
            ET.SubElement(n, "mean").text = "0.0"
            ET.SubElement(n, "stddev").text = str(noise_std)

    jt = ET.SubElement(robot_model, "joint", name="imu_joint", type="fixed")
    ET.SubElement(jt, "parent").text = "base_link"
    ET.SubElement(jt, "child").text = "imu_link"


def _build_debug_overhead_camera(world: ET.Element) -> None:
    """Append a static overhead debug camera at the track center."""
    debug_camera = ET.Element("model", name="debug_camera")
    ET.SubElement(debug_camera, "static").text = "true"
    ET.SubElement(debug_camera, "pose").text = "1.5 1.5 2.5 0 0 0"
    link = ET.SubElement(debug_camera, "link", name="link")
    sensor = ET.SubElement(link, "sensor", name="camera", type="camera")
    ET.SubElement(sensor, "update_rate").text = "30"
    ET.SubElement(sensor, "visualize").text = "true"
    ET.SubElement(sensor, "topic").text = "camera/image_raw"
    ET.SubElement(sensor, "always_on").text = "true"
    cam = ET.SubElement(sensor, "camera")
    ET.SubElement(cam, "horizontal_fov").text = "1.57"
    img = ET.SubElement(cam, "image")
    ET.SubElement(img, "width").text = "1280"
    ET.SubElement(img, "height").text = "720"
    ET.SubElement(img, "format").text = "R8G8B8"
    clip = ET.SubElement(cam, "clip")
    ET.SubElement(clip, "near").text = "0.1"
    ET.SubElement(clip, "far").text = "10.0"
    world.append(debug_camera)
