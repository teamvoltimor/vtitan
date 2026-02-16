#!/usr/bin/env python3
"""
WRO Training Data Generator

Generates randomized Gazebo simulations and records robot POV camera footage
for training computer vision models (YOLO, classical CV, etc.)

Usage:
    python3 generate_training_data.py --challenge open --num-scenarios 100
    python3 generate_training_data.py --challenge obstacles --num-scenarios 100 --randomize-all
"""

import os
import random
import argparse
import json

# Import WRO constants
from constants import (
    TrackDimensions, WallSpecs, CorridorDimensions,
    TrafficSignSpecs, ParkingLotSpecs, StartingZoneSpecs,
    RobotSpecs, TrackMarkings, LightingSpecs,
    ScenarioTypes, GridSections, FilePaths, RandomizationRanges,
    DictKeys, WidthTypes, ColorNames, ModelNames, FileExtensions, FolderNames
)
from enums import Section, Direction
from pathlib import Path
from xml.etree import ElementTree as ET
import cv2
import numpy as np

# ROS2 imports (if available)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("WARNING: ROS2 not available. Running in standalone mode.")


class ScenarioGenerator:
    """Generates randomized WRO track scenarios"""

    def __init__(self, base_world_path, output_dir, challenge_type='open'):
        self.base_world_path = base_world_path
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.challenge_type = challenge_type

        # WRO 2026 Track dimensions (using constants)
        self.track_bounds = {
            DictKeys.MIN: TrackDimensions.MIN_COORD,
            DictKeys.MAX: TrackDimensions.MAX_COORD,
            DictKeys.CENTER: TrackDimensions.CENTER_COORD,
            DictKeys.Z_SIGN: TrafficSignSpecs.Z_POSITION,
        }

        # Corridor width options for OPEN challenge
        self.corridor_widths = {
            WidthTypes.NARROW: CorridorDimensions.NARROW,
            WidthTypes.WIDE: CorridorDimensions.WIDE
        }

        # Track sections
        self.sections = GridSections.SECTIONS

        # Robot dimensions
        self.robot_width = RobotSpecs.WIDTH

        # Randomization parameters
        self.randomization = {
            DictKeys.LIGHTING: {
                DictKeys.INTENSITY_RANGE: (LightingSpecs.SUN_INTENSITY_MIN, LightingSpecs.SUN_INTENSITY_MAX),
                DictKeys.DIRECTION_VARIANCE: LightingSpecs.DIRECTION_VARIANCE
            },
            DictKeys.COLORS: {
                ColorNames.RED: {
                    DictKeys.MEAN: list(TrafficSignSpecs.RED_COLOR),
                    DictKeys.STD: list(TrafficSignSpecs.RED_STD)
                },
                ColorNames.GREEN: {
                    DictKeys.MEAN: list(TrafficSignSpecs.GREEN_COLOR),
                    DictKeys.STD: list(TrafficSignSpecs.GREEN_STD)
                }
            },
            DictKeys.PHYSICS: {
                DictKeys.FRICTION_RANGE: (RandomizationRanges.FRICTION_MIN, RandomizationRanges.FRICTION_MAX),
                DictKeys.MASS_VARIANCE: RandomizationRanges.MASS_VARIANCE
            }
        }

    # WRO 36 Predefined Scenarios (X, Y coordinates for South corridor)
    SCENARIOS = {
        # Single pillar scenarios (1-12)
        1: [('green', 1.0, 0.6)],  # Near, Inner
        2: [('red', 1.0, 0.6)],
        3: [('green', 1.5, 0.6)],  # Middle, Inner
        4: [('red', 1.5, 0.6)],
        5: [('green', 2.0, 0.6)],  # Far, Inner
        6: [('red', 2.0, 0.6)],
        7: [('green', 1.0, 0.4)],  # Near, Outer
        8: [('red', 1.0, 0.4)],
        9: [('green', 1.5, 0.4)],  # Middle, Outer
        10: [('red', 1.5, 0.4)],
        11: [('green', 2.0, 0.4)],  # Far, Outer (assuming typo in original: 0.6 → 0.4)
        12: [('red', 2.0, 0.4)],    # Far, Outer (assuming typo in original: 0.6 → 0.4)

        # Double pillar scenarios (13-36)
        13: [('green', 1.0, 0.4), ('green', 2.0, 0.6)],
        14: [('green', 1.0, 0.4), ('red', 2.0, 0.6)],
        15: [('red', 1.0, 0.4), ('green', 2.0, 0.6)],
        16: [('green', 1.0, 0.4), ('red', 2.0, 0.6)],  # Duplicate of 14
        17: [('red', 1.0, 0.4), ('green', 2.0, 0.6)],   # Duplicate of 15
        18: [('red', 1.0, 0.4), ('red', 2.0, 0.6)],
        19: [('green', 1.0, 0.6), ('green', 2.0, 0.4)],
        20: [('green', 1.0, 0.6), ('red', 2.0, 0.4)],
        21: [('red', 1.0, 0.6), ('green', 2.0, 0.4)],
        22: [('green', 1.0, 0.6), ('red', 2.0, 0.4)],   # Duplicate of 20
        23: [('red', 1.0, 0.6), ('green', 2.0, 0.4)],   # Duplicate of 21
        24: [('red', 1.0, 0.6), ('red', 2.0, 0.4)],
        25: [('green', 1.0, 0.6), ('green', 2.0, 0.6)],
        26: [('green', 1.0, 0.6), ('red', 2.0, 0.6)],
        27: [('red', 1.0, 0.6), ('green', 2.0, 0.6)],
        28: [('green', 1.0, 0.6), ('red', 2.0, 0.6)],   # Duplicate of 26
        29: [('red', 1.0, 0.6), ('green', 2.0, 0.6)],   # Duplicate of 27
        30: [('red', 1.0, 0.6), ('red', 2.0, 0.6)],
        31: [('green', 1.0, 0.4), ('green', 2.0, 0.4)],
        32: [('green', 1.0, 0.4), ('red', 2.0, 0.4)],
        33: [('red', 1.0, 0.4), ('green', 2.0, 0.4)],
        34: [('green', 1.0, 0.4), ('red', 2.0, 0.4)],   # Duplicate of 32
        35: [('red', 1.0, 0.4), ('green', 2.0, 0.4)],   # Duplicate of 33
        36: [('red', 1.0, 0.4), ('red', 2.0, 0.4)],
    }

    def apply_scenario_to_section(self, scenario_id, section):
        """Transform scenario coordinates from South corridor to target corridor

        Args:
            scenario_id: Scenario ID (1-36)
            section: Section enum (Section.NORTH, Section.SOUTH, Section.EAST, Section.WEST)
        """
        scenario = self.SCENARIOS[scenario_id]
        transformed = []

        for color, x_south, y_south in scenario:
            if section == Section.SOUTH:
                # Already in correct coordinates
                x, y = x_south, y_south
            elif section == Section.NORTH:
                # Mirror X, invert Y around center
                x = x_south  # X stays same
                y = TrackDimensions.MAX_COORD - y_south  # Mirror Y
            elif section == Section.EAST:
                # Swap and transform: South(X,Y) → East(MAX-Y, X)
                x = TrackDimensions.MAX_COORD - y_south  # Y becomes X, mirrored
                y = x_south
            elif section == Section.WEST:
                # Swap: South(X,Y) → West(Y, X)
                x = y_south  # Y becomes X
                y = x_south

            transformed.append((color, x, y))

        return transformed

    def generate_parking_lot_positions(self, starting_section):
        """Generate parking lot positions at grid intersections

        WRO Rules:
        - Parking lot must be in starting section's corner
        - Blocks perpendicular to corridor (rotated 90°)
        - Blocks touching outer wall (one side against wall)
        - First block positioned at one of 3 depth positions: 1.0, 1.5, or 2.0
        - Second block spaced 1.5 × robot_width away (along depth axis)
        - Both blocks must stay within 1000mm × 1000mm corner section
        - For south section: X ∈ [1.0, 2.0], Y ∈ [0.0, 1.0]

        Args:
            starting_section: Section enum (Section.NORTH, Section.SOUTH, Section.EAST, Section.WEST)

        Returns:
            dict with block1_pos, block2_pos, block1_yaw, block2_yaw
        """
        # Grid depth positions (same as traffic sign grid)
        depth_positions = [
            TrafficSignSpecs.GRID_DEPTH_NEAR,
            TrafficSignSpecs.GRID_DEPTH_MIDDLE,
            TrafficSignSpecs.GRID_DEPTH_FAR
        ]

        # Randomly select depth position for first block
        depth = random.choice(depth_positions)

        # Calculate spacing between blocks
        spacing = ParkingLotSpecs.BLOCK_SPACING_FACTOR * self.robot_width

        # Calculate second block position based on depth
        if depth == TrafficSignSpecs.GRID_DEPTH_NEAR:
            # Near edge: second block moves inward
            depth2 = depth + spacing
        elif depth == TrafficSignSpecs.GRID_DEPTH_FAR:
            # Far edge: second block moves inward
            depth2 = depth - spacing
        else:
            # Middle position: randomly choose direction
            if random.random() < 0.5:
                depth2 = depth + spacing
            else:
                depth2 = depth - spacing

        # Blocks perpendicular to corridor, touching outer wall
        wall_offset = ParkingLotSpecs.WALL_OFFSET

        # Transform based on actual starting section
        if starting_section == Section.SOUTH:
            # Blocks perpendicular to south corridor (standing along Y-axis)
            # Touching outer wall at Y=0
            block1_x, block1_y = depth, wall_offset
            block2_x, block2_y = depth2, wall_offset
            block1_yaw = 1.5708  # 90° (perpendicular to corridor)
            block2_yaw = 1.5708  # Parallel to first block

        elif starting_section == Section.NORTH:
            # Blocks perpendicular to north corridor (standing along Y-axis)
            # Touching outer wall at Y=MAX
            block1_x, block1_y = depth, TrackDimensions.MAX_COORD - wall_offset
            block2_x, block2_y = depth2, TrackDimensions.MAX_COORD - wall_offset
            block1_yaw = 1.5708  # 90° (perpendicular to corridor)
            block2_yaw = 1.5708  # Parallel to first block

        elif starting_section == Section.EAST:
            # Blocks perpendicular to east corridor (standing along X-axis)
            # Touching outer wall at X=MAX
            block1_x, block1_y = TrackDimensions.MAX_COORD - wall_offset, depth
            block2_x, block2_y = TrackDimensions.MAX_COORD - wall_offset, depth2
            block1_yaw = 0  # 0° (perpendicular to corridor)
            block2_yaw = 0  # Parallel to first block

        else:  # Section.WEST
            # Blocks perpendicular to west corridor (standing along X-axis)
            # Touching outer wall at X=0
            block1_x, block1_y = wall_offset, depth
            block2_x, block2_y = wall_offset, depth2
            block1_yaw = 0  # 0° (perpendicular to corridor)
            block2_yaw = 0  # Parallel to first block

        return {
            DictKeys.BLOCK1_POS: (block1_x, block1_y),
            DictKeys.BLOCK2_POS: (block2_x, block2_y),
            DictKeys.BLOCK1_YAW: block1_yaw,
            DictKeys.BLOCK2_YAW: block2_yaw,
            DictKeys.DEPTH: depth
        }

    def generate_sign_positions(self, num_signs=8, corridor_widths=None, exclude_section=None):
        """Generate traffic sign positions using WRO 36 predefined scenarios"""

        all_signs = []

        # For each corridor (except excluded starting section), pick a random scenario
        for section in self.sections:
            if exclude_section and section == exclude_section:
                continue

            # Randomly pick one of 36 scenarios for this corridor
            scenario_id = random.randint(1, 36)

            # Get pillars for this scenario transformed to this corridor
            pillars = self.apply_scenario_to_section(scenario_id, section)

            for color, x, y in pillars:
                all_signs.append({DictKeys.X: x, DictKeys.Y: y, DictKeys.COLOR: color})

        # Return positions and colors
        positions = [(sign[DictKeys.X], sign[DictKeys.Y]) for sign in all_signs]
        colors = []
        for sign in all_signs:
            color_name = sign[DictKeys.COLOR]
            # Get official WRO color RGB from randomization config
            color_rgb = self.randomization[DictKeys.COLORS][color_name][DictKeys.MEAN]
            colors.append((color_name, color_rgb))

        return positions, colors

    def randomize_lighting(self):
        """Generate randomized lighting parameters with diverse scenarios

        Creates realistic lighting conditions:
        - Direct sunlight (bright, harsh shadows)
        - Cloudy outdoor (diffuse, soft shadows)
        - Indoor artificial (uniform, moderate brightness)
        - Evening/dawn (warm tones, low angle)
        - Mixed lighting (sun + indoor lights)
        """
        # Choose lighting scenario
        scenarios = ['direct_sunlight', 'cloudy', 'indoor_bright', 'indoor_dim', 'evening', 'mixed']
        scenario = random.choice(scenarios)

        if scenario == 'direct_sunlight':
            # Bright, harsh shadows, high contrast
            sun_intensity = random.uniform(0.9, 1.0)
            ambient_intensity = random.uniform(0.3, 0.4)
            sun_direction = [
                random.uniform(-0.7, -0.3),  # Varied angle
                random.uniform(-0.7, -0.3),
                -1.0
            ]
            cast_shadows = True

        elif scenario == 'cloudy':
            # Diffuse lighting, soft shadows
            sun_intensity = random.uniform(0.6, 0.75)
            ambient_intensity = random.uniform(0.5, 0.6)
            sun_direction = [-0.5, -0.5, -1.0]  # Overhead
            cast_shadows = True

        elif scenario == 'indoor_bright':
            # Bright artificial lighting, minimal shadows
            sun_intensity = random.uniform(0.7, 0.85)
            ambient_intensity = random.uniform(0.6, 0.7)
            sun_direction = [0.0, 0.0, -1.0]  # Directly overhead
            cast_shadows = False

        elif scenario == 'indoor_dim':
            # Dimmer indoor lighting
            sun_intensity = random.uniform(0.5, 0.65)
            ambient_intensity = random.uniform(0.4, 0.5)
            sun_direction = [0.0, 0.0, -1.0]
            cast_shadows = False

        elif scenario == 'evening':
            # Warm, low-angle lighting
            sun_intensity = random.uniform(0.6, 0.8)
            ambient_intensity = random.uniform(0.3, 0.4)
            sun_direction = [
                random.uniform(-0.9, -0.7),  # Low angle
                random.uniform(-0.5, 0.5),
                -0.3  # More horizontal
            ]
            cast_shadows = True

        else:  # mixed
            # Combination of sun and indoor lights
            sun_intensity = random.uniform(0.7, 0.9)
            ambient_intensity = random.uniform(0.5, 0.65)
            sun_direction = [
                random.uniform(-0.6, -0.4),
                random.uniform(-0.6, -0.4),
                -1.0
            ]
            cast_shadows = True

        return {
            DictKeys.INTENSITY: sun_intensity,
            DictKeys.DIRECTION: sun_direction,
            DictKeys.AMBIENT_INTENSITY: ambient_intensity,
            'cast_shadows': cast_shadows,
            'scenario': scenario
        }

    def randomize_corridor_widths(self):
        """Randomize corridor width for each section (OPEN challenge only)

        Returns:
            dict: Dictionary with Section enum keys, each containing type and width
        """
        # WRO: Each section has corridor width either 600mm or 1000mm (coin toss)
        widths = {}
        for section in self.sections:
            # Coin toss: narrow (600mm) or wide (1000mm)
            width_type = random.choice([WidthTypes.NARROW, WidthTypes.WIDE])
            widths[section] = {
                DictKeys.TYPE: width_type,
                DictKeys.WIDTH: self.corridor_widths[width_type]
            }
        return widths

    def randomize_starting_conditions(self, corridor_widths=None):
        """Generate WRO-style randomized starting conditions

        Returns:
            dict: Starting conditions with direction (Direction enum), section (Section enum), etc.
        """
        # Randomize direction (coin toss): clockwise or counterclockwise
        direction = random.choice(list(Direction))

        # Randomize starting section (one of four sides)
        starting_section = random.choice(self.sections)
        print(f'[DEBUG] Available sections: {[str(s) for s in self.sections]}, Selected: {starting_section}')

        # Calculate starting position based on corridor width
        if corridor_widths and starting_section in corridor_widths:
            corridor_width = corridor_widths[starting_section][DictKeys.WIDTH]
        else:
            corridor_width = 0.65  # Default center position

        track_min = self.track_bounds[DictKeys.MIN]
        track_max = self.track_bounds[DictKeys.MAX]
        track_center = self.track_bounds[DictKeys.CENTER]

        # Calculate center of corridor for starting position
        if starting_section == Section.NORTH:
            y_pos = track_max - corridor_width / 2
            start_positions = [
                (track_center - 0.5, y_pos),  # Left
                (track_center, y_pos),        # Center
                (track_center + 0.5, y_pos)   # Right
            ]
        elif starting_section == Section.SOUTH:
            y_pos = corridor_width / 2
            start_positions = [
                (track_center - 0.5, y_pos),
                (track_center, y_pos),
                (track_center + 0.5, y_pos)
            ]
        elif starting_section == Section.EAST:
            x_pos = track_max - corridor_width / 2
            start_positions = [
                (x_pos, track_center - 0.5),
                (x_pos, track_center),
                (x_pos, track_center + 0.5)
            ]
        else:  # Section.WEST
            x_pos = corridor_width / 2
            start_positions = [
                (x_pos, track_center - 0.5),
                (x_pos, track_center),
                (x_pos, track_center + 0.5)
            ]

        starting_position = random.choice(start_positions)

        # Starting orientation based on direction and section
        # Yaw values to make robot face ALONG corridor
        # CW lap (viewed from above): N→E→S→W→N  (right at top, down at right, left at bottom, up at left)
        yaw_map = {
            Section.SOUTH: {Direction.CLOCKWISE: 3.14159, Direction.COUNTERCLOCKWISE: 0.0},        # CW=West(-X), CCW=East(+X)
            Section.NORTH: {Direction.CLOCKWISE: 0.0, Direction.COUNTERCLOCKWISE: 3.14159},        # CW=East(+X), CCW=West(-X)
            Section.EAST: {Direction.CLOCKWISE: -1.5708, Direction.COUNTERCLOCKWISE: 1.5708},      # CW=South(-Y), CCW=North(+Y)
            Section.WEST: {Direction.CLOCKWISE: 1.5708, Direction.COUNTERCLOCKWISE: -1.5708}       # CW=North(+Y), CCW=South(-Y)
        }

        print(f'[DEBUG] Robot starting in {starting_section.capitalized} corridor, {direction} direction, yaw={yaw_map[starting_section][direction]:.2f} rad')

        starting_yaw = yaw_map[starting_section][direction]

        return {
            DictKeys.DIRECTION: direction,
            DictKeys.SECTION: starting_section,
            DictKeys.SECTION_NAME: starting_section.capitalized,
            DictKeys.POSITION: starting_position,
            DictKeys.YAW: starting_yaw
        }

    def randomize_color(self, color_name):
        """Generate randomized color with Gaussian noise"""
        params = self.randomization[DictKeys.COLORS][color_name]
        color = np.random.normal(params[DictKeys.MEAN], params[DictKeys.STD])
        color = np.clip(color, 0.0, 1.0)
        return color.tolist()

    def create_scenario_world(self, scenario_id, randomize_all=True):
        """Create a randomized world SDF file"""
        print(f'\n[DEBUG] create_scenario_world called with randomize_all={randomize_all}')

        # Parse base world
        tree = ET.parse(self.base_world_path)
        root = tree.getroot()
        world = root.find('world')

        # Add Sensors system plugin (required for cameras/LIDAR in Gazebo Harmonic)
        existing_sensors = world.find(".//plugin[@name='gz::sim::systems::Sensors']")
        if existing_sensors is None:
            sensors_plugin = ET.Element('plugin',
                                       filename='gz-sim-sensors-system',
                                       name='gz::sim::systems::Sensors')
            ET.SubElement(sensors_plugin, 'render_engine').text = 'ogre2'
            world.insert(0, sensors_plugin)

        # Add Physics system plugin (CRITICAL: Required for DiffDrive to work!)
        existing_physics = world.find(".//plugin[@name='gz::sim::systems::Physics']")
        if existing_physics is None:
            physics_plugin = ET.Element('plugin',
                                       filename='gz-sim-physics-system',
                                       name='gz::sim::systems::Physics')
            world.insert(1, physics_plugin)

        # Randomize corridor widths for OPEN challenge (interior walls)
        # OBSTACLES challenge has fixed corridor width
        corridor_widths = None
        if self.challenge_type == ScenarioTypes.OPEN and randomize_all:
            corridor_widths = self.randomize_corridor_widths()
        elif self.challenge_type == ScenarioTypes.OPEN:
            # Default fixed widths for open challenge (1000mm wide corridors)
            corridor_widths = {section: {DictKeys.TYPE: WidthTypes.WIDE, DictKeys.WIDTH: CorridorDimensions.WIDE} for section in self.sections}
        else:
            # Obstacles challenge: fixed 1.0m corridor (creates 1.0m × 1.0m inner area)
            corridor_widths = {section: {DictKeys.TYPE: WidthTypes.FIXED, DictKeys.WIDTH: 1.0} for section in self.sections}

        # Calculate interior wall positions for all sections
        # Each section's interior wall is offset from exterior based on corridor width
        track_min = self.track_bounds[DictKeys.MIN]
        track_max = self.track_bounds[DictKeys.MAX]

        # Interior wall positions:
        # North: y = track_max - corridor_width_north
        # South: y = corridor_width_south
        # East: x = track_max - corridor_width_east
        # West: x = corridor_width_west
        north_interior_y = track_max - corridor_widths[Section.NORTH][DictKeys.WIDTH]
        south_interior_y = corridor_widths[Section.SOUTH][DictKeys.WIDTH]
        east_interior_x = track_max - corridor_widths[Section.EAST][DictKeys.WIDTH]
        west_interior_x = corridor_widths[Section.WEST][DictKeys.WIDTH]

        # North interior wall (spans from west to east, thickness extends inward)
        # Wall positioned so outer face is at north_interior_y, inner face extends inward
        wall_north = ET.Element('model', name=ModelNames.INTERIOR_WALL_NORTH)
        ET.SubElement(wall_north, 'static').text = 'true'
        north_length = east_interior_x - west_interior_x
        north_center_x = (east_interior_x + west_interior_x) / 2
        north_wall_y = north_interior_y - 0.05  # Shift inward by half thickness
        ET.SubElement(wall_north, 'pose').text = f"{north_center_x} {north_wall_y} 0.05 0 0 0"

        link_n = ET.SubElement(wall_north, 'link', name='link')
        visual_n = ET.SubElement(link_n, 'visual', name='visual')
        geom_n = ET.SubElement(visual_n, 'geometry')
        box_n = ET.SubElement(geom_n, 'box')
        ET.SubElement(box_n, 'size').text = f"{north_length} 0.1 0.1"

        material_n = ET.SubElement(visual_n, 'material')
        ET.SubElement(material_n, 'ambient').text = '0.0 0.0 0.0 1'
        ET.SubElement(material_n, 'diffuse').text = '0.0 0.0 0.0 1'
        ET.SubElement(material_n, 'specular').text = '0.0 0.0 0.0 1'

        collision_n = ET.SubElement(link_n, 'collision', name='collision')
        geom_n_col = ET.SubElement(collision_n, 'geometry')
        box_n_col = ET.SubElement(geom_n_col, 'box')
        ET.SubElement(box_n_col, 'size').text = f"{north_length} {WallSpecs.COLLISION_THICKNESS} 0.1"
        surf_n = ET.SubElement(collision_n, 'surface')
        cont_n = ET.SubElement(surf_n, 'contact')
        ode_n = ET.SubElement(cont_n, 'ode')
        ET.SubElement(ode_n, 'kp').text = '1e8'
        ET.SubElement(ode_n, 'kd').text = '1000'
        ET.SubElement(ode_n, 'max_vel').text = '0.0'
        ET.SubElement(ode_n, 'min_depth').text = '0.0'
        world.append(wall_north)

        # South interior wall (spans from west to east, thickness extends inward)
        # Wall positioned so outer face is at south_interior_y, inner face extends inward
        wall_south = ET.Element('model', name=ModelNames.INTERIOR_WALL_SOUTH)
        ET.SubElement(wall_south, 'static').text = 'true'
        south_length = east_interior_x - west_interior_x
        south_center_x = (east_interior_x + west_interior_x) / 2
        south_wall_y = south_interior_y + 0.05  # Shift inward by half thickness
        ET.SubElement(wall_south, 'pose').text = f"{south_center_x} {south_wall_y} 0.05 0 0 0"

        link_s = ET.SubElement(wall_south, 'link', name='link')
        visual_s = ET.SubElement(link_s, 'visual', name='visual')
        geom_s = ET.SubElement(visual_s, 'geometry')
        box_s = ET.SubElement(geom_s, 'box')
        ET.SubElement(box_s, 'size').text = f"{south_length} 0.1 0.1"

        material_s = ET.SubElement(visual_s, 'material')
        ET.SubElement(material_s, 'ambient').text = '0.0 0.0 0.0 1'
        ET.SubElement(material_s, 'diffuse').text = '0.0 0.0 0.0 1'
        ET.SubElement(material_s, 'specular').text = '0.0 0.0 0.0 1'

        collision_s = ET.SubElement(link_s, 'collision', name='collision')
        geom_s_col = ET.SubElement(collision_s, 'geometry')
        box_s_col = ET.SubElement(geom_s_col, 'box')
        ET.SubElement(box_s_col, 'size').text = f"{south_length} {WallSpecs.COLLISION_THICKNESS} 0.1"
        surf_s = ET.SubElement(collision_s, 'surface')
        cont_s = ET.SubElement(surf_s, 'contact')
        ode_s = ET.SubElement(cont_s, 'ode')
        ET.SubElement(ode_s, 'kp').text = '1e8'
        ET.SubElement(ode_s, 'kd').text = '1000'
        ET.SubElement(ode_s, 'max_vel').text = '0.0'
        ET.SubElement(ode_s, 'min_depth').text = '0.0'
        world.append(wall_south)

        # East interior wall (spans from south to north, thickness extends inward)
        # Wall positioned so outer face is at east_interior_x, inner face extends inward
        wall_east = ET.Element('model', name=ModelNames.INTERIOR_WALL_EAST)
        ET.SubElement(wall_east, 'static').text = 'true'
        east_length = north_interior_y - south_interior_y
        east_center_y = (north_interior_y + south_interior_y) / 2
        east_wall_x = east_interior_x - 0.05  # Shift inward by half thickness
        ET.SubElement(wall_east, 'pose').text = f"{east_wall_x} {east_center_y} 0.05 0 0 0"

        link_e = ET.SubElement(wall_east, 'link', name='link')
        visual_e = ET.SubElement(link_e, 'visual', name='visual')
        geom_e = ET.SubElement(visual_e, 'geometry')
        box_e = ET.SubElement(geom_e, 'box')
        ET.SubElement(box_e, 'size').text = f"0.1 {east_length} 0.1"

        material_e = ET.SubElement(visual_e, 'material')
        ET.SubElement(material_e, 'ambient').text = '0.0 0.0 0.0 1'
        ET.SubElement(material_e, 'diffuse').text = '0.0 0.0 0.0 1'
        ET.SubElement(material_e, 'specular').text = '0.0 0.0 0.0 1'

        collision_e = ET.SubElement(link_e, 'collision', name='collision')
        geom_e_col = ET.SubElement(collision_e, 'geometry')
        box_e_col = ET.SubElement(geom_e_col, 'box')
        ET.SubElement(box_e_col, 'size').text = f"{WallSpecs.COLLISION_THICKNESS} {east_length} 0.1"
        surf_e = ET.SubElement(collision_e, 'surface')
        cont_e = ET.SubElement(surf_e, 'contact')
        ode_e = ET.SubElement(cont_e, 'ode')
        ET.SubElement(ode_e, 'kp').text = '1e8'
        ET.SubElement(ode_e, 'kd').text = '1000'
        ET.SubElement(ode_e, 'max_vel').text = '0.0'
        ET.SubElement(ode_e, 'min_depth').text = '0.0'
        world.append(wall_east)

        # West interior wall (spans from south to north, thickness extends inward)
        # Wall positioned so outer face is at west_interior_x, inner face extends inward
        wall_west = ET.Element('model', name=ModelNames.INTERIOR_WALL_WEST)
        ET.SubElement(wall_west, 'static').text = 'true'
        west_length = north_interior_y - south_interior_y
        west_center_y = (north_interior_y + south_interior_y) / 2
        west_wall_x = west_interior_x + 0.05  # Shift inward by half thickness
        ET.SubElement(wall_west, 'pose').text = f"{west_wall_x} {west_center_y} 0.05 0 0 0"

        link_w = ET.SubElement(wall_west, 'link', name='link')
        visual_w = ET.SubElement(link_w, 'visual', name='visual')
        geom_w = ET.SubElement(visual_w, 'geometry')
        box_w = ET.SubElement(geom_w, 'box')
        ET.SubElement(box_w, 'size').text = f"0.1 {west_length} 0.1"

        material_w = ET.SubElement(visual_w, 'material')
        ET.SubElement(material_w, 'ambient').text = '0.0 0.0 0.0 1'
        ET.SubElement(material_w, 'diffuse').text = '0.0 0.0 0.0 1'
        ET.SubElement(material_w, 'specular').text = '0.0 0.0 0.0 1'

        collision_w = ET.SubElement(link_w, 'collision', name='collision')
        geom_w_col = ET.SubElement(collision_w, 'geometry')
        box_w_col = ET.SubElement(geom_w_col, 'box')
        ET.SubElement(box_w_col, 'size').text = f"{WallSpecs.COLLISION_THICKNESS} {west_length} 0.1"
        surf_w = ET.SubElement(collision_w, 'surface')
        cont_w = ET.SubElement(surf_w, 'contact')
        ode_w = ET.SubElement(cont_w, 'ode')
        ET.SubElement(ode_w, 'kp').text = '1e8'
        ET.SubElement(ode_w, 'kd').text = '1000'
        ET.SubElement(ode_w, 'max_vel').text = '0.0'
        ET.SubElement(ode_w, 'min_depth').text = '0.0'
        world.append(wall_west)

        # Randomize starting conditions (WRO-style)
        starting_conditions = self.randomize_starting_conditions(corridor_widths) if randomize_all else {
            'direction': Direction.CLOCKWISE,
            'section': Section.SOUTH,
            'section_name': 'South',
            'position': (1.5, 0.4),  # South corridor center, inside track (new coordinate system)
            'yaw': 3.14159  # π = facing West (CW in South corridor: S→W corner)
        }

        # Randomize lighting
        if randomize_all:
            lighting = self.randomize_lighting()

            # Update sun light (clamp intensity to [0.0, 1.0] for valid SDF)
            sun = world.find(f".//light[@name='{ModelNames.SUN_LIGHT}']")
            if sun is not None:
                diffuse = sun.find('diffuse')
                intensity = min(1.0, max(0.0, lighting[DictKeys.INTENSITY]))  # Clamp to [0.0, 1.0]
                diffuse.text = f"{intensity} {intensity} {intensity} 1"

                direction = sun.find('direction')
                direction.text = f"{lighting[DictKeys.DIRECTION][0]} {lighting[DictKeys.DIRECTION][1]} {lighting[DictKeys.DIRECTION][2]}"

                # Update shadow casting based on scenario
                cast_shadows = sun.find('cast_shadows')
                if cast_shadows is not None:
                    cast_shadows.text = 'true' if lighting.get('cast_shadows', True) else 'false'

            # Update ambient light (clamp to valid range)
            ambient = world.find(f".//light[@name='{ModelNames.AMBIENT_LIGHT}']")
            if ambient is not None:
                diffuse = ambient.find('diffuse')
                amb_intensity = min(1.0, max(0.0, lighting[DictKeys.AMBIENT_INTENSITY]))  # Clamp to [0.0, 1.0]
                diffuse.text = f"{amb_intensity} {amb_intensity} {amb_intensity} 1"

        # WRO: Traffic signs ONLY in obstacles challenge (Spec 13.19-13.22)
        # Open challenge has NO traffic signs
        sign_positions = []
        sign_colors = []
        parking_config = None

        if self.challenge_type == ScenarioTypes.OBSTACLES:
            # Generate traffic signs for obstacles challenge only
            # Using WRO 36 predefined scenarios (1-36) per corridor
            # Each corridor gets one random scenario, pillars placed at grid intersections
            # Note: Starting section will have signs, but they'll be moved away from parking blocks
            sign_positions, sign_colors = self.generate_sign_positions(num_signs=0, corridor_widths=corridor_widths, exclude_section=None)

            # Generate parking lot BEFORE adding signs so we can adjust sign positions
            starting_section = starting_conditions[DictKeys.SECTION]
            parking_config = self.generate_parking_lot_positions(starting_section)

            # Move traffic signs in parking section closer to inner wall to avoid collision
            # Parking blocks are at 0.1m from outer wall, so move signs from outer position (0.4) to inner (0.6)
            adjusted_signs = []
            adjusted_colors = []

            for (x, y), (color_name, color_rgb) in zip(sign_positions, sign_colors):
                adjusted_x, adjusted_y = x, y

                # Check if sign is in parking section and move to inner position if too close to outer wall
                if starting_section == Section.SOUTH:
                    # South: parking at Y~0.1, move signs from Y=0.4 to Y=0.6
                    if 1.0 <= x <= 2.0 and abs(y - 0.4) < 0.05:  # Sign at outer position
                        adjusted_y = 0.6  # Move to inner position
                elif starting_section == Section.NORTH:
                    # North: parking at Y~2.9, move signs from Y=2.6 to Y=2.4
                    if 1.0 <= x <= 2.0 and abs(y - 2.6) < 0.05:
                        adjusted_y = 2.4
                elif starting_section == Section.EAST:
                    # East: parking at X~2.9, move signs from X=2.6 to X=2.4
                    if 1.0 <= y <= 2.0 and abs(x - 2.6) < 0.05:
                        adjusted_x = 2.4
                elif starting_section == Section.WEST:
                    # West: parking at X~0.1, move signs from X=0.4 to X=0.6
                    if 1.0 <= y <= 2.0 and abs(x - 0.4) < 0.05:
                        adjusted_x = 0.6

                adjusted_signs.append((adjusted_x, adjusted_y))
                adjusted_colors.append((color_name, color_rgb))

            # Update sign positions with adjusted values
            sign_positions = adjusted_signs
            sign_colors = adjusted_colors

        # Add traffic signs to world (rectangular boxes per WRO Spec 13.19)
        for i, ((x, y), (color_name, color_rgb)) in enumerate(zip(sign_positions, sign_colors)):
            sign_prefix = ModelNames.RED_SIGN_PREFIX if color_name == ColorNames.RED else ModelNames.GREEN_SIGN_PREFIX
            sign_model = ET.Element('model', name=f'{sign_prefix}{i}')
            ET.SubElement(sign_model, 'static').text = 'true'
            ET.SubElement(sign_model, 'pose').text = f'{x} {y} {TrafficSignSpecs.Z_POSITION} 0 0 0'

            link = ET.SubElement(sign_model, 'link', name='link')

            # Visual (Rectangular parallelepiped per WRO Spec 13.19)
            visual = ET.SubElement(link, 'visual', name='visual')
            geom = ET.SubElement(visual, 'geometry')
            box = ET.SubElement(geom, 'box')
            ET.SubElement(box, 'size').text = f'{TrafficSignSpecs.WIDTH} {TrafficSignSpecs.DEPTH} {TrafficSignSpecs.HEIGHT}'

            material = ET.SubElement(visual, 'material')
            ET.SubElement(material, 'ambient').text = f'{color_rgb[0]} {color_rgb[1]} {color_rgb[2]} 1'
            ET.SubElement(material, 'diffuse').text = f'{color_rgb[0]} {color_rgb[1]} {color_rgb[2]} 1'
            ET.SubElement(material, 'specular').text = '0.2 0.2 0.2 1'

            # Collision
            collision = ET.SubElement(link, 'collision', name='collision')
            geom = ET.SubElement(collision, 'geometry')
            box = ET.SubElement(geom, 'box')
            ET.SubElement(box, 'size').text = f'{TrafficSignSpecs.WIDTH} {TrafficSignSpecs.DEPTH} {TrafficSignSpecs.HEIGHT}'

            world.append(sign_model)

        # NOTE: In WRO Future Engineers, there are NO separate obstacle objects!
        # Both "open" and "obstacles" challenges use the same traffic signs.
        # The difference is in the navigation rules, not the physical objects.
        # Parking limitations (magenta blocks) only appear in obstacles challenge.

        # Add parking lot if obstacles challenge
        # WRO Rule: Parking lot is always in the starting section
        # Note: parking_config was already generated earlier when adjusting sign positions
        if self.challenge_type == ScenarioTypes.OBSTACLES:
            block1_x, block1_y = parking_config[DictKeys.BLOCK1_POS]
            block2_x, block2_y = parking_config[DictKeys.BLOCK2_POS]
            block1_yaw = parking_config[DictKeys.BLOCK1_YAW]
            block2_yaw = parking_config[DictKeys.BLOCK2_YAW]

            # Parking limitation 1
            parking1 = ET.Element('model', name=ModelNames.PARKING_LIMITATION_1)
            ET.SubElement(parking1, 'static').text = 'true'
            ET.SubElement(parking1, 'pose').text = f'{block1_x} {block1_y} {ParkingLotSpecs.Z_POSITION} 0 0 {block1_yaw}'

            link1 = ET.SubElement(parking1, 'link', name='link')
            visual1 = ET.SubElement(link1, 'visual', name='visual')
            geom1 = ET.SubElement(visual1, 'geometry')
            box1 = ET.SubElement(geom1, 'box')
            ET.SubElement(box1, 'size').text = f'{ParkingLotSpecs.LENGTH} {ParkingLotSpecs.WIDTH} {ParkingLotSpecs.HEIGHT}'

            material1 = ET.SubElement(visual1, 'material')
            parking_color = ParkingLotSpecs.COLOR
            ET.SubElement(material1, 'ambient').text = f'{parking_color[0]} {parking_color[1]} {parking_color[2]} 1'
            ET.SubElement(material1, 'diffuse').text = f'{parking_color[0]} {parking_color[1]} {parking_color[2]} 1'

            collision1 = ET.SubElement(link1, 'collision', name='collision')
            geom1_col = ET.SubElement(collision1, 'geometry')
            box1_col = ET.SubElement(geom1_col, 'box')
            ET.SubElement(box1_col, 'size').text = f'{ParkingLotSpecs.LENGTH} {ParkingLotSpecs.WIDTH} {ParkingLotSpecs.HEIGHT}'

            world.append(parking1)

            # Parking limitation 2 (parallel to first block)
            parking2 = ET.Element('model', name=ModelNames.PARKING_LIMITATION_2)
            ET.SubElement(parking2, 'static').text = 'true'
            ET.SubElement(parking2, 'pose').text = f'{block2_x} {block2_y} {ParkingLotSpecs.Z_POSITION} 0 0 {block2_yaw}'

            link2 = ET.SubElement(parking2, 'link', name='link')
            visual2 = ET.SubElement(link2, 'visual', name='visual')
            geom2 = ET.SubElement(visual2, 'geometry')
            box2 = ET.SubElement(geom2, 'box')
            ET.SubElement(box2, 'size').text = f'{ParkingLotSpecs.LENGTH} {ParkingLotSpecs.WIDTH} {ParkingLotSpecs.HEIGHT}'

            material2 = ET.SubElement(visual2, 'material')
            ET.SubElement(material2, 'ambient').text = f'{parking_color[0]} {parking_color[1]} {parking_color[2]} 1'
            ET.SubElement(material2, 'diffuse').text = f'{parking_color[0]} {parking_color[1]} {parking_color[2]} 1'

            collision2 = ET.SubElement(link2, 'collision', name='collision')
            geom2_col = ET.SubElement(collision2, 'geometry')
            box2_col = ET.SubElement(geom2_col, 'box')
            ET.SubElement(box2_col, 'size').text = f'{ParkingLotSpecs.LENGTH} {ParkingLotSpecs.WIDTH} {ParkingLotSpecs.HEIGHT}'

            world.append(parking2)

        # Add starting zone visual marker (200×500mm grey rectangle, WRO Spec 13.10-13.11)
        # Positioned in one of 6 sections per corridor (2 length × 3 width divisions)
        starting_section = starting_conditions[DictKeys.SECTION]

        # Get corridor width for starting section to position zone correctly
        start_corridor_width = corridor_widths[starting_section][DictKeys.WIDTH]

        # Each corridor has 6 sections (2 along length × 3 across width)
        # Length sections: divided by centerline at 1.5 (for N/S) or 1.5 (for E/W)
        # Width sections: outer (400mm), middle (200mm), inner (400mm)

        # Define width section centers based on corridor divisions
        # For 1000mm corridor: outer=0.2, middle=0.5, inner=0.8 (from outer edge)
        # For 600mm corridor: outer=0.2, inner=0.5 (zone edge at inner wall)
        width_sections = []
        if start_corridor_width >= 1.0:
            # Wide corridor: all 3 width sections available
            width_sections = [0.2, 0.5, 0.8]  # Outer, middle, inner
        else:
            # Narrow corridor: outer and inner (right against inner wall)
            # Zone width is 0.2m, corridor is 0.6m, so inner position at 0.5m puts zone edge at 0.6m
            width_sections = [0.2, 0.5]  # Outer, inner (zone edge touches inner wall)

        # Randomly pick one of the width sections
        width_offset = random.choice(width_sections)

        # Randomly pick one of the 2 length sections (left/right or top/bottom)
        # Length divided by centerline at 1.5
        length_sections = [1.25, 1.75]  # Centers of [1.0-1.5] and [1.5-2.0]
        length_offset = random.choice(length_sections)

        # Calculate starting zone position and size based on section
        # Starting zone is 200mm wide (across corridor) × 500mm long (along corridor)
        # For obstacles challenge, adjust length based on parking block spacing
        zone_length = StartingZoneSpecs.DEFAULT_LENGTH

        if self.challenge_type == ScenarioTypes.OBSTACLES and parking_config:
            # Calculate available space between parking blocks
            # Parking blocks are 20mm wide in the spacing direction
            block_width_in_spacing_dir = ParkingLotSpecs.WIDTH

            # Get the two depth positions
            depth1 = parking_config[DictKeys.DEPTH]
            block1_pos = parking_config[DictKeys.BLOCK1_POS]
            block2_pos = parking_config[DictKeys.BLOCK2_POS]

            # Calculate spacing between blocks (center to center)
            if starting_section == Section.NORTH or starting_section == Section.SOUTH:
                spacing = abs(block2_pos[0] - block1_pos[0])
            else:  # Section.EAST or Section.WEST
                spacing = abs(block2_pos[1] - block1_pos[1])

            # Available gap = spacing - block_width
            available_gap = spacing - block_width_in_spacing_dir

            # Shrink zone length to fit in gap (with small margin for safety)
            if available_gap < zone_length:
                zone_length = available_gap * StartingZoneSpecs.OBSTACLES_SIZE_FACTOR

        # For obstacles challenge, position starting zone between parking blocks
        zone_center_along_depth = None
        zone_width_position = None
        if self.challenge_type == ScenarioTypes.OBSTACLES and parking_config:
            block1_pos = parking_config[DictKeys.BLOCK1_POS]
            block2_pos = parking_config[DictKeys.BLOCK2_POS]

            # Calculate center position between the two parking blocks (along depth)
            # And get width position from parking blocks (same as parking area)
            if starting_section == Section.NORTH or starting_section == Section.SOUTH:
                # Zone extends along X-axis, calculate center X (depth)
                zone_center_along_depth = (block1_pos[0] + block2_pos[0]) / 2
                # Zone width in Y, use same Y as parking blocks
                zone_width_position = block1_pos[1]  # Parking blocks Y position
            else:  # Section.EAST or Section.WEST
                # Zone extends along Y-axis, calculate center Y (depth)
                zone_center_along_depth = (block1_pos[1] + block2_pos[1]) / 2
                # Zone width in X, use same X as parking blocks
                zone_width_position = block1_pos[0]  # Parking blocks X position

        if starting_section == Section.NORTH or starting_section == Section.SOUTH:
            # Horizontal corridor: zone extends along X, width in Y
            zone_size = f'{zone_length} {StartingZoneSpecs.WIDTH} {StartingZoneSpecs.THICKNESS}'
            # Position zone between parking blocks for obstacles, otherwise use random position
            zone_x = zone_center_along_depth if zone_center_along_depth is not None else length_offset

            # Position zone width at parking area for obstacles, otherwise use random position
            if zone_width_position is not None:
                zone_y = zone_width_position
            elif starting_section == Section.SOUTH:
                zone_y = width_offset  # One of the 2-3 width sections
            else:  # Section.NORTH
                # Mirror for north: outer at 2.8, middle at 2.5, inner at 2.2
                if start_corridor_width >= 1.0:
                    zone_y = track_max - width_offset
                else:
                    # For narrow corridor, adjust
                    zone_y = track_max - width_offset
        else:  # Section.EAST or Section.WEST
            # Vertical corridor: zone extends along Y, width in X
            zone_size = f'{StartingZoneSpecs.WIDTH} {zone_length} {StartingZoneSpecs.THICKNESS}'
            # Position zone between parking blocks for obstacles, otherwise use random position
            zone_y = zone_center_along_depth if zone_center_along_depth is not None else length_offset

            # Position zone width at parking area for obstacles, otherwise use random position
            if zone_width_position is not None:
                zone_x = zone_width_position
            elif starting_section == Section.WEST:
                zone_x = width_offset  # One of the 2-3 width sections
            else:  # Section.EAST
                # Mirror for east
                if start_corridor_width >= 1.0:
                    zone_x = track_max - width_offset
                else:
                    zone_x = track_max - width_offset

        # Remove existing starting zone from base world if it exists
        existing_zones = world.findall(".//model[@name='starting_zone_south']")
        for zone in existing_zones:
            world.remove(zone)

        # Create new starting zone at randomized position
        starting_zone = ET.Element('model', name=f'{ModelNames.STARTING_ZONE_PREFIX}{str(starting_section)}')
        ET.SubElement(starting_zone, 'static').text = 'true'
        ET.SubElement(starting_zone, 'pose').text = f'{zone_x} {zone_y} 0.0002 0 0 0'

        link_zone = ET.SubElement(starting_zone, 'link', name='link')

        # Base rectangle (starting zone marker) - darker grey for better visibility
        visual_zone = ET.SubElement(link_zone, 'visual', name='visual_base')
        geom_zone = ET.SubElement(visual_zone, 'geometry')
        box_zone = ET.SubElement(geom_zone, 'box')
        ET.SubElement(box_zone, 'size').text = zone_size

        material_zone = ET.SubElement(visual_zone, 'material')
        zone_color = StartingZoneSpecs.COLOR
        ET.SubElement(material_zone, 'ambient').text = f'{zone_color[0]} {zone_color[1]} {zone_color[2]} 1'
        ET.SubElement(material_zone, 'diffuse').text = f'{zone_color[0]} {zone_color[1]} {zone_color[2]} 1'

        # Direction icon overlay (clockwise or counterclockwise)
        direction = starting_conditions[DictKeys.DIRECTION]

        # Try PNG first (better Gazebo support), fall back to SVG
        simulation_path = os.path.dirname(os.path.dirname(self.base_world_path))
        png_path = os.path.join(simulation_path, f'{str(direction)}{FileExtensions.PNG}')
        svg_path = os.path.join(simulation_path, f'{str(direction)}{FileExtensions.SVG}')

        if os.path.exists(png_path):
            icon_path = png_path
        elif os.path.exists(svg_path):
            icon_path = svg_path
        else:
            icon_path = None  # No icon available

        # Convert Windows path to forward slashes for Gazebo URI
        if icon_path:
            icon_path = icon_path.replace('\\', '/')

        # Simple colored indicator for direction
        # Blue for clockwise, Green for counterclockwise
        direction = starting_conditions['direction']
        if direction == Direction.CLOCKWISE:
            indicator_rgb = StartingZoneSpecs.CLOCKWISE_COLOR
        else:
            indicator_rgb = StartingZoneSpecs.COUNTERCLOCKWISE_COLOR

        indicator_color = f'{indicator_rgb[0]} {indicator_rgb[1]} {indicator_rgb[2]} 1'

        # Circle indicator (cylinder viewed from top)
        visual_indicator = ET.SubElement(link_zone, 'visual', name='visual_direction_indicator')
        ET.SubElement(visual_indicator, 'pose').text = '0 0 0.004 0 0 0'
        geom_indicator = ET.SubElement(visual_indicator, 'geometry')
        cylinder_indicator = ET.SubElement(geom_indicator, 'cylinder')
        ET.SubElement(cylinder_indicator, 'radius').text = str(StartingZoneSpecs.INDICATOR_RADIUS)
        ET.SubElement(cylinder_indicator, 'length').text = str(StartingZoneSpecs.THICKNESS)

        material_indicator = ET.SubElement(visual_indicator, 'material')
        ET.SubElement(material_indicator, 'ambient').text = indicator_color
        ET.SubElement(material_indicator, 'diffuse').text = indicator_color
        ET.SubElement(material_indicator, 'emissive').text = indicator_color  # Make it glow

        world.append(starting_zone)

        # Add robot model with integrated sensors (Ackermann steering, URDF-matched)
        start_yaw = starting_conditions[DictKeys.YAW]

        robot_x = zone_x
        robot_y = zone_y
        robot_z = RobotSpecs.WHEEL_RADIUS  # Spawn at wheel radius height

        # Pre-compute derived dimensions
        half_wheelbase = RobotSpecs.WHEELBASE / 2   # 0.085
        half_track = RobotSpecs.TRACK_WIDTH / 2      # 0.0525
        half_height = RobotSpecs.HEIGHT / 2           # 0.05
        wheel_r = RobotSpecs.WHEEL_RADIUS             # 0.0216
        wheel_w = RobotSpecs.WHEEL_WIDTH               # 0.020

        # Chassis inertia (solid box: I = (1/12)*m*(a^2+b^2))
        cm = RobotSpecs.CHASSIS_MASS
        chassis_ixx = (1/12) * cm * (RobotSpecs.WIDTH**2 + RobotSpecs.HEIGHT**2)
        chassis_iyy = (1/12) * cm * (RobotSpecs.LENGTH**2 + RobotSpecs.HEIGHT**2)
        chassis_izz = (1/12) * cm * (RobotSpecs.LENGTH**2 + RobotSpecs.WIDTH**2)

        # Wheel inertia (solid cylinder, axis along Y after rotation)
        wm = RobotSpecs.WHEEL_MASS
        wheel_ixx = (1/12) * wm * (3 * wheel_r**2 + wheel_w**2)
        wheel_iyy = 0.5 * wm * wheel_r**2
        wheel_izz = wheel_ixx

        # Yellow stripe dimensions (scaled proportionally to wheel radius)
        stripe_offset = round(wheel_r * 0.314, 4)
        stripe_x = round(0.004 * wheel_r / 0.035, 4)
        stripe_y = round(0.050 * wheel_r / 0.035, 4)
        stripe_z = round(0.008 * wheel_r / 0.035, 4)

        robot_model = ET.Element('model', name='wro_robot')
        ET.SubElement(robot_model, 'pose').text = f'{robot_x} {robot_y} {robot_z} 0 0 {start_yaw}'

        # ── Base link (chassis) ──────────────────────────────────────────
        base_link = ET.SubElement(robot_model, 'link', name='base_link')

        # Visual (blue box - robot body), raised so chassis clears ground
        visual_robot = ET.SubElement(base_link, 'visual', name='visual')
        ET.SubElement(visual_robot, 'pose').text = f'0 0 {half_height} 0 0 0'
        geom_robot = ET.SubElement(visual_robot, 'geometry')
        box_robot = ET.SubElement(geom_robot, 'box')
        ET.SubElement(box_robot, 'size').text = f'{RobotSpecs.LENGTH} {RobotSpecs.WIDTH} {RobotSpecs.HEIGHT}'

        material_robot = ET.SubElement(visual_robot, 'material')
        ET.SubElement(material_robot, 'ambient').text = '0 0 0.8 1'
        ET.SubElement(material_robot, 'diffuse').text = '0 0 0.8 1'

        # RED front indicator (thin top-mounted marker, above LIDAR scan plane)
        visual_front = ET.SubElement(base_link, 'visual', name='front_indicator')
        ET.SubElement(visual_front, 'pose').text = f'{RobotSpecs.LENGTH / 2 - 0.02} 0 {RobotSpecs.HEIGHT + 0.003} 0 0 0'
        geom_front = ET.SubElement(visual_front, 'geometry')
        box_front = ET.SubElement(geom_front, 'box')
        ET.SubElement(box_front, 'size').text = '0.04 0.04 0.005'

        material_front = ET.SubElement(visual_front, 'material')
        ET.SubElement(material_front, 'ambient').text = '1 0 0 1'
        ET.SubElement(material_front, 'diffuse').text = '1 0 0 1'

        # Collision (same raise as visual)
        collision_robot = ET.SubElement(base_link, 'collision', name='collision')
        ET.SubElement(collision_robot, 'pose').text = f'0 0 {half_height} 0 0 0'
        geom_collision = ET.SubElement(collision_robot, 'geometry')
        box_collision = ET.SubElement(geom_collision, 'box')
        ET.SubElement(box_collision, 'size').text = f'{RobotSpecs.LENGTH} {RobotSpecs.WIDTH} {RobotSpecs.HEIGHT}'

        # Inertial
        inertial = ET.SubElement(base_link, 'inertial')
        ET.SubElement(inertial, 'mass').text = f'{RobotSpecs.CHASSIS_MASS}'
        inertia = ET.SubElement(inertial, 'inertia')
        ET.SubElement(inertia, 'ixx').text = f'{chassis_ixx:.6f}'
        ET.SubElement(inertia, 'iyy').text = f'{chassis_iyy:.6f}'
        ET.SubElement(inertia, 'izz').text = f'{chassis_izz:.6f}'

        # ── Ackermann steering plugin ────────────────────────────────────
        plugin = ET.SubElement(robot_model, 'plugin',
                              filename='gz-sim-ackermann-steering-system',
                              name='gz::sim::systems::AckermannSteering')
        ET.SubElement(plugin, 'left_joint').text = 'rear_left_wheel_joint'
        ET.SubElement(plugin, 'right_joint').text = 'rear_right_wheel_joint'
        ET.SubElement(plugin, 'left_steering_joint').text = 'front_left_steering_joint'
        ET.SubElement(plugin, 'right_steering_joint').text = 'front_right_steering_joint'
        ET.SubElement(plugin, 'wheel_separation').text = f'{RobotSpecs.TRACK_WIDTH}'
        ET.SubElement(plugin, 'kingpin_width').text = f'{RobotSpecs.TRACK_WIDTH}'
        ET.SubElement(plugin, 'wheel_base').text = f'{RobotSpecs.WHEELBASE}'
        ET.SubElement(plugin, 'wheel_radius').text = f'{RobotSpecs.WHEEL_RADIUS}'
        ET.SubElement(plugin, 'min_steering_angle').text = f'-{RobotSpecs.MAX_STEERING_ANGLE}'
        ET.SubElement(plugin, 'max_steering_angle').text = f'{RobotSpecs.MAX_STEERING_ANGLE}'
        ET.SubElement(plugin, 'topic').text = '/wro_robot/cmd_vel'
        ET.SubElement(plugin, 'odom_topic').text = '/wro_robot/odom'
        ET.SubElement(plugin, 'odom_publish_frequency').text = '50'
        ET.SubElement(plugin, 'frame_id').text = 'odom'
        ET.SubElement(plugin, 'child_frame_id').text = 'base_link'

        # ── Helper: build a wheel link ───────────────────────────────────
        def _add_wheel_link(parent, name, pose_text, mu1='1.0', mu2='1.0'):
            """Create a wheel link with visual, collision, and inertia."""
            link = ET.SubElement(parent, 'link', name=name)
            ET.SubElement(link, 'pose', relative_to='base_link').text = pose_text

            # Main visual: dark grey cylinder (rotated so axis is along Y)
            vis = ET.SubElement(link, 'visual', name='visual')
            ET.SubElement(vis, 'pose').text = '0 0 0 1.5708 0 0'
            g = ET.SubElement(vis, 'geometry')
            c = ET.SubElement(g, 'cylinder')
            ET.SubElement(c, 'radius').text = f'{wheel_r}'
            ET.SubElement(c, 'length').text = f'{wheel_w}'
            mat = ET.SubElement(vis, 'material')
            ET.SubElement(mat, 'ambient').text = '0.1 0.1 0.1 1'
            ET.SubElement(mat, 'diffuse').text = '0.1 0.1 0.1 1'

            # Yellow stripe for rotation visibility
            sv = ET.SubElement(link, 'visual', name='stripe')
            ET.SubElement(sv, 'pose').text = f'{stripe_offset} 0 0 1.5708 0 0'
            sg = ET.SubElement(sv, 'geometry')
            sb = ET.SubElement(sg, 'box')
            ET.SubElement(sb, 'size').text = f'{stripe_x} {stripe_y} {stripe_z}'
            sm = ET.SubElement(sv, 'material')
            ET.SubElement(sm, 'ambient').text = '1.0 1.0 0.0 1'
            ET.SubElement(sm, 'diffuse').text = '1.0 1.0 0.0 1'

            # Collision cylinder (rotated to match visual)
            col = ET.SubElement(link, 'collision', name='collision')
            ET.SubElement(col, 'pose').text = '0 0 0 1.5708 0 0'
            cg = ET.SubElement(col, 'geometry')
            cc = ET.SubElement(cg, 'cylinder')
            ET.SubElement(cc, 'radius').text = f'{wheel_r}'
            ET.SubElement(cc, 'length').text = f'{wheel_w}'

            # Surface friction and contact
            surface = ET.SubElement(col, 'surface')
            friction = ET.SubElement(surface, 'friction')
            ode = ET.SubElement(friction, 'ode')
            ET.SubElement(ode, 'mu').text = mu1
            ET.SubElement(ode, 'mu2').text = mu2
            contact = ET.SubElement(surface, 'contact')
            ode_c = ET.SubElement(contact, 'ode')
            ET.SubElement(ode_c, 'kp').text = '1e7'
            ET.SubElement(ode_c, 'kd').text = '500'
            ET.SubElement(ode_c, 'max_vel').text = '0.01'
            ET.SubElement(ode_c, 'min_depth').text = '0.001'

            # Inertial
            iner = ET.SubElement(link, 'inertial')
            ET.SubElement(iner, 'mass').text = f'{RobotSpecs.WHEEL_MASS}'
            ix = ET.SubElement(iner, 'inertia')
            ET.SubElement(ix, 'ixx').text = f'{wheel_ixx:.8f}'
            ET.SubElement(ix, 'iyy').text = f'{wheel_iyy:.8f}'
            ET.SubElement(ix, 'izz').text = f'{wheel_izz:.8f}'
            return link

        # ── Rear wheels (powered) ────────────────────────────────────────
        _add_wheel_link(robot_model, 'rear_left_wheel',
                        f'{-half_wheelbase} {half_track} 0 0 0 0')
        _add_wheel_link(robot_model, 'rear_right_wheel',
                        f'{-half_wheelbase} {-half_track} 0 0 0 0')

        # Rear wheel joints (revolute, unlimited rotation)
        for jname, child in [('rear_left_wheel_joint', 'rear_left_wheel'),
                             ('rear_right_wheel_joint', 'rear_right_wheel')]:
            jt = ET.SubElement(robot_model, 'joint', name=jname, type='revolute')
            ET.SubElement(jt, 'parent').text = 'base_link'
            ET.SubElement(jt, 'child').text = child
            ax = ET.SubElement(jt, 'axis')
            ET.SubElement(ax, 'xyz').text = '0 1 0'
            lim = ET.SubElement(ax, 'limit')
            ET.SubElement(lim, 'lower').text = '-1e16'
            ET.SubElement(lim, 'upper').text = '1e16'
            ET.SubElement(lim, 'effort').text = '10.0'
            ET.SubElement(lim, 'velocity').text = '100.0'
            dyn = ET.SubElement(ax, 'dynamics')
            ET.SubElement(dyn, 'friction').text = '0.01'
            ET.SubElement(dyn, 'damping').text = '0.01'

        # ── Front steering assembly ──────────────────────────────────────
        # Each front corner: steering_link (Z-axis hinge) → wheel_link (Y-axis roll)
        for side, y_sign in [('left', 1), ('right', -1)]:
            y_pos = y_sign * half_track

            # Steering link (minimal inertia hinge)
            steer_name = f'front_{side}_steering'
            steer_link = ET.SubElement(robot_model, 'link', name=steer_name)
            ET.SubElement(steer_link, 'pose', relative_to='base_link').text = \
                f'{half_wheelbase} {y_pos} 0 0 0 0'
            si = ET.SubElement(steer_link, 'inertial')
            ET.SubElement(si, 'mass').text = '0.001'
            six = ET.SubElement(si, 'inertia')
            ET.SubElement(six, 'ixx').text = '0.00001'
            ET.SubElement(six, 'iyy').text = '0.00001'
            ET.SubElement(six, 'izz').text = '0.00001'

            # Front wheel (attached to steering link)
            wheel_name = f'front_{side}_wheel'
            fw_link = ET.SubElement(robot_model, 'link', name=wheel_name)
            ET.SubElement(fw_link, 'pose', relative_to=steer_name).text = '0 0 0 0 0 0'

            # Wheel visual
            vis = ET.SubElement(fw_link, 'visual', name='visual')
            ET.SubElement(vis, 'pose').text = '0 0 0 1.5708 0 0'
            g = ET.SubElement(vis, 'geometry')
            c = ET.SubElement(g, 'cylinder')
            ET.SubElement(c, 'radius').text = f'{wheel_r}'
            ET.SubElement(c, 'length').text = f'{wheel_w}'
            mat = ET.SubElement(vis, 'material')
            ET.SubElement(mat, 'ambient').text = '0.1 0.1 0.1 1'
            ET.SubElement(mat, 'diffuse').text = '0.1 0.1 0.1 1'

            # Yellow stripe
            sv = ET.SubElement(fw_link, 'visual', name='stripe')
            ET.SubElement(sv, 'pose').text = f'{stripe_offset} 0 0 1.5708 0 0'
            sg = ET.SubElement(sv, 'geometry')
            sb = ET.SubElement(sg, 'box')
            ET.SubElement(sb, 'size').text = f'{stripe_x} {stripe_y} {stripe_z}'
            sm = ET.SubElement(sv, 'material')
            ET.SubElement(sm, 'ambient').text = '1.0 1.0 0.0 1'
            ET.SubElement(sm, 'diffuse').text = '1.0 1.0 0.0 1'

            # Collision
            col = ET.SubElement(fw_link, 'collision', name='collision')
            ET.SubElement(col, 'pose').text = '0 0 0 1.5708 0 0'
            cg = ET.SubElement(col, 'geometry')
            cc = ET.SubElement(cg, 'cylinder')
            ET.SubElement(cc, 'radius').text = f'{wheel_r}'
            ET.SubElement(cc, 'length').text = f'{wheel_w}'
            surface = ET.SubElement(col, 'surface')
            friction = ET.SubElement(surface, 'friction')
            ode = ET.SubElement(friction, 'ode')
            ET.SubElement(ode, 'mu').text = '1.0'
            ET.SubElement(ode, 'mu2').text = '1.0'

            # Inertial
            iner = ET.SubElement(fw_link, 'inertial')
            ET.SubElement(iner, 'mass').text = f'{RobotSpecs.WHEEL_MASS}'
            ix = ET.SubElement(iner, 'inertia')
            ET.SubElement(ix, 'ixx').text = f'{wheel_ixx:.8f}'
            ET.SubElement(ix, 'iyy').text = f'{wheel_iyy:.8f}'
            ET.SubElement(ix, 'izz').text = f'{wheel_izz:.8f}'

            # Steering joint (base_link → steering_link, Z-axis)
            sj = ET.SubElement(robot_model, 'joint',
                               name=f'front_{side}_steering_joint', type='revolute')
            ET.SubElement(sj, 'parent').text = 'base_link'
            ET.SubElement(sj, 'child').text = steer_name
            sa = ET.SubElement(sj, 'axis')
            ET.SubElement(sa, 'xyz').text = '0 0 1'
            sl = ET.SubElement(sa, 'limit')
            ET.SubElement(sl, 'lower').text = f'-{RobotSpecs.MAX_STEERING_ANGLE}'
            ET.SubElement(sl, 'upper').text = f'{RobotSpecs.MAX_STEERING_ANGLE}'
            ET.SubElement(sl, 'effort').text = '5.0'
            ET.SubElement(sl, 'velocity').text = '5.0'
            sd = ET.SubElement(sa, 'dynamics')
            ET.SubElement(sd, 'damping').text = '0.1'
            ET.SubElement(sd, 'friction').text = '0.1'

            # Wheel rolling joint (steering_link → wheel, Y-axis)
            wj = ET.SubElement(robot_model, 'joint',
                               name=f'front_{side}_wheel_joint', type='revolute')
            ET.SubElement(wj, 'parent').text = steer_name
            ET.SubElement(wj, 'child').text = wheel_name
            wa = ET.SubElement(wj, 'axis')
            ET.SubElement(wa, 'xyz').text = '0 1 0'
            wl = ET.SubElement(wa, 'limit')
            ET.SubElement(wl, 'lower').text = '-1e16'
            ET.SubElement(wl, 'upper').text = '1e16'
            ET.SubElement(wl, 'effort').text = '10.0'
            ET.SubElement(wl, 'velocity').text = '100.0'
            wd = ET.SubElement(wa, 'dynamics')
            ET.SubElement(wd, 'damping').text = '0.01'
            ET.SubElement(wd, 'friction').text = '0.01'

        # ── Camera link ──────────────────────────────────────────────────
        camera_link = ET.SubElement(robot_model, 'link', name='camera_link')
        # Position matches URDF: front of chassis, slightly above center
        cam_x = RobotSpecs.LENGTH / 2 - 0.02   # 0.12
        cam_z = half_height + 0.01               # 0.06
        ET.SubElement(camera_link, 'pose', relative_to='base_link').text = \
            f'{cam_x} 0 {cam_z} 0 0.2 0'
        # Explicit lightweight inertial (prevents gz-sim auto-calculating heavy default)
        cam_inertial = ET.SubElement(camera_link, 'inertial')
        ET.SubElement(cam_inertial, 'mass').text = '0.005'
        cam_ix = ET.SubElement(cam_inertial, 'inertia')
        ET.SubElement(cam_ix, 'ixx').text = '0.00001'
        ET.SubElement(cam_ix, 'iyy').text = '0.00001'
        ET.SubElement(cam_ix, 'izz').text = '0.00001'

        camera_sensor = ET.SubElement(camera_link, 'sensor', name='robot_camera', type='camera')
        ET.SubElement(camera_sensor, 'update_rate').text = f'{RobotSpecs.CAMERA_UPDATE_RATE}'
        ET.SubElement(camera_sensor, 'visualize').text = 'false'
        ET.SubElement(camera_sensor, 'topic').text = 'robot/camera'
        ET.SubElement(camera_sensor, 'always_on').text = 'true'

        camera_elem = ET.SubElement(camera_sensor, 'camera')
        ET.SubElement(camera_elem, 'horizontal_fov').text = f'{RobotSpecs.CAMERA_HFOV}'

        image_elem = ET.SubElement(camera_elem, 'image')
        ET.SubElement(image_elem, 'width').text = f'{RobotSpecs.CAMERA_WIDTH}'
        ET.SubElement(image_elem, 'height').text = f'{RobotSpecs.CAMERA_HEIGHT}'
        ET.SubElement(image_elem, 'format').text = 'R8G8B8'

        clip_elem = ET.SubElement(camera_elem, 'clip')
        ET.SubElement(clip_elem, 'near').text = f'{RobotSpecs.CAMERA_NEAR_CLIP}'
        ET.SubElement(clip_elem, 'far').text = f'{RobotSpecs.CAMERA_FAR_CLIP}'

        camera_joint = ET.SubElement(robot_model, 'joint', name='camera_joint', type='fixed')
        ET.SubElement(camera_joint, 'parent').text = 'base_link'
        ET.SubElement(camera_joint, 'child').text = 'camera_link'

        # ── LIDAR link (Slamtec C1) ──────────────────────────────────────
        # IMPORTANT: gpu_lidar renders ALL visuals, including the robot's own body.
        # Position OUTSIDE the chassis to avoid self-detection.
        # Front-mounted, just beyond chassis edge, low for wall detection.
        lidar_link = ET.SubElement(robot_model, 'link', name='lidar_link')
        lidar_x = RobotSpecs.LENGTH / 2 + 0.03  # 30mm beyond front edge
        # Must be above chassis visual (top at z=HEIGHT=0.10) but below wall height
        # (walls are 0.1m tall from ground, LIDAR ground height = WHEEL_RADIUS + lidar_z)
        # Wall top = 0.1m, LIDAR ground height = 0.0216 + lidar_z
        # Need: 0.0216 + lidar_z < 0.1 → lidar_z < 0.0784
        # Also need: lidar_z > HEIGHT (0.10) to clear chassis... impossible!
        # Solution: keep LIDAR inside chassis height range but offset far enough forward
        # that backward rays miss. At x=0.17 (30mm beyond front), backward rays need
        # to travel ~0.03m in X before hitting chassis face at x=0.14.
        # Rays at ±90° don't intersect. Rays at ~170° hit at ~0.03/cos(10°)≈0.031m < min_range.
        # This is acceptable — only near-backward rays read inf, not side rays.
        lidar_z = 0.06  # 60mm above base_link, below wall top (0.1 - 0.0216 = 0.0784m max)
        ET.SubElement(lidar_link, 'pose', relative_to='base_link').text = \
            f'{lidar_x} 0 {lidar_z} 0 0 0'

        # Explicit lightweight inertial (prevents gz-sim auto-calculating heavy default)
        lidar_inertial = ET.SubElement(lidar_link, 'inertial')
        ET.SubElement(lidar_inertial, 'mass').text = '0.110'
        lidar_ix = ET.SubElement(lidar_inertial, 'inertia')
        ET.SubElement(lidar_ix, 'ixx').text = '0.00005'
        ET.SubElement(lidar_ix, 'iyy').text = '0.00005'
        ET.SubElement(lidar_ix, 'izz').text = '0.00004'

        # NOTE: No visual on lidar_link — gpu_lidar renders ALL visuals (including
        # same-link), and a surrounding cylinder causes self-detection → inf readings.

        lidar_sensor = ET.SubElement(lidar_link, 'sensor', name='lidar', type='gpu_lidar')
        ET.SubElement(lidar_sensor, 'update_rate').text = f'{RobotSpecs.LIDAR_UPDATE_RATE}'
        ET.SubElement(lidar_sensor, 'visualize').text = 'true'
        ET.SubElement(lidar_sensor, 'topic').text = 'lidar'
        ET.SubElement(lidar_sensor, 'always_on').text = 'true'

        lidar_elem = ET.SubElement(lidar_sensor, 'lidar')
        scan_elem = ET.SubElement(lidar_elem, 'scan')
        horizontal = ET.SubElement(scan_elem, 'horizontal')
        ET.SubElement(horizontal, 'samples').text = f'{RobotSpecs.LIDAR_SAMPLES}'
        ET.SubElement(horizontal, 'resolution').text = '1.0'
        ET.SubElement(horizontal, 'min_angle').text = '-3.14159'
        ET.SubElement(horizontal, 'max_angle').text = '3.14159'

        range_elem = ET.SubElement(lidar_elem, 'range')
        ET.SubElement(range_elem, 'min').text = f'{RobotSpecs.LIDAR_SIM_MIN_RANGE}'
        ET.SubElement(range_elem, 'max').text = f'{RobotSpecs.LIDAR_MAX_RANGE}'
        ET.SubElement(range_elem, 'resolution').text = '0.01'

        noise_elem = ET.SubElement(lidar_elem, 'noise')
        ET.SubElement(noise_elem, 'type').text = 'gaussian'
        ET.SubElement(noise_elem, 'mean').text = '0.0'
        ET.SubElement(noise_elem, 'stddev').text = f'{RobotSpecs.LIDAR_NOISE_STDDEV}'

        lidar_joint = ET.SubElement(robot_model, 'joint', name='lidar_joint', type='fixed')
        ET.SubElement(lidar_joint, 'parent').text = 'base_link'
        ET.SubElement(lidar_joint, 'child').text = 'lidar_link'

        # ── IMU link (Adafruit BNO085) ────────────────────────────────────
        imu_link = ET.SubElement(robot_model, 'link', name='imu_link')
        ET.SubElement(imu_link, 'pose', relative_to='base_link').text = '0 0 0.01 0 0 0'

        # Inertial
        imu_inertial = ET.SubElement(imu_link, 'inertial')
        ET.SubElement(imu_inertial, 'mass').text = f'{RobotSpecs.IMU_MASS}'
        imu_ix = ET.SubElement(imu_inertial, 'inertia')
        for tag in ['ixx', 'iyy', 'izz']:
            ET.SubElement(imu_ix, tag).text = '0.00001'
        for tag in ['ixy', 'ixz', 'iyz']:
            ET.SubElement(imu_ix, tag).text = '0'

        # Visual (small green PCB)
        imu_vis = ET.SubElement(imu_link, 'visual', name='visual')
        imu_geom = ET.SubElement(imu_vis, 'geometry')
        imu_box = ET.SubElement(imu_geom, 'box')
        ET.SubElement(imu_box, 'size').text = \
            f'{RobotSpecs.IMU_SIZE[0]} {RobotSpecs.IMU_SIZE[1]} {RobotSpecs.IMU_SIZE[2]}'
        imu_mat = ET.SubElement(imu_vis, 'material')
        ET.SubElement(imu_mat, 'ambient').text = '0.0 0.4 0.0 1'
        ET.SubElement(imu_mat, 'diffuse').text = '0.0 0.4 0.0 1'

        # IMU sensor
        imu_sensor = ET.SubElement(imu_link, 'sensor', name='imu', type='imu')
        ET.SubElement(imu_sensor, 'always_on').text = 'true'
        ET.SubElement(imu_sensor, 'update_rate').text = f'{RobotSpecs.IMU_UPDATE_RATE}'
        ET.SubElement(imu_sensor, 'topic').text = 'imu'

        imu_elem = ET.SubElement(imu_sensor, 'imu')
        angular_vel = ET.SubElement(imu_elem, 'angular_velocity')
        for axis in ['x', 'y', 'z']:
            ax = ET.SubElement(angular_vel, axis)
            noise = ET.SubElement(ax, 'noise', type='gaussian')
            ET.SubElement(noise, 'mean').text = '0.0'
            ET.SubElement(noise, 'stddev').text = f'{RobotSpecs.IMU_GYRO_NOISE}'

        linear_accel = ET.SubElement(imu_elem, 'linear_acceleration')
        for axis in ['x', 'y', 'z']:
            ax = ET.SubElement(linear_accel, axis)
            noise = ET.SubElement(ax, 'noise', type='gaussian')
            ET.SubElement(noise, 'mean').text = '0.0'
            ET.SubElement(noise, 'stddev').text = f'{RobotSpecs.IMU_ACCEL_NOISE}'

        # Fixed joint
        imu_joint = ET.SubElement(robot_model, 'joint', name='imu_joint', type='fixed')
        ET.SubElement(imu_joint, 'parent').text = 'base_link'
        ET.SubElement(imu_joint, 'child').text = 'imu_link'

        world.append(robot_model)

        # DEBUG: Add static overhead camera
        debug_camera = ET.Element('model', name='debug_camera')
        ET.SubElement(debug_camera, 'static').text = 'true'
        ET.SubElement(debug_camera, 'pose').text = '1.5 1.5 2.5 0 0 0'  # Center of track, 2.5m high, looking down

        debug_cam_link = ET.SubElement(debug_camera, 'link', name='link')

        debug_sensor = ET.SubElement(debug_cam_link, 'sensor', name='camera', type='camera')
        ET.SubElement(debug_sensor, 'update_rate').text = '30'
        ET.SubElement(debug_sensor, 'visualize').text = 'true'  # Show in GUI
        ET.SubElement(debug_sensor, 'topic').text = 'camera/image_raw'
        ET.SubElement(debug_sensor, 'always_on').text = 'true'

        debug_cam_elem = ET.SubElement(debug_sensor, 'camera')
        ET.SubElement(debug_cam_elem, 'horizontal_fov').text = '1.57'  # 90 degrees

        debug_image = ET.SubElement(debug_cam_elem, 'image')
        ET.SubElement(debug_image, 'width').text = '1280'
        ET.SubElement(debug_image, 'height').text = '720'
        ET.SubElement(debug_image, 'format').text = 'R8G8B8'

        debug_clip = ET.SubElement(debug_cam_elem, 'clip')
        ET.SubElement(debug_clip, 'near').text = '0.1'
        ET.SubElement(debug_clip, 'far').text = '10.0'

        world.append(debug_camera)

        # Save world file
        world_file = self.output_dir / f'{FilePaths.SCENARIO_PREFIX}{scenario_id:04d}{FileExtensions.SDF}'
        tree.write(world_file, encoding='utf-8', xml_declaration=True)

        # Save metadata
        metadata = {
            DictKeys.SCENARIO_ID: scenario_id,
            DictKeys.CHALLENGE_TYPE: self.challenge_type,
            DictKeys.WORLD_FILE: str(world_file),
            DictKeys.CORRIDOR_WIDTHS: {
                str(section): {
                    DictKeys.TYPE: corridor_widths[section][DictKeys.TYPE],
                    DictKeys.WIDTH_MM: int(corridor_widths[section][DictKeys.WIDTH] * 1000)
                }
                for section in self.sections
            } if corridor_widths else None,
            DictKeys.STARTING_CONDITIONS: {
                DictKeys.DIRECTION: str(starting_conditions[DictKeys.DIRECTION]),
                DictKeys.SECTION: starting_conditions[DictKeys.SECTION_NAME],
                DictKeys.POSITION: {DictKeys.X: starting_conditions[DictKeys.POSITION][0], DictKeys.Y: starting_conditions[DictKeys.POSITION][1]},
                DictKeys.YAW: starting_conditions[DictKeys.YAW]
            },
            DictKeys.NUM_SIGNS: len(sign_positions),
            DictKeys.HAS_PARKING_LOT: self.challenge_type == ScenarioTypes.OBSTACLES,
            DictKeys.SIGN_POSITIONS: [{DictKeys.X: x, DictKeys.Y: y, DictKeys.COLOR: color}
                              for (x, y), (color, _) in zip(sign_positions, sign_colors)],
            DictKeys.PARKING_LOT: {
                DictKeys.BLOCK1_POSITION: {DictKeys.X: parking_config[DictKeys.BLOCK1_POS][0], DictKeys.Y: parking_config[DictKeys.BLOCK1_POS][1]},
                DictKeys.BLOCK2_POSITION: {DictKeys.X: parking_config[DictKeys.BLOCK2_POS][0], DictKeys.Y: parking_config[DictKeys.BLOCK2_POS][1]},
                DictKeys.DEPTH: parking_config[DictKeys.DEPTH]
            } if parking_config else None
        }

        metadata_file = self.output_dir / f'{FilePaths.SCENARIO_PREFIX}{scenario_id:04d}{FilePaths.METADATA_SUFFIX}'
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)

        return world_file, metadata


class VideoRecorder:
    """Records camera feed from Gazebo simulation"""

    def __init__(self, output_dir, scenario_id):
        self.output_dir = Path(output_dir)
        self.scenario_id = scenario_id
        self.bridge = CvBridge() if ROS2_AVAILABLE else None
        self.frames = []
        self.recording = False

    def image_callback(self, msg):
        """ROS2 callback for camera images"""
        if self.recording and self.bridge:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frames.append(cv_image)

    def start_recording(self):
        """Start recording frames"""
        self.recording = True
        self.frames = []
        print(f"Started recording scenario {self.scenario_id}")

    def stop_recording(self):
        """Stop recording and save video"""
        self.recording = False

        if len(self.frames) == 0:
            print(f"No frames recorded for scenario {self.scenario_id}")
            return None

        # Save as video
        video_file = self.output_dir / f'{FilePaths.SCENARIO_PREFIX}{self.scenario_id:04d}_video{FileExtensions.MP4}'

        height, width = self.frames[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(str(video_file), fourcc, 30.0, (width, height))

        for frame in self.frames:
            out.write(frame)

        out.release()

        print(f"Saved video: {video_file} ({len(self.frames)} frames)")

        # Also save sample frames for dataset
        frames_dir = self.output_dir / FolderNames.FRAMES / f'{FilePaths.SCENARIO_PREFIX}{self.scenario_id:04d}'
        frames_dir.mkdir(parents=True, exist_ok=True)

        # Save every 10th frame
        for i, frame in enumerate(self.frames[::10]):
            frame_file = frames_dir / f'frame_{i:04d}{FileExtensions.JPG}'
            cv2.imwrite(str(frame_file), frame)

        return video_file


def main():
    parser = argparse.ArgumentParser(description='Generate WRO training data in Gazebo')
    parser.add_argument('--challenge', type=str, default='open',
                       choices=['open', 'obstacles'],
                       help='Challenge type: open or obstacles')
    parser.add_argument('--num-scenarios', type=int, default=10,
                       help='Number of scenarios to generate')
    parser.add_argument('--output-dir', type=str,
                       default='./training_data',
                       help='Output directory for generated data')
    parser.add_argument('--base-world', type=str,
                       default='../worlds/wro_track_2026.sdf',
                       help='Base world SDF file (default: wro_track_2026.sdf)')
    parser.add_argument('--randomize-all', action='store_true',
                       help='Enable full randomization (lighting, colors, physics)')
    parser.add_argument('--duration', type=int, default=30,
                       help='Recording duration per scenario (seconds)')

    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Initialize generator (separate folders per challenge type)
    challenge_output_dir = output_dir / args.challenge / FolderNames.SCENARIOS
    generator = ScenarioGenerator(
        base_world_path=args.base_world,
        output_dir=challenge_output_dir,
        challenge_type=args.challenge
    )

    print(f"Generating {args.num_scenarios} scenarios for '{args.challenge}' challenge")
    print(f"Output directory: {challenge_output_dir}")
    print(f"Randomization: {'FULL' if args.randomize_all else 'BASIC'}")
    print("-" * 60)

    # Generate scenarios
    for scenario_id in range(args.num_scenarios):
        print(f"\nGenerating scenario {scenario_id + 1}/{args.num_scenarios}...")

        world_file, metadata = generator.create_scenario_world(
            scenario_id,
            randomize_all=args.randomize_all
        )

        print(f"  World file: {world_file}")
        print(f"  Traffic Signs: {metadata[DictKeys.NUM_SIGNS]}")
        print(f"  Starting: {metadata[DictKeys.STARTING_CONDITIONS][DictKeys.SECTION]} section, {metadata[DictKeys.STARTING_CONDITIONS][DictKeys.DIRECTION]}")
        print(f"  Parking Lot: {'Yes' if metadata[DictKeys.HAS_PARKING_LOT] else 'No'}")

    print("\n" + "=" * 60)
    print("Scenario generation complete!")
    print(f"Challenge type: {args.challenge}")
    print(f"Total scenarios: {args.num_scenarios}")
    print(f"Output directory: {challenge_output_dir}")
    print("\nTo launch scenarios in Gazebo, use:")
    print(f"  gz sim {challenge_output_dir}/scenario_0000.sdf")
    print("\nTo record videos with ROS2:")
    print(f"  ros2 launch wro_simulation record_training_data.launch.py")


if __name__ == '__main__':
    main()
