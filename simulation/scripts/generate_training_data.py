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
import sys
import time
import random
import argparse
import subprocess
import json
import yaml
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

        # WRO 2026 Track dimensions (3000mm x 3000mm inner track)
        # Coordinate system: (0,0) at bottom-left, (3.0, 3.0) at top-right
        self.track_bounds = {
            'min': 0.0,      # Minimum coordinate (bottom-left corner)
            'max': 3.0,      # Maximum coordinate (top-right corner)
            'center': 1.5,   # Center of track
            'z_sign': 0.05,  # Half height of traffic sign (50mm)
        }

        # Corridor width options for OPEN challenge (randomized per section)
        # WRO: Each section's corridor width is either 600mm or 1000mm (coin toss)
        self.corridor_widths = {
            'narrow': 0.6,   # 600mm corridor
            'wide': 1.0      # 1000mm corridor
        }

        # Track sections for starting position and corridor randomization
        self.sections = ['north', 'south', 'east', 'west']

        # Randomization parameters (WRO official colors - Spec 13.21-13.22)
        self.randomization = {
            'lighting': {
                'intensity_range': (0.5, 1.5),
                'direction_variance': 0.3
            },
            'colors': {
                # WRO Official Colors (Spec 13.21-13.22)
                # Red: RGB(238, 39, 55) = (0.933, 0.153, 0.216)
                # Green: RGB(68, 214, 44) = (0.267, 0.839, 0.173)
                'red': {'mean': [0.933, 0.153, 0.216], 'std': [0.05, 0.02, 0.02]},
                'green': {'mean': [0.267, 0.839, 0.173], 'std': [0.02, 0.05, 0.02]}
            },
            'physics': {
                'friction_range': (0.6, 1.2),
                'mass_variance': 0.1
            }
        }

    def generate_sign_positions(self, num_signs=8, corridor_widths=None):
        """Generate random positions for traffic signs in corridor sections"""
        positions = []
        min_distance = 0.3  # Minimum distance between signs

        # Generate positions in the four corridor sections
        attempts = 0
        max_attempts = 1000

        while len(positions) < num_signs and attempts < max_attempts:
            # Pick a random section
            section = random.choice(self.sections)

            if corridor_widths and section in corridor_widths:
                corridor_width = corridor_widths[section]['width']
            else:
                corridor_width = 0.7

            # Calculate corridor bounds for this section
            track_min = self.track_bounds['min']
            track_max = self.track_bounds['max']

            # Interior boundary is corridor_width from exterior
            # North: y from (3.0 - corridor_width) to 3.0
            # South: y from 0.0 to corridor_width
            # East: x from (3.0 - corridor_width) to 3.0
            # West: x from 0.0 to corridor_width

            # Generate position in corridor
            if section == 'north':
                x = random.uniform(0.3, 2.7)  # Along track width
                y = random.uniform(track_max - corridor_width + 0.1, track_max - 0.1)
            elif section == 'south':
                x = random.uniform(0.3, 2.7)  # Along track width
                y = random.uniform(track_min + 0.1, corridor_width - 0.1)
            elif section == 'east':
                x = random.uniform(track_max - corridor_width + 0.1, track_max - 0.1)
                y = random.uniform(0.3, 2.7)  # Along track height
            else:  # west
                x = random.uniform(track_min + 0.1, corridor_width - 0.1)
                y = random.uniform(0.3, 2.7)  # Along track height

            # Check distance from existing signs
            too_close = False
            for px, py in positions:
                if np.sqrt((x - px)**2 + (y - py)**2) < min_distance:
                    too_close = True
                    break

            if not too_close:
                positions.append((x, y))

            attempts += 1

        return positions

    def generate_obstacle_positions(self, num_obstacles=4, sign_positions=None):
        """Generate random positions for obstacles (obstacles challenge)"""
        positions = []
        min_distance_to_sign = 0.2
        min_distance_to_obstacle = 0.25

        if sign_positions is None:
            sign_positions = []

        attempts = 0
        max_attempts = 1000

        while len(positions) < num_obstacles and attempts < max_attempts:
            x = random.uniform(self.track_bounds['x_min'], self.track_bounds['x_max'])
            y = random.uniform(self.track_bounds['y_min'], self.track_bounds['y_max'])

            # Check distance from traffic signs
            too_close = False
            for px, py in sign_positions:
                if np.sqrt((x - px)**2 + (y - py)**2) < min_distance_to_sign:
                    too_close = True
                    break

            # Check distance from other obstacles
            for ox, oy in positions:
                if np.sqrt((x - ox)**2 + (y - oy)**2) < min_distance_to_obstacle:
                    too_close = True
                    break

            if not too_close:
                positions.append((x, y))

            attempts += 1

        return positions

    def randomize_lighting(self):
        """Generate randomized lighting parameters"""
        intensity = random.uniform(*self.randomization['lighting']['intensity_range'])

        # Randomize direction slightly
        direction = [
            -0.5 + random.uniform(-0.3, 0.3),
            -0.5 + random.uniform(-0.3, 0.3),
            -1.0
        ]

        return {
            'intensity': intensity,
            'direction': direction,
            'ambient_intensity': intensity * 0.5
        }

    def randomize_corridor_widths(self):
        """Randomize corridor width for each section (OPEN challenge only)"""
        # WRO: Each section has corridor width either 600mm or 1000mm (coin toss)
        widths = {}
        for section in self.sections:
            # Coin toss: narrow (600mm) or wide (1000mm)
            width_type = random.choice(['narrow', 'wide'])
            widths[section] = {
                'type': width_type,
                'width': self.corridor_widths[width_type]
            }
        return widths

    def randomize_starting_conditions(self, corridor_widths=None):
        """Generate WRO-style randomized starting conditions"""
        # Randomize direction (coin toss): clockwise or counterclockwise
        direction = random.choice(['clockwise', 'counterclockwise'])

        # Randomize starting section (one of four sides)
        starting_section = random.choice(self.sections)

        # Calculate starting position based on corridor width
        if corridor_widths and starting_section in corridor_widths:
            corridor_width = corridor_widths[starting_section]['width']
        else:
            corridor_width = 0.65  # Default center position

        track_min = self.track_bounds['min']
        track_max = self.track_bounds['max']
        track_center = self.track_bounds['center']

        # Calculate center of corridor for starting position
        if starting_section == 'north':
            y_pos = track_max - corridor_width / 2
            start_positions = [
                (track_center - 0.5, y_pos),  # Left
                (track_center, y_pos),        # Center
                (track_center + 0.5, y_pos)   # Right
            ]
        elif starting_section == 'south':
            y_pos = corridor_width / 2
            start_positions = [
                (track_center - 0.5, y_pos),
                (track_center, y_pos),
                (track_center + 0.5, y_pos)
            ]
        elif starting_section == 'east':
            x_pos = track_max - corridor_width / 2
            start_positions = [
                (x_pos, track_center - 0.5),
                (x_pos, track_center),
                (x_pos, track_center + 0.5)
            ]
        else:  # west
            x_pos = corridor_width / 2
            start_positions = [
                (x_pos, track_center - 0.5),
                (x_pos, track_center),
                (x_pos, track_center + 0.5)
            ]

        starting_position = random.choice(start_positions)

        # Starting orientation based on direction and section
        yaw_map = {
            'north': {'clockwise': -1.5708, 'counterclockwise': 1.5708},
            'south': {'clockwise': 1.5708, 'counterclockwise': -1.5708},
            'east': {'clockwise': 3.14159, 'counterclockwise': 0.0},
            'west': {'clockwise': 0.0, 'counterclockwise': 3.14159}
        }

        starting_yaw = yaw_map[starting_section][direction]

        return {
            'direction': direction,
            'section': starting_section,
            'section_name': starting_section.capitalize(),
            'position': starting_position,
            'yaw': starting_yaw
        }

    def randomize_color(self, color_name):
        """Generate randomized color with Gaussian noise"""
        params = self.randomization['colors'][color_name]
        color = np.random.normal(params['mean'], params['std'])
        color = np.clip(color, 0.0, 1.0)
        return color.tolist()

    def create_scenario_world(self, scenario_id, randomize_all=True):
        """Create a randomized world SDF file"""
        # Parse base world
        tree = ET.parse(self.base_world_path)
        root = tree.getroot()
        world = root.find('world')

        # Randomize corridor widths for OPEN challenge (interior walls)
        # OBSTACLES challenge has fixed corridor width
        corridor_widths = None
        if self.challenge_type == 'open' and randomize_all:
            corridor_widths = self.randomize_corridor_widths()
        elif self.challenge_type == 'open':
            # Default fixed widths for open challenge (800mm - middle value)
            corridor_widths = {section: {'type': 'default', 'width': 0.8} for section in self.sections}
        else:
            # Obstacles challenge: fixed 1.0m corridor (creates 1.0m × 1.0m inner area)
            corridor_widths = {section: {'type': 'fixed', 'width': 1.0} for section in self.sections}

        # Calculate interior wall positions for all sections
        # Each section's interior wall is offset from exterior based on corridor width
        track_min = self.track_bounds['min']
        track_max = self.track_bounds['max']

        # Interior wall positions:
        # North: y = track_max - corridor_width_north
        # South: y = corridor_width_south
        # East: x = track_max - corridor_width_east
        # West: x = corridor_width_west
        north_interior_y = track_max - corridor_widths['north']['width']
        south_interior_y = corridor_widths['south']['width']
        east_interior_x = track_max - corridor_widths['east']['width']
        west_interior_x = corridor_widths['west']['width']

        # North interior wall (spans from west to east, thickness extends inward)
        # Wall positioned so outer face is at north_interior_y, inner face extends inward
        wall_north = ET.Element('model', name='interior_wall_north')
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
        ET.SubElement(box_n_col, 'size').text = f"{north_length} 0.1 0.1"
        world.append(wall_north)

        # South interior wall (spans from west to east, thickness extends inward)
        # Wall positioned so outer face is at south_interior_y, inner face extends inward
        wall_south = ET.Element('model', name='interior_wall_south')
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
        ET.SubElement(box_s_col, 'size').text = f"{south_length} 0.1 0.1"
        world.append(wall_south)

        # East interior wall (spans from south to north, thickness extends inward)
        # Wall positioned so outer face is at east_interior_x, inner face extends inward
        wall_east = ET.Element('model', name='interior_wall_east')
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
        ET.SubElement(box_e_col, 'size').text = f"0.1 {east_length} 0.1"
        world.append(wall_east)

        # West interior wall (spans from south to north, thickness extends inward)
        # Wall positioned so outer face is at west_interior_x, inner face extends inward
        wall_west = ET.Element('model', name='interior_wall_west')
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
        ET.SubElement(box_w_col, 'size').text = f"0.1 {west_length} 0.1"
        world.append(wall_west)

        # Randomize starting conditions (WRO-style)
        starting_conditions = self.randomize_starting_conditions(corridor_widths) if randomize_all else {
            'direction': 'clockwise',
            'section': 'south',
            'section_name': 'South',
            'position': (0.0, -1.0),
            'yaw': 1.5708
        }

        # Randomize lighting
        if randomize_all:
            lighting = self.randomize_lighting()

            # Update sun light (clamp intensity to [0.0, 1.0] for valid SDF)
            sun = world.find(".//light[@name='sun']")
            if sun is not None:
                diffuse = sun.find('diffuse')
                intensity = min(1.0, max(0.0, lighting['intensity']))  # Clamp to [0.0, 1.0]
                diffuse.text = f"{intensity} {intensity} {intensity} 1"

                direction = sun.find('direction')
                direction.text = f"{lighting['direction'][0]} {lighting['direction'][1]} {lighting['direction'][2]}"

            # Update ambient light (clamp to valid range)
            ambient = world.find(".//light[@name='ambient_light']")
            if ambient is not None:
                diffuse = ambient.find('diffuse')
                amb_intensity = min(1.0, max(0.0, lighting['ambient_intensity']))  # Clamp to [0.0, 1.0]
                diffuse.text = f"{amb_intensity} {amb_intensity} {amb_intensity} 1"

        # WRO: Traffic signs ONLY in obstacles challenge (Spec 13.19-13.22)
        # Open challenge has NO traffic signs
        sign_positions = []
        sign_colors = []

        if self.challenge_type == 'obstacles':
            # Generate traffic signs for obstacles challenge only
            # Up to 7 red and 7 green per round (total 14 signs max)
            num_signs = random.randint(6, 14) if randomize_all else 8
            sign_positions = self.generate_sign_positions(num_signs)

            for i in range(num_signs):
                # WRO rules: Randomly assign RED or GREEN (up to 7 each per Spec 13.20)
                if random.random() < 0.5:
                    color_name = 'red'
                else:
                    color_name = 'green'

                if randomize_all:
                    color_rgb = self.randomize_color(color_name)
                else:
                    color_rgb = self.randomization['colors'][color_name]['mean']

                sign_colors.append((color_name, color_rgb))

        # Add traffic signs to world (rectangular boxes per Spec 13.19)
        for i, ((x, y), (color_name, color_rgb)) in enumerate(zip(sign_positions, sign_colors)):
            sign_model = ET.Element('model', name=f'{color_name}_sign_{i}')
            ET.SubElement(sign_model, 'static').text = 'true'
            ET.SubElement(sign_model, 'pose').text = f'{x} {y} 0.05 0 0 0'  # 50mm height (half of 100mm)

            link = ET.SubElement(sign_model, 'link', name='link')

            # Visual (Rectangular parallelepiped 50×50×100mm per Spec 13.19)
            visual = ET.SubElement(link, 'visual', name='visual')
            geom = ET.SubElement(visual, 'geometry')
            box = ET.SubElement(geom, 'box')
            ET.SubElement(box, 'size').text = '0.05 0.05 0.10'  # 50mm × 50mm × 100mm

            material = ET.SubElement(visual, 'material')
            ET.SubElement(material, 'ambient').text = f'{color_rgb[0]} {color_rgb[1]} {color_rgb[2]} 1'
            ET.SubElement(material, 'diffuse').text = f'{color_rgb[0]} {color_rgb[1]} {color_rgb[2]} 1'
            ET.SubElement(material, 'specular').text = '0.2 0.2 0.2 1'

            # Collision
            collision = ET.SubElement(link, 'collision', name='collision')
            geom = ET.SubElement(collision, 'geometry')
            box = ET.SubElement(geom, 'box')
            ET.SubElement(box, 'size').text = '0.05 0.05 0.10'

            world.append(sign_model)

        # NOTE: In WRO Future Engineers, there are NO separate obstacle objects!
        # Both "open" and "obstacles" challenges use the same traffic signs.
        # The difference is in the navigation rules, not the physical objects.
        # Parking limitations (magenta blocks) only appear in obstacles challenge.

        # Add parking limitations if obstacles challenge
        if self.challenge_type == 'obstacles':
            # WRO Spec 13.26: One parking lot with two parking lot limitations
            # Position parking lot in a corner (example: northeast corner)
            parking_x = 2.7
            parking_y = 2.7

            # Parking limitation 1 (horizontal)
            parking1 = ET.Element('model', name='parking_limitation_1')
            ET.SubElement(parking1, 'static').text = 'true'
            ET.SubElement(parking1, 'pose').text = f'{parking_x} {parking_y} 0.05 0 0 0'  # 50mm height

            link1 = ET.SubElement(parking1, 'link', name='link')
            visual1 = ET.SubElement(link1, 'visual', name='visual')
            geom1 = ET.SubElement(visual1, 'geometry')
            box1 = ET.SubElement(geom1, 'box')
            ET.SubElement(box1, 'size').text = '0.20 0.02 0.10'  # 200×20×100mm

            material1 = ET.SubElement(visual1, 'material')
            ET.SubElement(material1, 'ambient').text = '1.0 0.0 1.0 1'  # Magenta
            ET.SubElement(material1, 'diffuse').text = '1.0 0.0 1.0 1'

            collision1 = ET.SubElement(link1, 'collision', name='collision')
            geom1_col = ET.SubElement(collision1, 'geometry')
            box1_col = ET.SubElement(geom1_col, 'box')
            ET.SubElement(box1_col, 'size').text = '0.20 0.02 0.10'

            world.append(parking1)

            # Parking limitation 2 (perpendicular)
            parking2 = ET.Element('model', name='parking_limitation_2')
            ET.SubElement(parking2, 'static').text = 'true'
            ET.SubElement(parking2, 'pose').text = f'{parking_x - 0.11} {parking_y - 0.09} 0.05 0 0 1.5708'  # Rotated 90°

            link2 = ET.SubElement(parking2, 'link', name='link')
            visual2 = ET.SubElement(link2, 'visual', name='visual')
            geom2 = ET.SubElement(visual2, 'geometry')
            box2 = ET.SubElement(geom2, 'box')
            ET.SubElement(box2, 'size').text = '0.20 0.02 0.10'

            material2 = ET.SubElement(visual2, 'material')
            ET.SubElement(material2, 'ambient').text = '1.0 0.0 1.0 1'
            ET.SubElement(material2, 'diffuse').text = '1.0 0.0 1.0 1'

            collision2 = ET.SubElement(link2, 'collision', name='collision')
            geom2_col = ET.SubElement(collision2, 'geometry')
            box2_col = ET.SubElement(geom2_col, 'box')
            ET.SubElement(box2_col, 'size').text = '0.20 0.02 0.10'

            world.append(parking2)

        # Save world file
        world_file = self.output_dir / f'scenario_{scenario_id:04d}.sdf'
        tree.write(world_file, encoding='utf-8', xml_declaration=True)

        # Save metadata
        metadata = {
            'scenario_id': scenario_id,
            'challenge_type': self.challenge_type,
            'world_file': str(world_file),
            'corridor_widths': {
                section: {
                    'type': corridor_widths[section]['type'],
                    'width_mm': int(corridor_widths[section]['width'] * 1000)
                }
                for section in self.sections
            } if corridor_widths else None,
            'starting_conditions': {
                'direction': starting_conditions['direction'],
                'section': starting_conditions['section_name'],
                'position': {'x': starting_conditions['position'][0], 'y': starting_conditions['position'][1]},
                'yaw': starting_conditions['yaw']
            },
            'num_signs': len(sign_positions),
            'has_parking_lot': self.challenge_type == 'obstacles',
            'sign_positions': [{'x': x, 'y': y, 'color': color}
                              for (x, y), (color, _) in zip(sign_positions, sign_colors)]
        }

        metadata_file = self.output_dir / f'scenario_{scenario_id:04d}_metadata.json'
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
        video_file = self.output_dir / f'scenario_{self.scenario_id:04d}_video.mp4'

        height, width = self.frames[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(str(video_file), fourcc, 30.0, (width, height))

        for frame in self.frames:
            out.write(frame)

        out.release()

        print(f"Saved video: {video_file} ({len(self.frames)} frames)")

        # Also save sample frames for dataset
        frames_dir = self.output_dir / 'frames' / f'scenario_{self.scenario_id:04d}'
        frames_dir.mkdir(parents=True, exist_ok=True)

        # Save every 10th frame
        for i, frame in enumerate(self.frames[::10]):
            frame_file = frames_dir / f'frame_{i:04d}.jpg'
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
    challenge_output_dir = output_dir / args.challenge / 'scenarios'
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
        print(f"  Traffic Signs: {metadata['num_signs']}")
        print(f"  Starting: {metadata['starting_conditions']['section']} section, {metadata['starting_conditions']['direction']}")
        print(f"  Parking Lot: {'Yes' if metadata['has_parking_lot'] else 'No'}")

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
