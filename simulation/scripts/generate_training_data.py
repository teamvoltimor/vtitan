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

        # Track dimensions (3m x 3m inner space)
        self.track_bounds = {
            'x_min': -1.4,
            'x_max': 1.4,
            'y_min': -1.4,
            'y_max': 1.4,
            'z_pillar': 0.15,  # Half height of pillar
            'z_obstacle': 0.05
        }

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

    def generate_sign_positions(self, num_signs=8):
        """Generate random positions for traffic signs (WRO Spec 13.20: up to 7 red + 7 green)"""
        positions = []
        min_distance = 0.3  # Minimum distance between signs

        # Define zones to avoid overcrowding start area
        start_zone = (-1.4, -0.8, -1.4, -0.9)  # x_min, x_max, y_min, y_max

        attempts = 0
        max_attempts = 1000

        while len(positions) < num_signs and attempts < max_attempts:
            x = random.uniform(self.track_bounds['x_min'], self.track_bounds['x_max'])
            y = random.uniform(self.track_bounds['y_min'], self.track_bounds['y_max'])

            # Skip start zone
            if (start_zone[0] <= x <= start_zone[1] and
                start_zone[2] <= y <= start_zone[3]):
                attempts += 1
                continue

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

        # Randomize lighting
        if randomize_all:
            lighting = self.randomize_lighting()

            # Update sun light
            sun = world.find(".//light[@name='sun']")
            if sun is not None:
                diffuse = sun.find('diffuse')
                intensity = lighting['intensity']
                diffuse.text = f"{intensity} {intensity} {intensity} 1"

                direction = sun.find('direction')
                direction.text = f"{lighting['direction'][0]} {lighting['direction'][1]} {lighting['direction'][2]}"

            # Update ambient light
            ambient = world.find(".//light[@name='ambient_light']")
            if ambient is not None:
                diffuse = ambient.find('diffuse')
                amb_intensity = lighting['ambient_intensity']
                diffuse.text = f"{amb_intensity} {amb_intensity} {amb_intensity} 1"

        # Generate traffic sign positions and colors (WRO Spec 13.19-13.22)
        # Up to 7 red and up to 7 green per round
        num_signs = random.randint(6, 14) if randomize_all else 8  # Total up to 14 (7 red + 7 green)
        sign_positions = self.generate_sign_positions(num_signs)

        sign_colors = []
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

        # Add obstacles if obstacles challenge
        # WRO: Obstacles are RED or GREEN blocks (same colors as signs)
        obstacle_positions = []
        obstacle_colors = []
        if self.challenge_type == 'obstacles':
            num_obstacles = random.randint(3, 6) if randomize_all else 4
            obstacle_positions = self.generate_obstacle_positions(num_obstacles, sign_positions)

            for i, (x, y) in enumerate(obstacle_positions):
                # WRO: Obstacles can be red or green
                obstacle_color_name = 'red' if random.random() < 0.5 else 'green'

                if randomize_all:
                    obstacle_color_rgb = self.randomize_color(obstacle_color_name)
                else:
                    obstacle_color_rgb = self.randomization['colors'][obstacle_color_name]['mean']

                obstacle_colors.append((obstacle_color_name, obstacle_color_rgb))

                obstacle_model = ET.Element('model', name=f'{obstacle_color_name}_obstacle_{i}')
                ET.SubElement(obstacle_model, 'static').text = 'false'
                ET.SubElement(obstacle_model, 'pose').text = f'{x} {y} {self.track_bounds["z_obstacle"]} 0 0 0'

                link = ET.SubElement(obstacle_model, 'link', name='link')

                # Visual
                visual = ET.SubElement(link, 'visual', name='visual')
                geom = ET.SubElement(visual, 'geometry')
                box = ET.SubElement(geom, 'box')
                ET.SubElement(box, 'size').text = '0.10 0.10 0.10'

                material = ET.SubElement(visual, 'material')
                ET.SubElement(material, 'ambient').text = f'{obstacle_color_rgb[0]} {obstacle_color_rgb[1]} {obstacle_color_rgb[2]} 1'
                ET.SubElement(material, 'diffuse').text = f'{obstacle_color_rgb[0]} {obstacle_color_rgb[1]} {obstacle_color_rgb[2]} 1'
                ET.SubElement(material, 'specular').text = '0.2 0.2 0.2 1'

                # Collision
                collision = ET.SubElement(link, 'collision', name='collision')
                geom = ET.SubElement(collision, 'geometry')
                box = ET.SubElement(geom, 'box')
                ET.SubElement(box, 'size').text = '0.10 0.10 0.10'

                # Inertial
                inertial = ET.SubElement(link, 'inertial')
                ET.SubElement(inertial, 'mass').text = '0.5'
                inertia = ET.SubElement(inertial, 'inertia')
                ET.SubElement(inertia, 'ixx').text = '0.001'
                ET.SubElement(inertia, 'iyy').text = '0.001'
                ET.SubElement(inertia, 'izz').text = '0.001'

                world.append(obstacle_model)

        # Save world file
        world_file = self.output_dir / f'scenario_{scenario_id:04d}.sdf'
        tree.write(world_file, encoding='utf-8', xml_declaration=True)

        # Save metadata
        metadata = {
            'scenario_id': scenario_id,
            'challenge_type': self.challenge_type,
            'world_file': str(world_file),
            'num_signs': len(sign_positions),
            'num_obstacles': len(obstacle_positions),
            'sign_positions': [{'x': x, 'y': y, 'color': color}
                              for (x, y), (color, _) in zip(sign_positions, sign_colors)],
            'obstacle_positions': [{'x': x, 'y': y, 'color': color}
                                  for (x, y), (color, _) in zip(obstacle_positions, obstacle_colors)] if self.challenge_type == 'obstacles' else []
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
                       default='../worlds/wro_track_base.sdf',
                       help='Base world SDF file')
    parser.add_argument('--randomize-all', action='store_true',
                       help='Enable full randomization (lighting, colors, physics)')
    parser.add_argument('--duration', type=int, default=30,
                       help='Recording duration per scenario (seconds)')

    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Initialize generator
    generator = ScenarioGenerator(
        base_world_path=args.base_world,
        output_dir=output_dir / 'scenarios',
        challenge_type=args.challenge
    )

    print(f"Generating {args.num_scenarios} scenarios for '{args.challenge}' challenge")
    print(f"Output directory: {output_dir}")
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
        print(f"  Pillars: {metadata['num_pillars']}")
        print(f"  Obstacles: {metadata['num_obstacles']}")

    print("\n" + "=" * 60)
    print("Scenario generation complete!")
    print(f"Total scenarios: {args.num_scenarios}")
    print(f"Output directory: {output_dir}")
    print("\nTo launch scenarios in Gazebo, use:")
    print(f"  gz sim {output_dir}/scenarios/scenario_0000.sdf")
    print("\nTo record videos with ROS2:")
    print(f"  ros2 launch wro_simulation record_training_data.launch.py")


if __name__ == '__main__':
    main()
