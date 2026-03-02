#!/usr/bin/env python3
"""
Extract frames from ROS2 bags and generate YOLO annotations

Uses scenario metadata to create ground-truth bounding boxes for traffic signs.

Usage:
    python3 extract_frames_and_annotate.py \
        --bag-dir ~/wro_bags \
        --metadata-dir ~/wro_training_data/scenarios \
        --output-dir ~/wro_yolo_dataset
"""

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np

# ROS2 bag imports
try:
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from sensor_msgs.msg import Image
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("WARNING: ROS2 not available. Some features disabled.")


class YOLOAnnotator:
    """Generates YOLO format annotations from simulation metadata"""

    def __init__(self, image_width=640, image_height=480):
        self.image_width = image_width
        self.image_height = image_height

        # Class labels (WRO official: only red and green pillars)
        self.class_map = {
            "red": 0,
            "green": 1,
            "obstacle": 2,  # For obstacles challenge
        }

    def project_3d_to_2d(self, x, y, z, camera_params):
        """
        Project 3D world coordinates to 2D image coordinates

        Simplified projection assuming camera looking forward with slight tilt
        """
        # Camera parameters (from URDF)
        camera_x = camera_params["pose"]["x"]
        camera_y = camera_params["pose"]["y"]
        camera_z = camera_params["pose"]["z"]
        camera_pitch = camera_params["pose"]["pitch"]  # radians
        fov_horizontal = camera_params["fov"]  # radians (2.094 = 120 degrees)
        fov_vertical = fov_horizontal * (self.image_height / self.image_width)

        # Transform to camera frame
        dx = x - camera_x
        dy = y - camera_y
        dz = z - camera_z

        # Apply camera pitch rotation (simplified)
        distance = np.sqrt(dx**2 + dy**2)
        angle_horizontal = np.arctan2(dy, dx)
        angle_vertical = np.arctan2(dz - camera_z, distance) - camera_pitch

        # Check if in FOV
        if abs(angle_horizontal) > fov_horizontal / 2:
            return None, None  # Outside horizontal FOV

        if abs(angle_vertical) > fov_vertical / 2:
            return None, None  # Outside vertical FOV

        # Project to image coordinates (normalized)
        u_normalized = (angle_horizontal / fov_horizontal) + 0.5
        v_normalized = (angle_vertical / fov_vertical) + 0.5

        # Convert to pixel coordinates
        u = int(u_normalized * self.image_width)
        v = int(v_normalized * self.image_height)

        # Check bounds
        if 0 <= u < self.image_width and 0 <= v < self.image_height:
            return u, v
        return None, None

    def estimate_bounding_box(self, x, y, z, object_radius, camera_params):
        """
        Estimate 2D bounding box for a cylindrical object

        Returns YOLO format: [class_id, x_center, y_center, width, height]
        All values normalized to [0, 1]
        """
        # Project center point
        u_center, v_center = self.project_3d_to_2d(x, y, z, camera_params)

        if u_center is None:
            return None  # Not visible

        # Estimate apparent size based on distance
        distance = np.sqrt((x - camera_params["pose"]["x"])**2 +
                          (y - camera_params["pose"]["y"])**2)

        if distance < 0.1:
            return None  # Too close

        # Apparent radius decreases with distance (perspective)
        focal_length = self.image_width / (2 * np.tan(camera_params["fov"] / 2))
        apparent_radius = (object_radius * focal_length) / distance

        # Bounding box size (pixels)
        box_width = int(apparent_radius * 2)
        box_height = int(apparent_radius * 3)  # Cylindrical objects are taller

        # Clamp to reasonable sizes
        box_width = max(10, min(box_width, self.image_width // 2))
        box_height = max(15, min(box_height, self.image_height // 2))

        # Normalize to YOLO format [0, 1]
        x_center_norm = u_center / self.image_width
        y_center_norm = v_center / self.image_height
        width_norm = box_width / self.image_width
        height_norm = box_height / self.image_height

        return x_center_norm, y_center_norm, width_norm, height_norm

    def generate_annotation(self, metadata, camera_params):
        """
        Generate YOLO annotation for a scenario

        Returns list of annotations in YOLO format:
        [class_id, x_center, y_center, width, height]
        """
        annotations = []

        # Annotate pillars
        for pillar in metadata.get("pillar_positions", []):
            x, y = pillar["x"], pillar["y"]
            z = 0.15  # Half height of pillar (30cm tall)
            color = pillar["color"]
            radius = 0.03  # Pillar radius

            bbox = self.estimate_bounding_box(x, y, z, radius, camera_params)

            if bbox is not None:
                class_id = self.class_map.get(color, 0)
                annotations.append([class_id] + list(bbox))

        # Annotate obstacles
        for obstacle in metadata.get("obstacle_positions", []):
            x, y = obstacle["x"], obstacle["y"]
            z = 0.05  # Half height of obstacle (10cm cube)
            radius = 0.07  # Approximate radius for cube

            bbox = self.estimate_bounding_box(x, y, z, radius, camera_params)

            if bbox is not None:
                class_id = self.class_map["obstacle"]
                annotations.append([class_id] + list(bbox))

        return annotations

    def save_yolo_annotation(self, annotations, output_file):
        """Save annotations to YOLO format text file"""
        with open(output_file, "w") as f:
            for ann in annotations:
                class_id, x_center, y_center, width, height = ann
                f.write(f"{class_id} {x_center:.6f} {y_center:.6f} "
                       f"{width:.6f} {height:.6f}\n")


def extract_frames_from_bag(bag_path, output_dir, frame_skip=10):
    """Extract frames from ROS2 bag"""
    if not ROS2_AVAILABLE:
        print("ERROR: ROS2 required for bag extraction")
        return []

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    bridge = CvBridge()
    frames = []

    # Setup bag reader
    storage_options = StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Read messages
    frame_count = 0
    saved_count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if "image_raw" in topic:
            frame_count += 1

            if frame_count % frame_skip != 0:
                continue

            # Deserialize message
            msg = deserialize_message(data, Image)
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Save frame
            frame_file = output_dir / f"frame_{saved_count:05d}.jpg"
            cv2.imwrite(str(frame_file), cv_image)

            frames.append({
                "file": frame_file,
                "timestamp": timestamp,
                "frame_id": saved_count,
            })

            saved_count += 1

            if saved_count % 100 == 0:
                print(f"Extracted {saved_count} frames...")

    print(f"Total frames extracted: {saved_count}")
    return frames


def create_yolo_dataset(metadata_dir, frames_dir, output_dir):
    """Create complete YOLO dataset with train/val split"""
    output_dir = Path(output_dir)
    images_dir = output_dir / "images"
    labels_dir = output_dir / "labels"

    # Create directories
    for split in ["train", "val"]:
        (images_dir / split).mkdir(parents=True, exist_ok=True)
        (labels_dir / split).mkdir(parents=True, exist_ok=True)

    # Load all metadata files
    metadata_files = sorted(Path(metadata_dir).glob("*_metadata.json"))

    annotator = YOLOAnnotator()

    # Camera parameters (from robot URDF)
    camera_params = {
        "pose": {
            "x": 0.08,   # Forward offset
            "y": 0.0,
            "z": 0.05,   # Height offset
            "pitch": 0.2,  # Downward tilt (radians)
        },
        "fov": 2.094,  # 120 degrees horizontal FOV
    }

    train_count = 0
    val_count = 0

    for metadata_file in metadata_files:
        with open(metadata_file) as f:
            metadata = json.load(f)

        scenario_id = metadata["scenario_id"]

        # Load frames for this scenario
        scenario_frames_dir = Path(frames_dir) / f"scenario_{scenario_id:04d}"

        if not scenario_frames_dir.exists():
            print(f"WARNING: No frames found for scenario {scenario_id}")
            continue

        frame_files = sorted(scenario_frames_dir.glob("*.jpg"))

        # Generate annotations
        annotations = annotator.generate_annotation(metadata, camera_params)

        # Split train/val (80/20)
        split = "train" if np.random.random() < 0.8 else "val"

        for frame_file in frame_files:
            # Copy image
            if split == "train":
                dest_image = images_dir / "train" / f"train_{train_count:06d}.jpg"
                dest_label = labels_dir / "train" / f"train_{train_count:06d}.txt"
                train_count += 1
            else:
                dest_image = images_dir / "val" / f"val_{val_count:06d}.jpg"
                dest_label = labels_dir / "val" / f"val_{val_count:06d}.txt"
                val_count += 1

            # Copy frame
            os.system(f'cp "{frame_file}" "{dest_image}"')

            # Save annotation
            annotator.save_yolo_annotation(annotations, dest_label)

    # Create data.yaml
    data_yaml = output_dir / "data.yaml"
    with open(data_yaml, "w") as f:
        f.write(f"""# WRO Traffic Sign Dataset (Official Colors)
path: {output_dir.absolute()}
train: images/train
val: images/val

nc: 3  # Number of classes (WRO official: red, green, obstacle)
names: ['red', 'green', 'obstacle']

# Training configuration
imgsz: 640
batch: 16
epochs: 100

# WRO Official Competition Notes:
# - Red pillars: Turn right indicator
# - Green pillars: Turn left indicator
# - Obstacles: Only in obstacles challenge (red/green blocks)
""")

    print("\nDataset created successfully!")
    print(f"  Train images: {train_count}")
    print(f"  Val images: {val_count}")
    print(f"  Output: {output_dir}")
    print("\nTo train YOLO:")
    print(f"  yolo task=detect mode=train model=yolo26n.pt data={data_yaml}")


def main():
    parser = argparse.ArgumentParser(
        description="Extract frames and generate YOLO annotations")
    parser.add_argument("--metadata-dir", type=str, required=True,
                       help="Directory containing scenario metadata JSON files")
    parser.add_argument("--frames-dir", type=str, required=True,
                       help="Directory containing extracted frames")
    parser.add_argument("--output-dir", type=str, required=True,
                       help="Output directory for YOLO dataset")
    parser.add_argument("--bag-dir", type=str,
                       help="Optional: Directory with ROS2 bags to extract frames from")
    parser.add_argument("--frame-skip", type=int, default=10,
                       help="Extract every Nth frame from bags")

    args = parser.parse_args()

    # Extract frames from bags if specified
    if args.bag_dir:
        bag_dir = Path(args.bag_dir)
        bag_files = sorted(bag_dir.glob("*.db3"))

        print(f"Found {len(bag_files)} ROS2 bags")

        for bag_file in bag_files:
            scenario_id = bag_file.stem.split("_")[-1]
            output_dir = Path(args.frames_dir) / f"scenario_{scenario_id}"

            print(f"\nExtracting frames from {bag_file.name}...")
            extract_frames_from_bag(bag_file, output_dir, args.frame_skip)

    # Create YOLO dataset
    print("\nGenerating YOLO dataset...")
    create_yolo_dataset(args.metadata_dir, args.frames_dir, args.output_dir)

    print("\n✅ Complete!")


if __name__ == "__main__":
    main()
