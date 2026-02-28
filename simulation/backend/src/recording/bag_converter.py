#!/usr/bin/env python3
"""
Convert ROS2 bags to videos

Extracts image messages from ROS2 bags and saves as MP4 videos.

Usage:
    # Convert a single bag
    python3 convert_bags_to_videos.py --bag-file ~/wro_recordings/recording_0.db3 --output-dir ~/videos

    # Convert all bags in directory
    python3 convert_bags_to_videos.py --bag-dir ~/wro_recordings --output-dir ~/videos

    # Convert with custom frame rate
    python3 convert_bags_to_videos.py --bag-dir ~/wro_recordings --output-dir ~/videos --fps 30
"""

import argparse
import os
import sys
from pathlib import Path

try:
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from sensor_msgs.msg import Image
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ERROR: Required packages not found")
    print("Install: sudo apt install ros-humble-rosbag2-py ros-humble-cv-bridge")
    print("         pip3 install opencv-python")
    sys.exit(1)


class BagToVideoConverter:
    """Converts ROS2 bag files to MP4 videos"""

    def __init__(self, topic="/camera/image_raw", fps=30):
        self.topic = topic
        self.fps = fps
        self.bridge = CvBridge()

    def convert_bag(self, bag_path, output_video_path):
        """Convert a single bag file to video"""
        bag_path = Path(bag_path)
        output_video_path = Path(output_video_path)

        print(f"\nConverting: {bag_path.name}")
        print(f"Output: {output_video_path}")

        # Check if bag exists
        if not bag_path.exists():
            print(f"  ERROR: Bag not found: {bag_path}")
            return False

        # Create output directory
        output_video_path.parent.mkdir(parents=True, exist_ok=True)

        try:
            # Setup bag reader
            storage_options = StorageOptions(uri=str(bag_path), storage_id="sqlite3")
            converter_options = ConverterOptions("", "")
            reader = SequentialReader()
            reader.open(storage_options, converter_options)

            # Read all image messages
            print(f"  Reading messages from topic: {self.topic}")
            frames = []
            message_count = 0
            image_count = 0

            while reader.has_next():
                topic, data, timestamp = reader.read_next()
                message_count += 1

                if topic == self.topic:
                    try:
                        # Deserialize image message
                        msg = deserialize_message(data, Image)
                        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                        frames.append(cv_image)
                        image_count += 1

                        if image_count % 100 == 0:
                            print(f"  Processed {image_count} images...")

                    except Exception as e:
                        print(f"  WARNING: Failed to process message: {e}")

            if len(frames) == 0:
                print(f"  ERROR: No images found on topic {self.topic}")
                return False

            print(f"  Total messages: {message_count}")
            print(f"  Total images: {image_count}")

            # Create video
            print("  Creating video...")
            height, width = frames[0].shape[:2]

            # Use H.264 codec for better compatibility
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            video_writer = cv2.VideoWriter(
                str(output_video_path),
                fourcc,
                self.fps,
                (width, height),
            )

            for i, frame in enumerate(frames):
                video_writer.write(frame)

                if (i + 1) % 100 == 0:
                    print(f"  Written {i + 1}/{len(frames)} frames...")

            video_writer.release()

            duration = len(frames) / self.fps
            print(f"  ✓ Video saved: {output_video_path}")
            print(f"  Duration: {duration:.2f}s, Frames: {len(frames)}, FPS: {self.fps}")

            return True

        except Exception as e:
            print(f"  ✗ ERROR: {e}")
            import traceback
            traceback.print_exc()
            return False

    def convert_directory(self, bag_dir, output_dir):
        """Convert all bags in a directory"""
        bag_dir = Path(bag_dir)
        output_dir = Path(output_dir)

        # Find all bag files (look for metadata.yaml or .db3 files)
        bag_folders = []

        # Method 1: Look for folders with metadata.yaml
        for item in bag_dir.iterdir():
            if item.is_dir():
                metadata_file = item / "metadata.yaml"
                if metadata_file.exists():
                    bag_folders.append(item)

        # Method 2: Look for .db3 files directly
        db3_files = list(bag_dir.glob("*.db3"))

        if len(bag_folders) == 0 and len(db3_files) == 0:
            print(f"No bag files found in {bag_dir}")
            return False

        print(f"Found {len(bag_folders)} bag folders and {len(db3_files)} db3 files")

        success_count = 0
        failed_count = 0

        # Process bag folders
        for bag_folder in bag_folders:
            output_video = output_dir / f"{bag_folder.name}.mp4"

            if self.convert_bag(bag_folder, output_video):
                success_count += 1
            else:
                failed_count += 1

        # Process standalone db3 files
        for db3_file in db3_files:
            output_video = output_dir / f"{db3_file.stem}.mp4"

            if self.convert_bag(db3_file, output_video):
                success_count += 1
            else:
                failed_count += 1

        # Summary
        print(f"\n{'='*60}")
        print("Conversion complete:")
        print(f"  Successful: {success_count}")
        print(f"  Failed: {failed_count}")
        print(f"  Output directory: {output_dir}")
        print(f"{'='*60}\n")

        return failed_count == 0


def main():
    parser = argparse.ArgumentParser(
        description="Convert ROS2 bags to MP4 videos",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Input options
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--bag-file", type=str,
                            help="Path to single bag file or folder")
    input_group.add_argument("--bag-dir", type=str,
                            help="Directory containing multiple bags")

    # Output options
    parser.add_argument("--output-dir", type=str, required=True,
                       help="Output directory for videos")

    # Conversion options
    parser.add_argument("--topic", type=str, default="/camera/image_raw",
                       help="Image topic to extract (default: /camera/image_raw)")
    parser.add_argument("--fps", type=int, default=30,
                       help="Output video frame rate (default: 30)")

    args = parser.parse_args()

    # Create converter
    converter = BagToVideoConverter(topic=args.topic, fps=args.fps)

    # Convert
    if args.bag_file:
        # Single file
        bag_path = Path(args.bag_file)
        output_video = Path(args.output_dir) / f"{bag_path.stem}.mp4"

        success = converter.convert_bag(bag_path, output_video)
        return 0 if success else 1

    # Directory
    success = converter.convert_directory(args.bag_dir, args.output_dir)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
