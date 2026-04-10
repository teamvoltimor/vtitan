#!/usr/bin/env python3
"""
Complete Pipeline: Generate and Record WRO Training Videos

This script automates the entire process:
1. Generates randomized scenarios
2. Launches each scenario in Gazebo
3. Records robot POV camera as video
4. Saves metadata for training

Usage:
    # Generate and record 10 open challenge scenarios
    python3 record_scenario_videos.py --challenge open --num-scenarios 10 --duration 30

    # Generate and record obstacles challenge with full randomization
    python3 record_scenario_videos.py \
        --challenge obstacles --num-scenarios 50 --duration 45 --randomize-all

    # Only record existing scenarios (skip generation)
    python3 record_scenario_videos.py \
        --challenge open --skip-generation --scenarios-dir ~/wro_data/open/scenarios
"""

import argparse
import json
import os
import subprocess
import sys
import time
from argparse import Namespace
from pathlib import Path
from typing import TYPE_CHECKING, Any

# ROS2 imports
if TYPE_CHECKING:
    from cv_bridge import CvBridge  # type: ignore[import]
    from rclpy.node import Node  # type: ignore[import]
    from sensor_msgs.msg import Image  # type: ignore[import]

try:
    import cv2  # type: ignore[import]
    import numpy as np  # type: ignore[import]
    import rclpy  # type: ignore[import]
    from cv_bridge import CvBridge  # type: ignore[import]
    from rclpy.node import Node  # type: ignore[import]
    from sensor_msgs.msg import Image  # type: ignore[import]

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ERROR: ROS2 not available. This script requires ROS2 Kilted Kaiju.")
    sys.exit(1)

# Import scenario generator
from shared.config.constants import DictKeys, FileExtensions, FilePaths, FolderNames

from src.generation.generator import ScenarioGenerator


# Custom exceptions for specific error scenarios
class RecordingError(Exception):
    """Base exception for recording-related errors."""


class FrameProcessingError(RecordingError):
    """Error processing camera frame."""


class VideoSaveError(RecordingError):
    """Error saving video file."""


class GazeboError(RecordingError):
    """Error launching or communicating with Gazebo."""


class BridgeError(RecordingError):
    """Error launching or communicating with ros_gz_bridge."""


class NavigatorError(RecordingError):
    """Error launching or communicating with robot navigator."""


class RobotSpawnError(RecordingError):
    """Error spawning robot in simulation."""


class FrameExtractionError(RecordingError):
    """Error extracting frames from video."""


class ScenarioGenerationError(RecordingError):
    """Error generating scenario."""


class ProcessTerminationError(RecordingError):
    """Error terminating subprocess."""


class VideoRecorderNode(Node):
    """ROS2 node that records camera feed to video file"""

    def __init__(
        self,
        output_path: Path | str,
        duration: float,
        fps: int = 30,
    ) -> None:
        super().__init__("video_recorder_node")

        self.output_path = Path(output_path)
        self.duration = duration
        self.fps = fps

        self.bridge = CvBridge()
        self.frames = []
        self.recording = True
        self.start_time = time.time()

        # Subscribe to camera (published by Gazebo camera sensor)
        # Using robot POV camera for training videos
        self.subscription = self.create_subscription(
            Image,
            "/robot/camera",  # Robot-mounted forward-looking camera
            self.image_callback,
            10,
        )

        self.get_logger().info(f"Recording started. Duration: {duration}s, Output: {output_path}")

        # Timer to check duration
        self.timer = self.create_timer(0.5, self.check_duration)

    def image_callback(self, msg: Image) -> None:
        """Callback for camera images"""
        if self.recording:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.frames.append(cv_image)

                # Log progress every 2 seconds
                elapsed = time.time() - self.start_time
                if len(self.frames) % (self.fps * 2) == 0:
                    self.get_logger().info(
                        f"Recording: {elapsed:.1f}s / {self.duration}s ({len(self.frames)} frames)"
                    )
             except (
                RuntimeError,
                OSError,
                TypeError,
            ) as e:
                self.get_logger().error(f"Error processing frame: {e}")

    def check_duration(self) -> None:
        """Check if recording duration has elapsed"""
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration:
            self.get_logger().info("Recording duration reached. Saving video...")
            self.save_video()
            self.recording = False
            rclpy.shutdown()

    def save_video(self) -> bool:
        """Save recorded frames as video"""
        if len(self.frames) == 0:
            self.get_logger().error("No frames recorded!")
            return False

        try:
            # Create output directory
            self.output_path.parent.mkdir(parents=True, exist_ok=True)

            # Get frame dimensions
            height, width = self.frames[0].shape[:2]

            # Create video writer (H.264 codec for better compatibility)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # type: ignore[attr-defined]
            video_writer = cv2.VideoWriter(
                str(self.output_path),
                fourcc,
                self.fps,
                (width, height),
            )

            # Write frames
            for frame in self.frames:
                video_writer.write(frame)

            video_writer.release()

            self.get_logger().info(f"Video saved: {self.output_path}")
            self.get_logger().info(
                f"Total frames: {len(self.frames)}, Duration: {len(self.frames) / self.fps:.2f}s"
            )

            return True

         except (OSError, cv2.error) as e:  # type: ignore[attr-defined]
             self.get_logger().error(f"Error saving video: {e}")
             return False
         except Exception as e:
             self.get_logger().error(f"Unexpected error saving video: {e}")
             return False


class PipelineOrchestrator:
    """Orchestrates the complete pipeline: generate scenarios -> record videos"""

    def __init__(self, args: Namespace) -> None:
        self.args = args
        self.output_dir = Path(args.output_dir)
        self.challenge_type = args.challenge

        # Create directory structure
        self.scenarios_dir = self.output_dir / args.challenge / FolderNames.SCENARIOS
        self.videos_dir = self.output_dir / args.challenge / "videos"
        self.frames_dir = self.output_dir / args.challenge / FolderNames.FRAMES

        self.scenarios_dir.mkdir(parents=True, exist_ok=True)
        self.videos_dir.mkdir(parents=True, exist_ok=True)
        self.frames_dir.mkdir(parents=True, exist_ok=True)

         self.gazebo_process = None
         self.recorder_process = None
         self.bridge_process = None
         self.driver_process = None
         
         # Gazebo operation timeout (seconds)
         self._gazebo_launch_timeout = 30  # Max time to wait for Gazebo to launch
         self._gazebo_start_time: float | None = None

    def generate_scenarios(self) -> bool:
        """Generate randomized scenarios"""
        if self.args.skip_generation:
            print("Skipping scenario generation (--skip-generation flag)")
            return True

        print(f"\n{'=' * 60}")
        print(f"STEP 1: Generating {self.args.num_scenarios} scenarios")
        print(f"Challenge: {self.challenge_type}")
        print(f"Output: {self.scenarios_dir}")
        print(f"{'=' * 60}\n")

        # Find base world file
        base_world = Path(__file__).parent.parent / "worlds" / "wro_track_2026.sdf"
        if not base_world.exists():
            print(f"ERROR: Base world file not found: {base_world}")
            return False

        # Create scenario generator
        generator = ScenarioGenerator(
            base_world_path=str(base_world),
            output_dir=str(self.scenarios_dir),
            challenge_type=self.challenge_type,
        )

        # Generate scenarios
        for i in range(self.args.num_scenarios):
            print(f"Generating scenario {i + 1}/{self.args.num_scenarios}...")

            try:
                world_file, metadata = generator.create_scenario_world(
                    scenario_index=i,
                    randomize_all=self.args.randomize_all,
                )

                start_section = metadata[DictKeys.STARTING_CONDITIONS][DictKeys.SECTION]
                start_direction = metadata[DictKeys.STARTING_CONDITIONS][DictKeys.DIRECTION]
                print(f"  ✓ World: {world_file.name}")
                print(f"  ✓ Signs: {metadata[DictKeys.NUM_SIGNS]}")
                print(f"  ✓ Start: {start_section} ({start_direction})")

            except ScenarioGenerationError as e:
                print(f"  ✗ ERROR: {e}")
                return False
            except (OSError, ValueError, KeyError) as e:
                print(f"  ✗ ERROR: Failed to generate scenario: {e}")
                return False
            except Exception as e:
                print(f"  ✗ UNEXPECTED ERROR: {e}")
                return False

        print(f"\n✓ Successfully generated {self.args.num_scenarios} scenarios\n")
        return True

    def get_scenario_files(self) -> list[Path]:
        """Get list of scenario files to process"""
        scenario_files = sorted(
            self.scenarios_dir.glob(f"{FilePaths.SCENARIO_PREFIX}*{FileExtensions.SDF}")
        )

        if len(scenario_files) == 0:
            print(f"ERROR: No scenario files found in {self.scenarios_dir}")
            return []

        # If specific scenarios requested, filter
        if hasattr(self.args, "scenario_ids") and self.args.scenario_ids:
            scenario_files = [
                f for f in scenario_files if self.get_scenario_id(f) in self.args.scenario_ids
            ]

        return scenario_files

    def get_scenario_id(self, scenario_file: Path) -> int:
        """Extract scenario ID from filename"""
        # scenario_0001.sdf -> 1
        filename = scenario_file.stem  # scenario_0001
        return int(filename.split("_")[-1])

    def load_metadata(self, scenario_file: Path) -> dict[str, Any] | None:
        """Load metadata for a scenario"""
        # Construct metadata filename: scenario_0001.sdf -> scenario_0001_metadata.json
        metadata_file = scenario_file.parent / f"{scenario_file.stem}{FilePaths.METADATA_SUFFIX}"

        if not metadata_file.exists():
            print(f"WARNING: Metadata not found for {scenario_file.name}")
            return None

        with open(metadata_file) as f:
            return json.load(f)

    def launch_gazebo(self, scenario_file: Path) -> bool:
        """Launch Gazebo with scenario with timeout protection."""
        print(f"  Launching Gazebo with {scenario_file.name}...")

        # Set environment variables
        env = os.environ.copy()
        models_dir = Path(__file__).parent.parent / "models"
        env["GZ_SIM_RESOURCE_PATH"] = f"{env.get('GZ_SIM_RESOURCE_PATH', '')}:{models_dir}"

        # Launch Gazebo headless (no GUI for faster processing)
        # Use -s for headless, -r for run immediately
        cmd = [
            "gz",
            "sim",
            "-r",  # Run immediately
            "-s",  # Headless (server only, no GUI)
            str(scenario_file),
        ]

        # If GUI requested, remove -s flag
        if self.args.show_gui:
            cmd.remove("-s")

        try:
            self.gazebo_process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            
            # Track launch time for timeout detection
            self._gazebo_start_time = time.time()

            # Wait for Gazebo to initialize (with timeout protection)
            init_wait = 5 if self.args.show_gui else 3
            time.sleep(init_wait)
            
            # Check if process is still alive after initialization
            if self.gazebo_process.poll() is not None:
                print(f"  ✗ ERROR: Gazebo process crashed during initialization (exit code: {self.gazebo_process.returncode})")
                return False

            return True

        except FileNotFoundError as e:
            print(f"  ✗ ERROR launching Gazebo: Gazebo executable not found: {e}")
            return False
        except (OSError, PermissionError) as e:
            print(f"  ✗ ERROR launching Gazebo: {e}")
            return False
        except Exception as e:
            print(f"  ✗ UNEXPECTED ERROR launching Gazebo: {e}")
            return False
    
    def _check_gazebo_timeout(self) -> bool:
        """Check if Gazebo has exceeded maximum runtime.
        
        Returns:
            True if Gazebo is still running and within timeout, False otherwise.
        """
        if self.gazebo_process is None or self._gazebo_start_time is None:
            return True
        
        # Check if process has crashed
        if self.gazebo_process.poll() is not None:
            print("  ⚠ WARNING: Gazebo process has terminated unexpectedly")
            return False
        
        # Check if exceeded timeout
        elapsed = time.time() - self._gazebo_start_time
        if elapsed > self._gazebo_launch_timeout:
            print(f"  ⚠ WARNING: Gazebo operation exceeded timeout ({elapsed:.1f}s > {self._gazebo_launch_timeout}s)")
            return False
        
        return True

    def launch_bridge(self) -> bool:
        """Launch ros_gz_bridge to connect Gazebo topics to ROS2"""
        print("  Launching ros_gz_bridge...")

        try:
            # Launch parameter bridge for camera and odometry topics
            # This bridges Gazebo topics to ROS2 topics
            camera_topic = "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"
            robot_camera_topic = "/robot/camera@sensor_msgs/msg/Image[gz.msgs.Image"
            bridge_cmd = [
                "ros2",
                "run",
                "ros_gz_bridge",
                "parameter_bridge",
                camera_topic,
                robot_camera_topic,
                "/wro_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/wro_robot/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "--ros-args",
                "--log-level",
                "error",
            ]

            self.bridge_process = subprocess.Popen(
                bridge_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            # Wait for bridge to initialize
            time.sleep(2)

            print("  ✓ Bridge launched (camera, cmd_vel, odom)")
            return True

        except FileNotFoundError as e:
            print(f"  ✗ ERROR launching bridge: ros_gz_bridge not found: {e}")
            return False
        except (OSError, PermissionError) as e:
            print(f"  ✗ ERROR launching bridge: {e}")
            return False
        except Exception as e:
            print(f"  ✗ UNEXPECTED ERROR launching bridge: {e}")
            return False

    def launch_robot_driver(self, metadata: dict[str, Any], metadata_path: Path) -> bool:
        """Launch robot driver to complete the challenge"""
        print("  Launching track navigator...")

        try:
            # Launch track navigator script (completes 3 laps using waypoint navigation)
            navigator_script = Path(__file__).parent / "track_navigator.py"

            self.driver_process = subprocess.Popen(
                [
                    "python3",
                    str(navigator_script),
                    "--metadata",
                    str(metadata_path),
                    "--laps",
                    "3",  # WRO requirement: 3 laps
                ],
                stdout=None,  # Show output in terminal
                stderr=None,  # Show errors in terminal
            )

            # Wait for navigator to initialize
            time.sleep(2)

            direction = metadata[DictKeys.STARTING_CONDITIONS][DictKeys.DIRECTION]
            print(f"  ✓ Track navigator launched ({direction} direction, 3 laps)")
            return True

        except FileNotFoundError as e:
            print(f"  ✗ ERROR launching track navigator: Python script not found: {e}")
            return False
        except (OSError, PermissionError) as e:
            print(f"  ✗ ERROR launching track navigator: {e}")
            return False
        except Exception as e:
            print(f"  ✗ UNEXPECTED ERROR launching track navigator: {e}")
            return False

    def spawn_robot(self, metadata: dict[str, Any]) -> bool:
        """Spawn robot at starting position"""
        print("  Spawning robot at starting position...")

        # Get starting position from metadata
        start_pos = metadata[DictKeys.STARTING_CONDITIONS][DictKeys.POSITION]
        start_yaw = metadata[DictKeys.STARTING_CONDITIONS][DictKeys.YAW]

        # URDF file path
        urdf_file = Path(__file__).parent.parent / "urdf" / "wro_robot.urdf"

        if not urdf_file.exists():
            print(f"  WARNING: URDF file not found: {urdf_file}")
            print("  Skipping robot spawn (recording will still work if robot exists in world)")
            return True

        # Spawn robot using gz service
        position_payload = f"x: {start_pos[DictKeys.X]}, y: {start_pos[DictKeys.Y]}, z: 0.05"
        orientation_payload = f"z: {start_yaw}"
        request_payload = (
            f'sdf_filename: "{urdf_file}", name: "wro_robot", '
            f"pose: {{position: {{{position_payload}}}, "
            f"orientation: {{{orientation_payload}}}}}"
        )

        cmd = [
            "gz",
            "service",
            "-s",
            "/world/wro_track/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "1000",
            "--req",
            request_payload,
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)

            if result.returncode == 0:
                print(
                    "  ✓ Robot spawned at "
                    f"({start_pos[DictKeys.X]:.2f}, {start_pos[DictKeys.Y]:.2f})"
                )
                time.sleep(2)  # Wait for robot to settle
                return True
            print("  WARNING: Robot spawn failed (may already exist)")
            return True  # Continue anyway

        except subprocess.TimeoutExpired:
            print(f"  WARNING: Robot spawn command timed out (may already exist)")
            return True  # Continue anyway
        except (FileNotFoundError, PermissionError) as e:
            print(f"  WARNING: Error spawning robot: {e}")
            return True  # Continue anyway
        except Exception as e:
            print(f"  WARNING: Unexpected error spawning robot: {e}")
            return True  # Continue anyway

    def record_video(self, scenario_id: int) -> Path | None:
        """Record video using ROS2 node with Gazebo timeout protection."""
        print(f"  Recording video for {self.args.duration} seconds...")
        
        # Check Gazebo health before recording
        if not self._check_gazebo_timeout():
            print("  ✗ ERROR: Gazebo not running or exceeded timeout, skipping video recording")
            return None

        # Video output path
        video_file = (
            self.videos_dir / f"{FilePaths.SCENARIO_PREFIX}{scenario_id:04d}{FileExtensions.MP4}"
        )

        # Initialize ROS2 (only if not already initialized)
        if not rclpy.ok():
            rclpy.init()

        try:
            # Create recorder node
            recorder = VideoRecorderNode(
                output_path=str(video_file),
                duration=self.args.duration,
                fps=self.args.fps,
            )

            # Spin until recording completes (with timeout protection)
            # The VideoRecorderNode has its own timeout via check_duration timer
            rclpy.spin(recorder)

            # Cleanup
            recorder.destroy_node()

            return video_file

        except RuntimeError as e:
            print(f"  ✗ ERROR recording video: ROS2 runtime error: {e}")
            return None
        except (OSError, IOError) as e:
            print(f"  ✗ ERROR recording video: File I/O error: {e}")
            return None
        except Exception as e:
            print(f"  ✗ UNEXPECTED ERROR recording video: {e}")
            return None

    def extract_sample_frames(
        self,
        video_file: Path | str,
        scenario_id: int,
        num_frames: int = 10,
    ) -> bool:
        """Extract sample frames from video for dataset"""
        if not self.args.extract_frames:
            return True

        print(f"  Extracting {num_frames} sample frames...")

        try:
            # Open video
            cap = cv2.VideoCapture(str(video_file))
            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

            if total_frames == 0:
                print("  WARNING: No frames in video")
                return False

            # Calculate frame indices to extract (evenly spaced)
            frame_indices = np.linspace(0, total_frames - 1, num_frames, dtype=int)

            # Create output directory
            frames_output_dir = self.frames_dir / f"{FilePaths.SCENARIO_PREFIX}{scenario_id:04d}"
            frames_output_dir.mkdir(parents=True, exist_ok=True)

            # Extract frames
            extracted_count = 0
            for idx in frame_indices:
                cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
                ret, frame = cap.read()

                if ret:
                    frame_file = (
                        frames_output_dir / f"frame_{extracted_count:04d}{FileExtensions.JPG}"
                    )
                    cv2.imwrite(str(frame_file), frame)
                    extracted_count += 1

            cap.release()

            print(f"  ✓ Extracted {extracted_count} frames to {frames_output_dir}")
            return True

        except (FileNotFoundError, IOError) as e:
            print(f"  ✗ ERROR extracting frames: File error: {e}")
            return False
        except (OSError, cv2.error) as e:  # type: ignore[attr-defined]
            print(f"  ✗ ERROR extracting frames: OpenCV error: {e}")
            return False
        except Exception as e:
            print(f"  ✗ UNEXPECTED ERROR extracting frames: {e}")
            return False

    def cleanup(self) -> None:
        """Cleanup processes with controlled fallback strategy."""
        self._terminate_process("driver", self.driver_process)
        self._terminate_process("bridge", self.bridge_process)
        self._terminate_process("gazebo", self.gazebo_process)
        self._kill_lingering_processes()

    def _terminate_process(self, process_name: str, process: Any) -> None:
        """Terminate a process with fallback to kill if terminate fails.

        Args:
            process_name: Human-readable name for logging.
            process: The subprocess.Popen object to terminate.
        """
        if not process:
            return

        try:
            process.terminate()
            process.wait(timeout=5)
            print(f"  ✓ {process_name} terminated cleanly")
        except subprocess.TimeoutExpired:
            print(f"  ⚠ {process_name} did not terminate, forcing kill...")
            try:
                process.kill()
                process.wait(timeout=2)
                print(f"  ✓ {process_name} killed")
            except OSError as e:
                print(f"  ✗ Failed to kill {process_name}: OS error: {e}")
            except ProcessTerminationError as e:
                print(f"  ✗ Failed to kill {process_name}: {e}")
        except OSError as e:
            print(f"  ⚠ Failed to terminate {process_name}: OS error: {e}. Attempting kill...")
            try:
                process.kill()
                print(f"  ✓ {process_name} killed")
            except OSError as kill_error:
                print(f"  ✗ Failed to kill {process_name}: {kill_error}")
        except ProcessTerminationError as e:
            print(f"  ⚠ Failed to terminate {process_name}: {e}. Attempting kill...")
            try:
                process.kill()
                print(f"  ✓ {process_name} killed")
            except Exception as kill_error:
                print(f"  ✗ Failed to kill {process_name}: {kill_error}")

    def _kill_lingering_processes(self) -> None:
        """Kill any leftover processes from previous runs."""
        try:
            subprocess.run(
                ["killall", "-9", "gz", "parameter_bridge", "python3"],
                stderr=subprocess.DEVNULL,
                timeout=5,
            )
            print("  ✓ Lingering processes killed")
        except subprocess.TimeoutExpired:
            print("  ⚠ killall command timed out")
        except FileNotFoundError:
            # killall not available (e.g., on some Windows systems)
            print("  ⚠ killall command not available on this system")
        except (OSError, PermissionError) as e:
            print(f"  ⚠ Error killing lingering processes: OS error: {e}")

    def process_scenario(self, scenario_file: Path) -> bool:
        """Process a single scenario: launch, record, cleanup"""
        scenario_id = self.get_scenario_id(scenario_file)

        print(f"\n{'=' * 60}")
        print(f"Processing Scenario {scenario_id}")
        print(f"{'=' * 60}")

        # Load metadata
        metadata = self.load_metadata(scenario_file)
        if metadata is None:
            return False

        # Construct metadata file path
        metadata_path = scenario_file.parent / f"{scenario_file.stem}{FilePaths.METADATA_SUFFIX}"

        try:
            # Step 1: Launch Gazebo
            if not self.launch_gazebo(scenario_file):
                return False

            # Step 2: Launch ros_gz_bridge
            if not self.launch_bridge():
                print("  WARNING: Bridge launch failed, recording may not work...")

            # Step 3: Launch track navigator (completes 3 laps)
            if not self.launch_robot_driver(metadata, metadata_path):
                print("  WARNING: Track navigator failed, robot won't move...")

            # Step 4: Record video
            video_file = self.record_video(scenario_id)
            if video_file is None:
                return False

            print(f"  ✓ Video saved: {video_file}")

            # Step 5: Extract frames (optional)
            if self.args.extract_frames:
                self.extract_sample_frames(
                    video_file, scenario_id, num_frames=self.args.num_frames
                )

            print(f"\n✓ Scenario {scenario_id} completed successfully")
            return True

        except (GazeboError, BridgeError, NavigatorError, VideoSaveError) as e:
            print(f"\n✗ ERROR processing scenario {scenario_id}: {e}")
            return False
        except (FileNotFoundError, OSError) as e:
            print(f"\n✗ ERROR processing scenario {scenario_id}: File error: {e}")
            return False
        except Exception as e:
            print(f"\n✗ UNEXPECTED ERROR processing scenario {scenario_id}: {e}")
            return False

        finally:
            # Cleanup
            self.cleanup()
            time.sleep(2)  # Wait before next scenario

    def run(self) -> bool:
        """Run the complete pipeline"""
        print(f"\n{'#' * 60}")
        print("# WRO Training Video Pipeline")
        print(f"# Challenge: {self.challenge_type}")
        print(f"# Output: {self.output_dir}")
        print(f"{'#' * 60}\n")

        # Step 1: Generate scenarios
        if not self.generate_scenarios():
            print("\n✗ Pipeline failed at scenario generation")
            return False

        # Step 2: Get scenarios to process
        print(f"\n{'=' * 60}")
        print("STEP 2: Recording Videos")
        print(f"{'=' * 60}\n")

        scenario_files = self.get_scenario_files()

        if len(scenario_files) == 0:
            print("✗ No scenarios to process")
            return False

        print(f"Found {len(scenario_files)} scenarios to process")

        # Step 3: Process each scenario
        success_count = 0
        failed_scenarios = []

        for i, scenario_file in enumerate(scenario_files):
            print(f"\n[{i + 1}/{len(scenario_files)}]")

            if self.process_scenario(scenario_file):
                success_count += 1
            else:
                failed_scenarios.append(self.get_scenario_id(scenario_file))

        # Summary
        print(f"\n{'#' * 60}")
        print("# Pipeline Complete")
        print(f"{'#' * 60}")
        print(f"Total scenarios: {len(scenario_files)}")
        print(f"Successful: {success_count}")
        print(f"Failed: {len(failed_scenarios)}")

        if failed_scenarios:
            print(f"Failed scenario IDs: {failed_scenarios}")

        print("\nOutput directories:")
        print(f"  Scenarios: {self.scenarios_dir}")
        print(f"  Videos: {self.videos_dir}")
        print(f"  Frames: {self.frames_dir}")

        # Shutdown ROS2
        if rclpy.ok():
            rclpy.shutdown()

        return len(failed_scenarios) == 0


def main() -> int | None:
    parser = argparse.ArgumentParser(
        description="Complete pipeline: Generate scenarios and record training videos",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate and record 10 open challenge scenarios
  python3 record_scenario_videos.py --challenge open --num-scenarios 10 --duration 30

  # Generate and record obstacles with full randomization
  python3 record_scenario_videos.py --challenge obstacles \
      --num-scenarios 50 --duration 45 --randomize-all

  # Only record existing scenarios (skip generation)
  python3 record_scenario_videos.py --challenge open --skip-generation --duration 30

  # Show Gazebo GUI while recording (slower but useful for debugging)
  python3 record_scenario_videos.py --challenge open --num-scenarios 5 --show-gui
        """,
    )

    # Scenario generation options
    parser.add_argument(
        "--challenge",
        type=str,
        required=True,
        choices=["open", "obstacles"],
        help="Challenge type: open or obstacles",
    )
    parser.add_argument(
        "--num-scenarios",
        type=int,
        default=10,
        help="Number of scenarios to generate (default: 10)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="./training_data",
        help="Output directory for all data (default: ./training_data)",
    )
    parser.add_argument(
        "--randomize-all",
        action="store_true",
        help="Enable full randomization (lighting, colors, physics)",
    )
    parser.add_argument(
        "--skip-generation",
        action="store_true",
        help="Skip scenario generation, only record existing scenarios",
    )

    # Recording options
    parser.add_argument(
        "--duration",
        type=int,
        default=30,
        help="Recording duration per scenario in seconds (default: 30)",
    )
    parser.add_argument("--fps", type=int, default=30, help="Video frame rate (default: 30)")
    parser.add_argument(
        "--show-gui", action="store_true", help="Show Gazebo GUI (slower but useful for debugging)"
    )

    # Frame extraction options
    parser.add_argument(
        "--extract-frames",
        action="store_true",
        help="Extract sample frames from videos for dataset",
    )
    parser.add_argument(
        "--num-frames",
        type=int,
        default=10,
        help="Number of frames to extract per video (default: 10)",
    )

    args = parser.parse_args()

    # Check ROS2 availability
    if not ROS2_AVAILABLE:
        print("ERROR: ROS2 is required for this script")
        print("Please install ROS2 Kilted Kaiju")
        return 1

    # Create and run pipeline
    try:
        pipeline = PipelineOrchestrator(args)
        success = pipeline.run()

        return 0 if success else 1

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        return 1
    except (RecordingError, GazeboError, BridgeError, NavigatorError) as e:
        print(f"\n✗ Pipeline error: {e}")
        return 1
    except (FileNotFoundError, OSError) as e:
        print(f"\n✗ Pipeline file error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    except Exception as e:
        print(f"\n✗ Pipeline error: {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
