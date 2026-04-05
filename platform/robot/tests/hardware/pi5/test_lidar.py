"""
ROS2 integration tests for Slamtec RPLiDAR C1.

Run on: Raspberry Pi 5 (with ROS2 Humble or later)

Run with: python -m pytest tests/hardware/pi5/test_lidar.py -v

Requirements:
    - ROS2 Humble or later installed
    - rplidar_ros (ROS2 official package):
      sudo apt install ros-humble-rplidar-ros
    - ros2_launch: pip install launch-ros
    - rclpy: pip install rclpy

Note: These tests assume the RPLiDAR C1 ROS2 node is running.
      The node handles hardware abstraction and serial communication.
"""

import logging
import os
import time
from typing import Any

import pytest

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
except ImportError:
    rclpy = None

from src.logger import LOG_LEVEL_DEFAULT, LOG_LEVEL_KEY, configure_json_logging

_log_level = getattr(logging, os.getenv(LOG_LEVEL_KEY, LOG_LEVEL_DEFAULT).upper(), logging.INFO)
configure_json_logging(level=_log_level)

logger = logging.getLogger(__name__)

LIDAR_TOPIC = "/scan"  # Standard ROS2 LaserScan topic from rplidar_ros
LIDAR_TIMEOUT = 5.0  # Seconds to wait for first scan


class LaserScanListener(Node):
    """ROS2 node to listen to LaserScan messages."""

    def __init__(self):
        super().__init__("laser_scan_listener")
        self.scan_data: LaserScan | None = None
        self.scan_received = False

        self.subscription = self.create_subscription(LaserScan, LIDAR_TOPIC, self.scan_callback, 10)

    def scan_callback(self, msg: LaserScan) -> None:
        """Callback for LaserScan messages."""
        self.scan_data = msg
        self.scan_received = True

    def wait_for_scan(self, timeout: float = LIDAR_TIMEOUT) -> LaserScan | None:
        """Wait for a LaserScan message with timeout."""
        start_time = time.time()

        while not self.scan_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self.scan_data if self.scan_received else None


@pytest.fixture(scope="module")
def ros_context():
    """Initialize and cleanup ROS2 context for the test module."""
    if rclpy is None:
        pytest.skip("ROS2 (rclpy) not installed")

    try:
        rclpy.init()
        yield
        rclpy.shutdown()
    except Exception as e:
        pytest.skip(f"ROS2 initialization failed: {e}")


@pytest.fixture
def laser_listener(ros_context):
    """Create a LaserScan listener node."""
    listener = LaserScanListener()
    yield listener
    listener.destroy_node()


class TestLidarROS2Connection:
    """Test ROS2 LIDAR connectivity via rplidar_ros node."""

    def test_lidar_topic_available(self, _ros_context):
        """Check if LIDAR ROS2 topic is available."""
        if rclpy is None:
            pytest.skip("ROS2 not installed")

        try:
            listener = LaserScanListener()
            scan = listener.wait_for_scan(timeout=5.0)
            listener.destroy_node()

            if scan is None:
                pytest.skip(f"LIDAR ROS2 node not running. Start with: ros2 launch rplidar_ros rplidar_a1_launch.py")

            logger.info("LIDAR ROS2 topic available", extra={"details": {"topic": LIDAR_TOPIC}})
            assert scan is not None
        except Exception as e:
            pytest.skip(f"Cannot connect to LIDAR ROS2 node: {e}")


class TestLidarScan:
    """Test LIDAR scanning via ROS2."""

    def test_single_scan(self, laser_listener):
        """Capture and verify a single LaserScan message."""
        scan = laser_listener.wait_for_scan(timeout=5.0)

        if scan is None:
            pytest.skip("No LaserScan received from ROS2 topic")

        logger.info(
            "Single scan captured",
            extra={
                "details": {
                    "num_ranges": len(scan.ranges),
                    "angle_min_deg": scan.angle_min * 180 / 3.14159,
                    "angle_max_deg": scan.angle_max * 180 / 3.14159,
                    "angle_increment_deg": scan.angle_increment * 180 / 3.14159,
                }
            },
        )

        assert len(scan.ranges) > 0, "No range data in scan"
        assert scan.angle_min < scan.angle_max, "Invalid angle range"

    def test_angle_coverage(self, laser_listener):
        """Verify 360° angle coverage."""
        scan = laser_listener.wait_for_scan(timeout=5.0)

        if scan is None:
            pytest.skip("No LaserScan received from ROS2 topic")

        angle_range_rad = scan.angle_max - scan.angle_min
        angle_range_deg = angle_range_rad * 180 / 3.14159

        logger.info(
            "Angle coverage measured",
            extra={
                "details": {
                    "min_angle_deg": scan.angle_min * 180 / 3.14159,
                    "max_angle_deg": scan.angle_max * 180 / 3.14159,
                    "coverage_deg": angle_range_deg,
                }
            },
        )

        assert angle_range_deg > 300, f"Insufficient angle coverage: {angle_range_deg}°"

    def test_range_validity(self, laser_listener):
        """Check range data validity."""
        scan = laser_listener.wait_for_scan(timeout=5.0)

        if scan is None:
            pytest.skip("No LaserScan received from ROS2 topic")

        valid_ranges = [r for r in scan.ranges if r > 0 and r != float("inf")]

        if not valid_ranges:
            pytest.skip("No valid range measurements in scan")

        min_range = min(valid_ranges)
        max_range = max(valid_ranges)

        logger.info(
            "Range validity checked",
            extra={
                "details": {
                    "min_range_m": min_range,
                    "max_range_m": max_range,
                    "valid_measurements": len(valid_ranges),
                }
            },
        )

        assert min_range > 0, f"Invalid minimum range: {min_range}"
        assert max_range > 0.1, f"Maximum range too low: {max_range}m"

    def test_intensity_data(self, laser_listener):
        """Verify intensity/signal strength data."""
        scan = laser_listener.wait_for_scan(timeout=5.0)

        if scan is None:
            pytest.skip("No LaserScan received from ROS2 topic")

        if not scan.intensities:
            pytest.skip("Intensity data not available from LIDAR")

        avg_intensity = sum(scan.intensities) / len(scan.intensities) if scan.intensities else 0

        logger.info(
            "Intensity data measured",
            extra={
                "details": {
                    "average_intensity": avg_intensity,
                    "num_readings": len(scan.intensities),
                }
            },
        )

        assert len(scan.intensities) > 0, "No intensity data received"


class TestLidarMessageFormat:
    """Test ROS2 LaserScan message format compliance."""

    def test_laser_scan_structure(self, laser_listener):
        """Verify LaserScan message structure."""
        scan = laser_listener.wait_for_scan(timeout=5.0)

        if scan is None:
            pytest.skip("No LaserScan received from ROS2 topic")

        # Check required fields
        assert hasattr(scan, "header"), "Missing header field"
        assert hasattr(scan, "angle_min"), "Missing angle_min field"
        assert hasattr(scan, "angle_max"), "Missing angle_max field"
        assert hasattr(scan, "angle_increment"), "Missing angle_increment field"
        assert hasattr(scan, "time_increment"), "Missing time_increment field"
        assert hasattr(scan, "scan_time"), "Missing scan_time field"
        assert hasattr(scan, "range_min"), "Missing range_min field"
        assert hasattr(scan, "range_max"), "Missing range_max field"
        assert hasattr(scan, "ranges"), "Missing ranges field"

        logger.info(
            "LaserScan structure verified",
            extra={
                "details": {
                    "frame_id": scan.header.frame_id,
                    "scan_time_s": scan.scan_time,
                    "range_min_m": scan.range_min,
                    "range_max_m": scan.range_max,
                }
            },
        )

    def test_timestamp_validity(self, laser_listener):
        """Verify message timestamp is valid."""
        scan = laser_listener.wait_for_scan(timeout=5.0)

        if scan is None:
            pytest.skip("No LaserScan received from ROS2 topic")

        assert scan.header.stamp.sec > 0, "Invalid timestamp seconds"
        logger.info("Message timestamp valid", extra={"details": {"timestamp_sec": scan.header.stamp.sec}})


def stream_laser_scan(duration: float = 10.0) -> None:
    """
    Stream LaserScan data from ROS2 topic for visual inspection.

    Usage:
        python -c "from tests.hardware.pi5.test_lidar import stream_laser_scan; stream_laser_scan()"
    """
    if rclpy is None:
        logger.error("ROS2 (rclpy) not installed")
        return

    logging.basicConfig(level=logging.INFO)
    log = logging.getLogger(__name__)

    try:
        rclpy.init()
        listener = LaserScanListener()
        log.info(f"Streaming LIDAR data for {duration}s - listening to {LIDAR_TOPIC}")

        start_time = time.time()
        scan_count = 0

        while (time.time() - start_time) < duration:
            rclpy.spin_once(listener, timeout_sec=0.1)

            if listener.scan_received:
                scan_count += 1
                scan = listener.scan_data
                valid_ranges = [r for r in scan.ranges if r > 0 and r != float("inf")]

                log.debug(
                    "LIDAR scan",
                    extra={
                        "details": {
                            "scan_number": scan_count,
                            "num_ranges": len(scan.ranges),
                            "valid_ranges": len(valid_ranges),
                            "frame_id": scan.header.frame_id,
                        }
                    },
                )

                listener.scan_received = False

        log.info(f"Captured {scan_count} scans in {duration}s")

    except KeyboardInterrupt:
        log.info("Streaming stopped by user")
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("Run tests with: python -m pytest tests/hardware/pi5/test_lidar.py -v")
    logger.info(
        'Or stream data with: python -c "from tests.hardware.pi5.test_lidar import stream_laser_scan; stream_laser_scan()"'
    )
    logger.info("Before running tests, start the LIDAR ROS2 node:")
    logger.info("  ros2 launch rplidar_ros rplidar_a1_launch.py")
