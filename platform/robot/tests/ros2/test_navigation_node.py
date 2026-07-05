"""Contract tests for ROS2HardwareGateway's real ROS2 wiring.

Nothing previously exercised ROS2HardwareGateway against a real rclpy.Node,
which is how it drifted onto a Twist/cmd_vel contract nothing subscribes to
while the rest of the stack (ackermann_motor_node, state_machine_node) moved
to AckermannDriveStamped/ackermann_cmd. These tests pin the real topic names
and message contract so that drift can't happen silently again.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from shared.config.constants import RobotSpecs
from shared.config.enums import Section
from shared.domain.steering import steering_norm_to_angle_rad

from src.navigation.ports import DriveCommand
from src.navigation.track_geometry import TrackWalls
from src.ros2.navigation.node import ROS2HardwareGateway

# tests/ros2/conftest.py mocks sys.modules["buildhat"] for this whole directory.

_WIDTHS = {Section.NORTH: 1.0, Section.SOUTH: 1.0, Section.EAST: 1.0, Section.WEST: 1.0}


@pytest.fixture()
def ros_context():
    """Initialize and cleanup ROS2 context for each test."""
    try:
        rclpy.init()
        yield
        rclpy.shutdown()
    except Exception as e:  # noqa: BLE001
        pytest.skip(f"ROS2 initialization failed: {e}")


def _make_host_node() -> Node:
    """A bare Node declaring the parameters ROS2HardwareGateway expects."""
    node = Node("test_track_navigator")
    node.declare_parameter("ackermann_cmd_topic", "/ackermann_cmd")
    node.declare_parameter("lidar_topic", "/scan")
    node.declare_parameter("vision_topic", "/vision/detections")
    node.declare_parameter("imu_topic", "/imu/data")
    return node


class TestGatewayTopicContract:
    """Pin the gateway onto the topics the deployed nodes actually use."""

    def test_default_topics_match_deployed_nodes(self, ros_context):
        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)

        topics = dict(node.get_publisher_names_and_types_by_node(node.get_name(), ""))
        assert "/ackermann_cmd" in topics
        assert topics["/ackermann_cmd"] == ["ackermann_msgs/msg/AckermannDriveStamped"]

        node.destroy_node()


class TestGatewayPublishDrive:
    """publish_drive() must emit the contract ackermann_motor_node decodes."""

    def test_publishes_ackermann_drive_stamped(self, ros_context):
        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)

        published: list[AckermannDriveStamped] = []
        gateway._drive_publisher.publish = published.append

        gateway.publish_drive(DriveCommand(speed_mps=0.42, steering_norm=0.5))

        assert len(published) == 1
        msg = published[0]
        assert isinstance(msg, AckermannDriveStamped)
        assert msg.drive.speed == pytest.approx(0.42)
        # steering_norm decodes through the same shared mapping the motor
        # node and simulator use — not a bare pass-through of the normalised
        # value (that was the CRIT-1 bug: 0.5 treated as 0.5 rad).
        expected_angle = steering_norm_to_angle_rad(0.5, RobotSpecs.MAX_STEERING_ANGLE)
        assert msg.drive.steering_angle == pytest.approx(expected_angle)
        assert expected_angle != pytest.approx(0.5), "must not be the raw normalised value"

        node.destroy_node()

    def test_zero_command_is_zero_speed_and_angle(self, ros_context):
        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)

        published: list[AckermannDriveStamped] = []
        gateway._drive_publisher.publish = published.append

        gateway.publish_drive(DriveCommand(speed_mps=0.0, steering_norm=0.0))

        assert published[0].drive.speed == pytest.approx(0.0)
        assert published[0].drive.steering_angle == pytest.approx(0.0)

        node.destroy_node()


class TestGatewayLidarLocalization:
    """get_current_pose() must reflect where the robot actually is — not just

    the seed start position — once LIDAR scans start arriving. Nothing
    publishes nav_msgs/Odometry on real hardware, so this is the only real
    position source (NEW-1 in the 2026-07-05 navigation review).
    """

    def test_pose_updates_from_lidar_scan_away_from_start(self, ros_context):
        node = _make_host_node()
        # Seed the gateway's position a few centimetres off from where the
        # robot actually is, exactly like real operation (the localizer
        # tracks locally from a good prior; it is not a global relocalizer).
        # Yaw is seeded accurately — the IMU's first reading calibrates its
        # own zero-point relative to start_yaw, so it isn't this test's
        # concern; only the LIDAR-derived position is under test here.
        true_x, true_y, true_yaw = 2.5, 1.5, math.pi / 2
        start_x, start_y, start_yaw = 2.47, 1.47, true_yaw
        gateway = ROS2HardwareGateway(node, start_x, start_y, start_yaw, _WIDTHS)

        walls = TrackWalls(_WIDTHS)
        angles = np.linspace(-math.pi, math.pi, RobotSpecs.LIDAR_SAMPLES, endpoint=False)
        ranges = walls.raycast(true_x, true_y, true_yaw, angles)

        msg = LaserScan()
        msg.angle_min = float(angles[0])
        msg.angle_max = float(angles[-1])
        msg.ranges = ranges.tolist()

        # IMU must report the true yaw before the scan arrives, since the
        # localizer is only asked to solve for (x, y), not yaw.
        from sensor_msgs.msg import Imu

        imu_msg = Imu()
        imu_msg.orientation.z = math.sin(true_yaw / 2)
        imu_msg.orientation.w = math.cos(true_yaw / 2)
        gateway._imu_callback(imu_msg)

        gateway._lidar_callback(msg)

        pose = gateway.get_current_pose()
        assert pose is not None
        assert pose.x == pytest.approx(true_x, abs=0.02)
        assert pose.y == pytest.approx(true_y, abs=0.02)
        assert (pose.x, pose.y) != pytest.approx((start_x, start_y), abs=0.01), (
            "pose must move off the seed start position once a scan arrives"
        )

        node.destroy_node()
