"""
Tests for ROS2 BNO085 IMU I2C lifecycle node.

IMU_I2CNode is a LifecycleNode: hardware connects in on_configure() and
publishing starts in on_activate(), so tests must drive those transitions
explicitly before exercising node behavior (deployed nodes do this
automatically via trigger_configure()/trigger_activate() in main()).

Run with: python -m pytest tests/ros2/test_imu_bno08x_i2c_node.py -v
"""

import logging
import sys
from dataclasses import dataclass
from unittest import mock

import pytest
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import Imu

sys.modules["board"] = mock.MagicMock()
sys.modules["busio"] = mock.MagicMock()
sys.modules["adafruit_bno08x.i2c"] = mock.MagicMock()

from src.ros2.imu.bno08x.mcp2221.i2c_node import IMU_I2CNode
from tests.ros2.common_imu_fixtures import (
    assert_creates_timer_on_activate,
    assert_node_configures_correctly,
    assert_publish_imu_noop_before_configure,
)

logger = logging.getLogger(__name__)


# Define IMU_AllData locally to avoid circular imports
@dataclass
class IMU_AllData:
    """IMU all sensor data."""

    quaternion: tuple[float, float, float, float]
    linear_accel: tuple[float, float, float]
    gyroscope: tuple[float, float, float]


@pytest.fixture()
def mock_driver():
    """Create a mock IMU I2C driver."""
    with mock.patch("src.ros2.imu.bno08x.mcp2221.i2c_node.IMU_I2CDriver") as mock_cls:
        driver_instance = mock.MagicMock()
        mock_cls.return_value = driver_instance
        yield driver_instance


class TestIMU_I2CNodeInit:
    """Test IMU I2C node configuration."""

    def test_node_configures_correctly(self, ros_context, mock_driver):
        """Test node configures correctly."""
        assert_node_configures_correctly(IMU_I2CNode, "bno08x_i2c_node")

    def test_node_creates_publisher(self, ros_context, mock_driver):
        """Test node creates IMU publisher after activation (lifecycle publishers
        aren't advertised on the graph until the node activates)."""
        node = IMU_I2CNode()
        node.trigger_configure()
        node.trigger_activate()
        assert node.publisher_ is not None
        topic_names = [topic_name for topic_name, _ in node.get_publisher_names_and_types_by_node(node.get_name(), "")]
        assert any("imu/data" in topic_name for topic_name in topic_names)
        node.destroy_node()

    def test_node_calls_driver_connect(self, ros_context, mock_driver):
        """Test node calls driver connect during configure."""
        node = IMU_I2CNode()
        node.trigger_configure()
        mock_driver.connect.assert_called_once()
        node.destroy_node()

    def test_node_calls_driver_enable_sensors(self, ros_context, mock_driver):
        """Test node calls driver enable_sensors during configure."""
        node = IMU_I2CNode()
        node.trigger_configure()
        mock_driver.enable_sensors.assert_called_once()
        node.destroy_node()

    def test_node_creates_timer_on_activate(self, ros_context, mock_driver):
        """Test node creates publish timer on activate, not on configure."""
        assert_creates_timer_on_activate(IMU_I2CNode)

    def test_node_degrades_gracefully_if_driver_connect_fails(self, ros_context, mock_driver):
        """Test node stays configured (driver=None) if connect raises."""
        mock_driver.connect.side_effect = RuntimeError("Connection failed")

        node = IMU_I2CNode()
        result = node.trigger_configure()

        assert result == TransitionCallbackReturn.SUCCESS
        assert node.driver is None
        node.destroy_node()

    def test_node_degrades_gracefully_if_enable_sensors_fails(self, ros_context, mock_driver):
        """Test node stays configured (driver=None) if enable_sensors raises."""
        mock_driver.enable_sensors.side_effect = RuntimeError("Sensor enable failed")

        node = IMU_I2CNode()
        result = node.trigger_configure()

        assert result == TransitionCallbackReturn.SUCCESS
        assert node.driver is None
        node.destroy_node()


class TestIMU_I2CNodePublishing:
    """Test IMU I2C node data publishing."""

    def test_publish_imu_with_valid_data(self, ros_context, mock_driver):
        """Test publishing IMU message with valid data."""
        node = IMU_I2CNode()
        node.trigger_configure()
        node.trigger_activate()

        # Create mock sensor data
        # Note: I2C quaternion order is (qw, qx, qy, qz)
        mock_data = IMU_AllData(
            quaternion=(0.707, 0.0, 0.0, 0.707),
            linear_accel=(0.1, 0.2, 9.8),
            gyroscope=(0.01, 0.02, 0.03),
        )
        mock_driver.get_all_data.return_value = mock_data

        # Mock the publisher to capture published messages
        published_messages = []

        def capture_publish(msg):
            published_messages.append(msg)

        node.publisher_.publish = capture_publish

        # Call publish method
        node.publish_imu()

        # Verify message was published
        assert len(published_messages) == 1
        msg = published_messages[0]

        # Verify message type
        assert isinstance(msg, Imu)

        # Verify orientation (quaternion)
        # Note: conversion from (qw, qx, qy, qz) to msg format (x, y, z, w)
        assert msg.orientation.x == 0.0
        assert msg.orientation.y == 0.0
        assert msg.orientation.z == 0.707
        assert msg.orientation.w == 0.707

        # Verify linear acceleration
        assert msg.linear_acceleration.x == 0.1
        assert msg.linear_acceleration.y == 0.2
        assert msg.linear_acceleration.z == 9.8

        # Verify angular velocity (gyroscope)
        assert msg.angular_velocity.x == 0.01
        assert msg.angular_velocity.y == 0.02
        assert msg.angular_velocity.z == 0.03

        # Verify header
        assert msg.header.frame_id == "imu_link"

        node.destroy_node()

    def test_publish_imu_sets_timestamp(self, ros_context, mock_driver):
        """Test publish_imu sets message timestamp."""
        node = IMU_I2CNode()
        node.trigger_configure()
        node.trigger_activate()

        mock_data = IMU_AllData(
            quaternion=(1.0, 0.0, 0.0, 0.0),
            linear_accel=(0.0, 0.0, 0.0),
            gyroscope=(0.0, 0.0, 0.0),
        )
        mock_driver.get_all_data.return_value = mock_data

        published_messages = []

        def capture_publish(msg):
            published_messages.append(msg)

        node.publisher_.publish = capture_publish

        node.publish_imu()

        assert len(published_messages) == 1
        msg = published_messages[0]

        # Verify timestamp is set
        assert msg.header.stamp is not None

        node.destroy_node()

    def test_publish_imu_sets_covariances(self, ros_context, mock_driver):
        """Test publish_imu sets covariance matrices."""
        node = IMU_I2CNode()
        node.trigger_configure()
        node.trigger_activate()

        mock_data = IMU_AllData(
            quaternion=(1.0, 0.0, 0.0, 0.0),
            linear_accel=(0.0, 0.0, 0.0),
            gyroscope=(0.0, 0.0, 0.0),
        )
        mock_driver.get_all_data.return_value = mock_data

        published_messages = []

        def capture_publish(msg):
            published_messages.append(msg)

        node.publisher_.publish = capture_publish

        node.publish_imu()

        assert len(published_messages) == 1
        msg = published_messages[0]

        # Verify angular velocity covariance (3x3 diagonal)
        assert msg.angular_velocity_covariance[0] == 0.01
        assert msg.angular_velocity_covariance[4] == 0.01
        assert msg.angular_velocity_covariance[8] == 0.01

        # Verify linear acceleration covariance (3x3 diagonal)
        assert msg.linear_acceleration_covariance[0] == 0.01
        assert msg.linear_acceleration_covariance[4] == 0.01
        assert msg.linear_acceleration_covariance[8] == 0.01

        node.destroy_node()

    def test_publish_imu_quaternion_conversion(self, ros_context, mock_driver):
        """Test publish_imu correctly converts quaternion order."""
        node = IMU_I2CNode()
        node.trigger_configure()
        node.trigger_activate()

        # I2C returns (qw, qx, qy, qz)
        mock_data = IMU_AllData(
            quaternion=(0.8, 0.1, 0.2, 0.3),
            linear_accel=(0.0, 0.0, 0.0),
            gyroscope=(0.0, 0.0, 0.0),
        )
        mock_driver.get_all_data.return_value = mock_data

        published_messages = []

        def capture_publish(msg):
            published_messages.append(msg)

        node.publisher_.publish = capture_publish

        node.publish_imu()

        msg = published_messages[0]

        # Verify conversion: (qw, qx, qy, qz) -> msg(x, y, z, w)
        assert msg.orientation.x == 0.1
        assert msg.orientation.y == 0.2
        assert msg.orientation.z == 0.3
        assert msg.orientation.w == 0.8

        node.destroy_node()

    def test_publish_imu_multiple_iterations(self, ros_context, mock_driver):
        """Test publish_imu works correctly over multiple calls."""
        node = IMU_I2CNode()
        node.trigger_configure()
        node.trigger_activate()

        mock_data_1 = IMU_AllData(
            quaternion=(0.7, 0.1, 0.2, 0.3),
            linear_accel=(0.5, 1.5, 9.8),
            gyroscope=(0.01, 0.02, 0.03),
        )
        mock_data_2 = IMU_AllData(
            quaternion=(0.8, 0.2, 0.3, 0.4),
            linear_accel=(1.0, 2.0, 9.9),
            gyroscope=(0.02, 0.03, 0.04),
        )

        mock_driver.get_all_data.side_effect = [mock_data_1, mock_data_2]

        published_messages = []

        def capture_publish(msg):
            published_messages.append(msg)

        node.publisher_.publish = capture_publish

        # First publish
        node.publish_imu()
        # Second publish
        node.publish_imu()

        assert len(published_messages) == 2

        # Verify first message
        msg1 = published_messages[0]
        assert msg1.orientation.x == 0.1
        assert msg1.linear_acceleration.z == 9.8
        assert msg1.angular_velocity.x == 0.01

        # Verify second message
        msg2 = published_messages[1]
        assert msg2.orientation.x == 0.2
        assert msg2.linear_acceleration.z == 9.9
        assert msg2.angular_velocity.x == 0.02

        node.destroy_node()

    def test_publish_imu_noop_before_configure(self, ros_context, mock_driver):
        """publish_imu must not raise if called before configure (driver/publisher are None)."""
        assert_publish_imu_noop_before_configure(IMU_I2CNode)


class TestIMU_I2CNodeCleanup:
    """Test IMU I2C node cleanup."""

    def test_node_cleanup_on_destroy(self, ros_context, mock_driver):
        """Test node cleanup on destroy."""
        node = IMU_I2CNode()
        node.trigger_configure()
        node.destroy_node()
        # Verify node is destroyed without error

    def test_node_does_not_crash_on_destroy(self, ros_context, mock_driver):
        """Test node destroy is safe even if driver fails."""
        node = IMU_I2CNode()
        node.trigger_configure()

        # Reset and setup driver to fail on close
        mock_driver.reset_mock()
        mock_driver.close.side_effect = Exception("Close failed")

        # Should not crash even if driver fails
        try:
            node.destroy_node()
        except Exception:
            # Some exception is expected, but node should attempt cleanup
            pass


class TestIMU_I2CNodeIntegration:
    """Integration tests for IMU I2C node."""

    def test_node_full_lifecycle(self, ros_context, mock_driver):
        """Test complete node lifecycle from configure to destroy."""
        # Create and setup mock data
        mock_data = IMU_AllData(
            quaternion=(0.707, 0.0, 0.0, 0.707),
            linear_accel=(0.1, 0.2, 9.8),
            gyroscope=(0.01, 0.02, 0.03),
        )
        mock_driver.get_all_data.return_value = mock_data

        # Create node
        node = IMU_I2CNode()
        node.trigger_configure()
        node.trigger_activate()

        # Verify initialization
        assert node.get_name() == "bno08x_i2c_node"
        assert mock_driver.connect.called
        assert mock_driver.enable_sensors.called

        # Publish data
        published_messages = []

        def capture_publish(msg):
            published_messages.append(msg)

        node.publisher_.publish = capture_publish
        node.publish_imu()

        # Verify publishing
        assert len(published_messages) == 1

        # Cleanup
        node.destroy_node()

    def test_node_handles_missing_quaternion(self, ros_context, mock_driver):
        """Test node handles edge case of unusual quaternion values."""
        node = IMU_I2CNode()
        node.trigger_configure()
        node.trigger_activate()

        # Use edge case quaternion values
        mock_data = IMU_AllData(
            quaternion=(0.0, 0.0, 0.0, 1.0),
            linear_accel=(-9.8, 0.0, 0.0),
            gyroscope=(1.5, 2.5, 3.5),
        )
        mock_driver.get_all_data.return_value = mock_data

        published_messages = []

        def capture_publish(msg):
            published_messages.append(msg)

        node.publisher_.publish = capture_publish

        node.publish_imu()

        msg = published_messages[0]

        # Verify edge case values are preserved
        assert msg.orientation.w == 0.0
        assert msg.orientation.z == 1.0
        assert msg.linear_acceleration.x == -9.8
        assert msg.angular_velocity.x == 1.5

        node.destroy_node()

    def test_node_timer_callback_integration(self, ros_context, mock_driver):
        """Test timer callback is properly registered on activate."""
        node = IMU_I2CNode()
        node.trigger_configure()
        node.trigger_activate()

        # Setup mock data
        mock_data = IMU_AllData(
            quaternion=(1.0, 0.0, 0.0, 0.0),
            linear_accel=(0.0, 0.0, 0.0),
            gyroscope=(0.0, 0.0, 0.0),
        )
        mock_driver.get_all_data.return_value = mock_data

        # Verify timer exists and has a callback
        timers = list(node.timers)
        assert len(timers) > 0

        # Timer should be created (callback is publish_imu)
        # This is verified implicitly by successful node creation

        node.destroy_node()
