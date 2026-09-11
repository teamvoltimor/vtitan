"""
Tests for ROS2 BNO085 IMU RVC lifecycle node.

IMU_UART_RVCNode is a LifecycleNode: hardware connects in on_configure() and
publishing starts in on_activate(), so tests must drive those transitions
explicitly before exercising node behavior (deployed nodes do this
automatically via trigger_configure()/trigger_activate() in main()).

Run with: python -m pytest tests/ros2/test_imu_uart_rvc_node.py -v
"""

import logging
import sys
from unittest import mock

import pytest
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from sensor_msgs.msg import Imu

from src.hardware.imu.readings import QuaternionReading, RVCReading
from tests.ros2.common_imu_fixtures import (
    assert_creates_timer_on_activate,
    assert_node_configures_correctly,
    assert_publish_imu_noop_before_configure,
)
from tests.ros2.common_node_fixtures import wait_for_graph_entry

logger = logging.getLogger(__name__)


@pytest.fixture(autouse=True)
def mock_buildhat():
    """Mock buildhat before importing node."""
    sys.modules["buildhat"] = mock.MagicMock()
    yield
    if "buildhat" in sys.modules:
        del sys.modules["buildhat"]


@pytest.fixture()
def imu_rvc_node_class():
    """Import IMU_UART_RVCNode with a mocked driver instance.

    ``get_data`` needs a real ``RVCReading`` by default, not MagicMock's
    default return: the publish timer can legitimately fire mid-test (e.g.
    while a graph-discovery wait spins the node), and a bare MagicMock
    unpacks as an empty sequence via its default ``__iter__``, crashing
    ``publish_imu``'s ``qw, qx, qy, qz = data.quaternion`` with an uncaught
    ValueError. Individual tests still override this with their own
    ``return_value``/``side_effect`` where the reading's content matters.
    """
    driver_instance = mock.MagicMock()
    driver_instance.get_data.return_value = RVCReading(
        yaw_deg=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
        x_accel=0.0,
        y_accel=0.0,
        z_accel=0.0,
        quaternion=QuaternionReading(1.0, 0.0, 0.0, 0.0),
    )

    with mock.patch(
        "src.ros2.imu.bno08x.mcp2221.uart_rvc_node.IMU_UART_RVCDriver", return_value=driver_instance
    ):
        from src.ros2.imu.bno08x.mcp2221.uart_rvc_node import IMU_UART_RVCNode

        yield IMU_UART_RVCNode, driver_instance


class TestIMU_UART_RVCNodeInit:
    """Test IMU RVC node configuration."""

    def test_node_configures_correctly(self, ros_context, imu_rvc_node_class):
        """Test node configures correctly."""
        IMU_UART_RVCNode, _ = imu_rvc_node_class
        assert_node_configures_correctly(IMU_UART_RVCNode, "bno08x_uart_rvc_node")

    def test_node_creates_publisher(self, ros_context, imu_rvc_node_class):
        """Test node creates IMU publisher after activation (lifecycle publishers
        aren't advertised on the graph until the node activates)."""
        IMU_UART_RVCNode, _ = imu_rvc_node_class

        node = IMU_UART_RVCNode()
        node.trigger_configure()
        node.trigger_activate()
        assert node.publisher_ is not None
        topics = wait_for_graph_entry(
            node,
            lambda: dict(node.get_publisher_names_and_types_by_node(node.get_name(), "")),
            "/imu/data",
            timeout_sec=5.0,
        )
        assert "/imu/data" in topics
        node.destroy_node()

    def test_node_calls_driver_connect(self, ros_context, imu_rvc_node_class):
        """Test node calls driver connect during configure."""
        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class

        node = IMU_UART_RVCNode()
        node.trigger_configure()
        mock_driver_instance.connect.assert_called_once()
        node.destroy_node()

    def test_node_calls_driver_start_polling(self, ros_context, imu_rvc_node_class):
        """Test node calls driver start_polling during configure."""
        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class

        node = IMU_UART_RVCNode()
        node.trigger_configure()
        mock_driver_instance.start_polling.assert_called_once()
        node.destroy_node()

    def test_node_creates_timer_on_activate(self, ros_context, imu_rvc_node_class):
        """Test node creates publish timer on activate, not on configure."""
        IMU_UART_RVCNode, _ = imu_rvc_node_class
        assert_creates_timer_on_activate(IMU_UART_RVCNode)

    def test_node_degrades_gracefully_on_known_connection_error(self, ros_context, imu_rvc_node_class):
        """A clean IMUConnectionError (hardware absent) must not fail configure --
        IMU data is not critical for motor control."""
        from src.hardware.exceptions import IMUConnectionError

        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class
        mock_driver_instance.connect.side_effect = IMUConnectionError("/dev/ttyUSB0", "not found")

        node = IMU_UART_RVCNode()
        result = node.trigger_configure()

        assert result == TransitionCallbackReturn.SUCCESS
        assert node._hardware_ready is False
        node.destroy_node()

    def test_node_fails_configure_on_unexpected_connect_error(self, ros_context, imu_rvc_node_class):
        """An unexpected error type (not IMUConnectionError) is a real bug --
        configure must fail rather than silently degrade."""
        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class
        mock_driver_instance.connect.side_effect = RuntimeError("Connection failed")

        node = IMU_UART_RVCNode()
        result = node.trigger_configure()

        assert result == TransitionCallbackReturn.FAILURE
        node.destroy_node()

    def test_node_fails_configure_on_unexpected_polling_error(self, ros_context, imu_rvc_node_class):
        """An unexpected error type during start_polling also fails configure."""
        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class
        mock_driver_instance.start_polling.side_effect = RuntimeError("Polling failed")

        node = IMU_UART_RVCNode()
        result = node.trigger_configure()

        assert result == TransitionCallbackReturn.FAILURE
        node.destroy_node()


class TestIMU_UART_RVCNodePublishing:
    """Test IMU RVC node data publishing."""

    def test_publish_imu_with_valid_data(self, ros_context, imu_rvc_node_class):
        """Test publishing IMU message with valid data."""
        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class

        node = IMU_UART_RVCNode()
        node.trigger_configure()
        node.trigger_activate()

        # Create mock sensor data
        mock_data = RVCReading(
            yaw_deg=45.0,
            pitch_deg=10.0,
            roll_deg=-5.0,
            x_accel=0.1,
            y_accel=0.2,
            z_accel=9.8,
            quaternion=QuaternionReading(0.707, 0.0, 0.0, 0.707),
        )
        mock_driver_instance.get_data.return_value = mock_data

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
        assert msg.orientation.x == 0.0
        assert msg.orientation.y == 0.0
        assert msg.orientation.z == 0.707
        assert msg.orientation.w == 0.707

        # Verify linear acceleration
        assert msg.linear_acceleration.x == 0.1
        assert msg.linear_acceleration.y == 0.2
        assert msg.linear_acceleration.z == 9.8

        # Verify header
        assert msg.header.frame_id == "imu_link"

        node.destroy_node()

    def test_publish_imu_with_none_data(self, ros_context, imu_rvc_node_class):
        """Test publish_imu returns early if no data available."""
        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class

        node = IMU_UART_RVCNode()
        node.trigger_configure()
        node.trigger_activate()
        mock_driver_instance.get_data.return_value = None

        # Mock the publisher to track calls
        node.publisher_.publish = mock.MagicMock()

        # Call publish method
        node.publish_imu()

        # Verify message was NOT published
        node.publisher_.publish.assert_not_called()

        node.destroy_node()

    def test_publish_imu_noop_before_configure(self, ros_context, imu_rvc_node_class):
        """publish_imu must not raise if called before configure."""
        IMU_UART_RVCNode, _ = imu_rvc_node_class
        assert_publish_imu_noop_before_configure(IMU_UART_RVCNode)

    def test_publish_imu_sets_timestamp(self, ros_context, imu_rvc_node_class):
        """Test publish_imu sets message timestamp."""
        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class

        node = IMU_UART_RVCNode()
        node.trigger_configure()
        node.trigger_activate()

        mock_data = RVCReading(
            yaw_deg=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            x_accel=0.0,
            y_accel=0.0,
            z_accel=0.0,
            quaternion=QuaternionReading(1.0, 0.0, 0.0, 0.0),
        )
        mock_driver_instance.get_data.return_value = mock_data

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

    def test_publish_imu_sets_angular_velocity_covariance(self, ros_context, imu_rvc_node_class):
        """Test publish_imu sets angular velocity covariance to -1."""
        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class

        node = IMU_UART_RVCNode()
        node.trigger_configure()
        node.trigger_activate()

        mock_data = RVCReading(
            yaw_deg=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            x_accel=0.0,
            y_accel=0.0,
            z_accel=0.0,
            quaternion=QuaternionReading(1.0, 0.0, 0.0, 0.0),
        )
        mock_driver_instance.get_data.return_value = mock_data

        published_messages = []

        def capture_publish(msg):
            published_messages.append(msg)

        node.publisher_.publish = capture_publish

        node.publish_imu()

        assert len(published_messages) == 1
        msg = published_messages[0]

        # Verify angular velocity covariance is set to -1 (indicates data is not available)
        assert msg.angular_velocity_covariance[0] == -1.0

        node.destroy_node()

    def test_publish_imu_multiple_iterations(self, ros_context, imu_rvc_node_class):
        """Test publish_imu works correctly over multiple calls."""
        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class

        node = IMU_UART_RVCNode()
        node.trigger_configure()
        node.trigger_activate()

        mock_data_1 = RVCReading(
            yaw_deg=10.0,
            pitch_deg=5.0,
            roll_deg=2.0,
            x_accel=0.5,
            y_accel=1.5,
            z_accel=9.8,
            quaternion=QuaternionReading(0.9, 0.1, 0.2, 0.3),
        )
        mock_data_2 = RVCReading(
            yaw_deg=20.0,
            pitch_deg=10.0,
            roll_deg=4.0,
            x_accel=1.0,
            y_accel=2.0,
            z_accel=9.9,
            quaternion=QuaternionReading(0.8, 0.2, 0.3, 0.4),
        )

        mock_driver_instance.get_data.side_effect = [mock_data_1, mock_data_2]

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

        # Verify second message
        msg2 = published_messages[1]
        assert msg2.orientation.x == 0.2
        assert msg2.linear_acceleration.z == 9.9

        node.destroy_node()


class TestIMU_UART_RVCNodeCleanup:
    """Test IMU RVC node cleanup."""

    def test_node_calls_driver_close_on_destroy(self, ros_context, imu_rvc_node_class):
        """Test node closes driver on destroy."""
        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class

        node = IMU_UART_RVCNode()
        node.trigger_configure()
        node.destroy_node()
        mock_driver_instance.close.assert_called_once()

    def test_node_cleanup_sequence(self, ros_context, imu_rvc_node_class):
        """Test node cleanup sequence is correct."""
        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class

        node = IMU_UART_RVCNode()
        node.trigger_configure()

        # Reset the mock to clear init calls
        mock_driver_instance.reset_mock()

        # Destroy node
        node.destroy_node()

        # Verify driver.close() was called
        mock_driver_instance.close.assert_called_once()


class TestIMU_UART_RVCNodeIntegration:
    """Integration tests for IMU RVC node."""

    def test_node_full_lifecycle(self, ros_context, imu_rvc_node_class):
        """Test complete node lifecycle from configure to destroy."""
        IMU_UART_RVCNode, mock_driver_instance = imu_rvc_node_class

        mock_data = RVCReading(
            yaw_deg=45.0,
            pitch_deg=10.0,
            roll_deg=-5.0,
            x_accel=0.1,
            y_accel=0.2,
            z_accel=9.8,
            quaternion=QuaternionReading(0.707, 0.0, 0.0, 0.707),
        )
        mock_driver_instance.get_data.return_value = mock_data

        # Create node
        node = IMU_UART_RVCNode()
        node.trigger_configure()
        node.trigger_activate()

        # Verify initialization
        assert node.get_name() == "bno08x_uart_rvc_node"
        assert mock_driver_instance.connect.called
        assert mock_driver_instance.start_polling.called

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
        mock_driver_instance.close.assert_called_once()
