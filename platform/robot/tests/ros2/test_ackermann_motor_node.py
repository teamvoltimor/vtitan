"""Mock-hardware tests for ackermann_motor_node — the real node deployed on the

Raspberry Pi Zero (registered console script, launched by rpi_zero_nodes.launch.py).
No real servo/dc_encoder/GPIO hardware is touched: the driver factory is mocked
so the test exercises the node's actual decode/clamp/watchdog logic against
fake drivers.
"""

from __future__ import annotations

import math
from unittest import mock

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped

from src.hardware.motors.enums import DriveBackend, SteeringBackend

# The mock motor config's own steering/speed parameters — module-level so
# every fixture and assertion that depends on them shares one source instead
# of repeating the literal and risking silent drift between them.
_MOCK_MAX_STEERING_DEG = 30.0
_MOCK_SPEED_SCALE = 30.0


@pytest.fixture()
def ros_context():
    """Initialize and cleanup ROS2 context for each test."""
    try:
        rclpy.init()
        yield
        rclpy.shutdown()
    except Exception as e:  # noqa: BLE001
        pytest.skip(f"ROS2 initialization failed: {e}")


@pytest.fixture()
def ackermann_node_class(monkeypatch):
    """Import AckermannMotorNode with a mocked Config and driver factory."""
    monkeypatch.setenv("STEERING_BACKEND", SteeringBackend.SERVO.value)
    monkeypatch.setenv("DRIVE_BACKEND", DriveBackend.DC_ENCODER.value)

    mock_config = mock.MagicMock()
    mock_config.steering.offset = 0.0
    mock_config.steering.max_steering_angle = _MOCK_MAX_STEERING_DEG
    mock_config.drive.reversed = False
    mock_config.drive.max_speed = 100
    mock_config.drive.speed_scale = _MOCK_SPEED_SCALE

    mock_steering = mock.MagicMock()
    mock_drive = mock.MagicMock()

    with (
        mock.patch("voldemorbot_robot.motors.ackermann_motor_node.Config", return_value=mock_config),
        mock.patch("voldemorbot_robot.motors.ackermann_motor_node._DriverFactory") as mock_factory_cls,
    ):
        mock_factory_cls.return_value.steering.return_value = mock_steering
        mock_factory_cls.return_value.drive.return_value = mock_drive

        from voldemorbot_robot.motors.ackermann_motor_node import AckermannMotorNode

        yield AckermannMotorNode, mock_steering, mock_drive, mock_config


class TestAckermannMotorNodeInit:
    def test_connects_and_centers_steering(self, ros_context, ackermann_node_class):
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()

        mock_steering.connect.assert_called_once()
        mock_steering.center_steering.assert_called_once()
        assert node.steering is mock_steering
        assert node.drive is mock_drive

        node.destroy_node()

    def test_subscribes_to_ackermann_cmd(self, ros_context, ackermann_node_class):
        AckermannMotorNode, _, _, _ = ackermann_node_class
        node = AckermannMotorNode()

        subs = node.get_subscriptions_info_by_topic("/ackermann_cmd")
        assert len(subs) == 1
        assert subs[0].topic_type == "ackermann_msgs/msg/AckermannDriveStamped"

        node.destroy_node()

    def test_driver_connect_failure_degrades_safely(self, ros_context, monkeypatch):
        """If the drivers fail to connect, the node must not crash — it disables itself."""
        monkeypatch.setenv("STEERING_BACKEND", SteeringBackend.SERVO.value)
        monkeypatch.setenv("DRIVE_BACKEND", DriveBackend.DC_ENCODER.value)
        mock_config = mock.MagicMock()
        mock_config.steering.offset = 0.0
        mock_config.steering.max_steering_angle = 30.0
        mock_config.drive.reversed = False
        mock_config.drive.max_speed = 100
        mock_config.drive.speed_scale = 30.0

        mock_steering = mock.MagicMock()
        mock_steering.connect.side_effect = RuntimeError("no such device")

        with (
            mock.patch("voldemorbot_robot.motors.ackermann_motor_node.Config", return_value=mock_config),
            mock.patch("voldemorbot_robot.motors.ackermann_motor_node._DriverFactory") as mock_factory_cls,
        ):
            mock_factory_cls.return_value.steering.return_value = mock_steering
            mock_factory_cls.return_value.drive.return_value = mock.MagicMock()

            from voldemorbot_robot.motors.ackermann_motor_node import AckermannMotorNode

            node = AckermannMotorNode()

        assert node.steering is None
        assert node.drive is None

        # A command arriving with no connected drivers must not raise.
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.3
        msg.drive.steering_angle = 0.1
        node._ackermann_callback(msg)

        node.destroy_node()


class TestAckermannMotorNodeDecode:
    """Pin the exact decode contract: radians -> degrees, offset, clamp, reversal."""

    def test_decodes_speed_and_steering(self, ros_context, ackermann_node_class):
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()

        msg = AckermannDriveStamped()
        msg.drive.speed = 0.3  # m/s
        msg.drive.steering_angle = math.radians(15.0)  # left

        node._ackermann_callback(msg)

        from voldemorbot_robot.motors.ackermann_motor_node import STEERING_COMMAND_SPEED

        mock_steering.move_steering_to.assert_called_once()
        angle_deg, kwargs = mock_steering.move_steering_to.call_args[0][0], mock_steering.move_steering_to.call_args[1]
        assert angle_deg == pytest.approx(15.0)
        assert kwargs["speed"] == STEERING_COMMAND_SPEED

        mock_drive.run_drive_forward.assert_called_once_with(pytest.approx(0.3 * _MOCK_SPEED_SCALE))
        mock_drive.run_drive_reverse.assert_not_called()
        mock_drive.stop_drive.assert_not_called()

        node.destroy_node()

    def test_negative_speed_reverses(self, ros_context, ackermann_node_class):
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()

        msg = AckermannDriveStamped()
        msg.drive.speed = -0.2
        msg.drive.steering_angle = 0.0

        node._ackermann_callback(msg)

        mock_drive.run_drive_reverse.assert_called_once_with(pytest.approx(abs(-0.2 * _MOCK_SPEED_SCALE)))
        mock_drive.run_drive_forward.assert_not_called()

        node.destroy_node()

    def test_zero_speed_stops(self, ros_context, ackermann_node_class):
        AckermannMotorNode, _, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()

        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0

        node._ackermann_callback(msg)

        mock_drive.stop_drive.assert_called_once()

        node.destroy_node()

    def test_steering_offset_applied(self, ros_context, ackermann_node_class):
        AckermannMotorNode, mock_steering, _, mock_config = ackermann_node_class
        mock_config.steering.offset = 5.0
        node = AckermannMotorNode()

        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0

        node._ackermann_callback(msg)

        angle_deg = mock_steering.move_steering_to.call_args[0][0]
        assert angle_deg == pytest.approx(5.0)

        node.destroy_node()

    def test_steering_clamped_to_max_angle(self, ros_context, ackermann_node_class):
        AckermannMotorNode, mock_steering, _, mock_config = ackermann_node_class
        node = AckermannMotorNode()

        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = math.radians(2 * _MOCK_MAX_STEERING_DEG)  # exceeds the configured limit

        node._ackermann_callback(msg)

        angle_deg = mock_steering.move_steering_to.call_args[0][0]
        assert angle_deg == pytest.approx(mock_config.steering.max_steering_angle)

        node.destroy_node()

    def test_drive_reversed_flips_direction(self, ros_context, ackermann_node_class):
        AckermannMotorNode, _, mock_drive, mock_config = ackermann_node_class
        mock_config.drive.reversed = True
        node = AckermannMotorNode()

        msg = AckermannDriveStamped()
        msg.drive.speed = 0.3
        msg.drive.steering_angle = 0.0

        node._ackermann_callback(msg)

        # A forward command on a reversed motor must run the reverse output.
        mock_drive.run_drive_reverse.assert_called_once()
        mock_drive.run_drive_forward.assert_not_called()

        node.destroy_node()


class TestAckermannMotorNodeWatchdog:
    def test_stops_motors_after_command_dropout(self, ros_context, ackermann_node_class):
        AckermannMotorNode, _, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()

        msg = AckermannDriveStamped()
        msg.drive.speed = 0.3
        msg.drive.steering_angle = 0.0
        node._ackermann_callback(msg)
        mock_drive.reset_mock()

        # Simulate the last command having arrived over 1s ago.
        node.last_command_time = node.get_clock().now().nanoseconds / 1e9 - 2.0
        node._watchdog_check()

        mock_drive.stop_drive.assert_called_once()
        assert node.current_speed == 0.0

        node.destroy_node()

    def test_no_watchdog_action_when_already_stopped(self, ros_context, ackermann_node_class):
        AckermannMotorNode, _, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()

        node.last_command_time = node.get_clock().now().nanoseconds / 1e9 - 2.0
        node.current_speed = 0.0
        node._watchdog_check()

        mock_drive.stop_drive.assert_not_called()

        node.destroy_node()
