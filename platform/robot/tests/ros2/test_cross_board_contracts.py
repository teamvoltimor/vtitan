"""Cross-board integration tests: RPi 5 <-> RPi Zero over their real message contracts.

The two boards only ever talk to each other through ROS2 topics (the "network"
in "Run on: Raspberry Pi Zero (connected to Raspberry Pi 5 via network)"). These
tests compose the REAL producer method on one board with the REAL consumer
method on the other — no real DDS transport, no real hardware — so a
topic/type/field/unit contract drift between them fails a test instead of
surfacing for the first time on the physical robot (exactly what happened with
the Twist/cmd_vel vs AckermannDriveStamped/ackermann_cmd mismatch this session
found and fixed).
"""

from __future__ import annotations

import math
from unittest import mock

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from shared.config.constants import RobotSpecs
from shared.config.enums import Section
from shared.domain.steering import steering_norm_to_angle_rad
from std_msgs.msg import String

from src.hardware.button.event import ButtonEvent
from src.hardware.button.state import ButtonState
from src.hardware.motors.enums import DriveBackend, SteeringBackend
from src.navigation.ports import DriveCommand
from src.ros2.navigation.node import ROS2HardwareGateway

_WIDTHS = {Section.NORTH: 1.0, Section.SOUTH: 1.0, Section.EAST: 1.0, Section.WEST: 1.0}

# The mock motor config's own steering limit (degrees) — shared between the
# fixture and the clamp assertion below so they can't silently drift apart.
_MOCK_MAX_STEERING_DEG = 30.0


@pytest.fixture()
def ros_context():
    """Initialize and cleanup ROS2 context for each test."""
    try:
        rclpy.init()
        yield
        rclpy.shutdown()
    except Exception as e:  # noqa: BLE001
        pytest.skip(f"ROS2 initialization failed: {e}")


def _make_navigator_host_node():
    from rclpy.node import Node

    node = Node("test_navigator_side")
    node.declare_parameter("ackermann_cmd_topic", "/ackermann_cmd")
    node.declare_parameter("lidar_topic", "/scan")
    node.declare_parameter("vision_topic", "/vision/detections")
    node.declare_parameter("imu_topic", "/imu/data")
    return node


def _make_motor_node():
    mock_config = mock.MagicMock()
    mock_config.steering.offset = 0.0
    mock_config.steering.max_steering_angle = _MOCK_MAX_STEERING_DEG
    mock_config.drive.reversed = False
    mock_config.drive.max_speed = 100
    mock_config.drive.speed_scale = 30.0

    mock_steering = mock.MagicMock()
    mock_drive = mock.MagicMock()

    with (
        mock.patch("voldemorbot_robot.motors.ackermann_motor_node.Config", return_value=mock_config),
        mock.patch("voldemorbot_robot.motors.ackermann_motor_node._DriverFactory") as mock_factory_cls,
    ):
        mock_factory_cls.return_value.steering.return_value = mock_steering
        mock_factory_cls.return_value.drive.return_value = mock_drive

        from voldemorbot_robot.motors.ackermann_motor_node import AckermannMotorNode

        return AckermannMotorNode(), mock_steering, mock_drive


class TestNavigatorToMotorNode:
    """RPi 5 (track_navigator_node) -> RPi Zero (ackermann_motor_node)."""

    @pytest.mark.parametrize(
        ("speed_mps", "steering_norm"),
        [
            (0.3, 0.0),
            (0.3, 0.5),  # left
            (0.3, -0.5),  # right
            (-0.2, 0.0),  # reverse
            (0.0, 0.0),  # stop
            (0.5, 1.0),  # full left, full speed
        ],
    )
    def test_drive_command_round_trips_correctly(self, ros_context, speed_mps, steering_norm, monkeypatch):
        monkeypatch.setenv("STEERING_BACKEND", SteeringBackend.SERVO.value)
        monkeypatch.setenv("DRIVE_BACKEND", DriveBackend.DC_ENCODER.value)

        nav_node = _make_navigator_host_node()
        gateway = ROS2HardwareGateway(nav_node, 0.0, 0.0, 0.0, _WIDTHS)

        published: list[AckermannDriveStamped] = []
        gateway._drive_publisher.publish = published.append
        gateway.publish_drive(DriveCommand(speed_mps=speed_mps, steering_norm=steering_norm))
        assert len(published) == 1

        motor_node, mock_steering, mock_drive = _make_motor_node()
        motor_node._ackermann_callback(published[0])

        expected_angle_deg = math.degrees(
            steering_norm_to_angle_rad(steering_norm, RobotSpecs.MAX_STEERING_ANGLE),
        )
        # The motor node clamps to its own configured max_steering_angle,
        # which can differ from RobotSpecs.MAX_STEERING_ANGLE (0.5236 rad) by
        # a fraction of a degree — clamp the expected value the same way the
        # real node does rather than asserting exact equality.
        expected_angle_deg = max(-_MOCK_MAX_STEERING_DEG, min(_MOCK_MAX_STEERING_DEG, expected_angle_deg))
        got_angle_deg = mock_steering.move_steering_to.call_args[0][0]
        assert got_angle_deg == pytest.approx(expected_angle_deg)

        # Sign convention: positive steering_norm (left) must decode to a
        # positive wheel angle here too — both boards agree "+ = left".
        if steering_norm > 0:
            assert got_angle_deg > 0
        elif steering_norm < 0:
            assert got_angle_deg < 0

        if speed_mps > 0:
            mock_drive.run_drive_forward.assert_called_once()
            mock_drive.run_drive_reverse.assert_not_called()
        elif speed_mps < 0:
            mock_drive.run_drive_reverse.assert_called_once()
            mock_drive.run_drive_forward.assert_not_called()
        else:
            mock_drive.stop_drive.assert_called_once()

        nav_node.destroy_node()
        motor_node.destroy_node()


class TestStateMachineStopToMotorNode:
    """RPi 5 (state_machine_node's e-stop) -> RPi Zero (ackermann_motor_node)."""

    def test_estop_stop_command_stops_the_motors(self, ros_context, monkeypatch):
        monkeypatch.setenv("STEERING_BACKEND", SteeringBackend.SERVO.value)
        monkeypatch.setenv("DRIVE_BACKEND", DriveBackend.DC_ENCODER.value)

        from voldemorbot_robot.state_machine_node import StateMachineNode

        sm_node = StateMachineNode()
        published: list[AckermannDriveStamped] = []
        sm_node.ackermann_pub.publish = published.append
        sm_node._publish_stop_command()
        assert len(published) == 1

        # The motor node must already have seen a nonzero command, so that a
        # stop is an observable change (not a no-op it would send regardless).
        motor_node, mock_steering, mock_drive = _make_motor_node()
        driving_msg = AckermannDriveStamped()
        driving_msg.drive.speed = 0.3
        driving_msg.drive.steering_angle = 0.2
        motor_node._ackermann_callback(driving_msg)
        mock_steering.reset_mock()
        mock_drive.reset_mock()

        motor_node._ackermann_callback(published[0])

        mock_drive.stop_drive.assert_called_once()
        angle_deg = mock_steering.move_steering_to.call_args[0][0]
        assert angle_deg == pytest.approx(0.0)

        sm_node.destroy_node()
        motor_node.destroy_node()


class TestButtonNodeToStateMachine:
    """RPi Zero (button_node) -> RPi 5 (state_machine_node)."""

    def test_short_press_event_starts_the_race(self, ros_context):
        from voldemorbot_robot.state_machine_node import StateMachineNode

        mock_button_driver = mock.MagicMock()
        with mock.patch("voldemorbot_robot.button_node.ButtonDriver", return_value=mock_button_driver):
            from voldemorbot_robot.button_node import ButtonNode

            button_node = ButtonNode()

        mock_button_driver.get_state.return_value = ButtonState(
            is_pressed=False, press_duration=0.0, last_event=ButtonEvent.SHORT_PRESS,
        )
        published: list[String] = []
        button_node.pub.publish = published.append
        button_node._poll()
        assert len(published) == 1

        sm_node = StateMachineNode()
        now = __import__("time").time()
        sm_node.imu_last_msg_time = now
        sm_node.lidar_last_msg_time = now
        sm_node.hailo_last_msg_time = now
        sm_node.hailo_fps = 30.0
        sm_node.ip_fetch_complete = True
        sm_node._handle_boot_check()

        from src.state_machine import RobotState

        assert sm_node.state_machine.current_state == RobotState.READY

        sm_node._button_event_callback(published[0])

        assert sm_node.state_machine.current_state == RobotState.RACING

        button_node.destroy_node()
        sm_node.destroy_node()
