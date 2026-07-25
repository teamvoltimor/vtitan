"""Mock-hardware tests for state_machine_node — the real node deployed on the

Raspberry Pi 5 (registered console script, launched by rpi5_nodes.launch.py).
No real IMU/LiDAR/Hailo/button hardware is involved: sensor freshness is driven
by directly setting the node's tracking fields (exactly what the real sensor
callbacks would do), and button events are synthetic std_msgs/String messages
matching what button_node actually publishes.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import pytest
import rclpy
from std_msgs.msg import String

from src.hardware.button.event import ButtonEvent
from src.state_machine import RobotState

if TYPE_CHECKING:
    from ackermann_msgs.msg import AckermannDriveStamped


@pytest.fixture()
def ros_context():
    """Initialize and cleanup ROS2 context for each test."""
    try:
        rclpy.init()
        yield
        rclpy.shutdown()
    except Exception as e:
        pytest.skip(f"ROS2 initialization failed: {e}")


@pytest.fixture()
def state_machine_node_class():
    from voldemorbot_state_machine.state_machine_node import StateMachineNode

    return StateMachineNode


def _mark_all_sensors_ready(node) -> None:
    """Fast-forward the node past BOOT_CHECK without waiting on real sensors/IP."""
    now = time.time()
    node.imu_last_msg_time = now
    node.lidar_last_msg_time = now
    node.hailo_last_msg_time = now
    node.hailo_fps = 30.0
    node.ip_fetch_complete = True
    node.ip_address = "192.0.2.1"


class TestStateMachineNodeInit:
    def test_starts_in_boot_check(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        assert node.state_machine.current_state == RobotState.BOOT_CHECK
        node.destroy_node()

    def test_publishes_ackermann_cmd_on_the_real_topic(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        topics = dict(node.get_publisher_names_and_types_by_node(node.get_name(), ""))
        assert "/ackermann_cmd" in topics
        assert topics["/ackermann_cmd"] == ["ackermann_msgs/msg/AckermannDriveStamped"]
        node.destroy_node()

    def test_subscribes_to_button_event(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        subs = node.get_subscriptions_info_by_topic("/button/event")
        assert len(subs) == 1
        assert subs[0].topic_type == "std_msgs/msg/String"
        node.destroy_node()


class TestBootCheckTransition:
    def test_all_sensors_ready_transitions_to_ready(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        _mark_all_sensors_ready(node)

        node._handle_boot_check()

        assert node.state_machine.current_state == RobotState.READY
        node.destroy_node()

    def test_missing_lidar_blocks_transition(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        _mark_all_sensors_ready(node)
        node.lidar_last_msg_time = None

        node._handle_boot_check()

        assert node.state_machine.current_state == RobotState.BOOT_CHECK
        node.destroy_node()

    def test_stale_imu_blocks_transition(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        _mark_all_sensors_ready(node)
        node.imu_last_msg_time = time.time() - 10.0  # older than the 3s freshness window

        node._handle_boot_check()

        assert node.state_machine.current_state == RobotState.BOOT_CHECK
        node.destroy_node()


class TestButtonEventDrivenTransitions:
    """Pin the wire contract this node relies on from button_node (Pi Zero)."""

    def test_short_press_in_ready_starts_race(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        _mark_all_sensors_ready(node)
        node._handle_boot_check()
        assert node.state_machine.current_state == RobotState.READY

        msg = String()
        msg.data = ButtonEvent.SHORT_PRESS.value  # exactly what button_node publishes
        node._button_event_callback(msg)

        assert node.state_machine.current_state == RobotState.RACING
        assert node.race_start_time is not None
        node.destroy_node()

    def test_short_press_ignored_outside_ready(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()  # still in BOOT_CHECK

        msg = String()
        msg.data = ButtonEvent.SHORT_PRESS.value
        node._button_event_callback(msg)

        assert node.state_machine.current_state == RobotState.BOOT_CHECK
        node.destroy_node()

    def test_long_press_during_racing_is_emergency_stop(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        _mark_all_sensors_ready(node)
        node._handle_boot_check()
        node._button_event_callback(_string_msg(ButtonEvent.SHORT_PRESS.value))
        assert node.state_machine.current_state == RobotState.RACING

        published: list[AckermannDriveStamped] = []
        node.ackermann_pub.publish = published.append

        node._button_event_callback(_string_msg(ButtonEvent.LONG_PRESS.value))

        assert node.state_machine.current_state == RobotState.FINISHED
        assert len(published) == 1
        assert published[0].drive.speed == pytest.approx(0.0)
        assert published[0].drive.steering_angle == pytest.approx(0.0)
        node.destroy_node()


class TestRacingCompletion:
    def test_laps_completed_transitions_to_finished_and_stops(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        _mark_all_sensors_ready(node)
        node._handle_boot_check()
        node._button_event_callback(_string_msg(ButtonEvent.SHORT_PRESS.value))
        assert node.state_machine.current_state == RobotState.RACING

        published: list[AckermannDriveStamped] = []
        node.ackermann_pub.publish = published.append
        node.laps_completed = 3  # TARGET_LAPS

        node._handle_racing()

        assert node.state_machine.current_state == RobotState.FINISHED
        assert len(published) == 1
        assert published[0].drive.speed == pytest.approx(0.0)
        node.destroy_node()

    def test_finished_state_keeps_publishing_stop(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        _mark_all_sensors_ready(node)
        node._handle_boot_check()
        node._button_event_callback(_string_msg(ButtonEvent.SHORT_PRESS.value))
        node.laps_completed = 3
        node._handle_racing()
        assert node.state_machine.current_state == RobotState.FINISHED

        published: list[AckermannDriveStamped] = []
        node.ackermann_pub.publish = published.append

        node._handle_finished()

        assert len(published) == 1
        assert published[0].drive.speed == pytest.approx(0.0)
        assert published[0].drive.steering_angle == pytest.approx(0.0)
        node.destroy_node()


def _string_msg(data: str) -> String:
    msg = String()
    msg.data = data
    return msg
