"""Mock-hardware tests for state_machine_node — the real node deployed on the

Raspberry Pi 5 (registered console script, launched by rpi5_nodes.launch.py).
No real IMU/LiDAR/Hailo/button hardware is involved: sensor freshness is driven
by directly setting the node's tracking fields (exactly what the real sensor
callbacks would do), and button events are synthetic std_msgs/String messages
matching what button_node actually publishes.
"""

from __future__ import annotations

import math
import time

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
from shared.config.constants import CompetitionSpecs
from std_msgs.msg import Bool, Int32, String

from src.hardware.button.event import ButtonEvent
from src.state_machine import RobotState, ScenarioType


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
    from vtitan_state_machine.state_machine_node import StateMachineNode

    return StateMachineNode


def _publish_jumper(node, *, inserted: bool) -> None:
    """Deliver a jumper reading the way the Pi Zero does.

    The jumper is wired to the ZERO's GPIO23, so the node consumes
    /challenge_mode/jumper_inserted rather than reading GPIO locally. These
    tests previously stubbed a local challenge_mode_driver, which stopped
    existing when that moved -- and reading Pi 5's GPIO23 (nothing attached,
    internal pull-up, always HIGH) had silently latched Open Challenge on every
    boot, so Obstacles could never be selected.
    """
    node._on_jumper_state(Bool(data=inserted))


def _mark_all_sensors_ready(node) -> None:
    """Fast-forward the node past BOOT_CHECK without waiting on real sensors/IP/jumper."""
    now = time.time()
    node.imu_last_msg_time = now
    node.lidar_last_msg_time = now
    node.hailo_last_msg_time = now
    node.hailo_fps = 30.0
    node.ip_fetch_complete = True
    node.ip_address = "192.0.2.1"
    node.challenge_mode = ScenarioType.OPEN


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

    def test_undetected_challenge_mode_blocks_transition(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        _mark_all_sensors_ready(node)
        node.challenge_mode = None  # jumper reading hasn't stabilized yet

        node._handle_boot_check()

        assert node.state_machine.current_state == RobotState.BOOT_CHECK
        node.destroy_node()


class TestChallengeModeDetection:
    """Pin the fail-closed sampling behavior from the jumper spec (no blocking sleeps)."""

    def test_stable_low_reading_selects_obstacles_and_its_lap_count(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        _publish_jumper(node, inserted=True)  # shorted to GND

        for _ in range(3):
            node._sample_challenge_mode()

        assert node.challenge_mode == ScenarioType.OBSTACLES
        assert node.target_laps == CompetitionSpecs.OBSTACLE_CHALLENGE_LAPS
        node.destroy_node()

    def test_stable_high_reading_selects_open(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        _publish_jumper(node, inserted=False)  # pulled up, no jumper

        for _ in range(3):
            node._sample_challenge_mode()

        assert node.challenge_mode == ScenarioType.OPEN
        assert node.target_laps == CompetitionSpecs.OPEN_CHALLENGE_LAPS
        node.destroy_node()

    def test_bouncing_reading_does_not_resolve_before_timeout(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        for inserted in (True, False, True, False, True, False):
            _publish_jumper(node, inserted=inserted)
            node._sample_challenge_mode()

        assert node.challenge_mode is None
        node.destroy_node()

    def test_bouncing_reading_times_out_to_open_challenge(self, ros_context, state_machine_node_class, monkeypatch):
        """A jumper that never settles must eventually fall back rather than block BOOT_CHECK forever.

        A floating (open) pin is weakly pulled up and far more noise-susceptible
        than a solid short to GND, so persistent bounce shows up specifically
        when the jumper is absent -- exactly the case this fallback covers.
        """
        node = state_machine_node_class()
        monkeypatch.setattr(node, "_challenge_mode_timed_out", lambda: True)

        for inserted in (True, False, True):
            _publish_jumper(node, inserted=inserted)
            node._sample_challenge_mode()

        assert node.challenge_mode == ScenarioType.OPEN
        assert node._challenge_mode_error is not None
        node.destroy_node()

    def test_explicit_target_laps_param_is_not_overridden_by_detection(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        node.target_laps = 5  # simulates an explicit launch-time override
        node._target_laps_explicit = True
        _publish_jumper(node, inserted=True)

        for _ in range(3):
            node._sample_challenge_mode()

        assert node.challenge_mode == ScenarioType.OBSTACLES
        assert node.target_laps == 5
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
        # 0 first: the navigator's post-reset publish, which is what marks the
        # count as belonging to this race.
        _report_laps(node, 0, 3)  # TARGET_LAPS

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
        _report_laps(node, 0, 3)
        node._handle_racing()
        assert node.state_machine.current_state == RobotState.FINISHED

        published: list[AckermannDriveStamped] = []
        node.ackermann_pub.publish = published.append

        node._handle_finished()

        assert len(published) == 1
        assert published[0].drive.speed == pytest.approx(0.0)
        assert published[0].drive.steering_angle == pytest.approx(0.0)
        node.destroy_node()


class TestRerunDoesNotInheritThePreviousRacesLaps:
    """A button-cycled re-run must start at zero laps, not the last race's total.

    Measured on hardware 2026-08-06: after a finished round, resetting and
    pressing start dropped straight back to FINISHED, and only a reboot cleared
    it. track_navigator_node publishes its lap count on every control tick
    regardless of state, so the finished race's total keeps arriving while this
    node sits in BOOT_CHECK/READY -- and _handle_racing runs on this node's own
    tick, which beats the round trip that would have delivered the navigator's
    post-reset zero.
    """

    def _finished_node(self, node_class):
        node = node_class()
        _mark_all_sensors_ready(node)
        node._handle_boot_check()
        node._button_event_callback(_string_msg(ButtonEvent.SHORT_PRESS.value))
        _report_laps(node, 0, 3)
        node._handle_racing()
        assert node.state_machine.current_state == RobotState.FINISHED
        return node

    def test_stale_count_during_rerun_does_not_finish_the_race(self, ros_context, state_machine_node_class):
        node = self._finished_node(state_machine_node_class)

        node._button_event_callback(_string_msg(ButtonEvent.LONG_PRESS.value))
        assert node.state_machine.current_state == RobotState.BOOT_CHECK
        # The navigator has not reset yet -- it resets on entering RACING -- so
        # it is still publishing the finished race's total.
        _report_laps(node, 3, 3)
        _mark_all_sensors_ready(node)
        node._handle_boot_check()
        node._button_event_callback(_string_msg(ButtonEvent.SHORT_PRESS.value))
        assert node.state_machine.current_state == RobotState.RACING

        # One more stale sample lands before the navigator's reset propagates.
        _report_laps(node, 3)
        node._handle_racing()

        assert node.state_machine.current_state == RobotState.RACING, (
            "the previous race's lap count finished the new race before it started"
        )
        assert node.laps_completed == 0
        node.destroy_node()

    def test_race_still_finishes_once_the_navigator_reports_afresh(self, ros_context, state_machine_node_class):
        node = self._finished_node(state_machine_node_class)
        node._button_event_callback(_string_msg(ButtonEvent.LONG_PRESS.value))
        _report_laps(node, 3)
        _mark_all_sensors_ready(node)
        node._handle_boot_check()
        node._button_event_callback(_string_msg(ButtonEvent.SHORT_PRESS.value))

        # The navigator resets and its zero arrives, re-arming the count.
        _report_laps(node, 0)
        node._handle_racing()
        assert node.state_machine.current_state == RobotState.RACING

        _report_laps(node, 1, 2, 3)
        node._handle_racing()

        assert node.state_machine.current_state == RobotState.FINISHED
        node.destroy_node()

    def test_first_race_after_boot_is_not_gated(self, ros_context, state_machine_node_class):
        """Nothing stale exists yet, and the navigator's very first publish is a
        genuine zero -- so the gate must not require a spurious extra sample."""
        node = state_machine_node_class()
        _mark_all_sensors_ready(node)
        node._handle_boot_check()
        node._button_event_callback(_string_msg(ButtonEvent.SHORT_PRESS.value))
        _report_laps(node, 0, 1, 2, 3)

        node._handle_racing()

        assert node.state_machine.current_state == RobotState.FINISHED
        node.destroy_node()


class TestRaceMetricsReflectRealTelemetry:
    """gyro_yaw/current_velocity/current_steering used to be dead: gyro_yaw was
    hardcoded to 0.0 in _imu_callback regardless of the message, and the other
    two were only ever set by _publish_stop_command (also to 0.0) -- /race_metrics
    reported zero for the entire race. Fixed 2026-08-03.
    """

    def test_imu_callback_derives_yaw_from_the_real_quaternion(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        msg = Imu()
        # Pure 90 deg yaw rotation: q = (0, 0, sin(yaw/2), cos(yaw/2)).
        msg.orientation.z = math.sin(math.radians(90) / 2)
        msg.orientation.w = math.cos(math.radians(90) / 2)

        node._imu_callback(msg)

        assert node.gyro_yaw == pytest.approx(90.0)
        node.destroy_node()

    def test_ackermann_callback_mirrors_last_commanded_drive(self, ros_context, state_machine_node_class):
        node = state_machine_node_class()
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.30
        msg.drive.steering_angle = math.radians(15.0)

        node._ackermann_callback(msg)

        assert node.current_velocity == pytest.approx(0.30)
        assert node.current_steering == pytest.approx(15.0)
        node.destroy_node()


def _string_msg(data: str) -> String:
    msg = String()
    msg.data = data
    return msg


def _report_laps(node, *counts: int) -> None:
    """Deliver lap counts the way track_navigator_node does.

    Setting ``node.laps_completed`` directly models a count that no navigator
    ever published, which is the one state the completion gate exists to reject
    -- so tests that mean "the navigator says N laps are done" have to arrive
    through the subscription like the real thing.
    """
    for count in counts:
        node._on_laps_completed(Int32(data=count))
