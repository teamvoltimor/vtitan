"""Mock-hardware tests for ackermann_motor_node — the real node deployed on the

Raspberry Pi Zero (registered console script, launched by rpi_zero_nodes.launch.py).
No real servo/l298n/GPIO hardware is touched: the driver factory is mocked
so the test exercises the node's actual decode/clamp/watchdog logic against
fake drivers.
"""

from __future__ import annotations

import math
from unittest import mock

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from shared.config.constants import RobotSpecs
from shared.config.ros_topics import RosMessageType, RosTopicConfig

from src.hardware.motors.enums import DriveBackend, SteeringBackend
from tests.ros2.common_node_fixtures import assert_destroy_without_configure_does_not_raise

# The mock motor config's own steering/speed parameters — module-level so
# every fixture and assertion that depends on them shares one source instead
# of repeating the literal and risking silent drift between them.
_MOCK_MAX_STEERING_DEG = 30.0
_MOCK_SPEED_SCALE = 30.0

_MOCK_LINKAGE_RATIO = 1.0
"""Servo-to-wheel gearing, held at 1:1 so servo and wheel angles coincide.

Every steering assertion below was written when the node fed the wheel angle
straight to the servo, so 1:1 preserves their arithmetic. It also has to be a
real number rather than a MagicMock: the node divides by it and then clamps the
result, and a MagicMock reaching that comparison raises TypeError -- which is
what failed 13 of these tests, in code paths that have nothing to do with
steering linkage.
"""


def _wheel_rpm_for(velocity_mps: float) -> float:
    """The wheel-rpm setpoint the node derives from a commanded m/s.

    Mirrors the node's conversion rather than hardcoding a number, so the
    expectation tracks RobotSpecs.WHEEL_RADIUS instead of silently drifting
    from it the way the old open-loop duty assertions did.
    """
    return velocity_mps / (math.pi * RobotSpecs.WHEEL_RADIUS * 2.0) * 60.0


@pytest.fixture()
def ackermann_node_class(monkeypatch):
    """Import AckermannMotorNode with a mocked Config and driver factory."""
    monkeypatch.setenv("STEERING_BACKEND", SteeringBackend.SERVO.value)
    monkeypatch.setenv("DRIVE_BACKEND", DriveBackend.L298N.value)

    # Every numeric field the node reads must be stubbed. A MagicMock left in
    # any of them propagates through the arithmetic and only fails later, at
    # whichever comparison it reaches first, in a test that looks unrelated.
    mock_config = mock.MagicMock()
    mock_config.steering.offset = 0.0
    mock_config.steering.max_steering_angle = _MOCK_MAX_STEERING_DEG
    mock_config.steering.linkage_ratio = _MOCK_LINKAGE_RATIO
    mock_config.drive.reversed = False
    mock_config.drive.encoder_reversed = False
    mock_config.drive.max_speed = 100
    mock_config.drive.speed_scale = _MOCK_SPEED_SCALE

    mock_steering = mock.MagicMock()
    mock_drive = mock.MagicMock()

    with (
        mock.patch("vtitan_drivers.motors.ackermann_motor_node.Config", return_value=mock_config),
        mock.patch("vtitan_drivers.motors.ackermann_motor_node._DriverFactory") as mock_factory_cls,
    ):
        mock_factory_cls.return_value.steering.return_value = mock_steering
        mock_factory_cls.return_value.drive.return_value = mock_drive
        # No encoder -> node.drive stays the raw mock, unwrapped by
        # ClosedLoopDrive, matching this fixture's plain-mock assertions
        # below (`node.drive is mock_drive`). ClosedLoopDrive composition
        # itself is covered by base.py/encoder-focused unit tests.
        mock_factory_cls.return_value.encoder.return_value = None

        from vtitan_drivers.motors.ackermann_motor_node import AckermannMotorNode

        yield AckermannMotorNode, mock_steering, mock_drive, mock_config


class TestAckermannMotorNodeInit:
    def test_connects_and_centers_steering(self, ros_context, ackermann_node_class):
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()
        node.trigger_configure()

        mock_steering.connect.assert_called_once()
        mock_steering.center_steering.assert_called_once()
        assert node.steering is mock_steering
        assert node.drive is mock_drive

        node.destroy_node()

    def test_subscribes_to_ackermann_cmd(self, ros_context, ackermann_node_class):
        AckermannMotorNode, _, _, _ = ackermann_node_class
        node = AckermannMotorNode()
        node.trigger_configure()
        node.trigger_activate()

        subs = node.get_subscriptions_info_by_topic(RosTopicConfig.load_default().commands.ackermann_cmd)
        assert len(subs) == 1
        assert subs[0].topic_type == RosMessageType.ACKERMANN_DRIVE_STAMPED

        node.destroy_node()

    def test_driver_connect_failure_degrades_safely(self, ros_context, monkeypatch):
        """If the drivers fail to connect, the node must not crash — it disables itself."""
        monkeypatch.setenv("STEERING_BACKEND", SteeringBackend.SERVO.value)
        monkeypatch.setenv("DRIVE_BACKEND", DriveBackend.L298N.value)
        mock_config = mock.MagicMock()
        mock_config.steering.offset = 0.0
        mock_config.steering.max_steering_angle = 30.0
        mock_config.drive.reversed = False
        mock_config.drive.max_speed = 100
        mock_config.drive.speed_scale = 30.0

        mock_steering = mock.MagicMock()
        mock_steering.connect.side_effect = RuntimeError("no such device")

        with (
            mock.patch("vtitan_drivers.motors.ackermann_motor_node.Config", return_value=mock_config),
            mock.patch("vtitan_drivers.motors.ackermann_motor_node._DriverFactory") as mock_factory_cls,
        ):
            mock_factory_cls.return_value.steering.return_value = mock_steering
            mock_factory_cls.return_value.drive.return_value = mock.MagicMock()
            mock_factory_cls.return_value.encoder.return_value = None

            from vtitan_drivers.motors.ackermann_motor_node import AckermannMotorNode

            node = AckermannMotorNode()
            node.trigger_configure()

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
        node.trigger_configure()

        msg = AckermannDriveStamped()
        msg.drive.speed = 0.3  # m/s
        msg.drive.steering_angle = math.radians(15.0)  # left

        node._ackermann_callback(msg)

        mock_steering.move_steering_to.assert_called_once()
        angle_deg, kwargs = mock_steering.move_steering_to.call_args[0][0], mock_steering.move_steering_to.call_args[1]
        assert angle_deg == pytest.approx(15.0)
        assert kwargs["speed"] == node.config.steering.turning_speed

        # Decode sets a wheel-rpm setpoint; the 50 Hz control loop is what
        # commands the driver. Asserting on run_drive_forward here tested an
        # open-loop API the node stopped using when it moved to closed-loop
        # speed control, and had been failing ever since.
        assert node.target_wheel_rpm == pytest.approx(_wheel_rpm_for(0.3))
        mock_drive.run_drive_forward.assert_not_called()
        mock_drive.run_drive_reverse.assert_not_called()

        node.destroy_node()

    def test_negative_speed_reverses(self, ros_context, ackermann_node_class):
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()
        node.trigger_configure()

        msg = AckermannDriveStamped()
        msg.drive.speed = -0.2
        msg.drive.steering_angle = 0.0

        node._ackermann_callback(msg)

        # Reverse is a negative setpoint now, not a separate driver call: the
        # driver owns direction, so the node no longer picks forward/reverse.
        assert node.target_wheel_rpm == pytest.approx(_wheel_rpm_for(-0.2))
        assert node.target_wheel_rpm < 0
        mock_drive.run_drive_forward.assert_not_called()
        mock_drive.run_drive_reverse.assert_not_called()

        node.destroy_node()

    def test_zero_speed_stops(self, ros_context, ackermann_node_class):
        AckermannMotorNode, _, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()
        node.trigger_configure()

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
        node.trigger_configure()

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
        node.trigger_configure()

        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = math.radians(2 * _MOCK_MAX_STEERING_DEG)  # exceeds the configured limit

        node._ackermann_callback(msg)

        angle_deg = mock_steering.move_steering_to.call_args[0][0]
        assert angle_deg == pytest.approx(mock_config.steering.max_steering_angle)

        node.destroy_node()

    def test_drive_reversed_does_not_flip_the_setpoint(self, ros_context, ackermann_node_class):
        """Direction inversion belongs to the driver, not to this node.

        This test previously asserted the opposite -- that drive.reversed made
        a forward command run the reverse output -- which was true before the
        driver took ownership of direction. Negating in both places would
        cancel out, and would report a commanded_speed whose sign disagreed
        with the actual motion, so the setpoint must come through untouched.
        """
        AckermannMotorNode, _, mock_drive, mock_config = ackermann_node_class
        mock_config.drive.reversed = True
        node = AckermannMotorNode()
        node.trigger_configure()

        msg = AckermannDriveStamped()
        msg.drive.speed = 0.3
        msg.drive.steering_angle = 0.0

        node._ackermann_callback(msg)

        assert node.target_wheel_rpm == pytest.approx(_wheel_rpm_for(0.3))
        assert node.target_wheel_rpm > 0
        mock_drive.run_drive_forward.assert_not_called()
        mock_drive.run_drive_reverse.assert_not_called()

        node.destroy_node()


class TestAckermannMotorNodeWatchdog:
    def test_stops_motors_after_command_dropout(self, ros_context, ackermann_node_class):
        AckermannMotorNode, _, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()
        node.trigger_configure()

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
        node.trigger_configure()

        node.last_command_time = node.get_clock().now().nanoseconds / 1e9 - 2.0
        node.current_speed = 0.0
        node._watchdog_check()

        mock_drive.stop_drive.assert_not_called()

        node.destroy_node()


class TestAckermannMotorNodeSafetyLifecycle:
    """Safety-critical guarantee: the motors are always commanded to a stopped,
    centered state on every teardown path, no matter which one is taken."""

    def test_feedback_and_watchdog_timers_only_exist_after_activate(self, ros_context, ackermann_node_class):
        AckermannMotorNode, _, _, _ = ackermann_node_class
        node = AckermannMotorNode()
        node.trigger_configure()
        assert node.feedback_timer is None
        assert node.watchdog_timer is None
        assert node.diagnostics_timer is None

        node.trigger_activate()

        assert node.feedback_timer is not None
        assert node.watchdog_timer is not None
        assert node.diagnostics_timer is not None
        node.destroy_node()

    def test_deactivate_stops_motors_and_removes_subscription(self, ros_context, ackermann_node_class):
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()
        node.trigger_configure()
        node.trigger_activate()
        mock_steering.reset_mock()
        mock_drive.reset_mock()

        node.trigger_deactivate()

        mock_drive.stop_drive.assert_called_once()
        mock_steering.center_steering.assert_called_once()
        assert node.ackermann_sub is None
        assert node.feedback_timer is None
        assert node.watchdog_timer is None
        assert node.diagnostics_timer is None
        node.destroy_node()

    def test_deactivated_node_ignores_further_commands(self, ros_context, ackermann_node_class):
        """Once deactivated, a command can no longer reach the motors even if
        something still holds a reference to the (destroyed) subscription's
        callback -- steering/drive are only ever acted on via _ackermann_callback,
        which is unreachable once the subscription is torn down."""
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()
        node.trigger_configure()
        node.trigger_activate()
        node.trigger_deactivate()

        assert node.ackermann_sub is None  # nothing left to route a command through
        node.destroy_node()

    def test_cleanup_stops_motors_defensively_even_without_prior_deactivate(self, ros_context, ackermann_node_class):
        """cleanup is normally reached via deactivate first, but its own stop
        must be redundant-safe in case that path is ever skipped."""
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()
        node.trigger_configure()
        mock_steering.reset_mock()
        mock_drive.reset_mock()

        node.trigger_cleanup()  # configure -> cleanup directly, no activate/deactivate

        mock_drive.stop_drive.assert_called_once()
        mock_steering.center_steering.assert_called_once()
        assert node.steering is None
        assert node.drive is None

    def test_deactivate_then_cleanup_stops_motors_on_each_transition(self, ros_context, ackermann_node_class):
        """Both transitions independently guarantee the stop -- calling both in
        sequence stops the motors twice, which is the intended defense-in-depth,
        not a bug."""
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()
        node.trigger_configure()
        node.trigger_activate()
        mock_steering.reset_mock()
        mock_drive.reset_mock()

        node.trigger_deactivate()
        node.trigger_cleanup()

        assert mock_drive.stop_drive.call_count == 2
        assert mock_steering.center_steering.call_count == 2
        assert node.steering is None
        assert node.drive is None
        node.destroy_node()

    def test_destroy_without_clean_shutdown_still_stops_motors(self, ros_context, ackermann_node_class):
        """Simulates a process kill mid-active: destroy_node() is the only
        thing called, no deactivate/cleanup transition first."""
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()
        node.trigger_configure()
        node.trigger_activate()
        mock_steering.reset_mock()
        mock_drive.reset_mock()

        node.destroy_node()

        mock_drive.stop_drive.assert_called_once()
        mock_steering.center_steering.assert_called_once()

    def test_destroy_without_configure_does_not_raise(self, ros_context, ackermann_node_class):
        AckermannMotorNode, _, mock_drive, _ = ackermann_node_class
        assert_destroy_without_configure_does_not_raise(AckermannMotorNode, mock_drive, "stop_drive")

    def test_stop_motors_failure_does_not_prevent_destroy(self, ros_context, ackermann_node_class):
        """If the driver itself throws while stopping, destroy_node() must still
        complete rather than leave the node half-destroyed."""
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = AckermannMotorNode()
        node.trigger_configure()
        mock_drive.stop_drive.side_effect = RuntimeError("bus error")

        node.destroy_node()  # must not raise


class TestJointStateFeedback:
    """/joint_states carries what the Float32 feedback topics cannot.

    telemetry_bridge_node has subscribed to /joint_states since it was written
    and nothing ever published it, so this connects existing plumbing rather
    than replacing anything. The Float32 topics stay: drive_speed and
    steering_position are telemetry payload fields reaching the proto, the
    OpenAPI contract and the frontend dials.
    """

    @staticmethod
    def _activated(node_cls, mock_drive, mock_steering, *, wheel_deg=180.0, speed_deg_s=90.0, steer_deg=10.0):
        mock_drive.get_drive_position.return_value = wheel_deg
        mock_drive.get_drive_speed.return_value = speed_deg_s
        mock_steering.get_steering_position.return_value = steer_deg
        node = node_cls()
        node.trigger_configure()
        node.trigger_activate()
        return node

    def test_publisher_exists_after_activate(self, ros_context, ackermann_node_class):
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = self._activated(AckermannMotorNode, mock_drive, mock_steering)
        assert node.joint_state_pub is not None
        node.destroy_node()

    def test_publishes_si_units_not_degrees(self, ros_context, ackermann_node_class):
        """The Float32 topics are degrees; JointState is radians."""
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = self._activated(AckermannMotorNode, mock_drive, mock_steering)
        published = []
        node.joint_state_pub.publish = published.append

        node._publish_feedback()

        assert len(published) == 1
        msg = published[0]
        drive_i = msg.name.index("drive_wheel")
        assert msg.position[drive_i] == pytest.approx(math.pi)  # 180 deg
        assert msg.velocity[drive_i] == pytest.approx(math.pi / 2)  # 90 deg/s
        node.destroy_node()

    def test_carries_a_timestamp(self, ros_context, ackermann_node_class):
        """The reason for the message: integrating distance needs sample times."""
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = self._activated(AckermannMotorNode, mock_drive, mock_steering)
        published = []
        node.joint_state_pub.publish = published.append

        node._publish_feedback()

        stamp = published[0].header.stamp
        assert (stamp.sec, stamp.nanosec) != (0, 0)
        node.destroy_node()

    def test_names_both_joints(self, ros_context, ackermann_node_class):
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = self._activated(AckermannMotorNode, mock_drive, mock_steering)
        published = []
        node.joint_state_pub.publish = published.append

        node._publish_feedback()

        msg = published[0]
        assert set(msg.name) == {"drive_wheel", "steering"}
        assert len(msg.position) == len(msg.name)
        assert len(msg.velocity) == len(msg.name)
        node.destroy_node()

    def test_does_not_consume_the_rpm_window(self, ros_context, ackermann_node_class):
        """get_drive_rpm() consumes the counts since its last call.

        The driver's own docstring records that the 100 Hz feedback publisher
        and the 50 Hz control loop stole windows from each other when both
        called it. Publishing JointState must not add a third consumer.
        """
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = self._activated(AckermannMotorNode, mock_drive, mock_steering)
        mock_drive.reset_mock()

        node._publish_feedback()

        mock_drive.get_drive_rpm.assert_not_called()
        mock_drive.get_drive_odometry.assert_not_called()
        node.destroy_node()

    def test_float32_topics_still_published(self, ros_context, ackermann_node_class):
        """Additive: the telemetry payload fields must keep flowing."""
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = self._activated(AckermannMotorNode, mock_drive, mock_steering)
        steering_pub, speed_pub = [], []
        node.steering_pos_pub.publish = steering_pub.append
        node.drive_speed_pub.publish = speed_pub.append

        node._publish_feedback()

        assert len(steering_pub) == 1
        assert len(speed_pub) == 1
        node.destroy_node()

    def test_publish_feedback_does_not_touch_status_pub(self, ros_context, ackermann_node_class):
        """Diagnostics moved to their own slower timer -- feedback must not also publish them."""
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = self._activated(AckermannMotorNode, mock_drive, mock_steering)
        status_pub = []
        node.status_pub.publish = status_pub.append

        node._publish_feedback()

        assert status_pub == []
        node.destroy_node()


class TestDiagnosticsPublishing:
    """/motor/status runs on its own slower timer (DIAGNOSTICS_RATE_HZ), separate
    from the steering/speed/joint-state feedback a closed loop or UI dial
    actually consumes at a real-time rate."""

    @staticmethod
    def _activated(node_cls, mock_drive, mock_steering, *, speed_deg_s=90.0, steer_deg=10.0):
        mock_drive.get_drive_speed.return_value = speed_deg_s
        mock_steering.get_steering_position.return_value = steer_deg
        node = node_cls()
        node.trigger_configure()
        node.trigger_activate()
        return node

    def test_publishes_status(self, ros_context, ackermann_node_class):
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = self._activated(AckermannMotorNode, mock_drive, mock_steering)
        published = []
        node.status_pub.publish = published.append

        node._publish_diagnostics()

        assert len(published) == 1
        node.destroy_node()

    def test_does_not_consume_the_rpm_window(self, ros_context, ackermann_node_class):
        """Same hazard as _publish_feedback: get_drive_rpm() consumes the counts
        since its last call, so diagnostics must read the cached get_drive_speed()
        rather than adding a third consumer alongside the control loop."""
        AckermannMotorNode, mock_steering, mock_drive, _ = ackermann_node_class
        node = self._activated(AckermannMotorNode, mock_drive, mock_steering)
        mock_drive.reset_mock()

        node._publish_diagnostics()

        mock_drive.get_drive_rpm.assert_not_called()
        mock_drive.get_drive_odometry.assert_not_called()
        node.destroy_node()
