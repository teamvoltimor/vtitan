"""Contract tests for ROS2HardwareGateway's real ROS2 wiring.

Nothing previously exercised ROS2HardwareGateway against a real rclpy.Node,
which is how it drifted onto a Twist/cmd_vel contract nothing subscribes to
while the rest of the stack (ackermann_motor_node, state_machine_node) moved
to AckermannDriveStamped/ackermann_cmd. These tests pin the real topic names
and message contract so that drift can't happen silently again.
"""

from __future__ import annotations

import json
import math
from unittest import mock

import numpy as np
import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from shared.config.constants import RobotSpecs
from shared.domain.enums import ScenarioType, Section
from shared.domain.steering import steering_norm_to_angle_rad
from std_msgs.msg import String

from src.navigation.ports import DriveCommand, LidarScan
from src.navigation.track_geometry import TrackWalls
from src.ros2.navigation.node import ROS2HardwareGateway, TrackNavigator, main

# tests/ros2/conftest.py mocks sys.modules["buildhat"] for this whole directory.

_WIDTHS = {Section.NORTH: 1.0, Section.SOUTH: 1.0, Section.EAST: 1.0, Section.WEST: 1.0}


def _write_metadata(tmp_path, metadata: dict) -> str:
    """Serialize a metadata fixture dict to a file TrackNavigator can load."""
    path = tmp_path / "metadata.json"
    path.write_text(json.dumps(metadata), encoding="utf-8")
    return str(path)


def _scan_at(x: float, y: float, yaw: float) -> LidarScan:
    """A scan as it would be taken standing at a known pose on the believed layout."""
    from src.navigation.track_geometry import corridor_geometry_from_widths

    walls = TrackWalls(corridor_geometry_from_widths(_WIDTHS))
    angles = np.linspace(-math.pi, math.pi, RobotSpecs.LIDAR_SAMPLES, endpoint=False)
    return LidarScan(
        ranges_m=tuple(walls.raycast(x, y, yaw, angles).tolist()),
        angles_rad=tuple(angles.tolist()),
    )


def _obstacles_metadata(*, with_parking: bool) -> dict:
    """Metadata matching ``ScenarioMetadata`` (simgen/scenario_builder schema).

    ``tests/conftest.py``'s ``sample_metadata_obstacles``/``sample_metadata_parking``
    fixtures use the *legacy* ``ScenarioSimulator`` sign/parking schema
    (section/color/depth/lane, blocks/zone_start/zone_end) -- a different shape
    from what ``TrackNavigator._plan`` actually validates against
    (``shared.domain.models.ScenarioMetadata``: sign_positions as x/y/color,
    parking_lot as block1_position/block2_position). Built directly here to
    match the schema the node itself round-trips through.
    """
    metadata = {
        "scenario_id": 1,
        "challenge_type": "obstacles",
        "corridor_widths": {
            "north": {"type": "wide", "width_mm": 1000},
            "south": {"type": "wide", "width_mm": 1000},
            "east": {"type": "wide", "width_mm": 1000},
            "west": {"type": "wide", "width_mm": 1000},
        },
        "starting_conditions": {
            "direction": "clockwise",
            "section": "South",
            "position": {"x": 1.5, "y": 0.5},
            "yaw": 3.14,
        },
        "num_signs": 2,
        "sign_positions": [
            {"x": 1.5, "y": 2.8, "color": "green"},
            {"x": 2.8, "y": 1.5, "color": "red"},
        ],
        "parking_lot": None,
    }
    if with_parking:
        metadata["parking_lot"] = {
            "block1_position": {"x": 0.1, "y": 0.4},
            "block2_position": {"x": 0.1, "y": 0.6},
        }
    return metadata


def _make_host_node() -> Node:
    """A bare Node declaring the parameters ROS2HardwareGateway expects."""
    node = Node("test_track_navigator")
    node.declare_parameter("ackermann_cmd_topic", "/ackermann_cmd")
    node.declare_parameter("lidar_topic", "/scan")
    node.declare_parameter("vision_topic", "/vision/detections")
    node.declare_parameter("imu_topic", "/imu/data")
    node.declare_parameter("joint_states_topic", "/joint_states")
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

        # Within RobotSpecs.MAX_SPEED_MPS (0.156) so this exercises the
        # steering contract, not the speed clamp (see TestGatewaySpeedClamp).
        gateway.publish_drive(DriveCommand(speed_mps=0.1, steering_norm=0.5))

        assert len(published) == 1
        msg = published[0]
        assert isinstance(msg, AckermannDriveStamped)
        assert msg.drive.speed == pytest.approx(0.1)
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


class TestGatewaySpeedClamp:
    """The published speed must never exceed what the drive motor can
    physically do (RobotSpecs.MAX_SPEED_MPS, measured top speed under load).

    Real motor behaviour is identical either way -- the PID's own output-duty
    clamp already saturates at the same point whether the setpoint is 0.156
    or 0.5 -- but an unclamped setpoint made commanded_speed_mps telemetry
    report an aspirational, unreachable value instead of what the motor was
    actually asked to do, matching neither reality nor the simulator (which
    AckermannKinematics already clamps to this same constant).
    """

    def test_speed_above_max_is_clamped(self, ros_context):
        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)

        published: list[AckermannDriveStamped] = []
        gateway._drive_publisher.publish = published.append

        gateway.publish_drive(DriveCommand(speed_mps=0.5, steering_norm=0.0))

        assert published[0].drive.speed == pytest.approx(RobotSpecs.MAX_SPEED_MPS)

        node.destroy_node()

    def test_reverse_speed_below_max_is_clamped(self, ros_context):
        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)

        published: list[AckermannDriveStamped] = []
        gateway._drive_publisher.publish = published.append

        gateway.publish_drive(DriveCommand(speed_mps=-0.5, steering_norm=0.0))

        assert published[0].drive.speed == pytest.approx(-RobotSpecs.MAX_SPEED_MPS)

        node.destroy_node()

    def test_speed_within_max_is_untouched(self, ros_context):
        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)

        published: list[AckermannDriveStamped] = []
        gateway._drive_publisher.publish = published.append

        gateway.publish_drive(DriveCommand(speed_mps=0.1, steering_norm=0.0))

        assert published[0].drive.speed == pytest.approx(0.1)

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

        # raycast()'s angles are already robot-frame (0 = forward), but
        # _lidar_callback now rotates whatever angle_min/max it's given by
        # the C1's real mount offset -- so a synthetic "as if from real
        # hardware" LaserScan has to be built pre-rotated by the inverse,
        # or this fake scan would come out offset from the true geometry
        # it's meant to represent.
        yaw_offset_rad = math.radians(RobotSpecs.LIDAR_MOUNT_YAW_OFFSET_DEG)
        msg = LaserScan()
        msg.angle_min = float(angles[0]) - yaw_offset_rad
        msg.angle_max = float(angles[-1]) - yaw_offset_rad
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


class TestWheelOdometryWiring:
    """The encoder's travel has to reach the navigation port.

    The driver has exposed distance since it was calibrated against a tape, and
    the motor node now publishes it, but the navigator is where it has to
    arrive to be usable as a motion prior.
    """

    def test_gateway_subscribes_to_joint_states(self, ros_context):
        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)

        subs = dict(node.get_subscriber_names_and_types_by_node(node.get_name(), ""))
        assert "/joint_states" in subs
        assert subs["/joint_states"] == ["sensor_msgs/msg/JointState"]

        node.destroy_node()

    def test_none_before_the_first_message(self, ros_context):
        """A drive backend with no encoder never reports, which is normal."""
        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)

        assert gateway.get_wheel_odometry() is None

        node.destroy_node()

    def test_wheel_angle_becomes_linear_travel(self, ros_context):
        from sensor_msgs.msg import JointState

        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)

        msg = JointState()
        msg.name = ["drive_wheel", "steering"]
        msg.position = [2 * math.pi, 0.0]  # exactly one wheel revolution
        msg.velocity = [1.0, 0.0]
        gateway._joint_state_callback(msg)

        odom = gateway.get_wheel_odometry()
        assert odom is not None
        # One revolution is one circumference of travel.
        assert odom.distance_m == pytest.approx(2 * math.pi * RobotSpecs.WHEEL_RADIUS)
        assert odom.speed_mps == pytest.approx(RobotSpecs.WHEEL_RADIUS)

        node.destroy_node()

    def test_indexed_by_name_not_array_position(self, ros_context):
        """JointState carries an arbitrary set of joints in an arbitrary order.

        Assuming index 0 is the drive wheel would break silently the first time
        another joint is added ahead of it.
        """
        from sensor_msgs.msg import JointState

        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)

        msg = JointState()
        msg.name = ["steering", "drive_wheel"]  # drive wheel second
        msg.position = [0.0, 2 * math.pi]
        msg.velocity = [0.0, 1.0]
        gateway._joint_state_callback(msg)

        odom = gateway.get_wheel_odometry()
        assert odom is not None
        assert odom.distance_m == pytest.approx(2 * math.pi * RobotSpecs.WHEEL_RADIUS)

        node.destroy_node()

    def test_a_message_without_the_drive_joint_is_ignored(self, ros_context):
        from sensor_msgs.msg import JointState

        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)

        msg = JointState()
        msg.name = ["steering"]
        msg.position = [0.5]
        gateway._joint_state_callback(msg)

        assert gateway.get_wheel_odometry() is None

        node.destroy_node()

    def test_the_message_the_motor_node_publishes_is_readable_by_the_gateway(self, ros_context):
        """Different packages, neither importing the other.

        This used to compare ``motor_node._DRIVE_JOINT`` against a re-export on
        the navigator, which stopped meaning anything once both sides began
        importing the name from ``src.hardware.motors.enums``: it compared a
        constant to itself down two import paths, and could only ever pass. It
        did not even do that -- the motor node's copy is public, so the
        assertion had been raising AttributeError rather than comparing
        anything, and the drift it was guarding was structurally impossible by
        then anyway.

        What is still possible is a literal creeping back in on either side, or
        the message shape changing. So assemble the JointState exactly as
        ackermann_motor_node does -- its own constants, its own order, its own
        units -- and require the gateway to read odometry out of it.
        """
        from sensor_msgs.msg import JointState
        from vtitan_drivers.motors.ackermann_motor_node import (
            DRIVE_JOINT as PUBLISHED_DRIVE,
            STEERING_JOINT as PUBLISHED_STEER,
        )

        node = _make_host_node()
        gateway = ROS2HardwareGateway(node, 0.0, 0.0, 0.0, _WIDTHS)

        # Same construction as ackermann_motor_node's publish block: both
        # joints, drive first, positions in radians.
        msg = JointState()
        msg.header.stamp.sec = 7
        msg.name = [PUBLISHED_DRIVE, PUBLISHED_STEER]
        msg.position = [math.radians(90.0), math.radians(10.0)]
        gateway._joint_state_callback(msg)

        odometry = gateway.get_wheel_odometry()
        assert odometry is not None, "the gateway could not find the drive joint the motor node publishes"
        assert odometry.distance_m == pytest.approx(math.radians(90.0) * RobotSpecs.WHEEL_RADIUS)

        node.destroy_node()


class TestLoadJson:
    """The path-vs-str dispatch at the top of the module, exercised via both branches."""

    def test_reads_a_path_object_directly(self, tmp_path):
        from src.ros2.navigation import track_navigator_node

        path = tmp_path / "data.json"
        path.write_text('{"a": 1}', encoding="utf-8")
        assert track_navigator_node._load_json(path) == {"a": 1}

    def test_wraps_a_str_path_first(self, tmp_path):
        from src.ros2.navigation import track_navigator_node

        path = tmp_path / "data.json"
        path.write_text('{"b": 2}', encoding="utf-8")
        assert track_navigator_node._load_json(str(path)) == {"b": 2}


class TestParamOverrideTypeDispatch:
    """``_apply_param_overrides``'s per-value-type dispatch, called directly.

    Called as an unbound method against a bare host node (like
    ``_make_host_node``) rather than a full ``TrackNavigator`` -- it only
    touches ``self.get_logger()`` and ``self.set_parameters()``, and the
    real node declares no int/float parameters to exercise those branches
    against.
    """

    def test_bool_int_float_str_applied_and_unsupported_type_skipped(self, tmp_path, ros_context):
        node = _make_host_node()
        node.declare_parameter("int_param", 0)
        node.declare_parameter("float_param", 0.0)
        node.declare_parameter("str_param", "x")
        node.declare_parameter("bool_param", value=False)
        node.declare_parameter("list_param", [1])

        params_file = tmp_path / "params.json"
        params_file.write_text(
            json.dumps(
                {
                    "bool_param": True,
                    "int_param": 7,
                    "float_param": 2.5,
                    "str_param": "hello",
                    "list_param": [9, 9],
                },
            ),
        )

        TrackNavigator._apply_param_overrides(node, params_file)

        assert node.get_parameter("bool_param").value is True
        assert node.get_parameter("int_param").value == 7
        assert node.get_parameter("float_param").value == pytest.approx(2.5)
        assert node.get_parameter("str_param").value == "hello"
        # Not bool/int/float/str -- must be skipped, not raise.
        assert node.get_parameter("list_param").value == [1]

        node.destroy_node()

    def test_missing_file_is_logged_and_ignored(self, ros_context):
        node = _make_host_node()
        TrackNavigator._apply_param_overrides(node, "/nonexistent/path/params.json")
        node.destroy_node()

    def test_invalid_json_is_logged_and_ignored(self, tmp_path, ros_context):
        node = _make_host_node()
        bad = tmp_path / "bad.json"
        bad.write_text("{not valid json", encoding="utf-8")
        TrackNavigator._apply_param_overrides(node, bad)
        node.destroy_node()

    def test_no_supported_params_never_calls_set_parameters(self, tmp_path, ros_context):
        node = _make_host_node()
        node.declare_parameter("list_param", [1])
        params_file = tmp_path / "params.json"
        params_file.write_text(json.dumps({"list_param": [9, 9]}))

        with mock.patch.object(node, "set_parameters") as set_params_mock:
            TrackNavigator._apply_param_overrides(node, params_file)
        set_params_mock.assert_not_called()

        node.destroy_node()


class TestParamOverridesReachTheRealNode:
    """The __init__ call site: params_path actually gets applied at construction."""

    def test_construction_with_params_path_applies_overrides(self, tmp_path, sample_metadata_open, ros_context):
        metadata_path = _write_metadata(tmp_path, sample_metadata_open)
        params_file = tmp_path / "params.json"
        params_file.write_text(json.dumps({"is_simulation": True, "ackermann_cmd_topic": "/custom_cmd"}))

        navigator = TrackNavigator(metadata_path=metadata_path, num_laps=1, params_path=params_file)
        try:
            assert navigator.get_parameter("is_simulation").value is True
            assert navigator.get_parameter("ackermann_cmd_topic").value == "/custom_cmd"
        finally:
            navigator.destroy_node()


class TestObstaclesConstruction:
    """The sign-router / park-controller wiring that only happens outside Open Challenge."""

    def test_sign_router_built_from_metadata_when_not_blind(self, tmp_path, ros_context):
        navigator = TrackNavigator(
            metadata_path=_write_metadata(tmp_path, _obstacles_metadata(with_parking=False)),
            num_laps=1,
        )
        try:
            router = navigator._core_navigator.sign_router
            assert router is not None
            assert len(router.signs) == 2
        finally:
            navigator.destroy_node()

    def test_sign_router_starts_empty_and_discovers_when_blind(self, tmp_path, ros_context):
        navigator = TrackNavigator(
            metadata_path=_write_metadata(tmp_path, _obstacles_metadata(with_parking=False)),
            num_laps=1,
            blind=True,
        )
        try:
            router = navigator._core_navigator.sign_router
            assert router is not None
            # Blind means nothing about the signs is known up front -- they
            # must be discovered from /vision/detections, not read from the file.
            assert router.signs == []
        finally:
            navigator.destroy_node()

    def test_no_sign_router_for_open_challenge(self, tmp_path, sample_metadata_open, ros_context):
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            assert navigator._core_navigator.sign_router is None
        finally:
            navigator.destroy_node()

    def test_park_controller_built_for_obstacles_with_a_parking_lot(self, tmp_path, ros_context):
        navigator = TrackNavigator(
            metadata_path=_write_metadata(tmp_path, _obstacles_metadata(with_parking=True)),
            num_laps=1,
        )
        try:
            assert navigator._core_navigator._park_controller is not None
        finally:
            navigator.destroy_node()

    def test_no_park_controller_for_open_challenge(self, tmp_path, sample_metadata_open, ros_context):
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            assert navigator._core_navigator._park_controller is None
        finally:
            navigator.destroy_node()


class TestToWidthsDictNonBlind:
    """The non-blind half of ``_to_widths_dict`` -- reads the told geometry, not an estimator."""

    def test_uses_the_told_geometry_when_not_blind(self, tmp_path, sample_metadata_open, ros_context):
        from tests.test_constants import CORRIDOR_WIDTH_NARROW_MM, CORRIDOR_WIDTH_WIDE_MM

        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            assert navigator._width_estimator is None
            widths = navigator._to_widths_dict()
            assert set(widths) == set(Section)
            assert widths[Section.NORTH] == pytest.approx(CORRIDOR_WIDTH_WIDE_MM / 1000)
            assert widths[Section.SOUTH] == pytest.approx(CORRIDOR_WIDTH_NARROW_MM / 1000)
        finally:
            navigator.destroy_node()


class TestRobotStateGating:
    """``_on_robot_state`` is the only thing standing between LIDAR and the motors."""

    def test_entering_racing_resets_and_starts_driving(self, tmp_path, sample_metadata_open, ros_context):
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            navigator._racing = False
            with mock.patch.object(navigator, "reset") as reset_mock:
                navigator._on_robot_state(String(data="racing"))
            assert navigator._racing is True
            reset_mock.assert_called_once()
        finally:
            navigator.destroy_node()

    def test_leaving_racing_stops_the_motors_immediately(self, tmp_path, sample_metadata_open, ros_context):
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            navigator._racing = True
            with mock.patch.object(navigator._gateway, "publish_drive") as publish_mock:
                navigator._on_robot_state(String(data="finished"))
            assert navigator._racing is False
            publish_mock.assert_called_once_with(DriveCommand(speed_mps=0.0, steering_norm=0.0))
        finally:
            navigator.destroy_node()

    def test_a_repeated_non_racing_state_is_a_no_op(self, tmp_path, sample_metadata_open, ros_context):
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            navigator._racing = False
            with (
                mock.patch.object(navigator, "reset") as reset_mock,
                mock.patch.object(navigator._gateway, "publish_drive") as publish_mock,
            ):
                navigator._on_robot_state(String(data="ready"))
            reset_mock.assert_not_called()
            publish_mock.assert_not_called()
        finally:
            navigator.destroy_node()


class TestControlLoop:
    """The 20 Hz tick: race gating, blind-vs-sighted dispatch, and the error backstop."""

    def test_not_racing_holds_zero_and_never_steps(self, tmp_path, sample_metadata_open, ros_context):
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            navigator._racing = False
            with (
                mock.patch.object(navigator._gateway, "publish_drive") as publish_mock,
                mock.patch.object(navigator._core_navigator, "step") as step_mock,
            ):
                navigator._control_loop()
            publish_mock.assert_called_once_with(DriveCommand(speed_mps=0.0, steering_norm=0.0))
            step_mock.assert_not_called()
        finally:
            navigator.destroy_node()

    def test_racing_and_sighted_steps_the_core_navigator(self, tmp_path, sample_metadata_open, ros_context):
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            navigator._racing = True
            with mock.patch.object(navigator._core_navigator, "step") as step_mock:
                navigator._control_loop()
            step_mock.assert_called_once()
        finally:
            navigator.destroy_node()

    def test_racing_blind_with_direction_unknown_skips_belief_and_step(self, ros_context):
        navigator = TrackNavigator(metadata_path=None, num_laps=1)
        try:
            navigator._racing = True
            with (
                mock.patch.object(navigator, "_resolve_direction", return_value=True) as resolve_mock,
                mock.patch.object(navigator, "_update_layout_belief") as belief_mock,
                mock.patch.object(navigator._core_navigator, "step") as step_mock,
            ):
                navigator._control_loop()
            resolve_mock.assert_called_once()
            belief_mock.assert_not_called()
            step_mock.assert_not_called()
        finally:
            navigator.destroy_node()

    def test_racing_blind_with_direction_known_updates_belief_then_steps(self, ros_context):
        navigator = TrackNavigator(metadata_path=None, num_laps=1)
        try:
            navigator._racing = True
            with (
                mock.patch.object(navigator, "_resolve_direction", return_value=False),
                mock.patch.object(navigator, "_update_layout_belief") as belief_mock,
                mock.patch.object(navigator._core_navigator, "step") as step_mock,
            ):
                navigator._control_loop()
            belief_mock.assert_called_once()
            step_mock.assert_called_once()
        finally:
            navigator.destroy_node()

    def test_runtime_error_from_step_is_caught_and_stops_the_motors(self, tmp_path, sample_metadata_open, ros_context):
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            navigator._racing = True
            with (
                mock.patch.object(navigator._core_navigator, "step", side_effect=RuntimeError("boom")),
                mock.patch.object(navigator._gateway, "publish_drive") as publish_mock,
            ):
                navigator._control_loop()  # must not propagate
            publish_mock.assert_called_once_with(DriveCommand(speed_mps=0.0, steering_norm=0.0))
        finally:
            navigator.destroy_node()

    def test_value_error_from_step_is_caught_and_stops_the_motors(self, tmp_path, sample_metadata_open, ros_context):
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            navigator._racing = True
            with (
                mock.patch.object(navigator._core_navigator, "step", side_effect=ValueError("bad")),
                mock.patch.object(navigator._gateway, "publish_drive") as publish_mock,
            ):
                navigator._control_loop()
            publish_mock.assert_called_once_with(DriveCommand(speed_mps=0.0, steering_norm=0.0))
        finally:
            navigator.destroy_node()


class TestReset:
    """``reset()`` is what actually runs at every RACING transition, not just node startup."""

    def test_rezeroes_heading_and_resets_the_core_navigator(self, tmp_path, sample_metadata_open, ros_context):
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            with (
                mock.patch.object(navigator._gateway, "reset_heading_reference") as reset_heading_mock,
                mock.patch.object(navigator._gateway, "reset_position") as reset_position_mock,
                mock.patch.object(navigator._core_navigator, "reset") as core_reset_mock,
            ):
                navigator.reset()
            reset_heading_mock.assert_called_once()
            reset_position_mock.assert_called_once_with(navigator._start_xy.x, navigator._start_xy.y)
            core_reset_mock.assert_called_once()
        finally:
            navigator.destroy_node()

    def test_resetting_actually_clears_position_drift_on_the_estimator(self, tmp_path, sample_metadata_open, ros_context):
        """2026-08-04: the bug behind hundreds-of-metres pose_x/pose_y on real hardware.

        The state machine can cycle FINISHED -> BOOT_CHECK -> READY -> RACING
        purely from the button, with no process restart, so reset() has to
        re-seed position the same way it already re-zeroed heading -- without
        it a new race inherits wherever the previous race's LIDAR localizer
        last drifted to. Goes through the real gateway/estimator, not a mock,
        to pin the actual value landing correctly.
        """
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            navigator._gateway._estimator.update_position(-125.4, -127.1)
            assert navigator._gateway.get_current_pose().x == pytest.approx(-125.4)

            navigator.reset()

            pose = navigator._gateway.get_current_pose()
            assert pose.x == pytest.approx(navigator._start_xy.x)
            assert pose.y == pytest.approx(navigator._start_xy.y)
        finally:
            navigator.destroy_node()

    def test_open_challenge_replaces_with_no_park_controller(self, tmp_path, sample_metadata_open, ros_context):
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            with mock.patch.object(navigator._core_navigator, "replace_park_controller") as replace_mock:
                navigator.reset()
            replace_mock.assert_called_once_with(None)
        finally:
            navigator.destroy_node()

    def test_obstacles_challenge_rebuilds_a_fresh_park_controller(self, tmp_path, ros_context):
        navigator = TrackNavigator(
            metadata_path=_write_metadata(tmp_path, _obstacles_metadata(with_parking=True)),
            num_laps=1,
        )
        try:
            with mock.patch.object(navigator._core_navigator, "replace_park_controller") as replace_mock:
                navigator.reset()
            (built,), _ = replace_mock.call_args
            assert built is not None
        finally:
            navigator.destroy_node()


class TestBlindChallengeSwitch:
    """A real blind run only learns its challenge from the jumper, forwarded by
    state_machine_node over /challenge_mode/active well after this node is
    constructed -- and the state machine can cycle FINISHED -> BOOT_CHECK ->
    READY -> RACING purely from the button, with no process restart, so the
    robot must be able to go from an Open round to an Obstacles round (or the
    reverse) without ever knowing direction, obstacle, or parking layout ahead
    of time. These exercise that path end to end: no metadata file, no
    foreknowledge, driven purely by the same topic/state transitions the real
    hardware uses.
    """

    @staticmethod
    def _blind_navigator():
        from shared.domain.enums import Direction

        return TrackNavigator(num_laps=1, blind=True, direction=Direction.CLOCKWISE)

    def test_defaults_to_open_before_any_jumper_reading_arrives(self, ros_context):
        navigator = self._blind_navigator()
        try:
            navigator._on_robot_state(String(data="racing"))
            assert navigator._is_open_challenge is True
            assert navigator._core_navigator.sign_router is None
            assert navigator._core_navigator._park_controller is None
            assert navigator._tuning is navigator._tuning_by_challenge[ScenarioType.OPEN]
        finally:
            navigator.destroy_node()

    def test_switches_to_obstacles_purely_from_the_button_no_restart(self, ros_context):
        navigator = self._blind_navigator()
        try:
            navigator._on_challenge_mode_active(String(data="obstacles"))
            navigator._on_robot_state(String(data="racing"))

            assert navigator._is_open_challenge is False
            assert navigator._core_navigator.sign_router is not None
            assert navigator._tuning is navigator._tuning_by_challenge[ScenarioType.OBSTACLES]
        finally:
            navigator.destroy_node()

    def test_round_trip_open_to_obstacles_and_back_in_one_process(self, ros_context):
        """Mirrors FINISHED -> BOOT_CHECK -> READY -> RACING cycled twice with the
        jumper moved in between -- the long-press reset re-arms challenge
        selection on state_machine_node; this pins the other half, that
        track_navigator_node actually picks up what it re-arms to.
        """
        navigator = self._blind_navigator()
        try:
            navigator._on_challenge_mode_active(String(data="open"))
            navigator._on_robot_state(String(data="racing"))
            assert navigator._core_navigator.sign_router is None
            navigator._on_robot_state(String(data="finished"))

            navigator._on_challenge_mode_active(String(data="obstacles"))
            navigator._on_robot_state(String(data="racing"))
            assert navigator._core_navigator.sign_router is not None

            navigator._on_robot_state(String(data="finished"))
            navigator._on_challenge_mode_active(String(data="open"))
            navigator._on_robot_state(String(data="racing"))
            assert navigator._core_navigator.sign_router is None
        finally:
            navigator.destroy_node()

    def test_metadata_driven_run_never_subscribes_to_the_jumper_topic(self, tmp_path, sample_metadata_open, ros_context):
        """A sim/test run with a metadata file must keep its pre-existing,
        deterministic behaviour -- the challenge is fixed for the whole
        process, exactly as before this change.
        """
        navigator = TrackNavigator(metadata_path=_write_metadata(tmp_path, sample_metadata_open), num_laps=1)
        try:
            assert navigator._tuning_by_challenge is None
        finally:
            navigator.destroy_node()


class TestMainEntryPoint:
    """The CLI entry point: argument parsing, the missing-file guard, and the spin loop."""

    def test_missing_metadata_file_exits_before_touching_rclpy(self, tmp_path):
        missing = tmp_path / "nope.json"
        with pytest.raises(SystemExit) as exc_info:
            main(["--metadata", str(missing)])
        assert exc_info.value.code == 1
        assert not rclpy.ok()

    def test_runs_one_spin_then_shuts_down_cleanly(self, tmp_path, sample_metadata_open, monkeypatch):
        metadata_path = _write_metadata(tmp_path, sample_metadata_open)

        def _raise_keyboard_interrupt(node, timeout_sec=0.1):  # noqa: ARG001
            raise KeyboardInterrupt

        monkeypatch.setattr(rclpy, "spin_once", _raise_keyboard_interrupt)

        main(["--metadata", metadata_path, "--laps", "1"])

        assert not rclpy.ok(), "the finally block must shut rclpy back down"

    def test_blind_ccw_with_no_metadata_file_at_all(self, monkeypatch):
        def _raise_keyboard_interrupt(node, timeout_sec=0.1):  # noqa: ARG001
            raise KeyboardInterrupt

        monkeypatch.setattr(rclpy, "spin_once", _raise_keyboard_interrupt)

        main(["--blind", "--direction", "ccw"])

        assert not rclpy.ok()


class TestStartMeasurementRetry:
    """A refused start measurement is retried, not settled for.

    ``measure_start_pose`` refuses when a cardinal ray has no valid return or
    the along-corridor pair does not span the mat, and its docstring says the
    caller must treat that as "do not race" rather than "use the old
    assumption". The node did use the old assumption, and it cost both rounds
    that refused on 2026-08-08: replaying those bags' whole scan stream, an
    operator stood in the rearward ray at 0.10-0.19 m and cleared it 0.6 s and
    1.5 s after the commit, by which point the node had already reseeded its
    position roughly 1.0 m behind the truth and was driving at a corner on it.
    """

    @staticmethod
    def _navigator():
        """A blind counterclockwise navigator with the retry already armed."""
        from shared.domain.enums import Direction

        navigator = TrackNavigator(num_laps=1, blind=True, direction=Direction.COUNTERCLOCKWISE)
        navigator._start_measurement_ticks_left = 100
        return navigator

    def _drive_one_tick(self, navigator, x: float, y: float, yaw: float):
        """Hand the retry one scan taken at ``(x, y, yaw)``, from a pose it does not know."""
        from shared.domain.models import Pose

        with (
            mock.patch.object(navigator._gateway, "get_lidar_scan", return_value=_scan_at(x, y, yaw)),
            # Deliberately not (x, y): the whole point is that the estimate is
            # wrong and the measurement is what corrects it.
            mock.patch.object(navigator._gateway, "get_current_pose", return_value=Pose(x=1.25, y=0.4, yaw=yaw)),
            mock.patch.object(navigator._gateway, "reset_position") as reset_mock,
        ):
            navigator._retry_start_measurement()
        return reset_mock

    def test_a_clean_scan_lands_the_measurement_and_reseeds(self, ros_context):
        navigator = self._navigator()
        try:
            reset_mock = self._drive_one_tick(navigator, 2.26, 0.30, 0.0)

            assert navigator._measured_start is not None
            assert navigator._measured_start.x == pytest.approx(2.26, abs=0.03)
            assert navigator._measured_start.y == pytest.approx(0.30, abs=0.03)
            reset_mock.assert_called_once()
            assert reset_mock.call_args.args[0] == pytest.approx(2.26, abs=0.03)
        finally:
            navigator.destroy_node()

    def test_the_budget_is_spent_once_a_measurement_lands(self, ros_context):
        """Latched: the start pose is measured once, and the localizer owns it after."""
        navigator = self._navigator()
        try:
            self._drive_one_tick(navigator, 2.26, 0.30, 0.0)
            landed = navigator._measured_start
            reset_mock = self._drive_one_tick(navigator, 1.40, 0.60, 0.0)

            assert navigator._measured_start is landed
            reset_mock.assert_not_called()
        finally:
            navigator.destroy_node()

    def test_a_chassis_turned_around_is_refused_though_the_rays_still_span_the_mat(self, ros_context):
        """The gate measure_start_pose cannot apply for itself.

        Facing back down the corridor, ``forward`` and ``back`` simply swap, so
        the closing check passes exactly as well and the along-corridor
        coordinate comes out mirrored about the mat's centre -- 2.26 read as
        0.74. Only the heading distinguishes them.
        """
        navigator = self._navigator()
        try:
            reset_mock = self._drive_one_tick(navigator, 2.26, 0.30, math.pi)

            assert navigator._measured_start is None
            reset_mock.assert_not_called()
        finally:
            navigator.destroy_node()

    def test_an_unarmed_navigator_never_measures(self, ros_context):
        """A round whose commit measured cleanly must not keep re-measuring while it drives."""
        navigator = self._navigator()
        navigator._start_measurement_ticks_left = 0
        try:
            reset_mock = self._drive_one_tick(navigator, 2.26, 0.30, 0.0)

            assert navigator._measured_start is None
            reset_mock.assert_not_called()
        finally:
            navigator.destroy_node()

    def test_the_budget_runs_out(self, ros_context):
        """Bounded, because the pose is read in the starting section's frame.

        The robot leaves that section for good at the first corner, and a
        measurement taken after it would be filed against a corridor the robot
        is no longer standing in.
        """
        navigator = self._navigator()
        navigator._start_measurement_ticks_left = 2
        try:
            # Two ticks of a scan nothing can be measured from, then a clean one.
            for _ in range(2):
                self._drive_one_tick(navigator, 2.26, 0.30, math.pi / 2)
            assert navigator._start_measurement_ticks_left == 0

            reset_mock = self._drive_one_tick(navigator, 2.26, 0.30, 0.0)

            assert navigator._measured_start is None
            reset_mock.assert_not_called()
        finally:
            navigator.destroy_node()
