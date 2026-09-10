"""Tests for telemetry_ingest_channel -- the robot->backend gRPC ingest channel.

Covers the dataclass->proto conversion helpers against representative
telemetry_bridge_node dataclasses, the keep-latest queue semantics, and the
start/stop/restart thread lifecycle. No real network/backend: _run_stream is
mocked out for the lifecycle tests so no actual grpc.insecure_channel
connection attempt is made.
"""

from __future__ import annotations

import queue
import time
from typing import TYPE_CHECKING
from unittest import mock

if TYPE_CHECKING:
    import threading

from vtitan_state_machine import telemetry_ingest_channel as tic
from vtitan_state_machine.telemetry_bridge_node import (
    RobotSnapshot,
    TelemetryMetrics,
    TopicsSnapshot,
    _IMUPayload,
    _MotorStatePayload,
    _TopicUpdatePayload,
    _VisionDetectionPayload,
)


def _metrics(**overrides) -> TelemetryMetrics:
    kwargs = {
        "timestamp": 1000.25,
        "nodeHealth": "nominal",
        "pointsCaptured": 42,
        "rangeMin": 0.1,
        "rangeMax": 5.0,
        "rangeMean": 1.5,
        "forward": 2.0,
        "left": 1.0,
        "right": 1.2,
        "back": 3.0,
        "speed": 0.5,
        "stage": "racing",
        "lidarAvailable": True,
        "imuAvailable": True,
        "cameraAvailable": True,
        "odometryAvailable": True,
    }
    kwargs.update(overrides)
    return TelemetryMetrics(**kwargs)


class TestPosition3d:
    def test_maps_xyz_in_order(self):
        proto = tic._position3d([1.0, 2.0, 3.0])
        assert (proto.x, proto.y, proto.z) == (1.0, 2.0, 3.0)


class TestTimestampFromEpoch:
    def test_splits_seconds_and_nanos(self):
        proto = tic._timestamp_from_epoch(1000.25)
        assert proto.seconds == 1000
        assert proto.nanos == 250_000_000

    def test_whole_second_has_zero_nanos(self):
        proto = tic._timestamp_from_epoch(500.0)
        assert proto.seconds == 500
        assert proto.nanos == 0


class TestNodeHealth:
    def test_known_value_maps_to_enum(self):
        assert tic._node_health("nominal") == tic.types_pb2.NODE_HEALTH_NOMINAL

    def test_is_case_insensitive(self):
        assert tic._node_health("WATCHDOG") == tic.types_pb2.NODE_HEALTH_WATCHDOG

    def test_unrecognized_value_falls_back_to_unspecified(self):
        assert tic._node_health("bogus_status") == tic.types_pb2.NODE_HEALTH_UNSPECIFIED

    def test_empty_string_falls_back_to_unspecified(self):
        assert tic._node_health("") == tic.types_pb2.NODE_HEALTH_UNSPECIFIED


class TestImuData:
    def test_maps_all_fields(self):
        imu = _IMUPayload(
            linearAcceleration=[0.1, 0.2, 9.8],
            angularVelocity=[0.01, 0.02, 0.03],
            orientationQuaternion=[0.0, 0.0, 0.707, 0.707],
        )
        proto = tic._imu_data(imu)
        assert (proto.linear_acceleration.x, proto.linear_acceleration.y, proto.linear_acceleration.z) == (
            0.1,
            0.2,
            9.8,
        )
        assert (proto.angular_velocity.x, proto.angular_velocity.y, proto.angular_velocity.z) == (0.01, 0.02, 0.03)
        assert proto.orientation_x == 0.0
        assert proto.orientation_y == 0.0
        assert proto.orientation_z == 0.707
        assert proto.orientation_w == 0.707


class TestMotorState:
    def test_maps_all_fields(self):
        motor = _MotorStatePayload(steeringAngle=0.35, driveSpeed=42.0, encoderPosition=1200)
        proto = tic._motor_state(motor)
        assert proto.steering_angle == 0.35
        assert proto.drive_speed == 42.0
        assert proto.encoder_position == 1200


class TestDetection:
    def test_maps_all_fields(self):
        det = _VisionDetectionPayload(className="red_sign", confidence=0.87, bbox=[0.1, 0.2, 0.3, 0.4])
        proto = tic._detection(det)
        assert proto.class_name == "red_sign"
        assert proto.confidence == 0.87
        assert (proto.bbox_x, proto.bbox_y, proto.bbox_w, proto.bbox_h) == (0.1, 0.2, 0.3, 0.4)

    def test_class_name_is_coerced_to_str(self):
        det = _VisionDetectionPayload(className=7, confidence=0.5, bbox=[0.0, 0.0, 0.0, 0.0])
        proto = tic._detection(det)
        assert proto.class_name == "7"


class TestMetrics:
    def test_maps_required_fields(self):
        metrics = _metrics()
        proto = tic._metrics(metrics)
        assert proto.timestamp.seconds == 1000
        assert proto.node_health == tic.types_pb2.NODE_HEALTH_NOMINAL
        assert proto.points_captured == 42
        assert proto.stage == "racing"
        assert proto.lidar_available is True
        assert proto.imu_available is True
        assert proto.camera_available is True
        assert proto.odometry_available is True

    def test_present_optional_fields_are_set(self):
        metrics = _metrics()
        proto = tic._metrics(metrics)
        for field in ("range_min", "range_max", "range_mean", "forward", "left", "right", "back", "speed"):
            assert proto.HasField(field), f"{field} should be set when the dataclass value is not None"
        assert proto.range_min == 0.1
        assert proto.speed == 0.5

    def test_none_optional_fields_are_left_unset(self):
        metrics = _metrics(
            rangeMin=None,
            rangeMax=None,
            rangeMean=None,
            forward=None,
            left=None,
            right=None,
            back=None,
            speed=None,
        )
        proto = tic._metrics(metrics)
        for field in ("range_min", "range_max", "range_mean", "forward", "left", "right", "back", "speed"):
            assert not proto.HasField(field), f"{field} should stay unset when the dataclass value is None"

    def test_partial_none_only_unsets_the_none_fields(self):
        metrics = _metrics(rangeMin=None, speed=None)
        proto = tic._metrics(metrics)
        assert not proto.HasField("range_min")
        assert not proto.HasField("speed")
        assert proto.HasField("range_max")
        assert proto.HasField("forward")


class TestStructFromDict:
    def test_json_serializable_dict_round_trips(self):
        struct = tic._struct_from_dict({"a": 1, "b": "text", "c": [1, 2, 3], "d": {"nested": True}})
        assert struct["a"] == 1
        assert struct["b"] == "text"
        assert list(struct["c"]) == [1, 2, 3]
        assert struct["d"]["nested"] is True

    def test_empty_dict_produces_empty_struct(self):
        struct = tic._struct_from_dict({})
        assert len(struct.fields) == 0

    def test_non_json_serializable_value_yields_a_clean_empty_struct(self):
        # protobuf's Struct.update() raises ValueError("Unexpected type") on
        # a bytes value, but not before it has already auto-vivified the map
        # entry for that key -- leaving a google.protobuf.Value with no oneof
        # case set, which protojson then refuses to marshal at all. So the
        # partially-converted struct is discarded entirely rather than
        # returned: the keys that did convert are lost along with the one
        # that didn't, and callers get an empty payload instead of a corrupt
        # one that 500s every /v1/telemetry/topics request.
        struct = tic._struct_from_dict({"ok": 1, "bad": b"\x00\x01"})

        assert len(struct.fields) == 0

    def test_value_type_that_raises_is_swallowed_without_propagating(self):
        class _Unconvertible:
            """Neither JSON-primitive nor iterable -- Struct.update() raises on this."""

        struct = tic._struct_from_dict({"bad": _Unconvertible()})  # must not raise

        assert len(struct.fields) == 0


class TestTopicUpdate:
    def test_maps_all_fields(self):
        update = _TopicUpdatePayload(
            topicName="/scan",
            messageType="sensor_msgs/LaserScan",
            timestamp=100.5,
            updateRateHz=9.8,
            data={"angle_min": -3.14},
        )
        proto = tic._topic_update(update)
        assert proto.topic_name == "/scan"
        assert proto.message_type == "sensor_msgs/LaserScan"
        assert proto.timestamp.seconds == 100
        assert proto.update_rate_hz == 9.8
        assert proto.data["angle_min"] == -3.14


class TestSnapshotToProto:
    def _snapshot(self, **overrides) -> RobotSnapshot:
        kwargs = {
            "timestamp": 2000.0,
            "missionName": "obstacle_challenge",
            "robotPosition": [1.0, 2.0, 0.0],
            "robotOrientation": 1.57,
            "lidarPoints": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            "pathHistory": [[0.0, 0.0, 0.0]],
            "logs": ["State: RACING"],
            "metrics": _metrics(),
            "imuData": _IMUPayload(
                linearAcceleration=[0.0, 0.0, 9.8],
                angularVelocity=[0.0, 0.0, 0.0],
                orientationQuaternion=[0.0, 0.0, 0.0, 1.0],
            ),
            "visionDetections": [_VisionDetectionPayload(className="green_sign", confidence=0.9, bbox=[0, 0, 1, 1])],
            "motorState": _MotorStatePayload(steeringAngle=0.0, driveSpeed=10.0, encoderPosition=5),
        }
        kwargs.update(overrides)
        return RobotSnapshot(**kwargs)

    def test_maps_scalar_and_repeated_fields(self):
        proto = tic._snapshot_to_proto(self._snapshot())
        assert proto.timestamp.seconds == 2000
        assert proto.mission_name == "obstacle_challenge"
        assert len(proto.lidar_points) == 2
        assert len(proto.path_history) == 1
        assert list(proto.logs) == ["State: RACING"]
        assert proto.metrics.stage == "racing"
        assert len(proto.vision_detections) == 1
        assert proto.vision_detections[0].class_name == "green_sign"

    def test_optional_present_fields_are_set(self):
        proto = tic._snapshot_to_proto(self._snapshot())
        assert proto.HasField("robot_position")
        assert proto.HasField("robot_orientation")
        assert proto.HasField("imu_data")
        assert proto.HasField("motor_state")
        assert proto.robot_orientation == 1.57
        assert proto.motor_state.drive_speed == 10.0

    def test_optional_none_fields_are_left_unset(self):
        proto = tic._snapshot_to_proto(
            self._snapshot(
                robotPosition=None,
                robotOrientation=None,
                imuData=None,
                motorState=None,
                visionDetections=None,
            ),
        )
        assert not proto.HasField("robot_position")
        assert not proto.HasField("robot_orientation")
        assert not proto.HasField("imu_data")
        assert not proto.HasField("motor_state")
        assert len(proto.vision_detections) == 0


class TestTopicsSnapshotToProto:
    def test_maps_timestamp_and_topics(self):
        snapshot = TopicsSnapshot(
            timestamp=100.0,
            topics=[
                _TopicUpdatePayload(
                    topicName="/imu/data",
                    messageType="sensor_msgs/Imu",
                    timestamp=100.0,
                    updateRateHz=50.0,
                    data={},
                ),
            ],
        )
        proto = tic._topics_snapshot_to_proto(snapshot)
        assert proto.timestamp.seconds == 100
        assert len(proto.topics) == 1
        assert proto.topics[0].topic_name == "/imu/data"


class TestPushLatest:
    def test_two_pushes_onto_a_full_queue_keep_only_the_latest(self):
        channel = tic.TelemetryIngestChannel(backend_target="localhost:9010", logger=mock.Mock())
        q: queue.Queue = queue.Queue(maxsize=1)

        channel._push_latest(q, "first")
        channel._push_latest(q, "second")

        assert q.qsize() == 1
        assert q.get_nowait() == "second"

    def test_never_blocks_and_never_raises(self):
        channel = tic.TelemetryIngestChannel(backend_target="localhost:9010", logger=mock.Mock())
        q: queue.Queue = queue.Queue(maxsize=1)

        start = time.perf_counter()
        for i in range(20):
            channel._push_latest(q, i)
        elapsed = time.perf_counter() - start

        assert elapsed < 0.5
        assert q.get_nowait() == 19

    def test_push_snapshot_and_push_topics_use_the_keep_latest_queue(self):
        channel = tic.TelemetryIngestChannel(backend_target="localhost:9010", logger=mock.Mock())
        snapshot = RobotSnapshot(
            timestamp=1.0,
            missionName="m",
            robotPosition=None,
            robotOrientation=None,
            lidarPoints=[],
            pathHistory=[],
            logs=[],
            metrics=_metrics(),
            imuData=None,
            visionDetections=None,
            motorState=None,
        )
        topics = TopicsSnapshot(timestamp=1.0, topics=[])

        channel.push_snapshot(snapshot)
        channel.push_topics(topics)

        assert channel._snapshot_queue.qsize() == 1
        assert channel._topics_queue.qsize() == 1


def _blocking_run_stream(stop_event: threading.Event):
    """Stand-in for TelemetryIngestChannel._run_stream: blocks like the real

    RPC call would, without touching grpc, until stop() sets stop_event.
    """

    def _run(*_args: object, **_kwargs: object) -> None:
        stop_event.wait(5)

    return _run


class TestLifecycle:
    """start/stop/restart, with _run_stream mocked out so no real grpc connection is attempted."""

    def _wait_until(self, predicate, timeout=2.0) -> bool:
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        return predicate()

    def test_start_spawns_two_daemon_threads(self):
        channel = tic.TelemetryIngestChannel(backend_target="localhost:9010", logger=mock.Mock())
        with mock.patch.object(channel, "_run_stream", side_effect=_blocking_run_stream(channel._stop_event)):
            channel.start()
            try:
                assert channel._snapshot_thread is not None
                assert channel._topics_thread is not None
                assert channel._snapshot_thread.daemon is True
                assert channel._topics_thread.daemon is True
                assert self._wait_until(channel._snapshot_thread.is_alive)
                assert self._wait_until(channel._topics_thread.is_alive)
            finally:
                channel.stop()

    def test_stop_sets_the_stop_event_and_threads_exit(self):
        channel = tic.TelemetryIngestChannel(backend_target="localhost:9010", logger=mock.Mock())
        with mock.patch.object(channel, "_run_stream", side_effect=_blocking_run_stream(channel._stop_event)):
            channel.start()
            snapshot_thread = channel._snapshot_thread
            topics_thread = channel._topics_thread
            self._wait_until(snapshot_thread.is_alive)

            channel.stop()

            assert channel._stop_event.is_set()
            assert self._wait_until(lambda: not snapshot_thread.is_alive())
            assert self._wait_until(lambda: not topics_thread.is_alive())

    def test_restart_after_stop_respawns_threads(self):
        channel = tic.TelemetryIngestChannel(backend_target="localhost:9010", logger=mock.Mock())
        with mock.patch.object(channel, "_run_stream", side_effect=_blocking_run_stream(channel._stop_event)):
            channel.start()
            first_snapshot_thread = channel._snapshot_thread
            channel.stop()
            self._wait_until(lambda: not first_snapshot_thread.is_alive())

            channel.restart()
            try:

                def _snapshot_thread_alive() -> bool:
                    return channel._snapshot_thread is not None and channel._snapshot_thread.is_alive()

                assert self._wait_until(_snapshot_thread_alive)
                assert not channel._stop_event.is_set()
            finally:
                channel.stop()

    def test_restart_is_a_no_op_when_already_running(self):
        channel = tic.TelemetryIngestChannel(backend_target="localhost:9010", logger=mock.Mock())
        with mock.patch.object(channel, "_run_stream", side_effect=_blocking_run_stream(channel._stop_event)):
            channel.start()
            self._wait_until(channel._snapshot_thread.is_alive)
            first_snapshot_thread = channel._snapshot_thread
            first_topics_thread = channel._topics_thread

            channel.restart()

            try:
                assert channel._snapshot_thread is first_snapshot_thread
                assert channel._topics_thread is first_topics_thread
            finally:
                channel.stop()


class TestOnStateChanged:
    def test_stop_fires_false(self):
        events = []
        channel = tic.TelemetryIngestChannel(backend_target="localhost:9010", logger=mock.Mock(), on_state_changed=events.append)

        channel.stop()

        assert events == [False]

    def test_run_stream_fires_true_on_connect(self):
        events = []
        fake_grpc_channel = mock.Mock()
        with mock.patch.object(tic.grpc, "insecure_channel", return_value=fake_grpc_channel):
            channel = tic.TelemetryIngestChannel(
                backend_target="localhost:9010",
                logger=mock.Mock(),
                on_state_changed=events.append,
            )
            stub = mock.Mock()
            stub.StreamSnapshots = mock.Mock(return_value=None)
            with mock.patch.object(tic.ingest_pb2_grpc, "TelemetryIngestServiceStub", return_value=stub):
                channel._run_stream("StreamSnapshots", channel._snapshot_queue)

        assert events == [True]
        fake_grpc_channel.close.assert_called_once()
