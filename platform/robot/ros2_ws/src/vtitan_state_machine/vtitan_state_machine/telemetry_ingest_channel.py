"""Robot->backend telemetry channel (gRPC TelemetryIngestService).

Replaces telemetry_bridge_node.py's old HTTP POST loop: the backend already
exposes a persistent client-streaming ingest RPC (one HTTP/2 connection for
the whole 10Hz feed instead of a POST per tick), and this module owns that
connection the same way command_channel.py owns the backend->robot
direction -- its own background threads, own protobuf imports, own
retry/backoff state.
"""

from __future__ import annotations

import queue
import sys
import threading
from contextlib import suppress
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator, Sequence

    from rclpy.impl.rcutils_logger import RcutilsLogger

    from vtitan_state_machine.telemetry_bridge_node import (
        RobotSnapshot,
        TelemetryMetrics,
        TopicsSnapshot,
        _IMUPayload,
        _MotorStatePayload,
        _TopicUpdatePayload,
        _VisionDetectionPayload,
    )

# Same sys.path bootstrap as command_channel.py -- buf's Python plugin
# generates gen/telemetry/v1/*_pb2*.py rooted so that "telemetry.v1.ingest_pb2"
# is a top-level import, so platform/robot/src/gen itself has to be on
# sys.path. Idempotent (both modules doing this insert is fine).
_GEN_ROOT = Path(__file__).resolve().parents[4] / "src" / "gen"
if str(_GEN_ROOT) not in sys.path:
    sys.path.insert(0, str(_GEN_ROOT))

import grpc  # noqa: E402
from google.protobuf import struct_pb2, timestamp_pb2  # noqa: E402
from telemetry.v1 import ingest_pb2, ingest_pb2_grpc, types_pb2  # noqa: E402

_BACKOFF_INITIAL = 1.0
_BACKOFF_MAX = 60.0


def _position3d(values: Sequence[float]) -> types_pb2.Position3D:
    x, y, z = values
    return types_pb2.Position3D(x=x, y=y, z=z)


def _timestamp_from_epoch(epoch: float) -> timestamp_pb2.Timestamp:
    seconds = int(epoch)
    nanos = round((epoch - seconds) * 1e9)
    return timestamp_pb2.Timestamp(seconds=seconds, nanos=nanos)


def _node_health(node_health: str) -> int:
    """Map the bridge's free-form nodeHealth string onto the NodeHealth enum.

    Falls back to NODE_HEALTH_UNSPECIFIED (the zero value) for anything that
    doesn't match a known enum name, rather than raising -- a stream thread
    dying on an unrecognized status string would drop the whole snapshot.
    """
    try:
        return types_pb2.NodeHealth.Value(f"NODE_HEALTH_{node_health.upper()}")
    except ValueError:
        return types_pb2.NODE_HEALTH_UNSPECIFIED


def _imu_data(imu: _IMUPayload) -> types_pb2.ImuData:
    ax, ay, az = imu.linearAcceleration
    gx, gy, gz = imu.angularVelocity
    qx, qy, qz, qw = imu.orientationQuaternion
    return types_pb2.ImuData(
        linear_acceleration=types_pb2.Position3D(x=ax, y=ay, z=az),
        angular_velocity=types_pb2.Position3D(x=gx, y=gy, z=gz),
        orientation_x=qx,
        orientation_y=qy,
        orientation_z=qz,
        orientation_w=qw,
    )


def _motor_state(motor: _MotorStatePayload) -> types_pb2.MotorState:
    return types_pb2.MotorState(
        steering_angle=motor.steeringAngle,
        drive_speed=motor.driveSpeed,
        encoder_position=motor.encoderPosition,
    )


def _detection(detection: _VisionDetectionPayload) -> types_pb2.Detection:
    bx, by, bw, bh = detection.bbox
    return types_pb2.Detection(
        class_name=str(detection.className),
        confidence=detection.confidence,
        bbox_x=bx,
        bbox_y=by,
        bbox_w=bw,
        bbox_h=bh,
    )


def _metrics(metrics: TelemetryMetrics) -> types_pb2.TelemetryMetrics:
    proto = types_pb2.TelemetryMetrics(
        timestamp=_timestamp_from_epoch(metrics.timestamp),
        node_health=_node_health(metrics.nodeHealth),
        points_captured=metrics.pointsCaptured,
        stage=metrics.stage,
        lidar_available=metrics.lidarAvailable,
        imu_available=metrics.imuAvailable,
        camera_available=metrics.cameraAvailable,
        odometry_available=metrics.odometryAvailable,
    )
    # optional double fields: only set when present, rather than coercing
    # None to 0.0 -- these are proto3 "optional" (explicit-presence) fields
    # and the backend distinguishes "not measured" from "measured as zero".
    if metrics.rangeMin is not None:
        proto.range_min = metrics.rangeMin
    if metrics.rangeMax is not None:
        proto.range_max = metrics.rangeMax
    if metrics.rangeMean is not None:
        proto.range_mean = metrics.rangeMean
    if metrics.forward is not None:
        proto.forward = metrics.forward
    if metrics.left is not None:
        proto.left = metrics.left
    if metrics.right is not None:
        proto.right = metrics.right
    if metrics.back is not None:
        proto.back = metrics.back
    if metrics.speed is not None:
        proto.speed = metrics.speed
    return proto


def _struct_from_dict(data: dict[str, Any]) -> struct_pb2.Struct:
    """Best-effort dict->Struct conversion.

    "data" is a recursive reflection of an arbitrary ROS2 message (see
    telemetry_bridge_node._msg_to_dict) -- it's expected to already be
    JSON-shaped, but a stray non-JSON-serializable value (e.g. bytes) must
    not kill the stream thread over one topic's diagnostic payload.
    """
    struct = struct_pb2.Struct()
    with suppress(TypeError, ValueError):
        struct.update(data)
    return struct


def _topic_update(update: _TopicUpdatePayload) -> types_pb2.TopicUpdate:
    return types_pb2.TopicUpdate(
        topic_name=update.topicName,
        message_type=update.messageType,
        timestamp=_timestamp_from_epoch(update.timestamp),
        update_rate_hz=update.updateRateHz,
        data=_struct_from_dict(update.data),
    )


def _snapshot_to_proto(snapshot: RobotSnapshot) -> types_pb2.RobotSnapshot:
    proto = types_pb2.RobotSnapshot(
        timestamp=_timestamp_from_epoch(snapshot.timestamp),
        mission_name=snapshot.missionName,
        lidar_points=[_position3d(p) for p in snapshot.lidarPoints],
        path_history=[_position3d(p) for p in snapshot.pathHistory],
        logs=list(snapshot.logs),
        metrics=_metrics(snapshot.metrics),
        vision_detections=[_detection(d) for d in snapshot.visionDetections or []],
    )
    if snapshot.robotPosition is not None:
        proto.robot_position.CopyFrom(_position3d(snapshot.robotPosition))
    if snapshot.robotOrientation is not None:
        proto.robot_orientation = snapshot.robotOrientation
    if snapshot.imuData is not None:
        proto.imu_data.CopyFrom(_imu_data(snapshot.imuData))
    if snapshot.motorState is not None:
        proto.motor_state.CopyFrom(_motor_state(snapshot.motorState))
    return proto


def _topics_snapshot_to_proto(topics: TopicsSnapshot) -> types_pb2.TopicsSnapshot:
    return types_pb2.TopicsSnapshot(
        timestamp=_timestamp_from_epoch(topics.timestamp),
        topics=[_topic_update(t) for t in topics.topics],
    )


class TelemetryIngestChannel:
    """Owns two persistent client-streaming RPCs: StreamSnapshots and StreamTopics.

    Each runs its own background thread for the same reason CommandChannel
    does -- stub.StreamSnapshots()/StreamTopics() blocks for the life of the
    connection, and would stall the owning node's single-threaded executor
    if run there instead. Producers (telemetry_bridge_node) call
    push_snapshot/push_topics, which never block: each stream has a
    maxsize=1 keep-latest queue, so a slow/reconnecting backend just means
    the next tick's data overwrites the last unsent one.
    """

    def __init__(
        self,
        backend_target: str,
        logger: RcutilsLogger,
        on_state_changed: Callable[[bool], None] | None = None,
    ) -> None:
        self._backend_target = backend_target
        self._logger = logger
        self._on_state_changed = on_state_changed

        self._snapshot_queue: queue.Queue[ingest_pb2.StreamSnapshotsRequest] = queue.Queue(maxsize=1)
        self._topics_queue: queue.Queue[ingest_pb2.StreamTopicsRequest] = queue.Queue(maxsize=1)

        self._stop_event = threading.Event()
        self._snapshot_thread: threading.Thread | None = None
        self._topics_thread: threading.Thread | None = None

    def start(self) -> None:
        if self._is_running():
            return
        self._stop_event.clear()
        self._snapshot_thread = threading.Thread(
            target=self._stream_loop,
            args=("StreamSnapshots", self._snapshot_queue),
            name="telemetry-ingest-snapshots",
            daemon=True,
        )
        self._topics_thread = threading.Thread(
            target=self._stream_loop,
            args=("StreamTopics", self._topics_queue),
            name="telemetry-ingest-topics",
            daemon=True,
        )
        self._snapshot_thread.start()
        self._topics_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._on_state_changed is not None:
            connected = False
            self._on_state_changed(connected)

    def restart(self) -> None:
        if self._is_running():
            return
        self._stop_event.clear()
        self.start()

    def push_snapshot(self, snapshot: RobotSnapshot) -> None:
        # StreamSnapshots' wire type is StreamSnapshotsRequest{snapshot=1},
        # not a bare RobotSnapshot -- sending the unwrapped message serializes
        # its fields under RobotSnapshot's own field numbers, which the
        # server then misparses as StreamSnapshotsRequest.snapshot's nested
        # bytes (both happen to be wire type 2 at field 1, so it "parses"
        # into an empty/garbage snapshot instead of failing outright).
        request = ingest_pb2.StreamSnapshotsRequest(snapshot=_snapshot_to_proto(snapshot))
        self._push_latest(self._snapshot_queue, request)

    def push_topics(self, topics: TopicsSnapshot) -> None:
        request = ingest_pb2.StreamTopicsRequest(topics=_topics_snapshot_to_proto(topics))
        self._push_latest(self._topics_queue, request)

    def _is_running(self) -> bool:
        return (self._snapshot_thread is not None and self._snapshot_thread.is_alive()) or (
            self._topics_thread is not None and self._topics_thread.is_alive()
        )

    @staticmethod
    def _push_latest(q: queue.Queue, item: object) -> None:
        """Non-blocking keep-latest push: never makes the caller wait on a stalled stream."""
        if q.full():
            with suppress(queue.Empty):
                q.get_nowait()
        with suppress(queue.Full):
            q.put_nowait(item)

    def _drain(self, q: queue.Queue) -> Iterator:
        """Generator fed to stub.StreamSnapshots/StreamTopics -- yields until stop() is set."""
        while not self._stop_event.is_set():
            try:
                item = q.get(timeout=0.5)
            except queue.Empty:
                continue
            yield item

    def _stream_loop(self, rpc_name: str, q: queue.Queue) -> None:
        """Background thread: keep the given RPC's client-stream open with backoff."""
        backoff = _BACKOFF_INITIAL
        while not self._stop_event.is_set():
            try:
                self._run_stream(rpc_name, q)
            except grpc.RpcError as exc:
                self._logger.warning(f"{rpc_name} stream error: {exc.code()} {exc.details()}")
            except Exception as exc:  # noqa: BLE001
                self._logger.warning(f"{rpc_name} stream error: {exc}")

            if self._on_state_changed is not None:
                connected = False
                self._on_state_changed(connected)

            if self._stop_event.is_set():
                return
            self._stop_event.wait(backoff)
            backoff = min(backoff * 2, _BACKOFF_MAX)

    def _run_stream(self, rpc_name: str, q: queue.Queue) -> None:
        channel = grpc.insecure_channel(self._backend_target)
        try:
            stub = ingest_pb2_grpc.TelemetryIngestServiceStub(channel)
            rpc = getattr(stub, rpc_name)
            self._logger.info(f"Opening {rpc_name} stream to {self._backend_target}")
            if self._on_state_changed is not None:
                connected = True
                self._on_state_changed(connected)
            rpc(self._drain(q))
        finally:
            channel.close()
