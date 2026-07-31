"""Backend->robot command channel (gRPC RobotCommandService).

Extracted from telemetry_bridge_node.py: this is a separate responsibility
from that module's sensor-telemetry POST loop (different direction of data
flow, own thread, own protobuf imports, own retry/backoff state) that shares
almost no state with it beyond the backend URL.
"""

from __future__ import annotations

import sys
import threading
from pathlib import Path
from typing import TYPE_CHECKING

import requests
from rcl_interfaces.msg import (
    Parameter as RosParameter,
    ParameterType,
    ParameterValue,
)
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String

if TYPE_CHECKING:
    from rclpy.client import Client
    from rclpy.impl.rcutils_logger import RcutilsLogger
    from rclpy.publisher import Publisher

# buf's Python plugin generates gen/telemetry/v1/*_pb2*.py rooted so that
# "telemetry.v1.commands_pb2" is a top-level import (its own internal imports
# assume that, e.g. commands_pb2_grpc.py does "from telemetry.v1 import
# commands_pb2") -- so platform/robot/src/gen itself, not just platform/robot,
# has to be on sys.path. Done here rather than via PYTHONPATH so this node
# works the same whether launched by a pixi task, `ros2 run`, or a test.
_GEN_ROOT = Path(__file__).resolve().parents[4] / "src" / "gen"
if str(_GEN_ROOT) not in sys.path:
    sys.path.insert(0, str(_GEN_ROOT))

import grpc  # noqa: E402
from telemetry.v1 import commands_pb2, commands_pb2_grpc  # noqa: E402

_BACKOFF_INITIAL = 1.0
_BACKOFF_MAX = 60.0


class CommandChannel:
    """Owns the StreamCommands loop: robot-id resolution, dispatch, and ack.

    Runs its own background thread deliberately -- stub.StreamCommands()
    blocks on stream.recv() for the life of the connection, and would stall
    the owning node's whole single-threaded executor (every sensor callback,
    every publish timer) if it ran on that thread instead.
    """

    def __init__(
        self,
        backend_url: str,
        command_channel_target: str,
        button_pub: Publisher,
        vision_params_client: Client,
        logger: RcutilsLogger,
    ) -> None:
        self._backend_url = backend_url
        self._command_channel_target = command_channel_target
        self._button_pub = button_pub
        self._vision_params_client = vision_params_client
        self._logger = logger

        self._session = requests.Session()

        # This is a single-robot system: one seeded default Robot per backend
        # process, with no self-registration/identity flow yet. The robot
        # can't know that backend-assigned UUID (StreamCommandsRequest.robot_id
        # must match it) any other way than asking, so the command-channel
        # thread resolves it from GET {backend_url}/v1/robots before it can
        # open a stream.
        self._robot_id: str | None = None
        self._last_command_id: str | None = None
        self._command_backoff_delay: float = _BACKOFF_INITIAL
        self._command_stream_stop = threading.Event()

    def start(self) -> None:
        threading.Thread(target=self._command_channel_loop, name="robot-command-channel", daemon=True).start()

    def stop(self) -> None:
        self._command_stream_stop.set()

    def _resolve_robot_id(self) -> str | None:
        """GET {backend_url}/v1/robots and return the first robot's id.

        Single-robot system: exactly one robot is seeded per backend process
        (see cmd/server/main.go's defaultRobotName), so "the first one" is
        unambiguous. Returns None (retry later) on any failure.
        """
        try:
            r = self._session.get(f"{self._backend_url}/v1/robots", timeout=2.0)
            r.raise_for_status()
            robots = r.json()
        except (requests.exceptions.RequestException, ValueError) as exc:
            self._logger.warning(f"Could not resolve robot id: {exc}", throttle_duration_sec=30.0)
            return None
        if not robots:
            self._logger.warning("Backend has no registered robots yet", throttle_duration_sec=30.0)
            return None
        return str(robots[0]["id"])

    def _command_channel_loop(self) -> None:
        """Background thread: resolve robot_id, then keep StreamCommands open."""
        while not self._command_stream_stop.is_set():
            if self._robot_id is None:
                self._robot_id = self._resolve_robot_id()
                if self._robot_id is None:
                    self._command_stream_stop.wait(_BACKOFF_INITIAL)
                    continue
                self._logger.info(f"Resolved robot id for command channel: {self._robot_id}")

            try:
                self._run_command_stream(self._robot_id)
            except grpc.RpcError as exc:
                self._logger.warning(f"Command stream error: {exc.code()} {exc.details()}")
            except Exception as exc:  # noqa: BLE001
                self._logger.warning(f"Command stream error: {exc}")

            if self._command_stream_stop.is_set():
                return
            self._command_stream_stop.wait(self._command_backoff_delay)
            self._command_backoff_delay = min(self._command_backoff_delay * 2, _BACKOFF_MAX)

    def _run_command_stream(self, robot_id: str) -> None:
        """Open StreamCommands and apply/ack commands until it drops."""
        channel = grpc.insecure_channel(self._command_channel_target)
        try:
            stub = commands_pb2_grpc.RobotCommandServiceStub(channel)
            request = commands_pb2.StreamCommandsRequest(robot_id=robot_id)
            if self._last_command_id:
                request.last_command_id = self._last_command_id
            self._logger.info(f"Opening command stream to {self._command_channel_target}")

            for cmd in stub.StreamCommands(request):
                self._command_backoff_delay = _BACKOFF_INITIAL
                self._last_command_id = cmd.command_id
                status, message = self._dispatch_command(cmd)
                self._ack_command(stub, robot_id, cmd.command_id, status, message)
                if self._command_stream_stop.is_set():
                    return
        finally:
            channel.close()

    def _dispatch_command(self, cmd: commands_pb2.RobotCommand) -> tuple[int, str]:
        """Apply an incoming RobotCommand and report the outcome to ack."""
        kind = cmd.WhichOneof("payload")
        if kind == "start_race":
            return self._dispatch_button_event("short_press", "start race")
        if kind in ("stop_race", "emergency_stop"):
            # state_machine_node draws no distinction between a graceful stop
            # and an e-stop today -- long_press is its only "stop driving"
            # trigger, so both command types map onto it rather than
            # fabricating a state-machine transition that doesn't exist.
            return self._dispatch_button_event("long_press", kind.replace("_", " "))
        if kind == "set_vision_debug":
            return self._dispatch_set_vision_debug(cmd.set_vision_debug)
        return (
            commands_pb2.COMMAND_EXECUTION_STATUS_FAILED,
            f"command type '{kind}' is not implemented on this robot",
        )

    def _dispatch_button_event(self, event: str, label: str) -> tuple[int, str]:
        """Publish a synthetic /button/event, the only trigger state_machine_node exposes.

        Fire-and-forget from here: state_machine_node only reacts when its
        current state matches the expected transition (e.g. "short_press" is
        a no-op outside READY), and there is no reply path to observe that,
        so this acks ACCEPTED (delivered) rather than COMPLETED.
        """
        msg = String()
        msg.data = event
        self._button_pub.publish(msg)
        return (
            commands_pb2.COMMAND_EXECUTION_STATUS_ACCEPTED,
            f"published synthetic /button/event '{event}' for {label}",
        )

    def _dispatch_set_vision_debug(self, params: commands_pb2.SetVisionDebugParams) -> tuple[int, str]:
        """Forward enabled/stream_fps to vision_detector's set_parameters service."""
        if not self._vision_params_client.service_is_ready():
            return commands_pb2.COMMAND_EXECUTION_STATUS_FAILED, "vision_detector parameter service unavailable"

        request = SetParameters.Request()
        request.parameters.append(
            RosParameter(
                name="publish_annotated",
                value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=params.enabled),
            ),
        )
        if params.HasField("stream_fps"):
            request.parameters.append(
                RosParameter(
                    name="debug_stream_fps",
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(params.stream_fps)),
                ),
            )

        # call_async's future is fulfilled by whichever thread is spinning
        # the owning node's executor (main()'s rclpy.spin call) -- not this
        # background thread -- so block on a plain threading.Event rather
        # than rclpy.spin_until_future_complete, which requires being called
        # from the spinning thread itself.
        future = self._vision_params_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=2.0):
            return commands_pb2.COMMAND_EXECUTION_STATUS_FAILED, "vision_detector set_parameters call timed out"

        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            return commands_pb2.COMMAND_EXECUTION_STATUS_FAILED, f"set_parameters call failed: {exc}"
        if response is None:
            return commands_pb2.COMMAND_EXECUTION_STATUS_FAILED, "vision_detector set_parameters returned no response"

        if all(result.successful for result in response.results):
            return commands_pb2.COMMAND_EXECUTION_STATUS_COMPLETED, "vision debug stream updated"
        reasons = "; ".join(result.reason for result in response.results if not result.successful)
        return commands_pb2.COMMAND_EXECUTION_STATUS_FAILED, f"vision_detector rejected parameters: {reasons}"

    def _ack_command(
        self,
        stub: commands_pb2_grpc.RobotCommandServiceStub,
        robot_id: str,
        command_id: str,
        status: int,
        message: str,
    ) -> None:
        try:
            stub.AckCommand(
                commands_pb2.AckCommandRequest(
                    robot_id=robot_id,
                    command_id=command_id,
                    status=status,
                    message=message,
                ),
                timeout=2.0,
            )
        except grpc.RpcError as exc:
            self._logger.warning(f"AckCommand failed: {exc.code()} {exc.details()}")
