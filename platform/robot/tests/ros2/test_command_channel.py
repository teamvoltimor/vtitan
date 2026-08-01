"""Tests for command_channel.CommandChannel.

No real network/rclpy node: CommandChannel takes its collaborators
(button_pub, vision_params_client, logger, telemetry_channel) as plain
constructor args, so it's testable in isolation the same way
telemetry_ingest_channel.py's TelemetryIngestChannel is -- see
test_telemetry_ingest_channel.py for that module's equivalent coverage.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from unittest import mock

import grpc
import pytest
import requests
from vtitan_state_machine.command_channel import CommandChannel
from vtitan_state_machine.telemetry_ingest_channel import TelemetryIngestChannel

# Same sys.path bootstrap command_channel.py itself does (its own import
# above already performed it as a side effect, but this makes the
# commands_pb2 import below self-contained rather than relying on that).
_GEN_ROOT = Path(__file__).resolve().parents[2] / "src" / "gen"
if str(_GEN_ROOT) not in sys.path:
    sys.path.insert(0, str(_GEN_ROOT))

from telemetry.v1 import commands_pb2


def _channel(**overrides) -> CommandChannel:
    kwargs = {
        "backend_url": "http://localhost:8010",
        "command_channel_target": "localhost:9010",
        "button_pub": mock.Mock(),
        "vision_params_client": mock.Mock(),
        "logger": mock.Mock(),
    }
    kwargs.update(overrides)
    return CommandChannel(**kwargs)


class TestRestartIdempotency:
    def test_restart_is_a_no_op_when_already_running(self):
        channel = _channel()
        channel.start()
        try:
            first_thread = channel._thread
            assert first_thread is not None
            assert first_thread.is_alive()

            channel.restart()

            assert channel._thread is first_thread, "restart() must not spawn a second thread while running"
        finally:
            channel.stop()

    def test_restart_after_stop_respawns_the_thread(self):
        channel = _channel()
        channel.start()
        try:
            first_thread = channel._thread
            channel.stop()
            # _resolve_robot_id's own GET carries a 2.0s timeout, and the
            # loop may be mid-call when stop() fires, so give this more
            # headroom than a typical fast-poll teardown.
            for _ in range(100):
                if not first_thread.is_alive():
                    break
                time.sleep(0.1)
            assert not first_thread.is_alive()

            channel._command_backoff_delay = 30.0
            channel.restart()

            assert channel._thread is not None
            assert channel._thread is not first_thread or channel._thread.is_alive()
            assert not channel._command_stream_stop.is_set()
            assert channel._command_backoff_delay == pytest.approx(1.0)
        finally:
            channel.stop()

    def test_start_is_a_no_op_when_already_running(self):
        channel = _channel()
        channel.start()
        try:
            first_thread = channel._thread
            channel.start()
            assert channel._thread is first_thread
        finally:
            channel.stop()


class TestOnStateChanged:
    def test_stop_fires_false(self):
        events = []
        channel = _channel(on_state_changed=events.append)

        channel.stop()

        assert events == [False]

    def test_stop_without_callback_does_not_raise(self):
        channel = _channel(on_state_changed=None)
        channel.stop()  # must not raise


class TestDispatchDisableCommandChannel:
    def test_routes_to_dispatch_disable_command_channel(self):
        channel = _channel()
        cmd = commands_pb2.RobotCommand(disable_command_channel=commands_pb2.DisableCommandChannelParams())

        with mock.patch.object(
            channel,
            "_dispatch_disable_command_channel",
            wraps=channel._dispatch_disable_command_channel,
        ) as spy:
            status, _message = channel._dispatch_command(cmd)

        spy.assert_called_once()
        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_COMPLETED

    def test_dispatch_disable_command_channel_stops_and_completes(self):
        channel = _channel()
        channel.stop = mock.Mock(wraps=channel.stop)

        status, message = channel._dispatch_disable_command_channel()

        channel.stop.assert_called_once()
        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_COMPLETED
        assert "command channel disabled" in message
        assert channel._command_stream_stop.is_set()


class TestDispatchSetTelemetryChannel:
    def test_routes_to_dispatch_set_telemetry_channel(self):
        telemetry_channel = mock.Mock(spec=TelemetryIngestChannel)
        channel = _channel(telemetry_channel=telemetry_channel)
        cmd = commands_pb2.RobotCommand(
            set_telemetry_channel=commands_pb2.SetTelemetryChannelParams(enabled=True),
        )

        with mock.patch.object(
            channel,
            "_dispatch_set_telemetry_channel",
            wraps=channel._dispatch_set_telemetry_channel,
        ) as spy:
            status, _message = channel._dispatch_command(cmd)

        spy.assert_called_once()
        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_COMPLETED

    def test_enabled_true_restarts_telemetry_channel(self):
        telemetry_channel = mock.Mock(spec=TelemetryIngestChannel)
        channel = _channel(telemetry_channel=telemetry_channel)
        params = commands_pb2.SetTelemetryChannelParams(enabled=True)

        status, message = channel._dispatch_set_telemetry_channel(params)

        telemetry_channel.restart.assert_called_once()
        telemetry_channel.stop.assert_not_called()
        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_COMPLETED
        assert "enabled" in message

    def test_enabled_false_stops_telemetry_channel(self):
        telemetry_channel = mock.Mock(spec=TelemetryIngestChannel)
        channel = _channel(telemetry_channel=telemetry_channel)
        params = commands_pb2.SetTelemetryChannelParams(enabled=False)

        status, message = channel._dispatch_set_telemetry_channel(params)

        telemetry_channel.stop.assert_called_once()
        telemetry_channel.restart.assert_not_called()
        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_COMPLETED
        assert "disabled" in message

    def test_returns_failed_when_no_telemetry_channel_configured(self):
        channel = _channel(telemetry_channel=None)
        params = commands_pb2.SetTelemetryChannelParams(enabled=True)

        status, message = channel._dispatch_set_telemetry_channel(params)

        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_FAILED
        assert "not configured" in message


class _FakeRpcError(grpc.RpcError):
    """grpc.RpcError itself exposes no code()/details() -- those come from grpc.Call,
    which real errors mix in. Production code only calls those two methods, so a
    minimal stand-in is enough.
    """

    def __init__(self, code=grpc.StatusCode.UNAVAILABLE, details="unavailable"):
        self._code = code
        self._details = details

    def code(self):
        return self._code

    def details(self):
        return self._details


class TestResolveRobotId:
    def test_connection_error_returns_none_and_warns(self):
        channel = _channel()
        channel._session.get = mock.Mock(side_effect=requests.exceptions.ConnectionError("refused"))

        result = channel._resolve_robot_id()

        assert result is None
        channel._logger.warning.assert_called_once()

    def test_malformed_json_returns_none_and_warns(self):
        channel = _channel()
        response = mock.Mock()
        response.raise_for_status = mock.Mock()
        response.json = mock.Mock(side_effect=ValueError("not json"))
        channel._session.get = mock.Mock(return_value=response)

        result = channel._resolve_robot_id()

        assert result is None
        channel._logger.warning.assert_called_once()

    def test_http_error_returns_none_and_warns(self):
        channel = _channel()
        response = mock.Mock()
        response.raise_for_status = mock.Mock(side_effect=requests.exceptions.HTTPError("500"))
        channel._session.get = mock.Mock(return_value=response)

        result = channel._resolve_robot_id()

        assert result is None
        channel._logger.warning.assert_called_once()

    def test_empty_robot_list_returns_none_and_warns(self):
        channel = _channel()
        response = mock.Mock()
        response.raise_for_status = mock.Mock()
        response.json = mock.Mock(return_value=[])
        channel._session.get = mock.Mock(return_value=response)

        result = channel._resolve_robot_id()

        assert result is None
        channel._logger.warning.assert_called_once()

    def test_returns_first_robot_id_as_string(self):
        channel = _channel()
        response = mock.Mock()
        response.raise_for_status = mock.Mock()
        response.json = mock.Mock(return_value=[{"id": 42}, {"id": 7}])
        channel._session.get = mock.Mock(return_value=response)

        result = channel._resolve_robot_id()

        assert result == "42"
        channel._session.get.assert_called_once_with("http://localhost:8010/v1/robots", timeout=2.0)


class TestDispatchLegacyBranches:
    def test_start_race_publishes_short_press_and_accepts(self):
        channel = _channel()
        cmd = commands_pb2.RobotCommand(start_race=commands_pb2.StartRaceParams(mission_name="m"))

        status, message = channel._dispatch_command(cmd)

        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_ACCEPTED
        published = channel._button_pub.publish.call_args[0][0]
        assert published.data == "short_press"
        assert "start race" in message

    def test_stop_race_publishes_long_press_and_accepts(self):
        channel = _channel()
        cmd = commands_pb2.RobotCommand(stop_race=commands_pb2.StopRaceParams())

        status, message = channel._dispatch_command(cmd)

        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_ACCEPTED
        published = channel._button_pub.publish.call_args[0][0]
        assert published.data == "long_press"
        assert "stop race" in message

    def test_emergency_stop_publishes_long_press_and_accepts(self):
        channel = _channel()
        cmd = commands_pb2.RobotCommand(emergency_stop=commands_pb2.EmergencyStopParams())

        status, message = channel._dispatch_command(cmd)

        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_ACCEPTED
        published = channel._button_pub.publish.call_args[0][0]
        assert published.data == "long_press"
        assert "emergency stop" in message

    def test_unimplemented_command_kind_returns_failed(self):
        channel = _channel()
        cmd = commands_pb2.RobotCommand(pause=commands_pb2.PauseParams())

        status, message = channel._dispatch_command(cmd)

        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_FAILED
        assert "pause" in message
        assert "not implemented" in message


class TestDispatchSetVisionDebug:
    def test_routes_to_dispatch_set_vision_debug(self):
        channel = _channel()
        cmd = commands_pb2.RobotCommand(set_vision_debug=commands_pb2.SetVisionDebugParams(enabled=True))

        with mock.patch.object(
            channel,
            "_dispatch_set_vision_debug",
            wraps=channel._dispatch_set_vision_debug,
        ) as spy:
            channel._vision_params_client.service_is_ready = mock.Mock(return_value=False)
            status, _message = channel._dispatch_command(cmd)

        spy.assert_called_once()
        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_FAILED

    def test_service_not_ready_returns_failed(self):
        channel = _channel()
        channel._vision_params_client.service_is_ready = mock.Mock(return_value=False)
        params = commands_pb2.SetVisionDebugParams(enabled=True)

        status, message = channel._dispatch_set_vision_debug(params)

        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_FAILED
        assert "unavailable" in message

    def test_call_timeout_returns_failed(self):
        channel = _channel()
        channel._vision_params_client.service_is_ready = mock.Mock(return_value=True)
        future = mock.Mock()
        future.add_done_callback = mock.Mock()  # deliberately never fires done
        channel._vision_params_client.call_async = mock.Mock(return_value=future)
        params = commands_pb2.SetVisionDebugParams(enabled=True)

        status, message = channel._dispatch_set_vision_debug(params)

        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_FAILED
        assert "timed out" in message

    def test_future_result_exception_returns_failed(self):
        channel = _channel()
        channel._vision_params_client.service_is_ready = mock.Mock(return_value=True)
        future = mock.Mock()

        def fake_add_done_callback(cb):
            cb(future)

        future.add_done_callback = mock.Mock(side_effect=fake_add_done_callback)
        future.result = mock.Mock(side_effect=RuntimeError("boom"))
        channel._vision_params_client.call_async = mock.Mock(return_value=future)
        params = commands_pb2.SetVisionDebugParams(enabled=True, stream_fps=5)

        status, message = channel._dispatch_set_vision_debug(params)

        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_FAILED
        assert "set_parameters call failed" in message

    def test_none_response_returns_failed(self):
        channel = _channel()
        channel._vision_params_client.service_is_ready = mock.Mock(return_value=True)
        future = mock.Mock()

        def fake_add_done_callback(cb):
            cb(future)

        future.add_done_callback = mock.Mock(side_effect=fake_add_done_callback)
        future.result = mock.Mock(return_value=None)
        channel._vision_params_client.call_async = mock.Mock(return_value=future)
        params = commands_pb2.SetVisionDebugParams(enabled=True)

        status, message = channel._dispatch_set_vision_debug(params)

        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_FAILED
        assert "no response" in message

    def test_partial_rejection_returns_failed_with_reasons(self):
        channel = _channel()
        channel._vision_params_client.service_is_ready = mock.Mock(return_value=True)
        future = mock.Mock()

        def fake_add_done_callback(cb):
            cb(future)

        future.add_done_callback = mock.Mock(side_effect=fake_add_done_callback)
        response = mock.Mock()
        response.results = [
            mock.Mock(successful=True, reason=""),
            mock.Mock(successful=False, reason="unsupported fps"),
        ]
        future.result = mock.Mock(return_value=response)
        channel._vision_params_client.call_async = mock.Mock(return_value=future)
        params = commands_pb2.SetVisionDebugParams(enabled=True)

        status, message = channel._dispatch_set_vision_debug(params)

        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_FAILED
        assert "unsupported fps" in message

    def test_happy_path_returns_completed(self):
        channel = _channel()
        channel._vision_params_client.service_is_ready = mock.Mock(return_value=True)
        future = mock.Mock()

        def fake_add_done_callback(cb):
            cb(future)

        future.add_done_callback = mock.Mock(side_effect=fake_add_done_callback)
        response = mock.Mock()
        response.results = [mock.Mock(successful=True, reason="")]
        future.result = mock.Mock(return_value=response)
        channel._vision_params_client.call_async = mock.Mock(return_value=future)
        params = commands_pb2.SetVisionDebugParams(enabled=True, stream_fps=15)

        status, message = channel._dispatch_set_vision_debug(params)

        assert status == commands_pb2.COMMAND_EXECUTION_STATUS_COMPLETED
        assert "vision debug stream updated" in message
        request = channel._vision_params_client.call_async.call_args[0][0]
        names = [p.name for p in request.parameters]
        assert "publish_annotated" in names
        assert "debug_stream_fps" in names


class TestAckCommand:
    def test_rpc_error_is_caught_and_logged_not_raised(self):
        channel = _channel()
        stub = mock.Mock()
        stub.AckCommand = mock.Mock(side_effect=_FakeRpcError())

        channel._ack_command(stub, "robot-1", "cmd-1", commands_pb2.COMMAND_EXECUTION_STATUS_COMPLETED, "ok")

        channel._logger.warning.assert_called_once()

    def test_success_does_not_log(self):
        channel = _channel()
        stub = mock.Mock()

        channel._ack_command(stub, "robot-1", "cmd-1", commands_pb2.COMMAND_EXECUTION_STATUS_COMPLETED, "ok")

        stub.AckCommand.assert_called_once()
        channel._logger.warning.assert_not_called()


class TestCommandChannelLoopBackoff:
    def test_robot_id_resolved_once_then_stream_reopened_until_stop(self):
        channel = _channel()
        channel._command_backoff_delay = 30.0
        channel._resolve_robot_id = mock.Mock(return_value="robot-1")

        call_count = 0

        def fake_run_command_stream(_robot_id):
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                return
            channel._command_stream_stop.set()

        channel._run_command_stream = mock.Mock(side_effect=fake_run_command_stream)
        channel._command_stream_stop.wait = mock.Mock(return_value=False)

        channel._command_channel_loop()

        assert call_count == 2
        channel._logger.info.assert_any_call("Resolved robot id for command channel: robot-1")

    def test_grpc_error_from_stream_is_caught_and_backoff_doubles(self):
        channel = _channel()
        channel._robot_id = "robot-1"
        channel._command_backoff_delay = 1.0
        channel._run_command_stream = mock.Mock(side_effect=_FakeRpcError())

        waits = []

        def fake_wait(timeout):
            waits.append(timeout)
            channel._command_stream_stop.set()
            return True

        channel._command_stream_stop.wait = mock.Mock(side_effect=fake_wait)

        channel._command_channel_loop()

        assert waits == [1.0]
        assert channel._command_backoff_delay == pytest.approx(2.0)
        channel._logger.warning.assert_called_once()

    def test_generic_exception_from_stream_is_caught_and_logged(self):
        channel = _channel()
        channel._robot_id = "robot-1"
        channel._run_command_stream = mock.Mock(side_effect=RuntimeError("boom"))

        def fake_wait(_timeout):
            channel._command_stream_stop.set()
            return True

        channel._command_stream_stop.wait = mock.Mock(side_effect=fake_wait)

        channel._command_channel_loop()

        channel._logger.warning.assert_called_once()
        assert "boom" in channel._logger.warning.call_args[0][0]

    def test_backoff_caps_at_backoff_max(self):
        channel = _channel()
        channel._robot_id = "robot-1"
        channel._command_backoff_delay = 45.0
        channel._run_command_stream = mock.Mock(side_effect=_FakeRpcError())

        def fake_wait(_timeout):
            channel._command_stream_stop.set()
            return True

        channel._command_stream_stop.wait = mock.Mock(side_effect=fake_wait)

        channel._command_channel_loop()

        assert channel._command_backoff_delay == pytest.approx(60.0)

    def test_stop_set_during_stream_returns_without_waiting(self):
        channel = _channel()
        channel._robot_id = "robot-1"

        def fake_run_command_stream(_robot_id):
            channel._command_stream_stop.set()

        channel._run_command_stream = mock.Mock(side_effect=fake_run_command_stream)
        channel._command_stream_stop.wait = mock.Mock()

        channel._command_channel_loop()

        channel._command_stream_stop.wait.assert_not_called()
