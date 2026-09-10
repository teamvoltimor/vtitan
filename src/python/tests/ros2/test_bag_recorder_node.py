"""Mock-hardware tests for bag_recorder_node.

No real `ros2 bag record` subprocess is spawned: subprocess.Popen is mocked
so these tests exercise the node's own gating/publishing logic against a
fake recorder process.
"""

from __future__ import annotations

from pathlib import Path
from unittest import mock

import pytest
from std_msgs.msg import String


@pytest.fixture(autouse=True)
def _posix_process_group_signals():
    """os.killpg/getpgid are POSIX-only and absent on this Windows dev/test
    machine, but every test's node.destroy_node() calls _stop_recording(),
    which uses them whenever a recording was left running -- patch them in
    for the duration of every test in this module, the same way they'd exist
    on the Pi 5 this node actually runs on."""
    with (
        mock.patch("vtitan_state_machine.bag_recorder_node.os.getpgid", create=True, return_value=1),
        mock.patch("vtitan_state_machine.bag_recorder_node.os.killpg", create=True),
    ):
        yield


@pytest.fixture()
def bag_recorder_node_class():
    from vtitan_state_machine.bag_recorder_node import BagRecorderNode

    return BagRecorderNode


def _node(bag_recorder_node_class, tmp_path):
    """A node whose bag_dir is a temp directory, not the real ~/vtitan_runs.

    BagRecorderNode's own ROS2 parameter declaration reads bag_dir once at
    construction with no live-override callback, so overriding it means
    reaching in directly after construction rather than a parameter override.
    """
    node = bag_recorder_node_class()
    node._bag_dir = tmp_path
    return node


def _fake_popen():
    proc = mock.MagicMock(name="ros2_bag_record_process")
    proc.pid = 12345
    proc.poll.return_value = 0  # already exited, so _stop_recording's grace loop doesn't spin
    return proc


def _fake_popen_creating_output_dir(argv, **_kwargs):
    """Mimic the one side effect of real `ros2 bag record` this test suite cares
    about: it creates its `-o <path>` output directory. Nothing else calls
    real bag_recorder_node code that relies on the directory's contents."""
    Path(argv[argv.index("-o") + 1]).mkdir(parents=True)
    return _fake_popen()


class TestRunPathPublish:
    def test_run_path_is_published_matching_the_popen_output_dir(
        self, ros_context, bag_recorder_node_class, tmp_path,
    ):
        node = _node(bag_recorder_node_class, tmp_path)
        published: list[String] = []
        node._run_path_pub.publish = published.append

        with mock.patch("vtitan_state_machine.bag_recorder_node.subprocess.Popen") as popen_mock:
            popen_mock.return_value = _fake_popen()
            node._on_robot_state(String(data="racing"))

        assert len(published) == 1
        argv = popen_mock.call_args[0][0]
        run_path_arg = argv[argv.index("-o") + 1]
        assert published[0].data == run_path_arg

        node.destroy_node()

    def test_no_publish_when_popen_fails(self, ros_context, bag_recorder_node_class, tmp_path):
        node = _node(bag_recorder_node_class, tmp_path)
        published: list[String] = []
        node._run_path_pub.publish = published.append

        with mock.patch("vtitan_state_machine.bag_recorder_node.subprocess.Popen", side_effect=OSError("boom")):
            node._on_robot_state(String(data="racing"))

        assert published == []

        node.destroy_node()

    def test_no_publish_when_disabled(self, ros_context, bag_recorder_node_class, tmp_path):
        node = _node(bag_recorder_node_class, tmp_path)
        node._enabled = False
        published: list[String] = []
        node._run_path_pub.publish = published.append

        with mock.patch("vtitan_state_machine.bag_recorder_node.subprocess.Popen") as popen_mock:
            node._on_robot_state(String(data="racing"))

        popen_mock.assert_not_called()
        assert published == []

        node.destroy_node()

    def test_run_path_published_again_on_a_second_race_after_stop(
        self, ros_context, bag_recorder_node_class, tmp_path,
    ):
        """A round can cycle FINISHED -> BOOT_CHECK -> READY -> RACING purely from
        the button -- the second race's run path must be published too, not just the first's."""
        node = _node(bag_recorder_node_class, tmp_path)
        published: list[String] = []
        node._run_path_pub.publish = published.append

        with mock.patch(
            "vtitan_state_machine.bag_recorder_node.subprocess.Popen",
            side_effect=_fake_popen_creating_output_dir,
        ):
            node._on_robot_state(String(data="racing"))
            node._on_robot_state(String(data="finished"))
            node._on_robot_state(String(data="racing"))

        assert len(published) == 2
        assert published[0].data != published[1].data  # distinct run directories

        node.destroy_node()
