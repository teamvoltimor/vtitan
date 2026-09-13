"""Mock-hardware tests for bag_recorder_node.

No real `ros2 bag record` subprocess is spawned: subprocess.Popen is mocked
so these tests exercise the node's own gating/publishing logic against a
fake recorder process.
"""

from __future__ import annotations

import json
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


class TestProvenanceStamp:
    """A bag records what the robot did and nothing about which code did it.

    On 2026-09-13 a 55-run competition corpus had to be attributed by comparing
    a deploy note against `git rev-list`, and the first answer was wrong: a
    rebase had rewritten the hashes, so counting commits overstated the gap.
    These tests pin the stamp that removes the guesswork.
    """

    def test_the_stamp_lands_in_the_run_directory(
        self, ros_context, bag_recorder_node_class, tmp_path,
    ):
        node = _node(bag_recorder_node_class, tmp_path)
        node._provenance = {"commit": "deadbeef", "branch": "master", "dirty": "false"}

        with mock.patch(
            "vtitan_state_machine.bag_recorder_node.subprocess.Popen",
            side_effect=_fake_popen_creating_output_dir,
        ):
            node._on_robot_state(String(data="racing"))
        # `ros2 bag record` has created the directory by now, so the first poll
        # finds it; the timer only exists to cover the case where it has not.
        node._poll_for_run_dir()

        stamps = list(tmp_path.glob("run_*/provenance.json"))
        assert len(stamps) == 1
        written = json.loads(stamps[0].read_text(encoding="utf-8"))
        assert written["commit"] == "deadbeef"
        assert written["branch"] == "master"
        assert written["dirty"] == "false"
        # Run-specific fields are added at write time, not resolved at startup.
        assert written["run"] == stamps[0].parent.name
        assert "started_at" in written

        node.destroy_node()

    def test_a_directory_that_never_appears_warns_instead_of_raising(
        self, ros_context, bag_recorder_node_class, tmp_path,
    ):
        """The recorder can fail to create its directory. That must cost the
        stamp and nothing else -- never the round, and never an exception out of
        a timer callback."""
        node = _node(bag_recorder_node_class, tmp_path)

        with mock.patch("vtitan_state_machine.bag_recorder_node.subprocess.Popen") as popen_mock:
            popen_mock.return_value = _fake_popen()  # does NOT create the directory
            node._on_robot_state(String(data="racing"))

        assert node._provenance_timer is not None
        node._provenance_deadline = 0.0  # expire it
        node._poll_for_run_dir()

        assert node._provenance_timer is None
        assert list(tmp_path.glob("run_*/provenance.json")) == []

        node.destroy_node()

    def test_no_stamp_is_armed_when_the_recorder_fails_to_start(
        self, ros_context, bag_recorder_node_class, tmp_path,
    ):
        node = _node(bag_recorder_node_class, tmp_path)

        with mock.patch(
            "vtitan_state_machine.bag_recorder_node.subprocess.Popen",
            side_effect=OSError("no ros2 on PATH"),
        ):
            node._on_robot_state(String(data="racing"))

        assert node._provenance_timer is None

        node.destroy_node()

    def test_an_unwritable_run_directory_does_not_raise(
        self, ros_context, bag_recorder_node_class, tmp_path,
    ):
        node = _node(bag_recorder_node_class, tmp_path)
        run_path = tmp_path / "run_20260913_120000"
        run_path.mkdir()

        with mock.patch(
            "vtitan_state_machine.bag_recorder_node.Path.write_text",
            side_effect=OSError("read-only card"),
        ):
            node._write_provenance(run_path)  # must not raise

        node.destroy_node()


class TestCodeProvenance:
    def test_outside_a_git_checkout_the_commit_is_unknown_not_a_crash(self):
        from vtitan_state_machine import bag_recorder_node

        with mock.patch.object(bag_recorder_node, "_git", return_value=None):
            provenance = bag_recorder_node._code_provenance()

        assert provenance["commit"] == "unknown"
        assert "commit_source" in provenance

    def test_a_dirty_tree_is_reported_dirty(self):
        """The field that decides whether a stamp can be trusted at all: a dirty
        tree means the recorded commit does NOT describe what ran."""
        from vtitan_state_machine import bag_recorder_node

        with mock.patch.object(bag_recorder_node, "_git", side_effect=lambda _repo, *args: {
            ("rev-parse", "--show-toplevel"): "/repo",
            ("status", "--porcelain"): " M src/python/foo.py",
        }.get(args, "x")):
            provenance = bag_recorder_node._code_provenance()

        assert provenance["dirty"] == "true"

    def test_a_clean_tree_is_reported_clean(self):
        from vtitan_state_machine import bag_recorder_node

        with mock.patch.object(bag_recorder_node, "_git", side_effect=lambda _repo, *args: {
            ("rev-parse", "--show-toplevel"): "/repo",
            ("status", "--porcelain"): "",
        }.get(args, "x")):
            provenance = bag_recorder_node._code_provenance()

        assert provenance["dirty"] == "false"

    def test_git_failure_collapses_to_none_rather_than_raising(self):
        from vtitan_state_machine import bag_recorder_node

        with mock.patch(
            "vtitan_state_machine.bag_recorder_node.subprocess.run",
            side_effect=OSError("git not installed"),
        ):
            assert bag_recorder_node._git(Path("/repo"), "rev-parse", "HEAD") is None
