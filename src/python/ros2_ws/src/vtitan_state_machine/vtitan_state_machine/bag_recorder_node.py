"""Race-gated rosbag recorder.

race.launch.py used to start ``ros2 bag record`` unconditionally through an
ExecuteProcess action. That was harmless while the race launch was something an
operator ran by hand for one round, but it stops being harmless the moment that
launch becomes a boot-time service: the robot would then record from power-on,
writing bag after bag of a stationary robot and filling the SD card with runs
that contain no race at all.

So recording is gated on ``/robot_state``: a bag is opened when the state
machine enters RACING and closed when it leaves (finish or E-STOP). That also
makes each bag correspond exactly to one round, which is what you want when
replaying a bad run.

Old runs are pruned so the card cannot fill, following the same rule the
backend's session recorder uses (``pruneOldSessions`` in
``apps/backend/domain/session/sqlite/recorder.go``): drop the oldest until
the cap is satisfied.
"""

from __future__ import annotations

import contextlib
import json
import os
import shutil
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import override

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from shared.config.ros_topics import RosTopicConfig
from shared.domain.enums import RobotState
from std_msgs.msg import String

from src.config import launch_settings as _launch_settings
from src.config.launch_settings import RaceLaunchDefaults
from src.ros2.params import (
    declare_and_get_bool_param,
    declare_and_get_float_param,
    declare_and_get_int_param,
    declare_and_get_str_param,
)
from src.ros2.qos import QOS_LATCHED_STATE

# Fallback only -- race.launch.py always passes "topics" explicitly, sourced
# from RaceLaunchDefaults.bag_topics (src/python/src/config/launch_settings.py)
# so it stays configurable via config/launch/race.toml like the rest of this
# node's parameters. This default only matters if the node is ever run
# standalone (e.g. `ros2 run vtitan_state_machine bag_recorder_node`).
#
# Deliberately excludes /camera/image_raw. The camera was measured publishing
# 63 MB/s, which on this hardware is the difference between a bag that fits on
# the card and one that does not -- and the frames are reconstructible from the
# detections for the purpose these bags serve (replaying a bad run's decisions).
# Add it explicitly via the "topics" parameter if a specific investigation needs
# imagery.
_RACE_DEFAULTS = RaceLaunchDefaults()
_DEFAULT_TOPICS = _RACE_DEFAULTS.bag_topics

_BYTES_PER_GB = 1024**3

# rosbag2 finalizes its metadata on SIGINT; killing it outright leaves a bag
# that cannot be reindexed. This is how long to wait for that clean exit before
# escalating to SIGKILL.
_SHUTDOWN_GRACE_SEC = 10.0

# A bag records what the robot DID and says nothing about which code did it.
# That gap is not theoretical: on 2026-09-13 a 55-run competition corpus had to
# be attributed by comparing the deploy note against `git rev-list`, and the
# first answer was wrong -- a rebase had rewritten the hashes, so counting
# commits overstated the gap and named two flags as missing that ship OFF while
# missing the three that actually changed behaviour. Without a stamp no track
# A/B is attributable, which is the whole point of running one.
#
# Written beside the mcap rather than published on a topic so it survives a bag
# that never finalizes, and read without rosbag tooling.
_PROVENANCE_FILENAME = "provenance.json"
# `ros2 bag record` creates the directory itself, and must: it refuses to write
# into one that already exists, so this node cannot pre-create it. Poll for it
# the same way vision_node's video recorder does rather than sleep blindly.
_PROVENANCE_POLL_INTERVAL_SEC = 0.25
_PROVENANCE_POLL_TIMEOUT_SEC = 10.0
# git on a cold page cache is not instant, and this runs once at startup rather
# than per round, so a generous bound costs nothing and a hang costs a round.
_GIT_TIMEOUT_SEC = 5.0


def _git(repo: Path, *args: str) -> str | None:
    """One git command, or ``None`` for any failure at all.

    Provenance is a diagnostic nicety and the race is not. Every failure mode --
    git absent, not a repository, a lock held by a concurrent command, a
    timeout -- collapses to ``None`` so the caller records "unknown" instead of
    raising inside a node whose job is to stay out of the way.
    """
    try:
        result = subprocess.run(  # noqa: S603 - fixed argv, no shell, no user input
            ["git", "-C", str(repo), *args],  # noqa: S607
            capture_output=True,
            text=True,
            timeout=_GIT_TIMEOUT_SEC,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def _code_provenance() -> dict[str, str]:
    """Which code and which hardware profile this process is running.

    Located through ``src.config.launch_settings``'s own file rather than by
    counting ``parents[]`` from here. The node imports that module anyway, so it
    is by construction inside the checkout that is running -- which the colcon
    install space is NOT, and a stamp taken from the install space would name
    whatever was built rather than what is executing.

    ``VTITAN_HARDWARE_PROFILE`` is recorded beside the commit because it is the
    other input that silently changes behaviour: a blank profile once cost a
    whole night of motor tests that ran against the unmodified base ceiling,
    and no bag records which servo and motor the run believed it had.
    """
    here = Path(_launch_settings.__file__).resolve().parent
    provenance = {"hardware_profile": os.environ.get("VTITAN_HARDWARE_PROFILE", "")}
    toplevel = _git(here, "rev-parse", "--show-toplevel")
    if toplevel is None:
        provenance["commit"] = "unknown"
        provenance["commit_source"] = f"not a git checkout at {here}"
        return provenance
    repo = Path(toplevel)
    status = _git(repo, "status", "--porcelain")
    provenance |= {
        "commit": _git(repo, "rev-parse", "HEAD") or "unknown",
        "branch": _git(repo, "rev-parse", "--abbrev-ref", "HEAD") or "unknown",
        "subject": _git(repo, "log", "-1", "--format=%s") or "unknown",
        # A dirty tree means the commit above does NOT describe what ran, so
        # this is the field that decides whether a stamp can be trusted at all.
        "dirty": "unknown" if status is None else str(bool(status)).lower(),
        "repo": str(repo),
    }
    return provenance


class BagRecorderNode(Node):
    """Records a rosbag for exactly the duration of each RACING state."""

    def __init__(self) -> None:
        super().__init__("bag_recorder")

        self.declare_parameter("topics", _DEFAULT_TOPICS)  # list param -- no scalar params.py getter fits

        # Default now points at the repo-root data/live/runs tree, shared by
        # the Python and Go stacks (see repo-root .gitignore). Resolved relative
        # to the repo root so a Pi deployment and a dev checkout both write to
        # the same place without an absolute path or ~/vtitan_runs. Taken from
        # RaceLaunchDefaults rather than counting parents[] a second time here:
        # the two counts agree today only because nobody has moved either file,
        # and a recorder writing somewhere the bag scripts do not read is a
        # silent loss of the whole session's evidence.
        self._bag_dir = Path(
            declare_and_get_str_param(self, "bag_dir", _RACE_DEFAULTS.bag_dir),
        ).expanduser()
        self._topics = list(self.get_parameter("topics").value)
        # Fallbacks only -- race.launch.py passes these from the same settings
        # object. Read from it rather than repeating the numbers: a literal here
        # would silently diverge the day someone retunes retention in
        # race.toml, and a node that quietly keeps a different number of runs
        # than a race does is the kind of drift you only notice as missing bags.
        self._max_runs = declare_and_get_int_param(self, "max_runs", _RACE_DEFAULTS.bag_max_runs)
        self._max_total_bytes = int(
            declare_and_get_float_param(self, "max_total_gb", _RACE_DEFAULTS.bag_max_total_gb) * _BYTES_PER_GB,
        )
        self._enabled = declare_and_get_bool_param(self, "enabled", default=True)

        self._recorder: subprocess.Popen[bytes] | None = None
        self._racing = False

        # Resolved ONCE, here, not per round. It cannot change while this
        # process lives -- a deploy restarts the service -- so paying for git
        # per round would add latency to the one moment that matters, the race
        # start, and a git failure surfaces at boot where it can be read rather
        # than mid-round where it cannot.
        self._provenance = _code_provenance()
        self._provenance_timer: Timer | None = None
        self._provenance_target: Path | None = None
        self._provenance_deadline = 0.0

        topics = RosTopicConfig.load_default()
        self.create_subscription(String, topics.state_machine.state, self._on_robot_state, QOS_LATCHED_STATE)
        # Lets a separate process (vision_node's per-run video recorder) write
        # into the exact same run directory as the mcap, without the two nodes
        # sharing any other state. Latched: a late-subscribing node still gets
        # the current run's path immediately rather than waiting for the next one.
        self._run_path_pub = self.create_publisher(String, topics.bag_recorder.run_path, QOS_LATCHED_STATE)

        if self._enabled:
            self.get_logger().info(
                f"Bag recorder armed: {len(self._topics)} topics → {self._bag_dir} "
                f"(keep ≤{self._max_runs} runs / ≤{self._max_total_bytes / _BYTES_PER_GB:.1f} GB) "
                "- idle until /robot_state reports racing",
            )
            # Logged at WARNING when it is not usable, because a corpus that
            # cannot be attributed is discovered weeks later by someone trying
            # to compare two sessions.
            commit = self._provenance.get("commit", "unknown")
            if commit == "unknown" or self._provenance.get("dirty") != "false":
                self.get_logger().warning(
                    f"Runs will be stamped commit={commit} dirty={self._provenance.get('dirty')} "
                    f"profile={self._provenance.get('hardware_profile') or '<unset>'} "
                    "- this corpus will NOT be attributable to a commit",
                )
            else:
                self.get_logger().info(
                    f"Runs stamped {commit[:8]} ({self._provenance.get('branch')}) "
                    f"profile={self._provenance.get('hardware_profile') or '<unset>'}",
                )
        else:
            self.get_logger().warning("Bag recorder disabled by parameter - no runs will be recorded")

    def _on_robot_state(self, msg: String) -> None:
        """Open a bag on entering RACING, close it on leaving."""
        was_racing = self._racing
        self._racing = msg.data.strip().lower() == RobotState.RACING.value

        if self._racing and not was_racing:
            self._start_recording()
        elif was_racing and not self._racing:
            self._stop_recording(reason=msg.data.strip() or "unknown")

    def _start_recording(self) -> None:
        if not self._enabled or self._recorder is not None:
            return

        # Pruned before opening the new bag, not after closing it: the point of
        # the cap is to guarantee room for the run about to be recorded, and
        # pruning afterwards would be too late for the card that just filled.
        self._prune_old_runs()

        self._bag_dir.mkdir(parents=True, exist_ok=True)
        run_path = self._unique_run_path()

        try:
            # start_new_session so the recorder gets its own process group and
            # can be signalled without the signal also hitting this node.
            self._recorder = subprocess.Popen(  # noqa: S603 - fixed argv, no shell, no user input
                ["ros2", "bag", "record", "-o", str(run_path), *self._topics],  # noqa: S607
                start_new_session=True,
            )
        except OSError as exc:
            # A failed recorder must never take the race down with it: the
            # robot can complete a round perfectly well without a bag.
            self.get_logger().error(f"Could not start rosbag recorder, racing without a bag: {exc}")
            self._recorder = None
            return

        # Published only once the recorder process actually launched -- a
        # failed Popen above must not point a subscriber (e.g. vision_node's
        # video recorder) at a directory that will never be created.
        self._run_path_pub.publish(String(data=str(run_path)))
        self._arm_provenance_write(run_path)
        self.get_logger().info(f"Race started - recording to {run_path}")

    def _arm_provenance_write(self, run_path: Path) -> None:
        """Write the stamp as soon as the recorder has created the directory.

        Deliberately at START rather than at stop: a round killed hard still
        leaves a partial bag somebody will read, and an unattributable partial
        bag is exactly the case this exists for.
        """
        self._cancel_provenance_timer()
        self._provenance_target = run_path
        self._provenance_deadline = time.monotonic() + _PROVENANCE_POLL_TIMEOUT_SEC
        self._provenance_timer = self.create_timer(
            _PROVENANCE_POLL_INTERVAL_SEC,
            self._poll_for_run_dir,
        )

    def _poll_for_run_dir(self) -> None:
        run_path = self._provenance_target
        if run_path is None:
            self._cancel_provenance_timer()
            return
        if run_path.is_dir():
            self._cancel_provenance_timer()
            self._write_provenance(run_path)
            return
        if time.monotonic() >= self._provenance_deadline:
            self._cancel_provenance_timer()
            self.get_logger().warning(
                f"Run directory {run_path} never appeared in {_PROVENANCE_POLL_TIMEOUT_SEC:.0f}s "
                "- this run is NOT stamped with a commit",
            )

    def _cancel_provenance_timer(self) -> None:
        if self._provenance_timer is not None:
            self._provenance_timer.cancel()
            self.destroy_timer(self._provenance_timer)
            self._provenance_timer = None
        self._provenance_target = None

    def _write_provenance(self, run_path: Path) -> None:
        """Never raises. A failed stamp must not cost the round the bag."""
        stamp = self._provenance | {
            "run": run_path.name,
            "started_at": datetime.now().astimezone().isoformat(),
            "topics": len(self._topics),
        }
        try:
            (run_path / _PROVENANCE_FILENAME).write_text(
                json.dumps(stamp, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        except OSError as exc:
            self.get_logger().warning(f"Could not write {_PROVENANCE_FILENAME} to {run_path}: {exc}")

    def _stop_recording(self, reason: str) -> None:
        recorder = self._recorder
        if recorder is None:
            return
        self._recorder = None

        try:
            os.killpg(os.getpgid(recorder.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError) as exc:
            self.get_logger().warning(f"Could not signal rosbag recorder: {exc}")
            return

        deadline = time.monotonic() + _SHUTDOWN_GRACE_SEC
        while recorder.poll() is None and time.monotonic() < deadline:
            time.sleep(0.1)

        if recorder.poll() is None:
            # Past the grace period the bag is already likely unusable, but a
            # lingering recorder would keep writing to a card this node is
            # meant to be protecting.
            self.get_logger().error("rosbag recorder did not exit on SIGINT - killing it; bag may be unreadable")
            with contextlib.suppress(OSError):
                os.killpg(os.getpgid(recorder.pid), signal.SIGKILL)
            recorder.wait()

        self.get_logger().info(f"Race state '{reason}' - recording stopped")

    def _unique_run_path(self) -> Path:
        """Timestamped run directory, disambiguated if one already exists.

        The robot is meant to be run repeatedly without restarting this node --
        the state machine cycles FINISHED -> BOOT_CHECK -> READY -> RACING from
        the button alone. Two rounds landing in the same second is unlikely but
        not impossible (an E-STOP immediately re-started), and `ros2 bag record`
        refuses to write into an existing directory, so that collision would
        silently cost a recording.
        """
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        run_path = self._bag_dir / f"run_{stamp}"
        suffix = 1
        while run_path.exists():
            run_path = self._bag_dir / f"run_{stamp}_{suffix}"
            suffix += 1
        return run_path

    def _prune_old_runs(self) -> None:
        """Drop oldest runs until both the count and size caps are satisfied."""
        if not self._bag_dir.is_dir():
            return

        runs = sorted((p for p in self._bag_dir.iterdir() if p.is_dir()), key=lambda p: p.stat().st_mtime)
        sizes = {run: self._dir_size(run) for run in runs}

        while runs and (len(runs) > self._max_runs or sum(sizes.values()) > self._max_total_bytes):
            oldest = runs.pop(0)
            freed = sizes.pop(oldest, 0)
            try:
                shutil.rmtree(oldest)
            except OSError as exc:
                self.get_logger().warning(f"Could not prune old run {oldest.name}: {exc}")
                # Leave it out of the running total anyway; retrying it every
                # race would just repeat the same failure each round.
                continue
            self.get_logger().info(f"Pruned old run {oldest.name} ({freed / _BYTES_PER_GB:.2f} GB)")

    @staticmethod
    def _dir_size(path: Path) -> int:
        return sum(f.stat().st_size for f in path.rglob("*") if f.is_file())

    @override
    def destroy_node(self) -> None:
        """Close an in-flight bag so a shutdown mid-race still leaves it readable."""
        self._stop_recording(reason="shutdown")
        # Before super(), which tears the timer's context down underneath it.
        self._cancel_provenance_timer()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Main entry point for the race-gated bag recorder."""
    rclpy.init(args=args)
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
