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
``platform/backend/domain/session/sqlite/recorder.go``): drop the oldest until
the cap is satisfied.
"""

from __future__ import annotations

import contextlib
import os
import shutil
import signal
import subprocess  # noqa: S404 - ros2 bag record has no Python API; a subprocess is the only way
import time
from datetime import datetime
from pathlib import Path
from typing import override

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from shared.domain.enums import RobotState
from src.config.launch_settings import RaceLaunchDefaults
from std_msgs.msg import String

# Must match state_machine_node's _QOS_TRANSIENT publisher on both policies, or
# this subscription receives nothing at all -- a RELIABLE reader against that
# BEST_EFFORT writer is an incompatible pair, and DDS resolves it by never
# delivering (observed on hardware: "offering incompatible QoS. No messages will
# be received").
#
# TRANSIENT_LOCAL matters for the same reason it does in track_navigator_node:
# without it, a recorder started mid-race would sit idle until the state next
# changed, so the round it was meant to capture would go unrecorded.
_QOS_ROBOT_STATE = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# Fallback only -- race.launch.py always passes "topics" explicitly, sourced
# from RaceLaunchDefaults.bag_topics (platform/robot/src/config/launch_settings.py)
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
_DEFAULT_TOPICS = RaceLaunchDefaults().bag_topics

_BYTES_PER_GB = 1024**3

# rosbag2 finalizes its metadata on SIGINT; killing it outright leaves a bag
# that cannot be reindexed. This is how long to wait for that clean exit before
# escalating to SIGKILL.
_SHUTDOWN_GRACE_SEC = 10.0


class BagRecorderNode(Node):
    """Records a rosbag for exactly the duration of each RACING state."""

    def __init__(self) -> None:
        super().__init__("bag_recorder")

        self.declare_parameter("bag_dir", "~/vtitan_runs")
        self.declare_parameter("topics", _DEFAULT_TOPICS)
        self.declare_parameter("max_runs", 20)
        self.declare_parameter("max_total_gb", 4.0)
        self.declare_parameter("enabled", True)

        self._bag_dir = Path(self.get_parameter("bag_dir").value).expanduser()
        self._topics = list(self.get_parameter("topics").value)
        self._max_runs = int(self.get_parameter("max_runs").value)
        self._max_total_bytes = int(float(self.get_parameter("max_total_gb").value) * _BYTES_PER_GB)
        self._enabled = bool(self.get_parameter("enabled").value)

        self._recorder: subprocess.Popen[bytes] | None = None
        self._racing = False

        self.create_subscription(String, "/robot_state", self._on_robot_state, _QOS_ROBOT_STATE)

        if self._enabled:
            self.get_logger().info(
                f"Bag recorder armed: {len(self._topics)} topics → {self._bag_dir} "
                f"(keep ≤{self._max_runs} runs / ≤{self._max_total_bytes / _BYTES_PER_GB:.1f} GB) "
                "- idle until /robot_state reports racing",
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

        self.get_logger().info(f"Race started - recording to {run_path}")

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
