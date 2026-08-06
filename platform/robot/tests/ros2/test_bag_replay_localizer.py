"""Assert recorded race bags show a position estimate that tracks forward.

The 2026-08-05 CCW failure was a position estimate that walked *backwards*
along the robot's own heading: the robot drove east into the wall while its
pose slid west, so pure pursuit never registered progress toward its waypoint,
never advanced the plan, and drove straight into a corner with the steering
near zero. Nothing in the logged pose looks wrong on its own -- the numbers are
in-track and the implied speed is plausible. Only the *sign* of the motion
relative to heading gives it away.

That is the invariant here, and it holds for any run regardless of layout,
direction or corridor widths: while the robot is being commanded forward, the
position estimate's displacement projected onto its own heading must be
positive. A run that violates it is mislocalising in the one way the rest of
the stack cannot detect or recover from.

Bags live in ``vtitan_runs_pulled/`` and are gitignored (pulled from the Pi 5
with ``scripts/pull-runs-from-pi5.sh``), so these tests skip when absent rather
than fail.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rosbag2_py", reason="ROS2 bag reading is only available in the ROS2 env")

import rosbag2_py
from rclpy.serialization import deserialize_message
from shared.config.constants import RobotSpecs
from std_msgs.msg import String

_BAG_ROOT = Path(__file__).resolve().parents[2] / "vtitan_runs_pulled"
# The window that matters is the opening drive, before any escape maneuver has
# had a chance to reorient the robot: escapes reverse on purpose, so "moved
# backwards along the heading" is correct behaviour once one is running, and
# including them would make the invariant untestable rather than more thorough.
_ANALYSIS_WINDOW_S = 6.0
# Blind direction inference re-seeds position outright when it overturns the
# assumed direction (see _commit_direction), which is a deliberate
# discontinuity, not motion -- both real captures committed by t=1.2s. Starting
# after it keeps the speed bound a statement about tracking rather than about
# a correction the robot is supposed to make.
_SETTLE_S = 1.5


def _bags() -> list[Path]:
    """The newest pulled run only.

    Deliberately not the whole corpus. Sweeping every bag was tried first and
    is genuinely informative -- 9 of 24 violate the forward-tracking invariant,
    which is the historical record of runs that really did mislocalise -- but
    those failures are permanent, so as a gate it would be red forever and stop
    meaning anything. Asserting on the latest run makes it the check you want
    after a race: it went green on the CW run that completed a lap and red on
    the CCW run that drove into a corner. Use ``scripts/diag_bag_summary.py``
    and its siblings for the full sweep.
    """
    if not _BAG_ROOT.is_dir():
        return []
    bags = sorted(p for p in _BAG_ROOT.iterdir() if p.is_dir() and any(p.glob("*.mcap")))
    return bags[-1:]


def _read(bag_dir: Path) -> list[tuple[int, dict]]:
    """Time-ordered /nav_debug snapshots that carry a pose."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    snapshots: list[tuple[int, dict]] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != "/nav_debug":
            continue
        payload = json.loads(deserialize_message(data, String).data)
        if payload.get("pose_x") is not None:
            snapshots.append((t, payload))
    return snapshots






def _forward_progress(bag_dir: Path) -> tuple[float, float] | None:
    """Return (along-heading displacement, peak implied speed) for the opening drive.

    Measured on the pose the run actually logged, not on a fresh replay. The
    logged pose is the one pure pursuit steered from, so it is the only
    trajectory whose sign means anything about the failure; a replay reseeded
    from a clean start diverges from it within a few ticks and then answers a
    question nobody asked. (Confirmed while writing this: a replayed version
    of this check passed the very run it was written to catch.)

    Positive displacement means the estimate advanced the way the robot was
    pointing. ``None`` when the bag has no usable forward-driving window.
    """
    snapshots = _read(bag_dir)
    if not snapshots:
        return None
    t0 = snapshots[0][0]
    window = [
        (t, s)
        for t, s in snapshots
        if _SETTLE_S <= (t - t0) / 1e9 <= _ANALYSIS_WINDOW_S and (s.get("commanded_speed_mps") or 0.0) > 0.0
    ]
    if len(window) < 20:
        return None

    first = window[0][1]
    start = (first["pose_x"], first["pose_y"])
    pos = start
    yaws: list[float] = []
    peak_speed = 0.0
    prev_t: int | None = None
    prev_pos = pos
    for t, snapshot in window:
        yaw = snapshot.get("pose_yaw")
        if yaw is None:
            continue
        pos = (snapshot["pose_x"], snapshot["pose_y"])
        yaws.append(yaw)
        if prev_t is not None:
            dt = (t - prev_t) / 1e9
            if dt > 0:
                peak_speed = max(peak_speed, math.hypot(pos[0] - prev_pos[0], pos[1] - prev_pos[1]) / dt)
        prev_t, prev_pos = t, pos

    if not yaws:
        return None
    # Mean heading as a unit vector, so wraparound near +-pi cannot average to
    # a direction the robot never faced.
    heading = math.atan2(float(np.mean(np.sin(yaws))), float(np.mean(np.cos(yaws))))
    along = (pos[0] - start[0]) * math.cos(heading) + (pos[1] - start[1]) * math.sin(heading)
    return along, peak_speed


@pytest.mark.slow()
@pytest.mark.parametrize("bag_dir", _bags(), ids=lambda p: p.name)
def test_estimate_does_not_track_backwards(bag_dir: Path) -> None:
    """The position estimate must advance along the heading, never against it."""
    result = _forward_progress(bag_dir)
    if result is None:
        pytest.skip(f"{bag_dir.name} has no usable forward-driving window")
    along, _ = result
    assert along > 0.0, (
        f"{bag_dir.name}: position estimate moved {abs(along):.3f} m *backwards* along its own "
        "heading while driving forward. Pure pursuit measures progress as distance to the next "
        "waypoint, so a receding estimate means the plan never advances and the robot drives "
        "straight on with near-zero steering (see the 2026-08-05 CCW run)."
    )


@pytest.mark.slow()
@pytest.mark.xfail(
    reason=(
        "The speed-bound guard does not currently bound anything. Measured across every pulled "
        "bag -- including run_20260805_195501, which completed its lap successfully, and both "
        "runs recorded after the guard shipped in 9e91981 -- the accepted estimates still imply "
        "multiples of the drivetrain maximum. The guard's hysteresis turns a rejection into a "
        "delay followed by acceptance in full, so a persistently wrong pull passes at its "
        "original magnitude and only a single non-repeating outlier is ever stopped. Recorded as "
        "xfail rather than a hard gate because it is red on good runs too: it is a known "
        "shortcoming, not a regression, and it should flip to XPASS when the guard is reworked."
    ),
    strict=False,
)
@pytest.mark.parametrize("bag_dir", _bags(), ids=lambda p: p.name)
def test_estimate_respects_physical_speed_bound(bag_dir: Path) -> None:
    """No accepted estimate may imply motion the drivetrain cannot produce."""
    result = _forward_progress(bag_dir)
    if result is None:
        pytest.skip(f"{bag_dir.name} has no usable forward-driving window")
    _, peak_speed = result
    # The same headroom LidarLocalizer's own guard allows over the measured
    # 0.156 m/s top speed, so this pins the guard's contract rather than a
    # second, independently-drifting number.
    assert peak_speed <= 0.25, (
        f"{bag_dir.name}: estimate implied {peak_speed:.3f} m/s, above the guard's 0.25 m/s bound "
        f"(real drivetrain maximum is {RobotSpecs.MAX_SPEED_MPS} m/s)."
    )
