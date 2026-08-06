"""Where a run actually got close to a wall, and what it did about it.

``nav_debug.min_lidar_range_m`` is the raw ``min(scan.ranges_m)``, and the
gateway deliberately leaves near-zero invalid returns in the scan (the
collision controller filters them at ``> 0.01``). So that field bottoms out
around 0.006 m on every run and says nothing about real clearance. This reads
``/scan`` directly and applies the sensor's own minimum range instead, which is
the only way to answer "how close did it really come".

Groups the result into episodes rather than listing ticks, because a slowdown
that matters is a stretch of seconds, not one sample.

Usage:
    pixi run -e dev python scripts/diag_bag_proximity.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.config.constants import RobotSpecs
from std_msgs.msg import String


def _read(bag_dir: Path) -> tuple[list[tuple[float, dict]], list[tuple[float, float]]]:
    """Return nav_debug ticks and (time, true_min_range) from /scan."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    ticks: list[tuple[float, dict]] = []
    scans: list[tuple[float, float]] = []
    t0: float | None = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp / 1e9
        if t0 is None:
            t0 = t
        rel = t - t0
        if topic.endswith("nav_debug"):
            ticks.append((rel, json.loads(deserialize_message(data, String).data)))
        elif topic.endswith("scan"):
            raw = np.array(deserialize_message(data, LaserScan).ranges, dtype=float)
            valid = raw[np.isfinite(raw) & (raw >= RobotSpecs.LIDAR_MIN_RANGE)]
            scans.append((rel, float(valid.min()) if valid.size else float("nan")))
    return ticks, scans


def _episodes(flagged: list[tuple[float, dict]], gap_s: float = 0.5) -> list[list[tuple[float, dict]]]:
    """Group consecutive flagged ticks separated by less than ``gap_s``."""
    out: list[list[tuple[float, dict]]] = []
    for item in flagged:
        if out and item[0] - out[-1][-1][0] <= gap_s:
            out[-1].append(item)
        else:
            out.append([item])
    return out


def main() -> None:
    """Print the proximity review for the bag named on the command line."""
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--slow-below", type=float, default=0.14)
    parser.add_argument("--min-episode-s", type=float, default=0.4)
    args = parser.parse_args()

    ticks, scans = _read(args.bag_dir)
    driving = [(t, d) for t, d in ticks if d.get("pose_x") is not None]
    scan_t = np.array([t for t, _ in scans])
    scan_r = np.array([r for _, r in scans])

    def true_min_at(t: float) -> float:
        if scan_t.size == 0:
            return float("nan")
        return float(scan_r[int(np.abs(scan_t - t).argmin())])

    print(f"bag: {args.bag_dir.name}   scans: {len(scans)}   driving ticks: {len(driving)}")
    if scan_r.size:
        finite = scan_r[np.isfinite(scan_r)]
        print(f"true min range over run: {finite.min():.3f} m   (debug field bottoms at ~0.006 m and is noise)")

    print(f"\n--- slowdown episodes (< {args.slow_below} m/s, forward motion only) ---")
    slow = [(t, d) for t, d in driving if 0.0 < (d.get("commanded_speed_mps") or 1.0) < args.slow_below]
    eps = [e for e in _episodes(slow) if e[-1][0] - e[0][0] >= args.min_episode_s]
    print(f"{len(eps)} episode(s) lasting >= {args.min_episode_s}s")
    print(f"  {'start':>7} {'dur':>6} {'minspd':>7} {'true_rng':>9} {'fwd_clr':>8} {'xtrack':>7} {'pose':>16} {'cause':>10}")
    for e in eps[:14]:
        t0, t1 = e[0][0], e[-1][0]
        worst = min(e, key=lambda p: p[1].get("commanded_speed_mps") or 0.0)[1]
        clear_v = worst.get("clearance_speed_mps")
        head_v = worst.get("heading_speed_mps")
        cause = "?" if clear_v is None or head_v is None else ("clearance" if clear_v <= head_v else "heading")
        rng = min(true_min_at(t) for t, _ in e)
        pose = f"({worst.get('pose_x') or 0:.2f},{worst.get('pose_y') or 0:.2f})"
        print(
            f"  {t0:7.1f} {t1 - t0:6.1f} {worst.get('commanded_speed_mps') or 0:7.3f} {rng:9.3f} "
            f"{worst.get('forward_clearance_m') or float('nan'):8.3f} "
            f"{worst.get('crosstrack_error_m') or float('nan'):7.3f} {pose:>16} {cause:>10}"
        )

    print("\n--- closest real approaches (true scan min) ---")
    with_rng = [(t, d, true_min_at(t)) for t, d in driving]
    for t, d, r in sorted(with_rng, key=lambda x: x[2])[:6]:
        print(
            f"  {t:7.1f}s  range {r:.3f} m  pose ({d.get('pose_x') or 0:.2f},{d.get('pose_y') or 0:.2f})  "
            f"fwd_clr {d.get('forward_clearance_m') or float('nan'):.3f}  "
            f"steer {d.get('commanded_steering_norm') or 0:.3f}  phase {d.get('phase')}"
        )

    print("\n--- escape / recovery timeline ---")
    esc = [(t, d) for t, d in driving if (d.get("escape_count") or 0) > 0]
    for e in _episodes(esc, gap_s=2.0)[:12]:
        t0, t1 = e[0][0], e[-1][0]
        peak = max(d.get("escape_count") or 0 for _, d in e)
        kinds = {d.get("active_maneuver_type") for _, d in e if d.get("active_maneuver_type")}
        d0 = e[0][1]
        print(
            f"  {t0:7.1f}s -> {t1:6.1f}s  peak escape_count {peak:2d}  "
            f"at ({d0.get('pose_x') or 0:.2f},{d0.get('pose_y') or 0:.2f})  {','.join(sorted(kinds)) or '-'}"
        )


if __name__ == "__main__":
    main()
