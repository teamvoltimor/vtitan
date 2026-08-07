"""Replay real hardware bags through LidarLocalizer's exact grid search.

Built to stress-test the ambiguous-match guard's thresholds
(``min_distinctiveness=0.02``, ``distinctiveness_cost_floor=3.0``,
``localization.py``) against runs beyond the 2 originally used to derive
them. For each pulled run:

1. Reconstruct per-tick pose (``/nav_debug``) and implied speed between
   consecutive ticks. Flag ticks whose implied speed exceeds what the real
   drivetrain can produce (``RobotSpecs.MAX_SPEED_MPS``, with margin) --
   those are "jump ticks": positions the bag's own (older, unguarded)
   localizer accepted that cannot be real motion.
2. At each jump tick, reconstruct the exact TrackWalls the robot believed in
   at that moment (``belief_{north,south,east,west}_m`` on the same
   snapshot -- blind mode's corridor-width belief, which is what the
   localizer was actually matching against) and the nearest-in-time real
   ``/scan``, with the same LIDAR yaw-mount correction production code
   applies.
3. Re-run the same coarse-to-fine grid search LidarLocalizer.estimate_position
   performs, seeded from the previous (non-jump) tick's pose, and report the
   winning cost and its margin over the runner-up -- the two numbers the
   ambiguous-match guard thresholds against.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/diag_localizer_guard_replay.py vtitan_runs_pulled/run_20260804_213147
    python scripts/diag_localizer_guard_replay.py vtitan_runs_pulled/*  # every pulled run
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from _bag_io import Topics, decode_nav_debug, open_reader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.config.constants import RobotSpecs
from shared.domain.enums import Section
from shared.domain.models import NavigatorDebugSnapshot

from src.navigation.localization import LidarLocalizer
from src.navigation.track_geometry import TrackWalls, corridor_geometry_from_widths

_LIDAR_YAW_OFFSET_RAD = math.radians(
    (180.0 if RobotSpecs.LIDAR_INVERTED else 0.0) + RobotSpecs.LIDAR_MOUNT_YAW_OFFSET_DEG,
)
# Real motion between LIDAR-refresh ticks tops out well under this even during
# a K-turn; anything faster is not real motion, it is the search snapping to a
# wrong candidate. 2x MAX_SPEED_MPS leaves margin for IMU/encoder noise in the
# implied-speed estimate itself without hiding a real jump.
_IMPOSSIBLE_SPEED_MPS = 2.0 * RobotSpecs.MAX_SPEED_MPS

# Ambiguous-match guard thresholds (tuned against pull data)
_DEFAULT_MIN_DISTINCTIVENESS = 0.02
_DEFAULT_COST_FLOOR = 3.0
_GRID_SEARCH_RADIUS_SCALE = 2.0


def _read_bag(bag_dir: Path) -> tuple[list[tuple[int, NavigatorDebugSnapshot]], list[tuple[int, LaserScan]]]:
    """Return (nav_debug snapshots, scans), each as (bag-time-ns, msg), time-ordered.

    Scans are kept as raw messages, decoded lazily only for the handful of
    jump ticks that actually need them (via _scan_to_ranges_angles below) --
    a bag can carry thousands of /scan messages and only a few matter here.
    """
    reader = open_reader(bag_dir)
    nav_debug: list[tuple[int, NavigatorDebugSnapshot]] = []
    scans: list[tuple[int, LaserScan]] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == Topics.NAV_DEBUG:
            snapshot = decode_nav_debug(data)
            if snapshot.pose_x is not None and snapshot.pose_y is not None:
                nav_debug.append((t, snapshot))
        elif topic == Topics.SCAN:
            scans.append((t, deserialize_message(data, LaserScan)))
    return nav_debug, scans


def _nearest_scan(scans: list[tuple[int, LaserScan]], t: int) -> LaserScan:
    idx = min(range(len(scans)), key=lambda i: abs(scans[i][0] - t))
    return scans[idx][1]


def _scan_to_ranges_angles(msg: LaserScan) -> tuple[np.ndarray, np.ndarray]:
    """Mirror ros2_hardware_gateway._lidar_callback's exact preprocessing.

    The raw Slamtec driver emits NaN/inf for no-return rays. Production
    replaces both with LIDAR_MAX_RANGE and clips before the localizer ever
    sees the scan -- skipping that step (as an earlier version of this script
    did) leaves ~20% of rays reading literal inf, which inflates cost
    uniformly across every tick, jump or not, and makes the ambiguity guard
    look untrustworthy when the real problem is unfiltered input.
    """
    raw = np.asarray(msg.ranges, dtype=float)
    raw[~np.isfinite(raw)] = RobotSpecs.LIDAR_MAX_RANGE
    raw = np.clip(raw, 0.0, RobotSpecs.LIDAR_MAX_RANGE)
    angles = np.linspace(msg.angle_min, msg.angle_max, len(raw)) + _LIDAR_YAW_OFFSET_RAD
    return raw, angles


def _final_walls(nav_debug: list[tuple[int, NavigatorDebugSnapshot]]) -> TrackWalls | None:
    """Ground-truth geometry for the whole run: the physical corridor widths
    never change mid-run, only the robot's blind-mode belief about them does
    (see CorridorWidthEstimator) -- so the LAST snapshot's belief, once
    evidence has accumulated over the whole run, is a far better stand-in for
    the true walls than any single tick's (possibly still-converging) belief.
    Using each tick's own belief was tried first and produced universally
    inflated cost even on non-jump ticks, because early-run geometry
    reconstructed from an unconverged belief doesn't match the track the
    scan was actually taken against.
    """
    for _, snapshot in reversed(nav_debug):
        widths = {
            Section.NORTH: snapshot.belief_north_m,
            Section.SOUTH: snapshot.belief_south_m,
            Section.EAST: snapshot.belief_east_m,
            Section.WEST: snapshot.belief_west_m,
        }
        if all(w is not None for w in widths.values()):
            return TrackWalls(corridor_geometry_from_widths(widths))
    return None


def _grid_search_costs(
    walls: TrackWalls,
    prior_xy: tuple[float, float],
    yaw: float,
    ranges: np.ndarray,
    angles: np.ndarray,
) -> tuple[float, float, float, float]:
    """Replicate LidarLocalizer's exact search, returning (x, y, best_cost, second_cost)."""
    localizer = LidarLocalizer(walls)
    best_x, best_y = prior_xy
    radius = localizer._search_radius  # noqa: SLF001 - diagnostic replay of internal state
    n = localizer._grid_points  # noqa: SLF001
    best_cost = second_cost = np.inf

    for _ in range(localizer._passes):  # noqa: SLF001
        offsets = np.linspace(-radius, radius, n)
        best_cost = second_cost = np.inf
        cand_x, cand_y = best_x, best_y
        for dx in offsets:
            x = best_x + dx
            for dy in offsets:
                y = best_y + dy
                predicted = walls.raycast(x, y, yaw, angles)
                residual = np.abs(predicted - ranges)
                np.minimum(residual, localizer._residual_clip, out=residual)  # noqa: SLF001
                cost = float(np.sum(residual**2))
                if cost < best_cost:
                    second_cost = best_cost
                    best_cost = cost
                    cand_x, cand_y = x, y
                elif cost < second_cost:
                    second_cost = cost
        best_x, best_y = cand_x, cand_y
        radius = _GRID_SEARCH_RADIUS_SCALE * radius / (n - 1)

    return best_x, best_y, best_cost, second_cost


def replay(bag_dir: Path, min_distinctiveness: float, cost_floor: float) -> None:
    nav_debug, scans = _read_bag(bag_dir)
    if len(nav_debug) < 2 or not scans:
        print(f"{bag_dir.name}: not enough data (nav_debug={len(nav_debug)}, scan={len(scans)})")
        return

    print(f"\n== {bag_dir.name} == ({len(nav_debug)} nav_debug ticks, {len(scans)} scans)")

    walls = _final_walls(nav_debug)
    if walls is None:
        print("  no belief widths recorded in this run, skipping replay")
        return

    jump_count = 0
    would_reject = 0
    for i in range(1, len(nav_debug)):
        t0, prev = nav_debug[i - 1]
        t1, cur = nav_debug[i]
        dt = (t1 - t0) / 1e9
        if dt <= 0:
            continue
        dist = math.hypot(cur.pose_x - prev.pose_x, cur.pose_y - prev.pose_y)
        implied_speed = dist / dt
        if implied_speed <= _IMPOSSIBLE_SPEED_MPS:
            continue

        jump_count += 1
        scan = _nearest_scan(scans, t1)
        ranges, angles = _scan_to_ranges_angles(scan)
        prior_xy = (prev.pose_x, prev.pose_y)
        _, _, best_cost, second_cost = _grid_search_costs(walls, prior_xy, cur.pose_yaw, ranges, angles)

        margin = (second_cost - best_cost) / best_cost if best_cost > 0 else float("inf")
        ambiguous = best_cost > cost_floor and margin < min_distinctiveness
        if ambiguous:
            would_reject += 1
        print(
            f"  tick {i}: jump {implied_speed:5.2f} m/s  best_cost={best_cost:7.3f}  "
            f"second_cost={second_cost:7.3f}  margin={margin * 100:6.2f}%  "
            f"guard={'REJECTS (correct)' if ambiguous else 'ACCEPTS (misses it!)'}",
        )

    print(f"  {jump_count} jump tick(s) found, guard would reject {would_reject}/{jump_count}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_dirs", nargs="+", type=Path)
    parser.add_argument("--min-distinctiveness", type=float, default=_DEFAULT_MIN_DISTINCTIVENESS)
    parser.add_argument("--cost-floor", type=float, default=_DEFAULT_COST_FLOOR)
    args = parser.parse_args()
    for bag_dir in args.bag_dirs:
        replay(bag_dir, args.min_distinctiveness, args.cost_floor)


if __name__ == "__main__":
    main()
