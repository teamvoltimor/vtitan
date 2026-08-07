"""Test whether a run's believed heading was 180 degrees wrong.

The localizer solves only for position, taking yaw as given, so a yaw that is
pi off produces a confidently-tracked but wrong position. This replays the
recorded ``/scan`` twice -- once at the heading the run actually believed, once
at that heading plus pi -- and reports the scan-match residual each yields.
The heading the robot was really at is the one whose scan the known wall
geometry can actually explain, so the lower residual identifies it.

Both replays are seeded from the same canonical start pose and stepped forward
tick by tick, so each is a self-consistent trajectory rather than a per-tick
re-fit: a heading that is wrong by pi does not merely cost more on one scan, it
walks the position estimate the wrong way down the corridor.

Usage:
    pixi run -e dev python scripts/bag/diag_yaw_flip_replay.py vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
from shared.config.enums import Section

from scripts.bag.diag_localizer_guard_replay import (
    _final_walls,
    _nearest_scan,
    _read_bag,
    _scan_to_ranges_angles,
)
from src.navigation.localization import LidarLocalizer
from src.navigation.track_geometry import TrackWalls, corridor_geometry_from_widths

_RESIDUAL_CLIP_M = 0.25
_DEFAULT_SEED_X_M = 1.5
_DEFAULT_SEED_Y_M = 0.25
_DEFAULT_UNTIL_S = 8.0
_YAW_FLIP_OFFSET_RAD = math.pi
_DOWNSAMPLE_FACTOR = 8


def _residual(walls: TrackWalls, x: float, y: float, yaw: float, ranges: np.ndarray, angles: np.ndarray) -> float:
    """Clipped least-squares scan-match cost, as LidarLocalizer scores candidates."""
    predicted = walls.raycast(x, y, yaw, angles)
    residual = np.abs(predicted - ranges)
    np.minimum(residual, _RESIDUAL_CLIP_M, out=residual)
    return float(np.sum(residual**2))


def _tick_walls(snapshot: dict) -> TrackWalls | None:
    """The geometry the robot believed at this tick, not the run's final belief.

    Blind mode's belief changes mid-run (notably the moment direction inference
    commits and the buffered creep measurements are folded in), and the
    localizer only ever matches against the belief it holds *now*.
    """
    widths = {
        Section.NORTH: snapshot.get("belief_north_m"),
        Section.SOUTH: snapshot.get("belief_south_m"),
        Section.EAST: snapshot.get("belief_east_m"),
        Section.WEST: snapshot.get("belief_west_m"),
    }
    if any(w is None for w in widths.values()):
        return None
    return TrackWalls(corridor_geometry_from_widths(widths))


def _replay(
    walls: TrackWalls,
    snapshots: list,
    scans: list,
    seed: tuple[float, float],
    yaw_offset: float,
    per_tick_walls: bool = False,
) -> tuple[list[tuple[float, float, float]], float]:
    """Step the localizer through the run with ``yaw_offset`` added to every believed yaw."""
    localizer = LidarLocalizer(walls)
    pos = seed
    track: list[tuple[float, float, float]] = []
    costs: list[float] = []
    t0 = snapshots[0][0]
    for t, snapshot in snapshots:
        yaw = snapshot.get("pose_yaw")
        if yaw is None:
            continue
        yaw = yaw + yaw_offset
        active = walls
        if per_tick_walls:
            active = _tick_walls(snapshot) or walls
            localizer._walls = active  # noqa: SLF001 - mirrors gateway.set_believed_walls
        ranges, angles = _scan_to_ranges_angles(_nearest_scan(scans, t))
        pos = localizer.estimate_position(pos, yaw, ranges.tolist(), angles.tolist(), now_s=(t - t0) / 1e9)
        track.append(((t - t0) / 1e9, pos[0], pos[1]))
        costs.append(_residual(active, pos[0], pos[1], yaw, ranges, angles))
    return track, float(np.median(costs)) if costs else float("nan")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--seed", type=float, nargs=2, default=(_DEFAULT_SEED_X_M, _DEFAULT_SEED_Y_M))
    parser.add_argument("--until", type=float, default=_DEFAULT_UNTIL_S)
    args = parser.parse_args()

    snapshots, scans = _read_bag(args.bag_dir)
    walls = _final_walls(snapshots)
    if walls is None or not scans:
        print("bag lacks belief widths or /scan")
        return
    t0 = snapshots[0][0]
    snapshots = [(t, s) for t, s in snapshots if (t - t0) / 1e9 <= args.until]

    variants = (
        ("as-believed", 0.0, False),
        ("flipped +pi", _YAW_FLIP_OFFSET_RAD, False),
        ("as-believed/per-tick walls", 0.0, True),
    )
    for label, offset, per_tick in variants:
        track, median_cost = _replay(walls, snapshots, scans, tuple(args.seed), offset, per_tick_walls=per_tick)
        first, last = track[0], track[-1]
        print(
            f"{label:26} median_cost={median_cost:8.2f}  "
            f"start=({first[1]:.3f},{first[2]:.3f}) end=({last[1]:.3f},{last[2]:.3f})  "
            f"net=({last[1] - first[1]:+.3f},{last[2] - first[2]:+.3f}) over {last[0]:.1f}s",
        )
        for ts, x, y in track[:: max(1, len(track) // _DOWNSAMPLE_FACTOR)]:
            print(f"    {ts:6.2f}s ({x:6.3f},{y:6.3f})")


if __name__ == "__main__":
    main()
