"""Between two escapes, is the robot BLOCKED, COMMANDED to stop, or STALLED?

``diag_bag_escape_aftermath.py`` shows the escape releasing with healthy room
ahead and the next one firing seconds later, and on the 2026-09-10 evening runs
the robot covers 2-7 cm in that gap. Half a metre of clear space and almost no
motion is not a navigation failure; it is the robot failing to drive. Three
causes produce that same symptom and want completely different fixes:

* **COMMANDED STOP** -- the speed controller itself asks for ~0 (a crawl cut, a
  sign-aware slowdown, a heading gate). Tell: |cmd_speed| at or near zero.
  Wants a fix to the speed policy.
* **STALLED** -- speed is commanded and the wheel does not turn. Tell:
  cmd_speed well above the deadband, drive_speed ~0. Wants more torque, less
  steering load, or a higher floor -- see the stall floor rising with steering.
* **DRIVING** -- the wheel turns and the robot still does not advance, so it is
  pushing against something or the pose is frozen. Tell: drive_speed nonzero
  with no displacement.

Reported per inter-escape interval, plus the steering held during it, since the
measured stall floor rises steeply with steering load (0.2% straight against
19.4% at full lock).

Usage::

    pixi run -e dev python scripts/bag/diag_bag_escape_interval.py \
        data/live/runs/run_2026091*
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
from shared.config.navigation_tuning import NavigationTuning

from scripts.common.bag_io import Topics, create_bags_parser, decode_nav_debug, open_reader, read_motion_streams
from scripts.common.stats import nearest_by_time
from src.navigation.utils import wrap_angle

_STALL_SPEED_MPS = 0.02
"""Wheel speed below this is 'not turning' -- an encoder tick or two of noise."""


def _series_at(series: list[tuple[float, float]], times: list[float], t: float) -> float:
    return nearest_by_time(series, times, t) if series else 0.0


def analyse(bag_dir: Path) -> list[tuple]:
    """One row per interval between consecutive escape episodes."""
    reader = open_reader(bag_dir)
    t0 = None
    rows: list[tuple[float, object]] = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        if topic == Topics.NAV_DEBUG:
            rows.append(((t - t0) / 1e9, decode_nav_debug(data)))
    if not rows:
        return []

    streams = read_motion_streams(bag_dir)
    cmd = streams.cmd_speed_mps
    cmd_t = [t for t, _ in cmd]
    drv = streams.drive_speed_mps()
    drv_t = [t for t, _ in drv]
    steer = streams.cmd_steer_rad
    steer_t = [t for t, _ in steer]

    # Escape episodes are contiguous runs of ticks in an active maneuver.
    active = [(t, bool(getattr(d, "escape_active", False) or str(getattr(d, "phase", "")) == "active_maneuver")) for t, d in rows]
    episodes: list[tuple[float, float]] = []
    start = None
    for t, on in active:
        if on and start is None:
            start = t
        elif not on and start is not None:
            episodes.append((start, t))
            start = None

    out = []
    for i in range(len(episodes) - 1):
        gap_start, gap_end = episodes[i][1], episodes[i + 1][0]
        if gap_end - gap_start <= 0:
            continue
        window = [(t, d) for t, d in rows if gap_start <= t <= gap_end]
        if len(window) < 2:
            continue
        poses = [(d.pose_x, d.pose_y) for _, d in window if d.pose_x is not None]
        moved = (
            float(np.hypot(poses[-1][0] - poses[0][0], poses[-1][1] - poses[0][1])) if len(poses) >= 2 else 0.0
        )
        ts = [t for t, _ in window]
        cmds = np.array([abs(_series_at(cmd, cmd_t, t)) for t in ts])
        signed = np.array([_series_at(drv, drv_t, t) for t in ts])
        drvs = np.abs(signed)
        strs = np.array([abs(_series_at(steer, steer_t, t)) for t in ts])
        # Wheel path travelled, two ways. A PENDULUM (the bay ratchet
        # mechanism) turns the wheel a long way while going nowhere, so its
        # tell is a large ABSOLUTE path against a small SIGNED one, and a net
        # yaw that comes back to where it started.
        dts = np.diff(ts, prepend=ts[0])
        path_abs = float(np.sum(drvs * dts))
        path_signed = abs(float(np.sum(signed * dts)))
        yaws = [d.pose_yaw for _, d in window if d.pose_yaw is not None]
        net_yaw = abs(math.degrees(wrap_angle(yaws[-1] - yaws[0]))) if len(yaws) >= 2 else 0.0
        out.append(
            (
                bag_dir.name.replace("run_", ""),
                gap_end - gap_start,
                moved,
                float(np.median(cmds)),
                float(np.median(drvs)),
                float(np.degrees(np.median(strs))),
                path_abs,
                path_signed,
                net_yaw,
            )
        )
    return out


def main() -> None:
    parser = create_bags_parser(__doc__)
    args = parser.parse_args()
    deadband = NavigationTuning.load_default().escape.STUCK_MOVE_THRESHOLD
    all_rows = []
    for bag in args.bag_dirs:
        try:
            all_rows.extend(analyse(Path(bag)))
        except (RuntimeError, OSError, ValueError):
            continue
    if not all_rows:
        print("No inter-escape intervals found.")
        return

    print(f"== {len(all_rows)} inter-escape intervals   (stuck move threshold {deadband} m)")
    print(f"{'run':>16} {'gap s':>7} {'moved m':>8} {'cmd m/s':>8} {'wheel m/s':>10} {'steer deg':>10}  verdict")
    tally: dict[str, int] = {}
    for run, gap, moved, c, d, st, *_rest in sorted(all_rows, key=lambda r: r[2])[:25]:
        verdict = (
            "COMMANDED STOP" if c < _STALL_SPEED_MPS else "STALLED" if d < _STALL_SPEED_MPS else "DRIVING"
        )
        tally[verdict] = tally.get(verdict, 0) + 1
        print(f"{run:>16} {gap:7.2f} {moved:8.3f} {c:8.3f} {d:10.3f} {st:10.1f}  {verdict}")
    for run, gap, moved, c, d, st, *_rest in sorted(all_rows, key=lambda r: r[2])[25:]:
        verdict = (
            "COMMANDED STOP" if c < _STALL_SPEED_MPS else "STALLED" if d < _STALL_SPEED_MPS else "DRIVING"
        )
        tally[verdict] = tally.get(verdict, 0) + 1

    print("\n  verdict over ALL intervals:")
    for k, v in sorted(tally.items(), key=lambda kv: -kv[1]):
        print(f"    {k:>16}: {v:4d}  ({100 * v / len(all_rows):.1f}%)")
    stuck = [r for r in all_rows if r[2] < 0.10]
    print(f"\n  intervals where the robot moved under 10 cm: {len(stuck)}/{len(all_rows)}")
    if stuck:
        print(f"    their median commanded speed: {np.median([r[3] for r in stuck]):.3f} m/s")
        print(f"    their median wheel speed:     {np.median([r[4] for r in stuck]):.3f} m/s")
        print(f"    their median |steering|:      {np.median([r[5] for r in stuck]):.1f} deg")
        print(f"    wheel path travelled, ABSOLUTE: {np.median([r[6] for r in stuck]):.3f} m")
        print(f"    wheel path travelled, SIGNED:   {np.median([r[7] for r in stuck]):.3f} m")
        print(f"    net yaw turned:                 {np.median([r[8] for r in stuck]):.1f} deg")
        print(f"    wheel path / displacement:      x{np.median([r[6] / max(r[2], 1e-3) for r in stuck]):.1f}")


if __name__ == "__main__":
    main()
