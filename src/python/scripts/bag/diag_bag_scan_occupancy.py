r"""What was PHYSICALLY THERE, at this place, during this window?

Every other instrument in this folder reduces the scan to a number -- a
clearance, a cluster, a verdict. That is the right move when the question is
"how close did it come", and the wrong one when the question is "what was the
car actually looking at", because a number cannot show you a pillar standing
where no pillar was supposed to be. This renders it instead: accumulate the real
``/scan`` returns over a time window, project them into world coordinates with
the pose the robot BELIEVED at the time, and print the occupancy grid as text.

It is how the 2026-09-15 wedges were identified as the parking lot's west fin
rather than a mis-seen pillar, and how a pillar was found sitting 0.38 m from
the wall in one round and 0.53 m in its two sibling rounds -- 15 cm of
disagreement between rounds of the same layout, which is either an object that
moved or a pose with a 15 cm bias, and which no scalar clearance metric would
ever have surfaced.

METHOD

Bin every return into 5 cm world cells over the requested rectangle, sampling
every third posed tick. Cells print as ``#`` above a quarter of the busiest
cell, ``+`` above a twentieth, ``.`` for anything seen at all, blank for never.
The robot's own pose extent over the window is printed above the grid, so the
viewing geometry is visible: a wall that only shows on one side of the box
usually means the car only ever looked at it from one side.

TRAP, and it is structural: the projection uses the BELIEVED pose. If the
localizer had diverged, this renders a picture of the robot's delusion, not of
the mat -- and the picture will look perfectly self-consistent either way, since
every ray in it is wrong by the same amount. Run
``diag_bag_localizer_divergence.py`` over the same window before believing a
layout read off this grid, and prefer comparing the SAME object across sibling
rounds, where a shared bias cancels.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_scan_occupancy.py RUN_DIR \
        --window 120 180 --box 0.6 1.6 0.0 1.0
"""

from __future__ import annotations

import argparse

import numpy as np

from scripts.common.bag_io import create_bag_parser, read_posed_bag, scan_to_ranges_angles

_CELL_M = 0.05
_MATCH_TOLERANCE_NS = 0.2e9
"""How stale a scan may be relative to the pose tick it is projected with.

At 0.22 m/s a 0.2 s mismatch smears a return by 4.4 cm, under one cell. Looser
than that and thin structure -- a 5 cm pillar, a parking fin -- blurs into the
background before the grid can show it.
"""

_DENSE = 0.25
_SPARSE = 0.05


def main() -> int:
    parser = create_bag_parser(__doc__)
    parser.formatter_class = argparse.RawDescriptionHelpFormatter
    parser.add_argument("--window", type=float, nargs=2, metavar=("T0", "T1"), required=True, help="Seconds from bag start")
    parser.add_argument("--box", type=float, nargs=4, metavar=("X0", "X1", "Y0", "Y1"), required=True, help="World metres")
    parser.add_argument("--stride", type=int, default=3, help="Use every Nth posed tick (default 3)")
    parser.add_argument("--max-range", type=float, default=2.5, help="Ignore returns beyond this, metres (default 2.5)")
    args = parser.parse_args()

    t_start, t_end = args.window
    x0, x1, y0, y1 = args.box
    if x1 <= x0 or y1 <= y0:
        parser.error("--box needs X0<X1 and Y0<Y1")

    snapshots, scans = read_posed_bag(args.bag_dir)
    if not snapshots or not scans:
        print(f"{args.bag_dir.name}: no posed snapshots or no scans")
        return 0

    base = snapshots[0][0]
    poses = [(t, s) for t, s in snapshots if t_start <= (t - base) / 1e9 <= t_end]
    if not poses:
        print(f"{args.bag_dir.name}: no posed ticks in {t_start}-{t_end}s (bag spans 0-{(snapshots[-1][0] - base) / 1e9:.0f}s)")
        return 0

    scan_times = np.array([t for t, _ in scans])
    grid = np.zeros((int((y1 - y0) / _CELL_M) + 1, int((x1 - x0) / _CELL_M) + 1), dtype=int)
    used = 0
    for t, snap in poses[:: args.stride]:
        k = int(np.argmin(np.abs(scan_times - t)))
        if abs(scan_times[k] - t) > _MATCH_TOLERANCE_NS:
            continue
        ranges, angles = scan_to_ranges_angles(scans[k][1])
        keep = (ranges > _CELL_M) & (ranges < args.max_range)
        px = snap.pose_x + ranges[keep] * np.cos(snap.pose_yaw + angles[keep])
        py = snap.pose_y + ranges[keep] * np.sin(snap.pose_yaw + angles[keep])
        inside = (px >= x0) & (px < x1) & (py >= y0) & (py < y1)
        cols = ((px[inside] - x0) / _CELL_M).astype(int)
        rows = ((py[inside] - y0) / _CELL_M).astype(int)
        np.add.at(grid, (rows, cols), 1)
        used += 1

    xs = [s.pose_x for _t, s in poses]
    ys = [s.pose_y for _t, s in poses]
    print(
        f"=== {args.bag_dir.name}  t={t_start:.0f}-{t_end:.0f}s  posed ticks={len(poses)} projected={used}\n"
        f"    robot pose over the window: x {min(xs):.2f}..{max(xs):.2f}  y {min(ys):.2f}..{max(ys):.2f}"
    )
    if used == 0:
        print("    no scan matched a posed tick within 0.2 s -- nothing to draw")
        return 0

    peak = grid.max() or 1
    header = "".join(f"{x0 + _CELL_M * i:4.1f}"[-1] if i % 2 == 0 else " " for i in range(grid.shape[1]))
    print(f"      {header}")
    for row in range(grid.shape[0] - 1, -1, -1):
        cells = "".join(
            "#" if v > peak * _DENSE else ("+" if v > peak * _SPARSE else ("." if v > 0 else " ")) for v in grid[row]
        )
        print(f"y{y0 + _CELL_M * row:5.2f} {cells}")
    print(
        f"\npeak cell={peak} returns; '#'>{_DENSE:.0%} of peak, '+'>{_SPARSE:.0%}, '.' seen once.\n"
        "Projected with the BELIEVED pose: if the localizer drifted, this is a self-consistent\n"
        "picture of the drift. Check diag_bag_localizer_divergence.py before reading a layout off it."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
