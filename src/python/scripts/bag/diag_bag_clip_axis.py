r"""Which AXIS is a clipped red box clipped on, and does the aspect test survive it?

``sign_discovery`` skips the wall-shape test for ANY frame-clipped box. The
stated reason is that a pillar being approached "grows out of frame" so its
height stops rising while its width keeps going, pushing w/h past 1.0 with
nothing about the pillar having changed.

That mechanism needs the HEIGHT truncated, i.e. clipping on the TOP or BOTTOM
edge. Horizontal clipping does the opposite: it truncates WIDTH, so w/h
UNDER-reads and the test becomes conservative rather than wrong. The sign and
the parking barrier are both 0.10 m tall (track.toml [sign] and [parking]), so
with an intact height the aspect ratio is exactly the object's implied width in
units of 0.10 m -- the aspect test already IS a width test, and a lower bound
on width is still sound evidence that an object is too wide to be a pillar.

So the exemption only has to cover VERTICAL clipping. This measures how much of
the current blanket exemption that would keep, on real bags.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_clip_axis.py RUN_DIR [RUN_DIR ...]
"""

from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.bag.diag_bag_barrier_gate import (  # noqa: E402
    FRAME_EDGE_TOLERANCE_PX,
    MAX_PILLAR_ASPECT,
    _frame_size,
    collect,
)
from scripts.common.tables import print_table  # noqa: E402


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    all_reds = []
    for bag in sys.argv[1:]:
        reds, _magenta = collect(Path(bag))
        all_reds.extend(reds)

    frame_w, frame_h = _frame_size(all_reds)
    edge = FRAME_EDGE_TOLERANCE_PX

    buckets: Counter[str] = Counter()
    wall_shaped_by_axis: Counter[str] = Counter()

    for (x_min, y_min, x_max, y_max), _corridor in all_reds:
        height = y_max - y_min
        width = x_max - x_min
        if height <= 0:
            continue
        horizontal = x_min <= edge or x_max >= frame_w - edge
        vertical = y_min <= edge or y_max >= frame_h - edge
        if not (horizontal or vertical):
            axis = "not clipped"
        elif horizontal and vertical:
            axis = "BOTH axes"
        elif vertical:
            axis = "vertical only"
        else:
            axis = "horizontal only"
        buckets[axis] += 1
        if width / height > MAX_PILLAR_ASPECT:
            wall_shaped_by_axis[axis] += 1

    total = sum(buckets.values())
    print(f"frame measured from the boxes themselves: {frame_w:.0f} x {frame_h:.0f} px")
    print(f"red detections: {total}\n")

    rows = []
    for axis in ("not clipped", "horizontal only", "vertical only", "BOTH axes"):
        n = buckets[axis]
        wall = wall_shaped_by_axis[axis]
        rows.append(
            [
                axis,
                n,
                f"{100.0 * n / total:.1f}%" if total else "-",
                wall,
                f"{100.0 * wall / n:.1f}%" if n else "-",
            ]
        )
    print_table(rows, ["clipped on", "n", "share", "wall-shaped (w/h>1)", "of that axis"])

    exempt_now = sum(wall_shaped_by_axis[a] for a in ("horizontal only", "vertical only", "BOTH axes"))
    exempt_after = sum(wall_shaped_by_axis[a] for a in ("vertical only", "BOTH axes"))
    print(
        f"\nwall-shaped reds waved through by the CURRENT blanket exemption: {exempt_now}"
        f"\nstill waved through if it covered VERTICAL clipping only:        {exempt_after}"
        f"\n  -> newly testable: {exempt_now - exempt_after}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
