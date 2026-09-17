r"""At each commit, how many pillars does the LIDAR see where the map believes N?

The 2026-09-16 pass-side work found that the crossing passes which fail are
the ones whose PREVIOUS claim was an opposite-colour sign 0.50 m behind -- the
lattice's row step -- and the WRO table never places two pillars 0.5 m apart
(every double is depth 1.0 + 2.0). On the 13 such cases the LIDAR, read in the
ROBOT frame, showed one pillar where the map held two to four beliefs at
adjacent depths. This generalises that check to every commit in the rounds.

Robot frame on purpose: the believed map carries 0.1-0.4 m of along-track
error, so counting returns around a believed WORLD position judges the map
with the map. A pillar here is a short LIDAR run (chord under 0.16 m) bounded
by range jumps on both sides, ahead of the nose, inside the corridor, and away
from the chassis's own returns and the walls.

Per commit (first tick the router names a new sign):

* ``believed`` -- signs the router holds within the window ahead
  (along -0.3..1.3 m, |lateral| < 0.5 m), and whether any two of them sit at
  ADJACENT depths (0.5 m apart along the section);
* ``seen`` -- pillar clusters the LIDAR shows in that window, as the median
  count over five scans around the tick;
* ``along error`` -- committed sign's along-track distance minus the nearest
  LIDAR pillar's, when exactly one pillar is seen.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_commit_belief_vs_lidar.py RUN_DIR [RUN_DIR ...]
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401  (imported first: models <-> enums cycle)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from shared.domain.enums import Direction
from shared.domain.models import Pose, SignColor

from scripts.common.bag_io import (
    create_bags_parser,
    decode_detections,
    read_vision_rows_and_scans,
    scan_to_ranges_angles,
    settled_direction,
)
from scripts.common.stats import nearest_by_time, percentile
from scripts.common.tables import print_table
from src.config.tuning_helpers import get_tuning
from src.navigation.planning.sign_discovery import detection_to_observation
from src.navigation.planning.sign_router import SignRouter

ALONG_MIN, ALONG_MAX, LAT_MAX = 0.28, 1.30, 0.30
"""Window ahead: past the chassis's own returns, inside the pillar lines."""


def pillar_clusters(rr: np.ndarray, aa: np.ndarray) -> list[tuple[float, float]]:
    """Pillar-sized runs in the robot frame as (along, lateral)."""
    order = np.argsort(aa)
    rr, aa = rr[order], aa[order]
    x = rr * np.cos(aa)
    y = rr * np.sin(aa)
    n = len(rr)
    out: list[tuple[float, float]] = []
    i = 0
    while i < n:
        if not (ALONG_MIN < x[i] < ALONG_MAX and abs(y[i]) < LAT_MAX and rr[i] < 1.6):
            i += 1
            continue
        j = i
        while j + 1 < n and abs(rr[j + 1] - rr[j]) < 0.05 and math.hypot(x[j + 1] - x[i], y[j + 1] - y[i]) < 0.16:
            j += 1
        before = rr[i - 1] - rr[i] if i > 0 else 1.0
        after = rr[j + 1] - rr[j] if j + 1 < n else 1.0
        if j - i >= 1 and before > 0.15 and after > 0.15:
            cx, cy = float(np.mean(x[i : j + 1])), float(np.mean(y[i : j + 1]))
            # Merge with a cluster already found within 0.12 m (the far side of a pillar).
            if not any(math.hypot(cx - a, cy - b) < 0.12 for a, b in out):
                out.append((cx, cy))
        i = j + 1
    return out


def commits_for_run(rows, frames, scans, tuning) -> list[dict]:  # noqa: ANN001
    direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)
    scan_times = [t for t, _ in scans]
    frame_i = 0
    last_key = None
    out: list[dict] = []

    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        pose = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
        ranges = angles = None
        if scan_times:
            ranges, angles = scan_to_ranges_angles(
                deserialize_message(nearest_by_time(scans, scan_times, rel), LaserScan)
            )
        obs = []
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            for det in decode_detections(frames[frame_i][1]):
                if det.color not in (SignColor.RED, SignColor.GREEN):
                    continue
                o = detection_to_observation(det, pose, tuning, ranges, angles)
                if o is not None:
                    obs.append(o)
            frame_i += 1
        if d.steer_target_x is None:
            continue
        router.deform_waypoint(
            (d.steer_target_x, d.steer_target_y), (d.pose_x, d.pose_y), d.pose_yaw, d.current_corridor, obs
        )
        anchor = router.committed_sign_position
        key = None if anchor is None else (round(anchor.x, 2), round(anchor.y, 2))
        if key is None or key == last_key:
            last_key = key
            continue
        last_key = key

        c, s = math.cos(d.pose_yaw), math.sin(d.pose_yaw)

        def frame(x: float, y: float) -> tuple[float, float]:
            dx, dy = x - d.pose_x, y - d.pose_y
            return dx * c + dy * s, -dx * s + dy * c

        believed = []
        for sg in router.signs:
            along, lat = frame(sg.x, sg.y)
            if -0.3 <= along <= ALONG_MAX and abs(lat) < 0.5:
                believed.append((along, lat, sg.x, sg.y, str(sg.color)))
        commit_along, _ = frame(anchor.x, anchor.y)
        # Adjacent depths: two beliefs whose WORLD positions differ by ~0.5 m
        # along one axis and by 0 or 0.2 m across (same or other lateral line).
        adjacent = False
        for i in range(len(believed)):
            for j in range(i + 1, len(believed)):
                dx = abs(believed[i][2] - believed[j][2])
                dy = abs(believed[i][3] - believed[j][3])
                if (abs(dx - 0.5) < 0.06 and dy < 0.26) or (abs(dy - 0.5) < 0.06 and dx < 0.26):
                    adjacent = True

        counts = []
        nearest_along = []
        if scan_times:
            for dt in (-0.4, -0.2, 0.0, 0.2, 0.4):
                sc = nearest_by_time(scans, scan_times, rel + dt)
                if sc is None:
                    continue
                r2, a2 = scan_to_ranges_angles(deserialize_message(sc, LaserScan))
                cl = pillar_clusters(np.asarray(r2), np.asarray(a2))
                counts.append(len(cl))
                if cl:
                    # Shift the cluster back to the commit tick by the robot's own travel is
                    # not attempted; 0.4 s at 0.22 m/s is under 0.09 m and the error is read at p50.
                    nearest_along.append(min(cl, key=lambda p: abs(p[0] - commit_along))[0])
        seen = int(np.median(counts)) if counts else None
        err = (commit_along - float(np.median(nearest_along))) if (seen == 1 and nearest_along) else None
        out.append(
            {
                "run": Path(str(getattr(d, "run", ""))).name,
                "believed": len(believed),
                "adjacent": adjacent,
                "seen": seen,
                "along_err": err,
                "commit_along": commit_along,
            }
        )
    return out


def main() -> int:
    """Replay the router over each bag and compare its beliefs with the LIDAR at every commit."""
    parser = create_bags_parser(__doc__)
    args = parser.parse_args()
    tuning = get_tuning(None)
    commits: list[dict] = []
    skipped = 0
    for bag in args.bag_dirs:
        try:
            rows, frames, scans = read_vision_rows_and_scans(Path(bag))
        except (RuntimeError, OSError, ValueError):
            skipped += 1
            continue
        for cmt in commits_for_run(rows, frames, scans, tuning):
            cmt["run"] = Path(bag).name.replace("run_", "")
            commits.append(cmt)

    if skipped:
        print(f"== SKIPPED {skipped} unreadable bag(s)")
    if not commits:
        print("No commits reconstructed.")
        return 0
    with_scan = [c for c in commits if c["seen"] is not None]
    print(f"== {len(commits)} commits over {len(set(c['run'] for c in commits))} run(s); {len(with_scan)} with a scan")
    print()

    print(
        "== believed signs in the window ahead (along -0.3..1.3 m) vs pillar-sized LIDAR clusters (median of 5 scans)"
    )
    table = Counter((min(c["believed"], 4), min(c["seen"], 3)) for c in with_scan)
    rows_out = []
    for b in range(5):
        row = [f"{b}{'+' if b == 4 else ''} believed"]
        for s_ in range(4):
            row.append(str(table.get((b, s_), 0)))
        rows_out.append(row)
    print_table(rows_out, ["", "seen 0", "seen 1", "seen 2", "seen 3+"])
    print()

    adj = [c for c in with_scan if c["adjacent"]]
    print(
        f"== commits with beliefs at ADJACENT depths (0.5 m apart, an illegal layout): {len(adj)}/{len(with_scan)}"
        f" ({100 * len(adj) / max(1, len(with_scan)):.1f}%)"
    )
    if adj:
        cnt = Counter(min(c["seen"], 3) for c in adj)
        print("   LIDAR pillars seen in those: " + ", ".join(f"{k}: {v}" for k, v in sorted(cnt.items())))
    print()

    errs = sorted(c["along_err"] for c in with_scan if c["along_err"] is not None)
    if errs:
        print(f"== along-track error of the COMMITTED belief vs the one LIDAR pillar seen (n={len(errs)}):")
        print(
            f"   believed - seen: p10 {percentile(errs, 0.1):+.3f}  p50 {percentile(errs, 0.5):+.3f}  p90 {percentile(errs, 0.9):+.3f} m"
            f"   |err| p50 {percentile(sorted(abs(e) for e in errs), 0.5):.3f}, > 0.25 m on {100 * sum(1 for e in errs if abs(e) > 0.25) / len(errs):.0f}%"
        )
    print()

    print("== per run")
    rows_out = []
    for run in sorted(set(c["run"] for c in with_scan)):
        cs = [c for c in with_scan if c["run"] == run]
        a = [c for c in cs if c["adjacent"]]
        e = sorted(abs(c["along_err"]) for c in cs if c["along_err"] is not None)
        rows_out.append(
            [
                run,
                str(len(cs)),
                str(len(a)),
                str(sum(1 for c in a if c["seen"] == 1)),
                f"{percentile(e, 0.5):.3f}" if e else "--",
            ]
        )
    print_table(rows_out, ["run", "commits", "adjacent-depth", "of those, 1 seen", "|along err| p50"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
