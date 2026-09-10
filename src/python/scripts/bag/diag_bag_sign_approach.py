"""How EARLY the sign lane commits, and how CLOSE the robot actually passes.

The sign path already anticipates -- `ACTIVATION_DIST_M` 1.4 m with a
`SIGN_LANE_RAMP_M` 0.9 m ramp -- so the open question is not "is there
anticipation" but "does the anticipation that exists convert into clearance".
This measures both ends of that on recorded hardware:

* **commit range** -- distance to the sign when the router first committed to
  it (`committed_sign_x_m`/`_y_m`). If this is far below ACTIVATION_DIST_M the
  lane never gets its ramp, and no amount of lane tuning can help.
* **closest approach** -- the minimum robot-to-sign distance over that
  commitment. This is the clearance that decides a collision, measured against
  the pose the robot actually reached rather than the offset it was commanded.

Deliberately NOT read: `sign_deform_magnitude_m` and `sign_target_*` are a dead
path while `SIGN_LANE_SUPPRESS_DEFORM` ships True.

Distances are robot-centre to sign-centre; subtract the chassis half-width
(~0.097 m) and the sign half-width (~0.025 m) for a physical gap. A closest
approach under ~0.122 m therefore means CONTACT, not a near miss.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_sign_approach.py \
        data/live/runs/run_2026090*
"""

from __future__ import annotations

import math
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402

# Chassis half-width + sign half-width. Below this the bodies overlap.
CONTACT_GAP_M = 0.122

# A committed sign that jumps further than this is a DIFFERENT sign, not the
# same one re-estimated -- used to split commitment episodes.
SAME_SIGN_M = 0.35


@dataclass
class Approach:
    run: str
    commit_range_m: float
    closest_m: float
    samples: int
    ramp_reached: bool


@dataclass
class RunStats:
    run: str
    approaches: list[Approach] = field(default_factory=list)


def _num(v) -> bool:  # noqa: ANN001
    return isinstance(v, (int, float))


def _episodes(rows, same_sign_m: float):  # noqa: ANN001
    """Split rows into contiguous commitments to a single sign."""
    current: list[tuple[float, float, float, float]] = []  # sx, sy, rx, ry
    last: tuple[float, float] | None = None
    for _, s in rows:
        if not (_num(s.committed_sign_x_m) and _num(s.committed_sign_y_m) and _num(s.pose_x) and _num(s.pose_y)):
            if current:
                yield current
                current, last = [], None
            continue
        sx, sy = s.committed_sign_x_m, s.committed_sign_y_m
        if last is not None and math.hypot(sx - last[0], sy - last[1]) > same_sign_m:
            yield current
            current = []
        last = (sx, sy)
        current.append((sx, sy, s.pose_x, s.pose_y))
    if current:
        yield current


def _analyse(run: str, rows, same_sign_m: float) -> RunStats:  # noqa: ANN001
    stats = RunStats(run)
    for ep in _episodes(rows, same_sign_m):
        if len(ep) < 2:
            continue
        dists = [math.hypot(sx - rx, sy - ry) for sx, sy, rx, ry in ep]
        stats.approaches.append(
            Approach(
                run=run,
                commit_range_m=dists[0],
                closest_m=min(dists),
                samples=len(ep),
                ramp_reached=dists[0] >= 0.9,
            )
        )
    return stats


def main() -> None:
    parser = create_bags_parser(__doc__ or "")
    parser.add_argument("--per-run", action="store_true", help="also print a row per run")
    parser.add_argument(
        "--same-sign",
        type=float,
        default=SAME_SIGN_M,
        help="jump (m) in the committed sign position that starts a NEW commitment. Raise it to tell a "
        "genuinely late commit apart from one stable approach re-split by a jittering estimate.",
    )
    args = parser.parse_args()

    all_runs: list[RunStats] = []
    for bag_dir in args.bag_dirs:
        if not bag_dir.is_dir():
            continue
        try:
            rows, _ = load_nav_debug_rows(bag_dir)
        except Exception as exc:  # noqa: BLE001 - one bad bag must not stop the sweep
            print(f"!! {bag_dir.name}: {exc}")
            continue
        if rows:
            st = _analyse(bag_dir.name, rows, args.same_sign)
            if st.approaches:
                all_runs.append(st)

    approaches = [a for st in all_runs for a in st.approaches]
    print(f"\n== SIGN APPROACHES  ({len(approaches)} commitments over {len(all_runs)} runs)")
    if not approaches:
        print("  no committed_sign_* samples -- nothing to measure")
        return

    if args.per_run:
        print_table(
            [
                [
                    st.run.replace("run_", ""),
                    str(len(st.approaches)),
                    f"{statistics.median([a.commit_range_m for a in st.approaches]):.2f}",
                    f"{min(a.closest_m for a in st.approaches):.3f}",
                    f"{statistics.median([a.closest_m for a in st.approaches]):.3f}",
                    str(sum(1 for a in st.approaches if a.closest_m < CONTACT_GAP_M)),
                ]
                for st in all_runs
            ],
            ["run", "signs", "med commit m", "min close m", "med close m", "contacts"],
        )

    commits = sorted(a.commit_range_m for a in approaches)
    close = sorted(a.closest_m for a in approaches)

    def pct(vals: list[float], p: float) -> float:
        return vals[min(len(vals) - 1, int(p * len(vals)))]

    print(
        f"  commit range m:    p10 {pct(commits, 0.1):.2f} / median {statistics.median(commits):.2f} "
        f"/ p90 {pct(commits, 0.9):.2f} / max {commits[-1]:.2f}"
    )
    print(f"  reached the 0.9 m ramp: {sum(1 for a in approaches if a.ramp_reached)}/{len(approaches)}")
    print(
        f"  closest approach m: p10 {pct(close, 0.1):.3f} / median {statistics.median(close):.3f} "
        f"/ p90 {pct(close, 0.9):.3f}"
    )
    contacts = sum(1 for a in approaches if a.closest_m < CONTACT_GAP_M)
    print(f"  passes INSIDE the {CONTACT_GAP_M} m contact gap: {contacts}/{len(approaches)} ({contacts / len(approaches):.0%})")

    # Does committing EARLY buy clearance? If anticipation is the lever, these
    # two groups separate; if they don't, more anticipation is not the fix.
    early = [a.closest_m for a in approaches if a.commit_range_m >= statistics.median(commits)]
    late = [a.closest_m for a in approaches if a.commit_range_m < statistics.median(commits)]
    if early and late:
        print(
            f"  closest approach by commit range: EARLY half median {statistics.median(early):.3f} m "
            f"vs LATE half median {statistics.median(late):.3f} m"
        )


if __name__ == "__main__":
    main()
