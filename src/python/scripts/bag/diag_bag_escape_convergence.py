r"""Does an escape ACHIEVE anything? Gap gained, ground kept, and how soon the next one fires.

An escape exists to buy room. Nothing in this repo has ever asked whether it
does. The escape work to date has counted episodes, tuned their trigger and
argued about their side; all of that is upstream of the only question that
decides whether the manoeuvre is worth running at all. See
``adr:0092-escape-does-not-retire-committed-sign`` for the measured verdict.

Four things are measured per episode, and they fail in different ways:

* **GAP GAINED** -- `forward_clearance_m` at the end minus at the start. This is
  the escape's entire purpose. Negative means it finished closer to the thing it
  was escaping.
* **EFFICIENCY** -- net displacement divided by path length. 1.0 is a straight
  move; near 0 is a pendulum that drove a long way and ended where it began. The
  escape's reverse/forward pair already gives back most of what it covers, so
  this is expected to be low -- the question is how low, and whether the low
  ones are the ones that fail.
* **YAW TURNED** -- how far the chassis actually rotated. An escape that buys no
  room may still have bought a heading.
* **TICKS TO THE NEXT ESCAPE** -- the outcome measure. An escape followed
  immediately by another escape did not work, whatever its other numbers say.

CONTROLS, because a crashed diagnostic here exits 0:

* Episodes still running at the end of the bag are excluded, not truncated.
* `forward_clearance_m` is optional on the wire; episodes missing it at either
  end are counted separately rather than scored as zero gain.
* The gap is read from the SAME field the CRITICAL verdict is computed from, so
  a gain here is the same quantity the trigger spends.

TRAP: `escape_risk` is lower-cased on the wire ("critical"), not the enum repr.
A case-sensitive test reads zero episodes across a whole session and looks
exactly like a clean null.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_escape_convergence.py RUN_DIR...
"""

from __future__ import annotations

import contextlib
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import (  # noqa: E402
    Topics,
    create_bags_parser,
    decode_nav_debug,
    open_reader,
)
from scripts.common.tables import print_table  # noqa: E402


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _pct(xs: list[float], q: float) -> float:
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))]


def main() -> int:
    """Per-escape outcome, pooled per bag."""
    parser = create_bags_parser(__doc__ or "")
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--soon", type=int, default=40, help="ticks within which a follow-up escape means failure")
    args = parser.parse_args()

    rows = []
    pooled_gain: list[float] = []
    pooled_eff: list[float] = []
    pooled_yaw: list[float] = []
    pooled_moves: list[float] = []
    pooled_soon = 0
    pooled_n = 0
    for bag_dir in args.bag_dirs:
        reader = open_reader(bag_dir)
        ticks: list[tuple[float, float, float, float | None, bool, tuple[float, float] | None]] = []
        while reader.has_next():
            topic, data, _ts = reader.read_next()
            if topic != Topics.NAV_DEBUG:
                continue
            snap = None
            with contextlib.suppress(Exception):
                snap = decode_nav_debug(data)
            if snap is None or not isinstance(snap.pose_x, (int, float)):
                continue
            fc = getattr(snap, "forward_clearance_m", None)
            tx = getattr(snap, "steer_target_x", None)
            ty = getattr(snap, "steer_target_y", None)
            tgt = (float(tx), float(ty)) if isinstance(tx, (int, float)) and isinstance(ty, (int, float)) else None
            ticks.append(
                (
                    float(snap.pose_x),
                    float(snap.pose_y),
                    float(snap.pose_yaw or 0.0),
                    float(fc) if isinstance(fc, (int, float)) else None,
                    snap.active_maneuver_type is not None,
                    tgt,
                )
            )

        episodes: list[tuple[int, int]] = []
        start = None
        for i, t in enumerate(ticks):
            if t[4] and start is None:
                start = i
            elif not t[4] and start is not None:
                episodes.append((start, i - 1))
                start = None
        # A manoeuvre still latched at the end of the bag never finished; it is
        # not a short escape, it is an unterminated one.
        unfinished = 1 if start is not None else 0

        gains: list[float] = []
        effs: list[float] = []
        yaws: list[float] = []
        moves: list[float] = []
        no_gap = 0
        soon = 0
        for k, (a, b) in enumerate(episodes):
            # Sampled OUTSIDE the episode, because the robot stops computing
            # forward clearance while a manoeuvre is latched: only a small
            # fraction of manoeuvre ticks carry the field, against most ticks
            # overall. The escape drives blind with respect to the very quantity
            # it exists to improve, so the boundary ticks are usually empty and
            # the last reading BEFORE and first reading AFTER are what exist.
            g0 = next((ticks[i][3] for i in range(a, max(a - 40, -1), -1) if ticks[i][3] is not None), None)
            g1 = next((ticks[i][3] for i in range(b, min(b + 40, len(ticks))) if ticks[i][3] is not None), None)
            if g0 is None or g1 is None:
                no_gap += 1
            else:
                gains.append(g1 - g0)
            path = sum(math.dist(ticks[i][:2], ticks[i + 1][:2]) for i in range(a, b))
            net = math.dist(ticks[a][:2], ticks[b][:2])
            effs.append(net / path if path > 1e-6 else 0.0)
            yaws.append(abs(math.degrees(_wrap(ticks[b][2] - ticks[a][2]))))
            # Does the PLAN change? An escape that buys room and is then handed
            # back the same target is an escape the planner will undo.
            t0 = next((ticks[i][5] for i in range(a, max(a - 40, -1), -1) if ticks[i][5] is not None), None)
            t1 = next((ticks[i][5] for i in range(b, min(b + 40, len(ticks))) if ticks[i][5] is not None), None)
            if t0 is not None and t1 is not None:
                moves.append(math.dist(t0, t1))
            if k + 1 < len(episodes) and episodes[k + 1][0] - b <= args.soon:
                soon += 1

        if not episodes:
            rows.append([bag_dir.name.replace("run_", ""), 0, "-", "-", "-", "-", "-", "-", "-"])
            continue
        pooled_gain.extend(gains)
        pooled_eff.extend(effs)
        pooled_yaw.extend(yaws)
        pooled_moves.extend(moves)
        pooled_soon += soon
        pooled_n += len(episodes)
        rows.append(
            [
                bag_dir.name.replace("run_", ""),
                len(episodes),
                f"{statistics.median(gains) * 100:+.1f}" if gains else "-",
                f"{100 * sum(1 for g in gains if g <= 0) / len(gains):.0f}%" if gains else "-",
                f"{statistics.median(effs):.2f}",
                f"{statistics.median(yaws):.0f}",
                f"{100 * soon / len(episodes):.0f}%",
                f"{statistics.median(moves) * 100:.0f}" if moves else "-",
                f"{100 * sum(1 for m in moves if m < 0.05) / len(moves):.0f}%" if moves else "-",
            ]
        )
    if pooled_n:
        rows.append(
            [
                "POOLED",
                pooled_n,
                f"{statistics.median(pooled_gain) * 100:+.1f}" if pooled_gain else "-",
                f"{100 * sum(1 for g in pooled_gain if g <= 0) / len(pooled_gain):.0f}%" if pooled_gain else "-",
                f"{statistics.median(pooled_eff):.2f}",
                f"{statistics.median(pooled_yaw):.0f}",
                f"{100 * pooled_soon / pooled_n:.0f}%",
                f"{statistics.median(pooled_moves) * 100:.0f}" if pooled_moves else "-",
                f"{100 * sum(1 for m in pooled_moves if m < 0.05) / len(pooled_moves):.0f}%" if pooled_moves else "-",
            ]
        )
    print_table(
        rows,
        ["run", "escapes", "gap gained cm", "gained NOTHING", "efficiency", "yaw deg", "another within 2 s", "target moved cm", "target UNCHANGED"],
    )
    print(
        "An escape exists to buy room. `gap gained` is the whole point; `gained NOTHING`"
        " counts the episodes that ended no further from the threat than they started."
        " `efficiency` is net over path -- near 0 means it drove a long way to nowhere."
        " `another within 2 s` is the outcome: whatever the other columns say, an escape"
        " followed at once by another escape did not work."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
