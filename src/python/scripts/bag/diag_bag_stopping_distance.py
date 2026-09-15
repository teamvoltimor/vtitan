r"""How far does the chassis travel between the CRITICAL tick and standstill?

STATUS: this answers a question `clearance.toml` records as blocked. Beside
`obstacles_contact_dist` it says, in the shipped tree: "STILL UNRUN, and it is
the only thing that can settle this: Session A's stopping-distance bench. Nobody
has measured how far the chassis travels between the CRITICAL tick and
standstill at Obstacles cruise. If that chain exceeds the value here, this
threshold generates the collisions it exists to prevent -- and the sim cannot say
so. If the bench shows standstill well inside 4 cm, take the corpus optimum and
go to 0.04. If it shows 6-8 cm, 0.10 was accidentally right and the trigger
needs rethinking rather than retuning."

A bench was never run. It does not have to be: every recorded Obstacles round
contains the same experiment, once per escape. The escape's CRITICAL verdict is
on the wire (`escape_risk`), the pose is on the wire, and what happens between
them is the measurement.

WHY IT MATTERS. `obstacles_contact_dist` is the range at which the reactive
layer declares an obstacle a threat, and it does three jobs at once:

* it fires the escape, so too small means the escape fires too late to stop;
* it is SUBTRACTED from the rear gap when a reverse escape is fitted, so too
  LARGE starves the reverse -- measured in the corpus, 0.07 against a 0.078 m
  bumper gap leaves 8 mm, one frame, one centimetre of travel, which cannot
  clear a stuck detector that wants 2.7 cm;
* the corpus optimum is 0.04, which wins 3 collisions against 28, but 0.04 sits
  BELOW `min_valid_range_m` (0.044) so it is inert rather than good.

METHOD. From each tick where `escape_risk` first reads CRITICAL, accumulate
pose-to-pose path length until the chassis is stationary -- `--stop-speed` for
`--stop-ticks` consecutive ticks -- or until it REVERSES along its approach, whichever comes first.
Reversal is a stop for this purpose: the chassis has given up its forward
momentum, which is what the threshold has to buy.

CONTROLS, because a crashed diagnostic here exits 0:

* Episodes that never reach standstill inside `--max-ticks` are reported
  SEPARATELY and excluded, not silently folded in as long stops.
* The speed at the CRITICAL tick is reported alongside. A stopping distance
  measured from a chassis that was already crawling says nothing about cruise,
  and the Obstacles cruise figure is the one the TOML is asking about.
* Pose path length is known to over-read by ~10% on this robot, so the numbers
  here are a slight OVER-estimate of the true distance -- which is the safe
  direction for a threshold that exists to prevent contact.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_stopping_distance.py RUN_DIR...
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


def _pct(xs: list[float], q: float) -> float:
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))]


def main() -> int:
    """Stopping distance per CRITICAL episode, pooled over the given bags."""
    parser = create_bags_parser(__doc__ or "")
    parser.add_argument("--stop-speed", type=float, default=0.02, help="m/s below which the chassis is stationary")
    parser.add_argument("--stop-ticks", type=int, default=3, help="consecutive stationary ticks that end an episode")
    parser.add_argument("--max-ticks", type=int, default=100, help="give up on an episode after this many ticks")
    parser.add_argument("--approach-ticks", type=int, default=5, help="ticks before CRITICAL that define approach speed")
    parser.add_argument("--moving", type=float, default=0.05, help="approach speed above which the episode counts as CRUISING")
    parser.add_argument("--hz", type=float, default=20.0, help="control rate, for speed from pose deltas")
    args = parser.parse_args()

    rows = []
    gap_rows = []
    pooled_consumed: list[float] = []
    pooled_at: list[float] = []
    touched_total = 0
    pooled: list[float] = []
    pooled_entry: list[float] = []
    unfinished_total = 0
    for bag_dir in args.bag_dirs:
        reader = open_reader(bag_dir)
        poses: list[tuple[float, float]] = []
        gaps: list[float | None] = []
        critical: list[int] = []
        prev_crit = False
        while reader.has_next():
            topic, data, _ts = reader.read_next()
            if topic != Topics.NAV_DEBUG:
                continue
            snap = None
            with contextlib.suppress(Exception):
                snap = decode_nav_debug(data)
            if snap is None or not isinstance(snap.pose_x, (int, float)):
                continue
            poses.append((float(snap.pose_x), float(snap.pose_y)))
            fc = getattr(snap, "forward_clearance_m", None)
            gaps.append(float(fc) if isinstance(fc, (int, float)) else None)
            # Lower-cased on the wire ("critical"), not the enum repr. A
            # case-sensitive test here read 0 episodes across five rounds and
            # looked exactly like a clean null; the control that caught it was
            # dumping the field's actual value counts.
            crit = "critical" in str(getattr(snap, "escape_risk", "")).lower()
            if crit and not prev_crit:
                critical.append(len(poses) - 1)
            prev_crit = crit

        dists: list[float] = []
        entry: list[float] = []
        already_stopped = [0]
        unfinished = 0
        for start in critical:
            still = 0
            travelled = 0.0
            # APPROACH speed, averaged over the ticks BEFORE the verdict. The
            # first version read it from the tick pair at the verdict itself and
            # got 0.000 m/s in every round -- which is a finding, not a bug: the
            # CRITICAL verdict mostly fires on a chassis that has ALREADY
            # stopped, so there is no momentum left for the threshold to buy.
            # Averaging backwards separates "fired while cruising" from "fired
            # while already wedged", and only the first is a braking distance.
            back = poses[max(0, start - args.approach_ticks) : start + 1]
            entry_speed = 0.0
            if len(back) > 1:
                entry_speed = sum(math.dist(a, b) for a, b in zip(back, back[1:], strict=False)) / (len(back) - 1) * args.hz
            # Heading at the verdict, so a REVERSAL can be detected. Without
            # this the episode keeps accumulating path length through the
            # escape manoeuvre, which drives on purpose -- that is not braking
            # and it inflated the first version of this measurement.
            hx = hy = 0.0
            if start + 1 < len(poses):
                hx = poses[start + 1][0] - poses[start][0]
                hy = poses[start + 1][1] - poses[start][1]
            hn = math.hypot(hx, hy)
            for i in range(start, min(start + args.max_ticks, len(poses) - 1)):
                dx = poses[i + 1][0] - poses[i][0]
                dy = poses[i + 1][1] - poses[i][1]
                step = math.hypot(dx, dy)
                if hn > 0.0 and (dx * hx + dy * hy) / hn < -args.stop_speed / args.hz:
                    # Moving BACKWARDS along the approach: the forward momentum
                    # the threshold had to buy is already spent. Stop here.
                    dists.append(travelled) if entry_speed >= args.moving else already_stopped.__setitem__(0, already_stopped[0] + 1)
                    if entry_speed >= args.moving:
                        entry.append(entry_speed)
                    break
                travelled += step
                still = still + 1 if step * args.hz < args.stop_speed else 0
                if still >= args.stop_ticks:
                    if entry_speed >= args.moving:
                        dists.append(travelled)
                        entry.append(entry_speed)
                    else:
                        already_stopped[0] += 1
                    break
            else:
                unfinished += 1
        unfinished_total += unfinished

        # GAP CONSUMED: how much closer the chassis gets after the verdict.
        consumed: list[float] = []
        at_verdict: list[float] = []
        touched = 0
        for start in critical:
            g0 = gaps[start] if start < len(gaps) else None
            if g0 is None:
                continue
            window = [g for g in gaps[start : start + args.max_ticks] if g is not None]
            if len(window) < 2:
                continue
            lo = min(window)
            consumed.append(max(0.0, g0 - lo))
            at_verdict.append(g0)
            touched += int(lo <= 0.0)
        if consumed:
            gap_rows.append(
                [
                    bag_dir.name.replace("run_", ""),
                    len(consumed),
                    f"{statistics.median(at_verdict) * 100:.1f}",
                    f"{statistics.median(consumed) * 100:.1f}",
                    f"{_pct(consumed, 0.90) * 100:.1f}",
                    f"{max(consumed) * 100:.1f}",
                    touched,
                ]
            )
            pooled_consumed.extend(consumed)
            pooled_at.extend(at_verdict)
            touched_total += touched
        else:
            gap_rows.append([bag_dir.name.replace("run_", ""), 0, "-", "-", "-", "-", 0])
        if not dists:
            rows.append([bag_dir.name.replace("run_", ""), len(critical), already_stopped[0], unfinished, "-", "-", "-", "-"])
            continue
        pooled.extend(dists)
        pooled_entry.extend(entry)
        rows.append(
            [
                bag_dir.name.replace("run_", ""),
                len(critical),
                already_stopped[0],
                unfinished,
                f"{statistics.median(dists) * 100:.1f}",
                f"{_pct(dists, 0.90) * 100:.1f}",
                f"{max(dists) * 100:.1f}",
                f"{statistics.median(entry):.3f}",
            ]
        )
    if pooled:
        rows.append(
            [
                "POOLED",
                sum(r[1] for r in rows if isinstance(r[1], int)),
                sum(r[2] for r in rows if isinstance(r[2], int)),
                unfinished_total,
                f"{statistics.median(pooled) * 100:.1f}",
                f"{_pct(pooled, 0.90) * 100:.1f}",
                f"{max(pooled) * 100:.1f}",
                f"{statistics.median(pooled_entry):.3f}",
            ]
        )
    print_table(
        rows,
        ["run", "CRITICAL", "already stopped", "never stopped", "median cm", "p90 cm", "max cm", "approach m/s"],
    )
    print(
        "\nTIME-TO-REST is NOT braking: it includes the escape manoeuvre, which"
        " drives on purpose. What a CONTACT threshold has to buy is the table below --"
        " how much of the forward gap is consumed AFTER the verdict, before the chassis"
        " stops closing. Taken from `forward_clearance_m` on the wire, so a manoeuvre"
        " that drives sideways or backwards cannot inflate it."
    )
    print_table(gap_rows, ["run", "episodes", "gap at verdict cm", "gap CONSUMED median cm", "p90 cm", "worst cm", "reached <=0"])
    print(
        "\nThe shipped `obstacles_contact_dist` is 0.07 m = 7.0 cm. Read the MEDIAN against\n"
        "it, and the p90 against the case it has to survive. Pose path length over-reads\n"
        "by about 10% on this robot, so these are slight over-estimates -- the safe\n"
        "direction for a threshold that exists to prevent contact."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
