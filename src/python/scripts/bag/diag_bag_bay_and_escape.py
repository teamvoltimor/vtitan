"""Measure, across many bags, how often the in-bay exit SUCCEEDS and what the
reactive escape actually does.

Two questions the corpus cannot answer, both settled from recorded hardware:

1. **Does the bay exit get out?** The simulator at the measured minimum turn
   radius says 0/32 -- never. The bags say usually. One of those is wrong, and
   only the bags are evidence. "Got out" is defined POSITIONALLY, not by the
   phase machine: the manoeuvre's own `rotation_complete` tests |rotation| and
   so reports done on a wrong-way exit, which makes phase departure a
   self-report rather than an observation. Here a run counts as out when it
   reaches the `normal_drive` phase AND accumulates real travel afterwards.

2. **What does obstacle evasion do?** `escape_count` alone conflates a single
   long manoeuvre with many re-triggers, so this reports episodes (contiguous
   runs of a latched `active_maneuver_type`), their durations, the bearing and
   range that triggered them, and how often one fires again immediately -- the
   re-trigger loop that costs both stacks ~20% of the corpus.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_bay_and_escape.py \
        data/live/runs/run_2026090*
"""

from __future__ import annotations

import math
import statistics
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402

BAY_PHASE = "bay_exit"
DRIVE_PHASE = "normal_drive"

# Travel after leaving the bay phase that counts as "actually left the pocket".
# The pocket is ~0.20 m deep, so a run that departs the phase but then moves a
# couple of centimetres never got out -- it only decided it had.
ESCAPED_TRAVEL_M = 0.30

# Gap between latched-maneuver samples that still counts as the SAME episode.
# Debug snapshots arrive at ~10-20 Hz; a hole longer than this is a new trigger.
EPISODE_GAP_S = 1.0


def _phase_of(snapshot) -> str | None:  # noqa: ANN001
    phase = getattr(snapshot, "phase", None)
    if phase is None:
        return None
    return getattr(phase, "value", str(phase))


def _maneuver_of(snapshot) -> str | None:  # noqa: ANN001
    kind = getattr(snapshot, "active_maneuver_type", None)
    if kind is None:
        return None
    return getattr(kind, "value", str(kind))


def _path_length(points: list[tuple[float, float]]) -> float:
    return sum(math.hypot(bx - ax, by - ay) for (ax, ay), (bx, by) in zip(points, points[1:]))


def _unwrapped_yaw_delta(yaws: list[float]) -> tuple[float, float]:
    """(net, total absolute) yaw change in degrees, unwrapped tick to tick."""
    net = 0.0
    total = 0.0
    for a, b in zip(yaws, yaws[1:]):
        d = math.atan2(math.sin(b - a), math.cos(b - a))
        net += d
        total += abs(d)
    return math.degrees(net), math.degrees(total)


@dataclass
class BayResult:
    run: str
    had_bay: bool
    bay_s: float
    net_deg: float
    abs_deg: float
    travel_after_m: float
    reached_drive: bool
    got_out: bool
    guard_blocked_frac: float
    leg_flips: int


@dataclass
class EscapeResult:
    run: str
    escape_count: int | None
    episodes: int
    median_s: float
    max_s: float
    retrigger_frac: float
    kinds: Counter
    trigger_bearings_deg: list[float]
    trigger_ranges_m: list[float]


def _analyse_bay(run: str, rows) -> BayResult:  # noqa: ANN001
    bay = [(t, s) for t, s in rows if _phase_of(s) == BAY_PHASE]
    if not bay:
        return BayResult(run, False, 0.0, 0.0, 0.0, 0.0, False, False, 0.0, 0)

    bay_s = bay[-1][0] - bay[0][0]
    yaws = [s.pose_yaw for _, s in bay if isinstance(s.pose_yaw, (int, float))]
    net_deg, abs_deg = _unwrapped_yaw_delta(yaws) if len(yaws) > 1 else (0.0, 0.0)

    gaps = [s.bay_guard_gap_m for _, s in bay if isinstance(s.bay_guard_gap_m, (int, float))]
    blocked = sum(1 for g in gaps if g <= 0.0) / len(gaps) if gaps else 0.0

    legs = [s.bay_leg_is_reverse for _, s in bay if s.bay_leg_is_reverse is not None]
    flips = sum(1 for a, b in zip(legs, legs[1:]) if a != b)

    bay_end_t = bay[-1][0]
    after = [
        (s.pose_x, s.pose_y)
        for t, s in rows
        if t > bay_end_t and isinstance(s.pose_x, (int, float)) and isinstance(s.pose_y, (int, float))
    ]
    travel_after = _path_length(after)
    reached_drive = any(_phase_of(s) == DRIVE_PHASE for t, s in rows if t > bay_end_t)

    return BayResult(
        run=run,
        had_bay=True,
        bay_s=bay_s,
        net_deg=net_deg,
        abs_deg=abs_deg,
        travel_after_m=travel_after,
        reached_drive=reached_drive,
        got_out=reached_drive and travel_after >= ESCAPED_TRAVEL_M,
        guard_blocked_frac=blocked,
        leg_flips=flips,
    )


def _analyse_escape(run: str, rows) -> EscapeResult:  # noqa: ANN001
    counts = [s.escape_count for _, s in rows if isinstance(s.escape_count, int)]
    final_count = max(counts) if counts else None

    latched = [(t, s) for t, s in rows if _maneuver_of(s) is not None]
    episodes: list[tuple[float, float]] = []
    kinds: Counter = Counter()
    bearings: list[float] = []
    ranges: list[float] = []

    start = prev = None
    for t, s in latched:
        if start is None:
            start = t
        elif t - prev > EPISODE_GAP_S:
            episodes.append((start, prev))
            start = t
        prev = t
        kinds[_maneuver_of(s)] += 1
        if isinstance(s.escape_trigger_angle_rad, (int, float)):
            # Wrap to +/-180. The field is published on a 0..2*pi convention, so
            # a raw degrees() gives medians like +293.9 that read as a bearing
            # the LIDAR cannot report and silently break the front/rear split.
            a = s.escape_trigger_angle_rad
            bearings.append(math.degrees(math.atan2(math.sin(a), math.cos(a))))
        if isinstance(s.escape_trigger_range_m, (int, float)):
            ranges.append(s.escape_trigger_range_m)
    if start is not None:
        episodes.append((start, prev))

    durations = [b - a for a, b in episodes]
    # A re-trigger is a new episode starting within EPISODE_GAP_S*3 of the last
    # one ending: the manoeuvre "finished" and immediately fired again.
    retrig = sum(1 for (_, e), (s2, _) in zip(episodes, episodes[1:]) if s2 - e <= EPISODE_GAP_S * 3)

    return EscapeResult(
        run=run,
        escape_count=final_count,
        episodes=len(episodes),
        median_s=statistics.median(durations) if durations else 0.0,
        max_s=max(durations) if durations else 0.0,
        retrigger_frac=(retrig / (len(episodes) - 1)) if len(episodes) > 1 else 0.0,
        kinds=kinds,
        trigger_bearings_deg=bearings,
        trigger_ranges_m=ranges,
    )


def main() -> None:
    parser = create_bags_parser(__doc__ or "")
    args = parser.parse_args()

    bays: list[BayResult] = []
    escapes: list[EscapeResult] = []
    for bag_dir in args.bag_dirs:
        if not bag_dir.is_dir():
            continue
        try:
            rows, _ = load_nav_debug_rows(bag_dir)
        except Exception as exc:  # noqa: BLE001 - a corrupt bag must not stop the sweep
            print(f"!! {bag_dir.name}: {exc}")
            continue
        if not rows:
            continue
        bays.append(_analyse_bay(bag_dir.name, rows))
        escapes.append(_analyse_escape(bag_dir.name, rows))

    with_bay = [b for b in bays if b.had_bay]
    print(f"\n== BAY EXIT  ({len(with_bay)} of {len(bays)} runs reached the bay_exit phase)")
    if with_bay:
        print_table(
            [
                [
                    b.run.replace("run_", ""),
                    f"{b.bay_s:.1f}",
                    f"{b.net_deg:+.1f}",
                    f"{b.abs_deg:.1f}",
                    f"{b.travel_after_m:.2f}",
                    "Y" if b.reached_drive else ".",
                    "Y" if b.got_out else ".",
                    f"{b.guard_blocked_frac:.0%}",
                    str(b.leg_flips),
                ]
                for b in with_bay
            ],
            ["run", "bay s", "net deg", "|deg|", "after m", "drive?", "OUT?", "blocked", "flips"],
        )
        out = sum(1 for b in with_bay if b.got_out)
        secs = sorted(b.bay_s for b in with_bay)
        print(f"  GOT OUT {out}/{len(with_bay)} ({out / len(with_bay):.0%})")
        print(f"  bay_exit duration s: min {secs[0]:.1f} / median {statistics.median(secs):.1f} / max {secs[-1]:.1f}")
        rot = sorted(abs(b.net_deg) for b in with_bay)
        print(f"  |net rotation| deg:  min {rot[0]:.1f} / median {statistics.median(rot):.1f} / max {rot[-1]:.1f}")
        wrong = sum(1 for b in with_bay if b.got_out and b.abs_deg > 3 * abs(b.net_deg))
        print(f"  rotated back and forth (|total| > 3x |net|): {wrong}/{len(with_bay)}")

    active = [e for e in escapes if e.episodes]
    print(f"\n== OBSTACLE EVASION  ({len(active)} of {len(escapes)} runs latched a manoeuvre)")
    if active:
        print_table(
            [
                [
                    e.run.replace("run_", ""),
                    str(e.escape_count),
                    str(e.episodes),
                    f"{e.median_s:.1f}",
                    f"{e.max_s:.1f}",
                    f"{e.retrigger_frac:.0%}",
                    ",".join(f"{k}:{v}" for k, v in e.kinds.most_common(3)),
                ]
                for e in active
            ],
            ["run", "esc cnt", "episodes", "med s", "max s", "retrig", "kinds"],
        )
        all_bearings = [b for e in active for b in e.trigger_bearings_deg]
        all_ranges = [r for e in active for r in e.trigger_ranges_m]
        print(f"  episodes total {sum(e.episodes for e in active)}")
        if all_bearings:
            front = sum(1 for b in all_bearings if abs(b) <= 45)
            print(
                f"  trigger bearing deg: n={len(all_bearings)} "
                f"median {statistics.median(all_bearings):+.1f}  |bearing|<=45 {front / len(all_bearings):.0%}"
            )
        if all_ranges:
            rs = sorted(all_ranges)
            print(
                f"  trigger range m:     min {rs[0]:.3f} / median {statistics.median(rs):.3f} / max {rs[-1]:.3f}"
            )


if __name__ == "__main__":
    main()
