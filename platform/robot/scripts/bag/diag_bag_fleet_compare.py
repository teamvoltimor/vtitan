"""Compare several race bags side by side, to rank failure modes across a session.

The per-bag tools answer "what happened in this run". After a session of five
rounds the question is different: which failure is worth fixing first, and is a
given symptom shared or specific. That needs one table per dimension with a
column per run, not five separate reports.

Reports, per run: how the time was spent by phase, whether and when direction
settled (with the gate verdict histogram when it did not), the speed profile,
how much of the run was spent in recovery, path-following error, and where the
start was measured.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_fleet_compare.py \
        vtitan_runs_pulled/run_A vtitan_runs_pulled/run_B ...
"""

from __future__ import annotations

import statistics
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.domain.enums import NavigatorPhase

from scripts.common.bag_io import create_bags_parser, load_nav_debug_rows, measured_start, settled_direction
from src.navigation.planning.waypoints import corridor_for_position

_REPORTED_PHASES = (
    NavigatorPhase.BLIND_CREEP,
    NavigatorPhase.NORMAL_DRIVE,
    NavigatorPhase.ACTIVE_MANEUVER,
    NavigatorPhase.STUCK_ESCAPE_HOLDING,
    NavigatorPhase.ESCAPE_TRIGGERED,
    NavigatorPhase.FINISHED_HOLD,
)
"""Phases worth a row. Named from the enum rather than spelled as strings, so a
renamed member breaks here instead of silently reporting 0% forever."""

_CREEP_SPEED_MPS = 0.06
"""At or below this, the robot is creeping rather than racing.

Sits just above the 0.05 creep setpoint so float noise doesn't split the
bucket, and well below the 0.15 normal cruise.

Deliberately pinned to the PRE-2026-08-09 setpoint. The creep tier was raised
to 0.075 m/s on that date (speed.CREEP_FRAC), so this threshold reads bags
recorded before the change correctly and will report 0% creep for bags
recorded after it. Raise it to ~0.09 when the older bags stop mattering --
changing it now would silently reinterpret every historical run this script
exists to compare.
"""

_STOPPED_SPEED_MPS = 0.005
"""Below this a command is a stop, not a slow crawl. Well under CREEP_SPEED."""

_STALLED_MOVEMENT_M = 0.01
"""Movement over the navigator's own recent-movement window that counts as none.

Compared against ``recent_movement_m``, the same signal the stuck detector
uses, so "commanded but still" here means what the navigator itself would call
not moving.
"""


def _pct(count: int, total: int) -> str:
    return f"{100.0 * count / total:.0f}%" if total else "-"


def _num(values: list[float], fmt: str = "{:.2f}") -> str:
    return fmt.format(statistics.median(values)) if values else "-"


def _summarize(bag_dir: Path) -> dict[str, str]:
    rows, _topics = load_nav_debug_rows(bag_dir)
    if not rows:
        return {"run": bag_dir.name, "note": "empty"}

    total = len(rows)
    span = rows[-1][0] - rows[0][0]
    out: dict[str, str] = {"run": bag_dir.name.replace("run_2026", ""), "span_s": f"{span:.0f}"}

    phases = Counter(s.phase for _, s in rows if s.phase)
    for phase in _REPORTED_PHASES:
        out[phase.value] = _pct(phases.get(phase, 0), total)

    laps = [s.laps_completed for _, s in rows if isinstance(s.laps_completed, int)]
    out["laps"] = str(max(laps)) if laps else "-"

    settled = next((t for t, s in rows if s.direction), None) if settled_direction(rows) else None
    out["dir_at_s"] = f"{settled:.1f}" if settled is not None else "NEVER"
    if settled is None:
        verdicts = Counter(
            s.direction_gate_verdict.split(" ")[0] for _, s in rows if s.direction_gate_verdict
        )
        out["gate"] = ", ".join(f"{k} {_pct(v, sum(verdicts.values()))}" for k, v in verdicts.most_common(2)) or "-"
    else:
        out["gate"] = "-"

    speeds = [s.commanded_speed_mps for _, s in rows if isinstance(s.commanded_speed_mps, (int, float))]
    forward = [s for s in speeds if s > 0]
    out["med_spd"] = _num(forward)
    out["creep%"] = _pct(sum(1 for s in forward if s <= _CREEP_SPEED_MPS), len(forward))
    out["reverse%"] = _pct(sum(1 for s in speeds if s < 0), len(speeds))
    # Two different kinds of "not moving", and they have different causes:
    # commanded zero is the navigator choosing to hold, while a non-zero command
    # with no measured movement is the robot failing to execute one.
    out["cmd_zero%"] = _pct(sum(1 for s in speeds if abs(s) < _STOPPED_SPEED_MPS), len(speeds))
    stalled = [
        s
        for _, s in rows
        if isinstance(s.recent_movement_m, (int, float))
        and isinstance(s.commanded_speed_mps, (int, float))
        and abs(s.commanded_speed_mps) >= _STOPPED_SPEED_MPS
    ]
    out["cmd_but_still%"] = _pct(sum(1 for s in stalled if s.recent_movement_m < _STALLED_MOVEMENT_M), len(stalled))

    out["escapes"] = str(max((s.escape_count for _, s in rows if isinstance(s.escape_count, int)), default=0))
    maneuvers = Counter(s.active_maneuver_type for _, s in rows if s.active_maneuver_type)
    out["maneuvers"] = ", ".join(f"{k}x{v}" for k, v in maneuvers.most_common(2)) or "-"

    xtrack = [abs(s.crosstrack_error_m) for _, s in rows if isinstance(s.crosstrack_error_m, (int, float))]
    out["xtrack_med"] = _num(xtrack)
    out["xtrack_max"] = f"{max(xtrack):.2f}" if xtrack else "-"

    clear = [s.forward_clearance_m for _, s in rows if isinstance(s.forward_clearance_m, (int, float))]
    out["min_clear"] = f"{min(clear):.2f}" if clear else "-"

    start = measured_start(rows)
    out["start"] = f"({start[0]:.2f},{start[1]:.2f})" if start else "-"
    out["start_sec"] = corridor_for_position(*start).value if start else "-"
    return out


def main() -> None:
    parser = create_bags_parser("Analyze multiple bags")
    args = parser.parse_args()

    summaries = [_summarize(d) for d in args.bag_dirs]
    keys = list(summaries[0].keys())

    # One row per metric, one column per run: five runs read far better compared
    # down a row than as five separate blocks.
    width = max(len(k) for k in keys)
    header = f"{'metric':<{width}} | " + " | ".join(f"{s['run']:>16}" for s in summaries)
    print(header)
    print("-" * len(header))
    for key in keys:
        if key == "run":
            continue
        print(f"{key:<{width}} | " + " | ".join(f"{s.get(key, '-'):>16}" for s in summaries))


if __name__ == "__main__":
    main()
