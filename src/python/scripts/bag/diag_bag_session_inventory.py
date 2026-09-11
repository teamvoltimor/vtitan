"""One row per bag: how a whole hardware session ended, not how one run did.

A session of 27 short runs cannot be read one bag at a time -- the question
"which of these actually raced, and what stopped the rest" is a property of
the SET, and reading it run-by-run is where the 2026-09-08 first answer went
wrong (the runs were there; the directory being read was not). This replays
every bag under a directory and prints one row each: duration, final phase,
laps, escapes, and the last /race_metrics payload.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_session_inventory.py data/live/runs --prefix run_20260908
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from rclpy.serialization import deserialize_message
from std_msgs.msg import String

from scripts.common.bag_io import Topics, decode_nav_debug, elapsed_seconds, open_reader


def summarize(bag_dir: Path) -> dict[str, object]:
    reader = open_reader(bag_dir)
    t0 = None
    last_ts = 0.0
    first_snap = None
    last_snap = None
    escape_max = 0
    escape_phase_ticks = 0
    escape_entries = 0
    was_escaping = False
    laps_max = 0
    num_laps_seen = 0
    nav_ticks = 0
    corridors: list[str] = []
    phases: dict[str, int] = {}
    last_metrics: dict[str, object] | None = None
    states: list[str] = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        last_ts = elapsed_seconds(t, t0)

        if topic == Topics.NAV_DEBUG:
            snap = decode_nav_debug(data)
            nav_ticks += 1
            if first_snap is None:
                first_snap = snap
            ph = str(getattr(snap.phase, "value", snap.phase))
            phases[ph] = phases.get(ph, 0) + 1
            # The node republishes a NOT_YET_STEPPED snapshot once the run
            # stops, which zeroes laps/num_laps. Taking it as the outcome
            # reported 0/0 laps on runs that had plainly driven, so the last
            # STEPPED snapshot is the run's end state, not the last message.
            if ph != "not_yet_stepped":
                last_snap = snap
                laps_max = max(laps_max, snap.laps_completed)
                num_laps_seen = max(num_laps_seen, snap.num_laps)
            escaping = "escape" in ph or "maneuver" in ph
            if escaping:
                escape_phase_ticks += 1
                if not was_escaping:
                    escape_entries += 1
            was_escaping = escaping
            if snap.escape_count is not None:
                escape_max = max(escape_max, snap.escape_count)
            if snap.current_corridor is not None:
                c = str(getattr(snap.current_corridor, "value", snap.current_corridor))
                if not corridors or corridors[-1] != c:
                    corridors.append(c)
        elif topic == Topics.ROBOT_STATE:
            s = deserialize_message(data, String).data
            if not states or states[-1] != s:
                states.append(s)
        elif topic == "/race_metrics":
            try:
                last_metrics = json.loads(deserialize_message(data, String).data)
            except (ValueError, TypeError):
                last_metrics = None

    return {
        "run": bag_dir.name,
        "dur": last_ts,
        "nav_ticks": nav_ticks,
        "final_phase": str(getattr(last_snap.phase, "value", last_snap.phase)) if last_snap else "-",
        "laps": laps_max,
        "num_laps": num_laps_seen,
        "escape_entries": escape_entries,
        "direction": str(getattr(last_snap.direction, "value", last_snap.direction)) if last_snap and last_snap.direction else "-",
        "escapes": escape_max,
        "escape_ticks": escape_phase_ticks,
        "corridor_flaps": max(len(corridors) - 1, 0),
        "states": states,
        "phases": phases,
        "metrics": last_metrics,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("root", type=Path)
    ap.add_argument("--prefix", default="run_")
    ap.add_argument("--json", action="store_true", help="dump full per-run detail as JSON")
    args = ap.parse_args()

    bags = sorted(d for d in args.root.iterdir() if d.is_dir() and d.name.startswith(args.prefix))
    rows = []
    for b in bags:
        try:
            rows.append(summarize(b))
        except Exception as exc:  # noqa: BLE001 - inventory must not stop on one bad bag
            rows.append({"run": b.name, "dur": -1.0, "final_phase": f"ERROR {exc}", "laps": 0,
                         "num_laps": 0, "direction": "-", "escapes": 0, "escape_ticks": 0, "escape_entries": 0,
                         "corridor_flaps": 0, "nav_ticks": 0, "states": [], "phases": {}, "metrics": None})

    if args.json:
        print(json.dumps(rows, indent=1, default=str))
        return

    hdr = f"{'run':22} {'dur':>7} {'ticks':>6} {'laps':>5} {'escN':>5} {'escTk':>6} {'esc%':>5} {'flaps':>6} {'dir':>16}  final_phase"
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        pct = 100.0 * r['escape_ticks'] / r['nav_ticks'] if r['nav_ticks'] else 0.0
        print(f"{r['run']:22} {r['dur']:7.1f} {r['nav_ticks']:6} "
              f"{r['laps']}/{r['num_laps']:>3} {r['escape_entries']:5} {r['escape_ticks']:6} "
              f"{pct:5.1f} {r['corridor_flaps']:6} {r['direction']:>16}  {r['final_phase']}")


if __name__ == "__main__":
    main()
