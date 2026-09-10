r"""Did the robot ever drive BACKWARDS around the circuit?

Reported from the track on an Open layout with a 1x1 m island: the robot
crossed, corrected, and then went the other way round. That is rule 9.21
territory, so it ends the round rather than costing time.

Neither ``diag_bag_lap_flip_events.py`` nor ``diag_bag_direction_gate.py`` sees
it, and both have the same blind spot: they read the DIRECTION ESTIMATOR. If
the estimator is what went wrong -- and there is history for exactly that, see
``direction_never_adopted_and_corridor_flap_2026_09_06`` -- then asking it
whether the robot reversed is asking the suspect.

This measures the trajectory instead, and never reads the estimator:

* take the bearing of the pose about the track centre,
* UNWRAP it, so a lap is a monotonic 2*pi of accumulated angle,
* and look for sustained sign reversals of its rate.

A robot going round the circuit accumulates angle in ONE direction. One that
turns and comes back reverses the sign and holds it. That is true whatever the
estimator believed, and true even if the estimator never settled at all.

Sustained is doing real work here. Angular rate about a distant centre is noisy
when the chassis is near it, and a corner naturally slows accumulation toward
zero, so single-tick sign flips are the normal case and mean nothing. A
reversal counts only when it holds for ``MIN_REVERSAL_S`` AND sweeps at least
``MIN_REVERSAL_RAD`` -- i.e. the robot committed to going back the way it came.

Reported per run with the angle swept, so a genuine U-turn (a large negative
sweep) reads differently from a wobble at a corner.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_reverse_lap.py data/live/runs/run_*
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import TrackDimensions

from scripts.common.bag_io import Topics, create_bags_parser, decode_nav_debug, elapsed_seconds, open_reader

CENTER = (TrackDimensions.CENTER_COORD, TrackDimensions.CENTER_COORD)

MIN_REVERSAL_S = 1.5
"""Seconds a reversed heading-about-centre must hold to count.

A corner slows angular accumulation toward zero, so brief sign flips are the
normal case. This is long enough that a corner cannot produce one."""

MIN_REVERSAL_RAD = 0.35
"""Radians (~20 deg of arc about the centre) a reversal must sweep.

Guards the other way: a slow reversal that holds for the duration but sweeps
nothing is the chassis sitting still, not driving back round."""

EXCLUDED_PHASES = frozenset({"bay_exit", "parking", "finished_hold"})
"""Phases whose reversing is the manoeuvre, not a wrong-way lap."""

MIN_RADIUS_M = 0.30
"""Ticks closer than this to the centre are dropped, not counted as reversals.

Bearing about a point is undefined at the point and violently noisy near it.
The track is a loop around the centre so this should never fire, but a
diverged pose can place the robot anywhere -- and a localizer failure must not
be reported as a U-turn."""


def _unwrapped_bearings(rows) -> list[tuple[float, float]]:
    """(elapsed_s, unwrapped bearing about the track centre) for usable ticks."""
    out: list[tuple[float, float]] = []
    prev: float | None = None
    turns = 0.0
    for t, snap in rows:
        dx, dy = snap.pose_x - CENTER[0], snap.pose_y - CENTER[1]
        if math.hypot(dx, dy) < MIN_RADIUS_M:
            continue
        raw = math.atan2(dy, dx)
        if prev is not None:
            delta = raw - prev
            if delta > math.pi:
                turns -= 2 * math.pi
            elif delta < -math.pi:
                turns += 2 * math.pi
        prev = raw
        out.append((t, raw + turns))
    return out


def _tolerant_rows(bag_dir: Path) -> tuple[list, int]:
    """Nav-debug rows, skipping payloads a killed run left truncated.

    Per ROW, not per bag: aborting the whole run on one bad tail would drop a
    complete trajectory because of its last few bytes, and a run that ended
    badly is exactly the kind this script is looking for.
    """
    reader = open_reader(bag_dir)
    rows, skipped, t0 = [], 0, None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        if topic != Topics.NAV_DEBUG:
            continue
        try:
            rows.append((elapsed_seconds(t, t0), decode_nav_debug(data)))
        except Exception:  # noqa: BLE001
            skipped += 1
    return rows, skipped


def analyse(bag_dir: Path) -> bool:
    rows, skipped = _tolerant_rows(bag_dir)
    # bay_exit REVERSES BY DESIGN -- it is a ratchet of forward and backward
    # legs inside a pocket. Leaving those ticks in reports the manoeuvre
    # working as a wrong-way lap, which is how run_20260908_012813 first read.
    posed = [
        (t, s)
        for t, s in rows
        if isinstance(s.pose_x, (int, float))
        and isinstance(s.pose_y, (int, float))
        and str(getattr(s.phase, "value", s.phase)) not in EXCLUDED_PHASES
    ]
    series = _unwrapped_bearings(posed)
    if len(series) < 3:
        print(f"{bag_dir.name:<26} no usable pose")
        return False

    total = series[-1][1] - series[0][1]
    dominant = 1.0 if total >= 0 else -1.0

    # Walk the series, accumulating stretches that run AGAINST the dominant
    # direction. A stretch ends the moment the sign agrees again.
    reversals: list[tuple[float, float]] = []
    start_t: float | None = None
    start_a = 0.0
    for (t_prev, a_prev), (t, a) in zip(series, series[1:]):
        against = (a - a_prev) * dominant < 0
        if against and start_t is None:
            start_t, start_a = t_prev, a_prev
        elif not against and start_t is not None:
            held, swept = t_prev - start_t, abs(a_prev - start_a)
            if held >= MIN_REVERSAL_S and swept >= MIN_REVERSAL_RAD:
                reversals.append((held, swept))
            start_t = None
    if start_t is not None:
        held, swept = series[-1][0] - start_t, abs(series[-1][1] - start_a)
        if held >= MIN_REVERSAL_S and swept >= MIN_REVERSAL_RAD:
            reversals.append((held, swept))

    laps = abs(total) / (2 * math.pi)
    flag = "  <-- REVERSAL" if reversals else ""
    print(
        f"{bag_dir.name:<26} swept {math.degrees(total):>8.1f} deg "
        f"({laps:4.2f} laps {'ccw' if dominant > 0 else 'cw'})  "
        f"reversals={len(reversals)}{flag}"
        + (f"  [{skipped} rows skipped]" if skipped else "")
    )
    for held, swept in reversals:
        print(f"      held {held:5.2f} s, swept {math.degrees(swept):6.1f} deg back")
    return bool(reversals)


def main() -> int:
    parser = create_bags_parser(
        description=__doc__ or "", formatter_class=argparse.RawDescriptionHelpFormatter
    )
    args = parser.parse_args()
    found = 0
    for bag_dir in args.bag_dirs:
        try:
            found += int(analyse(Path(bag_dir)))
        except Exception as exc:  # noqa: BLE001
            # ascii(): a corrupt payload puts raw bytes in the message, which a
            # cp1252 console cannot print -- and a crash in the error path
            # would hide the runs that still read fine.
            print(f"{Path(bag_dir).name:<26} unreadable: {ascii(exc)[:120]}")
    print(f"\nruns with a sustained reversal: {found}/{len(args.bag_dirs)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
