"""Show what the pursuit controller does through each corner of a race bag.

A corner is taken as a ``current_corridor`` transition. Around each one this
prints the crosstrack error, the selected lookahead and the commanded steering
in the seconds before and after -- so "it turned too lazily and ran wide" shows
up as steering that stays low while the lookahead stays long, followed by a
crosstrack spike that only then pulls the lookahead short.

Usage:
    pixi run -e dev python scripts/diag_bag_corner_overshoot.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from _bag_io import open_reader, read_nav_debug_rows

WINDOW_S = 4.0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--max-corners", type=int, default=8)
    args = parser.parse_args()

    reader = open_reader(args.bag_dir)
    rows, _topics = read_nav_debug_rows(reader)

    corners = []
    prev = None
    for t, snap in rows:
        corridor = snap.current_corridor
        if corridor is not None and corridor != prev:
            if prev is not None:
                corners.append((t, prev, corridor))
            prev = corridor

    print(f"== {args.bag_dir.name} == {len(corners)} corridor transitions")

    def g(snap, k: str, spec: str = "5.2f") -> str:
        v = getattr(snap, k)
        return format(v, spec) if isinstance(v, (int, float)) else " None"

    for t_corner, a, b in corners[: args.max_corners]:
        print(f"\n-- {a} -> {b} at {t_corner:.1f}s")
        window = [(t, snap) for t, snap in rows if abs(t - t_corner) <= WINDOW_S]
        for t, snap in window[::4]:
            rel = t - t_corner
            print(
                f"   {rel:+5.1f}s xtrack={g(snap, 'crosstrack_error_m')} "
                f"turn={g(snap, 'path_turn_ahead_rad')} "
                f"look={g(snap, 'lookahead_distance_m')} steer={g(snap, 'commanded_steering_norm')} "
                f"aerr={g(snap, 'angle_error_rad')} spd={g(snap, 'commanded_speed_mps')} "
                f"fwd={g(snap, 'forward_clearance_m')}"
            )
        xt = [
            snap.crosstrack_error_m
            for t, snap in rows
            if 0 <= t - t_corner <= 8.0 and isinstance(snap.crosstrack_error_m, (int, float))
        ]
        st = [
            abs(snap.commanded_steering_norm)
            for t, snap in window
            if isinstance(snap.commanded_steering_norm, (int, float))
        ]
        if xt and st:
            print(f"   => peak xtrack in the 8s after: {max(xt):.2f} m; peak |steer| through: {max(st):.2f}")

        # The question the 2026-08-06 fix exists to answer: did the short
        # lookahead arm on the way IN, or only once the corner had been run
        # wide of? Anything but "before" means the preview fired too late.
        armed = next(
            (t - t_corner for t, snap in window if snap.lookahead_distance_m == 0.20),
            None,
        )
        if armed is not None:
            when = "before the corner" if armed < 0 else "after the corner"
            print(f"   => short lookahead armed at {armed:+.1f}s ({when})")
        else:
            print("   => short lookahead never armed through this corner")


if __name__ == "__main__":
    main()
