"""Show what the pursuit controller does through each corner of a race bag.

A corner is taken as a ``current_corridor`` transition. Around each one this
prints the crosstrack error, the selected lookahead and the commanded steering
in the seconds before and after -- so "it turned too lazily and ran wide" shows
up as steering that stays low while the lookahead stays long, followed by a
crosstrack spike that only then pulls the lookahead short.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_corner_overshoot.py \
        vtitan_runs_pulled/run_XXXXXXXX_XXXXXX
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.navigation_tuning import NavigationTuning

from scripts.common.bag_io import create_bag_parser, load_nav_debug_rows
from scripts.common.tables import print_table

_CORNER_WINDOW_S = 4.0
_SHORT_LOOKAHEAD_M = NavigationTuning.load_default().pursuit.LOOKAHEAD_SHORT
_PEAK_XTRACK_WINDOW_S = 8.0


def main() -> None:
    parser = create_bag_parser("Show pursuit controller behavior through corners")
    parser.add_argument("--max-corners", type=int, default=8, help="Max corners to display")
    args = parser.parse_args()

    rows, _topics = load_nav_debug_rows(args.bag_dir)

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
        window = [(t, snap) for t, snap in rows if abs(t - t_corner) <= _CORNER_WINDOW_S]
        table_rows = []
        for t, snap in window[::4]:
            rel = t - t_corner
            table_rows.append((
                rel,
                snap.crosstrack_error_m,
                snap.path_turn_ahead_rad,
                snap.lookahead_distance_m,
                snap.commanded_steering_norm,
                snap.angle_error_rad,
                snap.commanded_speed_mps,
                snap.forward_clearance_m
            ))
        if table_rows:
            print_table(table_rows, ["rel_t", "xtrack", "turn", "look", "steer", "aerr", "spd", "fwd"])

        xt = [
            snap.crosstrack_error_m
            for t, snap in rows
            if 0 <= t - t_corner <= _PEAK_XTRACK_WINDOW_S and isinstance(snap.crosstrack_error_m, (int, float))
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
            (t - t_corner for t, snap in window if snap.lookahead_distance_m == _SHORT_LOOKAHEAD_M),
            None,
        )
        if armed is not None:
            when = "before the corner" if armed < 0 else "after the corner"
            print(f"   => short lookahead armed at {armed:+.1f}s ({when})")
        else:
            print("   => short lookahead never armed through this corner")


if __name__ == "__main__":
    main()
