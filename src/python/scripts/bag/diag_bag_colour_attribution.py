r"""Does the camera's colour land on the RIGHT pillar, and would pooling fix it?

STATUS: scores against the operator's stated layout, so it only runs on rounds
whose pillar map reconstructs to that layout. It reports the control per round
and refuses to score the ones that fail it.

THE QUESTION. A sign's colour decides which side of it the round must pass, and
a wrong-side pass ends the round. Colour is decided by a vote over detections
attributed to a track by PROXIMITY. Measured on 2026-09-15, the camera bearing
carries +/-12 deg of zero-mean scatter, which at 1.5 m is 0.31 m of lateral
miss, while ``sign_discovery.association_dist_m`` is 0.25 and the router's
``detection_match_dist_m`` is 0.30. **The scatter is wider than the radius that
is supposed to contain it**, so detections of one pillar land on its neighbour.

That is a claim about geometry. This measures whether it actually costs a
colour, which is the only thing that matters.

WHAT IT MEASURES, against LIDAR-located pillars coloured by the layout:

* **Attribution**: for every detection, the distance to the nearest pillar and
  whether that pillar's TRUE colour matches what the camera said. A detection
  whose colour contradicts its nearest pillar either landed on the wrong object
  or the classifier was wrong; both are the same failure downstream.
* **Verdict per pillar under three schemes**, so the fix is scored and not
  argued:
  - ``nearest``  -- today's rule, winner of the votes within ``--match``
  - ``exclusive`` -- votes within ``--match`` that are NOT within ``--match`` of
    any OTHER pillar, i.e. drop the ambiguous ones instead of splitting them
  - ``pooled``   -- ``sign_discovery.colour_pool_radius_m``, which SHIPS AT 0.0:
    sum the votes of every pillar within ``--pool`` and take the winner

The comparison is the point. ``pooled`` is a knob that already exists and is
switched off, and the simulator cannot screen it -- its vision emulator never
lies about colour, so every scheme scores identically there. The bags are the
only instrument that can see it.

CONTROLS, because a crashed diagnostic here exits 0:

* The reconstructed layout must come out 5 GREEN / 3 RED, the operator's stated
  count. A round that misses it is printed and NOT scored.
* Detection counts are printed per pillar. A scheme that "fixes" a pillar with
  three votes has not fixed anything.
* ``nearest`` is the shipped rule, so its column is the baseline. If all three
  columns agree, the attribution error costs no colour and this whole line of
  work is dead -- which is a result, not a failure.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/bag/diag_bag_colour_attribution.py RUN_DIR...
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

# isort: off
from scripts.bag.diag_bag_pass_side_truth import (  # noqa: E402
    _EXPECTED_GREEN,
    _EXPECTED_RED,
    _FIN_BAND_M,
    _assign_sections,
    _layout_colour,
    _near_wall,
    _on_lattice,
    _read,
)
from scripts.common.bag_io import create_bags_parser  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from shared.domain.models import SignColor  # noqa: E402

# isort: on


def _margin(votes: dict[SignColor, int]) -> float:
    """Winner's share of the two-colour vote. 0.5 is a coin flip.

    The WINNER being right is not the same as the vote being sound. A pillar
    decided 68 to 61 is one bad frame from flipping a rule that ends the round,
    and a pillar decided by ONE vote is not decided at all.
    """
    red, green = votes.get(SignColor.RED, 0), votes.get(SignColor.GREEN, 0)
    total = red + green
    return max(red, green) / total if total else 0.0


def _winner(votes: dict[SignColor, int]) -> SignColor | None:
    """Majority colour, or None when nothing voted."""
    if not votes or sum(votes.values()) == 0:
        return None
    return max(votes, key=lambda c: votes[c])


def _schemes(
    pillars: list[tuple[float, float, SignColor]],
    dets: list[tuple[float, float, SignColor]],
    match: float,
    pool: float,
) -> list[dict[str, object]]:
    """Per-pillar colour under each attribution scheme."""
    near: list[Counter] = [Counter() for _ in pillars]
    excl: list[Counter] = [Counter() for _ in pillars]
    for dx, dy, colour in dets:
        within = [i for i, (px, py, _c) in enumerate(pillars) if math.hypot(dx - px, dy - py) <= match]
        if not within:
            continue
        best = min(within, key=lambda i: math.hypot(dx - pillars[i][0], dy - pillars[i][1]))
        near[best][colour] += 1
        # EXCLUSIVE: a detection that two pillars could both claim says nothing
        # about which, so it is discarded rather than handed to the closer one.
        if len(within) == 1:
            excl[within[0]][colour] += 1

    rows = []
    for i, (px, py, truth) in enumerate(pillars):
        pooled: Counter = Counter()
        for j, (qx, qy, _c) in enumerate(pillars):
            if math.hypot(px - qx, py - qy) <= pool:
                pooled.update(near[j])
        rows.append(
            {
                "pos": (px, py),
                "truth": truth,
                "n": sum(near[i].values()),
                "nearest": _winner(near[i]),
                "margin": _margin(near[i]),
                "exclusive": _winner(excl[i]),
                "pooled": _winner(pooled),
            }
        )
    return rows


def main() -> int:
    """Score the colour vote against the operator layout, three ways."""
    parser = create_bags_parser(__doc__ or "")
    parser.add_argument("--wall-margin", type=float, default=0.25)
    parser.add_argument("--cell", type=float, default=0.05)
    parser.add_argument("--min-returns", type=int, default=250)
    parser.add_argument("--max-pillars", type=int, default=10)
    parser.add_argument("--match", type=float, default=0.35, help="a detection this close to a pillar votes for it")
    parser.add_argument("--pool", type=float, default=0.40, help="colour_pool_radius_m to evaluate (0 = shipped)")
    parser.add_argument("--detail", action="store_true")
    args = parser.parse_args()

    totals = {"nearest": 0, "exclusive": 0, "pooled": 0, "signs": 0, "rounds": 0}
    summary = []
    for bag_dir in args.bag_dirs:
        track, direction, raw_pillars, dets = _read(
            bag_dir, args.wall_margin, args.cell, args.min_returns, args.max_pillars
        )
        name = bag_dir.name.replace("run_", "")
        if direction is None or not raw_pillars:
            summary.append([name, "-", "no direction or no pillars", "", "", "", "", ""])
            continue
        signs = [(x, y, n) for x, y, n in raw_pillars if _near_wall(x, y) >= _FIN_BAND_M and _on_lattice(x, y)]
        secs = _assign_sections(signs)
        pillars: list[tuple[float, float, SignColor]] = []
        for i, (x, y, _n) in enumerate(signs):
            colour = _layout_colour(x, y, secs[i])
            if colour is not None:
                pillars.append((x, y, colour))
        greens = sum(1 for _x, _y, c in pillars if c is SignColor.GREEN)
        reds = len(pillars) - greens
        if (greens, reds) != (_EXPECTED_GREEN, _EXPECTED_RED):
            summary.append(
                [name, str(direction).split(".")[-1][:4], f"{greens}G/{reds}R CONTROL FAILED", "", "", "", "", ""]
            )
            continue

        rows = _schemes(pillars, dets, args.match, args.pool)
        counts = {k: sum(1 for r in rows if r[k] is not r["truth"]) for k in ("nearest", "exclusive", "pooled")}
        for k in counts:
            totals[k] += counts[k]
        totals["signs"] += len(rows)
        totals["rounds"] += 1

        if args.detail:
            print(f"\n=== {bag_dir.name}  direction={direction}")
            print_table(
                [
                    [
                        f"({r['pos'][0]:.2f},{r['pos'][1]:.2f})",
                        r["truth"].value,
                        r["n"],
                        f"{r['margin']:.2f}",
                        *[
                            ("-" if r[k] is None else r[k].value) + ("" if r[k] is r["truth"] else "  WRONG")
                            for k in ("nearest", "exclusive", "pooled")
                        ],
                    ]
                    for r in rows
                ],
                ["pillar", "layout", "votes", "margin", "nearest (ships)", "exclusive", f"pooled@{args.pool}"],
            )
        summary.append(
            [
                name,
                str(direction).split(".")[-1][:4],
                f"{greens}G/{reds}R",
                f"{counts['nearest']}/{len(rows)}",
                f"{counts['exclusive']}/{len(rows)}",
                f"{counts['pooled']}/{len(rows)}",
                f"{min(r['margin'] for r in rows):.2f}",
                str(min(r["n"] for r in rows)),
            ]
        )

    print()
    print_table(
        summary,
        [
            "run", "dir", "control", "nearest WRONG", "exclusive WRONG",
            f"pooled@{args.pool} WRONG", "thinnest margin", "fewest votes",
        ],
    )
    if totals["rounds"]:
        print(
            f"\nOver {totals['rounds']} scoreable round(s), {totals['signs']} signs:  "
            f"nearest {totals['nearest']}   exclusive {totals['exclusive']}   pooled {totals['pooled']}"
        )
    print(
        "\n`nearest` is the SHIPPED rule and is the baseline. `pooled` is\n"
        "sign_discovery.colour_pool_radius_m, which ships at 0.0. If all three columns\n"
        "agree, mis-attribution costs no colour here and the knob is not worth turning."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
