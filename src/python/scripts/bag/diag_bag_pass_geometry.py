r"""Which TRACK GEOMETRIES lose the pass side?

Named from the track on 2026-09-11: the pass keeps going wrong in two specific
layouts, and they want counting before they want fixing.

* **BOTH INNER** -- a section carrying two pillars, both on the INNER division
  line. Each has to be passed on its rule-mandated side with no room between
  them to recover, so the second commitment arrives before the first crossing
  has finished.
* **AT THE PARKING LOT** -- a pillar standing immediately in front of the
  parking lot, where the post-lap pursuit already has to be.

Both are predictions that a CROSSING is what costs the pass, which is what
``diag_bag_exec_failures.py`` measured on the same corpus: a pass that must
cross ends on the legal side 34.1% of the time against 91.3% for one that only
has to hold its own.

THE CLASSIFICATION NEEDS NO GROUND TRUTH. A pillar may only stand on 24 legal
positions -- three depth rows crossed with the two corridor division lines, in
four sections -- so snapping the BELIEVED position to the nearest of them says
which section it is in and whether it sits on the INNER or the OUTER line. The
believed position is the right input regardless: it is what the pass was
planned against.

The parking lot is located from the detector's own MAGENTA class (the
parking-block class the GMR detector emits alongside RED/GREEN), so a run
without a lot simply reports none rather than guessing.

READ THE BASELINE COLUMN FIRST. A geometry is only interesting if its pass rate
is WORSE than the corpus as a whole; one that matches the baseline is a
geometry the robot handles like any other, however uncomfortable it looks from
the trackside.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_pass_geometry.py \
        $(cat corpus_obstacles.txt)
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from scripts.bag.diag_bag_pass_side import Pass, _load, _passes  # noqa: E402
from scripts.common.bag_io import create_bags_parser, decode_detections  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from shared.config.constants.track import TrackDimensions, TrafficSignSpecs  # noqa: E402
from shared.domain.models import SignColor  # noqa: E402
from src.config.tuning_helpers import get_tuning, tuning_with_overrides  # noqa: E402
from src.navigation.planning.sign_discovery import _detection_to_world  # noqa: E402

INNER = TrafficSignSpecs.GRID_WIDTH_INNER
OUTER = TrafficSignSpecs.GRID_WIDTH_OUTER
NEAR_LOT_M = 0.50
"""How close a pillar must be to the lot to count as standing in front of it.

Half a metre rather than a tighter number because the believed position carries
the map's own error (p50 15.3 cm at best, and the lot is located from detections
with the same error), and because the pass is planned from ~1.4 m out -- a
pillar 40 cm from the lot is in the way of the same approach as one at 10 cm.
"""


def classify_lattice(x: float, y: float) -> tuple[str, bool] | None:
    """Section, and whether the pillar is on the INNER division line.

    Returns None when the position is not near any legal lattice point, which
    on hardware means the map believed in a pillar that cannot exist -- worth
    excluding rather than forcing into a section.
    """
    size = TrackDimensions.TRACK_SIZE
    # Each section is named by the wall its pillars line up along, and the
    # WIDTH coordinate is the one pinned to a division line.
    for section, width, depth in (
        ("south", y, x),
        ("north", size - y, x),
        ("west", x, y),
        ("east", size - x, y),
    ):
        for line, is_inner in ((INNER, True), (OUTER, False)):
            # Half the 0.20 m gap between the two lines, so the two are never
            # both matched and a position between them matches neither.
            if abs(width - line) <= 0.10 and TrafficSignSpecs.GRID_DEPTH_NEAR - 0.25 <= depth <= TrafficSignSpecs.GRID_DEPTH_FAR + 0.25:
                return section, is_inner
    return None


def find_parking_lot(frames, rows, tuning) -> tuple[float, float] | None:  # noqa: ANN001
    """Mean world position of the MAGENTA detections, or None if there are none.

    The lot does not move during a round, so averaging every sighting is the
    right estimator: it beats any single frame's range error, and a run with no
    lot yields nothing rather than a guess.
    """
    poses = {}
    for rel, d in rows:
        if d.pose_x is not None and d.pose_y is not None and d.pose_yaw is not None:
            poses[rel] = (d.pose_x, d.pose_y, d.pose_yaw)
    if not poses:
        return None
    pose_times = sorted(poses)

    xs: list[float] = []
    ys: list[float] = []
    for rel, payload in frames:
        # Nearest pose at or before this frame; the lag correction inside
        # _detection_to_world handles the rest.
        stamp = min(pose_times, key=lambda t: abs(t - rel))
        px, py, pyaw = poses[stamp]
        for det in decode_detections(payload):
            if det.class_name is not SignColor.MAGENTA:
                continue
            world = _detection_to_world(det, (px, py), pyaw, tuning=tuning)
            if world is not None:
                xs.append(world[0])
                ys.append(world[1])
    if not xs:
        return None
    return sum(xs) / len(xs), sum(ys) / len(ys)


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE")
    args = parser.parse_args()

    overrides = dict(pair.split("=", 1) for pair in args.set)
    tuning = tuning_with_overrides(overrides, group="sign_discovery") if overrides else get_tuning(None)

    tagged: list[tuple[Pass, set[str]]] = []
    skipped: list[str] = []
    lots_found = runs = 0
    unplaceable = 0

    for bag in args.bag_dirs:
        try:
            rows, frames, scans = _load(Path(bag))
        except (RuntimeError, OSError, ValueError) as exc:
            skipped.append(f"{Path(bag).name} ({type(exc).__name__})")
            continue
        runs += 1
        run = Path(bag).name.replace("run_", "")
        passes, _peak = _passes(run, rows, frames, scans, tuning)
        if not passes:
            continue

        placed = {}
        for i, p in enumerate(passes):
            cell = classify_lattice(p.sign_x, p.sign_y)
            if cell is None:
                unplaceable += 1
            else:
                placed[i] = cell

        # A section is "both inner" when TWO of its pillars sit on the inner
        # line. Counted per run, since that is the layout the round was set up
        # with.
        inner_per_section: dict[str, int] = {}
        for section, is_inner in placed.values():
            if is_inner:
                inner_per_section[section] = inner_per_section.get(section, 0) + 1

        lot = find_parking_lot(frames, rows, tuning)
        if lot is not None:
            lots_found += 1

        for i, p in enumerate(passes):
            tags: set[str] = set()
            cell = placed.get(i)
            if cell is not None:
                section, is_inner = cell
                tags.add("inner line" if is_inner else "outer line")
                if is_inner and inner_per_section.get(section, 0) >= 2:
                    tags.add("BOTH INNER")
            if lot is not None and math.dist((p.sign_x, p.sign_y), lot) <= NEAR_LOT_M:
                tags.add("AT THE LOT")
            tagged.append((p, tags))

    if skipped:
        print(f"== SKIPPED {len(skipped)} unreadable bag(s): {', '.join(skipped[:6])}")
        print()
    if not tagged:
        print("No sign passes reconstructed from these bags.")
        return

    def rate(members: list[Pass]) -> str:
        if not members:
            return "      --"
        ok = sum(1 for p in members if p.commanded >= 0 and p.achieved >= 0)
        return f"{ok:4d} / {len(members):4d}  {100 * ok / len(members):5.1f}%"

    everything = [p for p, _ in tagged]
    print(f"== {len(everything)} passes over {runs} runs; a parking lot was located in {lots_found}")
    print(f"   {unplaceable} believed positions matched NO legal lattice point (excluded from the geometry rows)")
    print()
    print("== PASS RATE BY GEOMETRY  (ok = commanded right AND went right)")
    rows_out = [["ALL PASSES (baseline)", rate(everything)]]
    for tag in ("inner line", "outer line", "BOTH INNER", "AT THE LOT"):
        rows_out.append([tag, rate([p for p, tags in tagged if tag in tags])])
    both = [p for p, tags in tagged if "BOTH INNER" in tags and "AT THE LOT" in tags]
    if both:
        rows_out.append(["BOTH INNER + AT THE LOT", rate(both)])
    print_table(rows_out, ["geometry", "passed correctly"])
    print()

    # Split the failures the same way diag_bag_exec_failures.py does, because
    # the two geometries predict a CROSSING problem specifically, not a general
    # loss of quality.
    print("== AND WHEN THEY FAIL, IS IT THE CROSSING?")
    rows_x = []
    for label, members in (
        ("all passes", everything),
        ("BOTH INNER", [p for p, tags in tagged if "BOTH INNER" in tags]),
        ("AT THE LOT", [p for p, tags in tagged if "AT THE LOT" in tags]),
    ):
        wanted = [p for p in members if p.commanded >= 0]
        if not wanted:
            rows_x.append([label, "--", "--"])
            continue
        crossers = [p for p in wanted if p.commit_lateral_m < 0]
        share = f"{100 * len(crossers) / len(wanted):5.1f}%"
        won = sum(1 for p in crossers if p.achieved >= 0)
        rows_x.append([label, share, f"{won}/{len(crossers)}" if crossers else "--"])
    print_table(rows_x, ["population", "had to cross", "crossings won"])


if __name__ == "__main__":
    main()
