r"""Is a sign's pass outcome predicted by the PREVIOUS sign, across a corner?

Claimed from the track on 2026-09-11: after evading one pillar, the NEXT pillar
in the following section is sometimes very hard to reach a line for. The named
geometries are green->green CCW both INNER, red->red CW both INNER,
red->green CCW, green->red CW -- and the proposed remedy is a full-lock
rotation at the corner.

That claim is testable exactly one way: take the SHIPPED three-way verdict of
``diag_bag_pass_side.py`` (ROUTING / EXECUTION / ok) for pillar B, and split it
by whether B was preceded, across a corner, by a pillar A -- and by the colour
pair, the direction and the lattice line each stands on. If the named pairs are
not worse than the passes that follow NOTHING across a corner, the claim is
refuted.

NOTHING about the verdict, the lattice snap or the router replay is reinvented
here: ``_passes`` and ``classify_lattice`` are imported as they ship.

ORDERING. ``_passes`` builds its result from a dict first written at the tick a
sign is FIRST committed to, so the returned list is already in first-commitment
order. That is the order used to form pairs, and it is verified rather than
assumed: the ring-transition table below must be dominated by ONE rotation
sense per direction, which is only true if the order is temporal.

BOUNDS, stated because they decide what the n means:
* One physical pillar passed on three laps collapses to ONE record (the anchor
  key is its believed position), and a phantom duplicate becomes a SEPARATE
  record. So a "pair" is a pair of believed anchors, not certainly of laps.
* ~half of believed positions match no legal lattice point. Those are DROPPED
  from the pair analysis (counted, never imputed) and kept only in the pooled
  control row, where they reproduce diag_bag_pass_side's own tally.

MEASURED 2026-09-11 on the 2026-09-06..09-11 window (131 bags, 126 readable,
68 carrying sign passes, 952 passes; 536 of them -- 56.3% -- unplaceable and
dropped from the pairs). REFUTED, and refuted twice over:

* ``--skip-unplaceable``: 110 cross-corner followers fail 49.1% against 39.7%
  for a clean control with no predecessor at all (Fisher p=0.235).
* strict adjacency: 35 followers fail 28.6% against 49.0% (Fisher p=0.038) --
  the OPPOSITE sign. An effect that flips sign with the pairing definition is
  not an effect.
* none of the four named pairs separates from its matched clean control; the
  two with usable n (red->green CCW 65.0% vs 68.8%, green->red CW 43.5% vs
  71.4%) are if anything BETTER, and the two the operator named most precisely
  reach only n=4 and n=6.
* the POSITIVE CONTROL in the same query does separate, hugely: a pass that
  must cross wins 40.5% against 85.1% for one that only holds (odds 8.4,
  p<0.001), reproducing diag_bag_exec_failures' 34.1% vs 91.3%. The null above
  is therefore a null, not a dead pipeline.

The one measure that leans the operator's way is UPSTREAM of the verdict:
followers stand on the wrong side at commit 54.5% of the time against 39.7%
(p=0.055). It does not carry through to the outcome at this n.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_pass_pairs.py --jobs 8 \
        ../../data/live/runs/run_202609*
"""

from __future__ import annotations

import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from scripts.bag.diag_bag_pass_geometry import classify_lattice  # noqa: E402
from scripts.bag.diag_bag_pass_side import _load, _passes  # noqa: E402
from scripts.common.bag_io import create_bags_parser, settled_direction  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from shared.domain.enums import Direction  # noqa: E402
from src.config.tuning_helpers import get_tuning  # noqa: E402

# The ring, as travelled. Each section is named by the wall its pillars line up
# along (classify_lattice's own naming), so counterclockwise -- bottom corridor
# travelled +x -- runs south, east, north, west. Asserted against the data in
# the RING TRANSITIONS control table, not taken on faith.
CCW_NEXT = {"south": "east", "east": "north", "north": "west", "west": "south"}
CW_NEXT = {v: k for k, v in CCW_NEXT.items()}


@dataclass
class Rec:
    run: str
    direction: str
    order: int
    colour: str
    section: str | None
    line: str | None
    verdict: str
    commit_range_m: float
    commit_lateral_m: float
    commit_speed_mps: float | None
    sign_x: float
    sign_y: float
    peak: int


def _verdict(p) -> str:  # noqa: ANN001
    if p.commanded < 0:
        return "routing"
    if p.achieved < 0:
        return "execution"
    return "ok"


def _one_bag(bag: str) -> tuple[str, list[Rec], str | None]:
    try:
        rows, frames, scans = _load(Path(bag))
    except (RuntimeError, OSError, ValueError) as exc:
        return Path(bag).name, [], type(exc).__name__
    tuning = get_tuning(None)
    run = Path(bag).name.replace("run_", "")
    direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
    passes, peak = _passes(run, rows, frames, scans, tuning)
    out: list[Rec] = []
    for i, p in enumerate(passes):
        cell = classify_lattice(p.sign_x, p.sign_y)
        out.append(
            Rec(
                run=run,
                direction=str(direction),
                order=i,
                colour=str(p.colour),
                section=None if cell is None else cell[0],
                line=None if cell is None else ("inner" if cell[1] else "outer"),
                verdict=_verdict(p),
                commit_range_m=p.commit_range_m,
                commit_lateral_m=p.commit_lateral_m,
                commit_speed_mps=p.commit_speed_mps,
                sign_x=p.sign_x,
                sign_y=p.sign_y,
                peak=peak,
            )
        )
    return Path(bag).name, out, None


def _rate_row(label: str, members: list[Rec]) -> list:
    n = len(members)
    if n == 0:
        return [label, 0, "--", "--", "--", "--"]
    r = sum(1 for m in members if m.verdict == "routing")
    e = sum(1 for m in members if m.verdict == "execution")
    o = n - r - e
    return [
        label,
        n,
        f"{r:3d} {100 * r / n:5.1f}%",
        f"{e:3d} {100 * e / n:5.1f}%",
        f"{o:3d} {100 * o / n:5.1f}%",
        f"{100 * (r + e) / n:5.1f}%",
    ]


HEAD = ["population", "n", "routing", "execution", "ok", "fail rate"]


def _fisher(label: str, a: list[Rec], b: list[Rec]) -> None:
    """Fisher exact on fail/ok for two populations, reported whatever it says."""
    try:
        from scipy.stats import fisher_exact  # noqa: PLC0415
    except ImportError:
        print(f"   {label}: scipy unavailable")
        return
    if not a or not b:
        print(f"   {label}: empty cell, no test")
        return
    af = sum(1 for m in a if m.verdict != "ok")
    bf = sum(1 for m in b if m.verdict != "ok")
    odds, p = fisher_exact([[af, len(a) - af], [bf, len(b) - bf]])
    print(f"   FISHER {label}: {af}/{len(a)} vs {bf}/{len(b)} failed, odds={odds:.2f}  p={p:.3f}")


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument(
        "--skip-unplaceable",
        action="store_true",
        help="chain pairs over the placeable subsequence instead of strict adjacency",
    )
    args = parser.parse_args()

    recs: list[Rec] = []
    skipped: list[str] = []
    runs = 0
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        futs = {pool.submit(_one_bag, str(b)): b for b in args.bag_dirs}
        for fut in as_completed(futs):
            name, got, err = fut.result()
            if err is not None:
                skipped.append(f"{name}({err})")
                continue
            runs += 1
            recs.extend(got)

    print(f"== {len(recs)} passes over {runs} readable runs, {len(skipped)} skipped")
    if skipped:
        print(f"   skipped: {', '.join(skipped[:8])}")
    if not recs:
        print("No sign passes reconstructed.")
        return
    peaks = {r.run: r.peak for r in recs}
    over = sum(1 for v in peaks.values() if v > 8)
    print(f"   runs whose peak believed sign count EXCEEDS the physical max of 8: {over}/{len(peaks)}")
    unplaceable = sum(1 for r in recs if r.section is None)
    print(
        f"   {unplaceable} ({100 * unplaceable / len(recs):.1f}%) believed positions match NO legal "
        "lattice point -> dropped from pairs"
    )
    print(f"   direction mix (passes): {dict(Counter(r.direction for r in recs))}")
    print()

    # ---- CONTROL 0: reproduce diag_bag_pass_side's own pooled tally.
    print("== CONTROL: pooled three-way verdict over every pass (must match diag_bag_pass_side)")
    print_table([_rate_row("all passes", recs)], HEAD)
    print()

    # ---- Build consecutive pairs, per run, in first-commitment order.
    by_run: dict[str, list[Rec]] = {}
    for r in recs:
        by_run.setdefault(r.run, []).append(r)
    for v in by_run.values():
        v.sort(key=lambda r: r.order)

    transitions: Counter = Counter()
    pairs: list[tuple[Rec, Rec]] = []
    same_section = 0
    dropped_unplaceable = 0
    non_adjacent = 0
    for v in by_run.values():
        for a, b in zip(v, v[1:]):
            if a.section is None or b.section is None:
                dropped_unplaceable += 1
                continue
            transitions[(a.direction, a.section, b.section)] += 1
            if a.section == b.section:
                same_section += 1
                continue
            nxt = CCW_NEXT if a.direction == "counterclockwise" else CW_NEXT
            if nxt[a.section] != b.section:
                non_adjacent += 1
                continue
            pairs.append((a, b))

    # A SECOND pairing, because half the records are unplaceable and a single
    # phantom between A and B destroys an otherwise real pair. Here the chain
    # is formed over the PLACEABLE subsequence only -- the adjacency test still
    # requires B to sit in the section immediately after A's in the travel
    # sense, which bounds how far back the skipped predecessor may be.
    if args.skip_unplaceable:
        pairs = []
        for v in by_run.values():
            placeable = [r for r in v if r.section is not None]
            for a, b in zip(placeable, placeable[1:]):
                nxt = CCW_NEXT if a.direction == "counterclockwise" else CW_NEXT
                if a.section != b.section and nxt[a.section] == b.section:
                    pairs.append((a, b))
        print("== PAIRING MODE: chained over PLACEABLE passes only (--skip-unplaceable)")

    print("== RING TRANSITIONS control (does first-commitment order really travel the ring?)")
    for direction in sorted({r.direction for r in recs}):
        fwd = sum(n for (d, s1, s2), n in transitions.items() if d == direction and CCW_NEXT[s1] == s2)
        bwd = sum(n for (d, s1, s2), n in transitions.items() if d == direction and CW_NEXT[s1] == s2)
        same = sum(n for (d, s1, s2), n in transitions.items() if d == direction and s1 == s2)
        opp = sum(
            n
            for (d, s1, s2), n in transitions.items()
            if d == direction and s1 != s2 and CCW_NEXT[s1] != s2 and CW_NEXT[s1] != s2
        )
        print(f"   {direction:17s} ccw-sense {fwd:5d}   cw-sense {bwd:5d}   same {same:5d}   opposite-corner {opp:5d}")
    print(f"   pairs kept (straddle ONE corner, in the travel sense): {len(pairs)}")
    print(f"   rejected: {same_section} same-section, {non_adjacent} not the next section, {dropped_unplaceable} unplaceable")
    print()

    followers = {id(b) for _, b in pairs}

    # THREE predecessor states, because "not a cross-corner follower" pools two
    # very different things: a pillar whose predecessor sat in the SAME section
    # (the already-named BOTH-INNER geometry, known bad) and a pillar with no
    # recent predecessor at all. Pooling them inflates the control's failure
    # rate and would flatter the hypothesis.
    same_sec_followers: set[int] = set()
    for v in by_run.values():
        placeable = [r for r in v if r.section is not None]
        for a, b in zip(placeable, placeable[1:]):
            if a.section == b.section:
                same_sec_followers.add(id(b))
    placeable_recs = [r for r in recs if r.section is not None]
    control = [r for r in placeable_recs if id(r) not in followers]
    clean = [r for r in placeable_recs if id(r) not in followers and id(r) not in same_sec_followers]

    print("== THE CONTROL ROW: what preceded B?")
    print_table(
        [
            _rate_row("B follows a pass ACROSS A CORNER", [b for _, b in pairs]),
            _rate_row("B follows a pass in the SAME section", [r for r in placeable_recs if id(r) in same_sec_followers]),
            _rate_row("CONTROL: no recent placeable predecessor", clean),
            _rate_row("(pooled non-corner control)", control),
        ],
        HEAD,
    )
    _fisher("cross-corner follower vs clean control", [b for _, b in pairs], clean)
    print()

    # ---- The pair table.
    print("== VERDICT OF B, BY (A colour -> B colour, direction, A line -> B line)")
    rows = []
    groups: dict[tuple, list[Rec]] = {}
    for a, b in pairs:
        groups.setdefault((a.direction, a.colour, b.colour, a.line, b.line), []).append(b)
    for key in sorted(groups, key=lambda k: -len(groups[k])):
        d, ca, cb, la, lb = key
        rows.append(_rate_row(f"{d[:3].upper()} {ca}->{cb}  {la}->{lb}", groups[key]))
    print_table(rows, HEAD)
    print()

    print("== COLLAPSED: colour pair x direction (lines pooled)")
    rows = []
    g2: dict[tuple, list[Rec]] = {}
    for a, b in pairs:
        g2.setdefault((a.direction, a.colour, b.colour), []).append(b)
    for key in sorted(g2, key=lambda k: -len(g2[k])):
        d, ca, cb = key
        rows.append(_rate_row(f"{d[:3].upper()} {ca}->{cb}", g2[key]))
    rows.append(_rate_row("CLEAN CONTROL (no predecessor)", clean))
    print_table(rows, HEAD)
    for key in sorted(g2, key=lambda k: -len(g2[k])):
        d, ca, cb = key
        _fisher(
            f"{d[:3].upper()} {ca}->{cb} vs matched clean control",
            g2[key],
            [r for r in clean if r.colour == cb and r.direction == d],
        )
    print()

    print("== MATCHED CONTROL: same B colour+line, cross-corner follower vs not")
    rows = []
    for colour in sorted({r.colour for r in recs}):
        for line in ("inner", "outer"):
            foll = [b for _, b in pairs if b.colour == colour and b.line == line]
            ctrl = [r for r in clean if r.colour == colour and r.line == line]
            if not foll and not ctrl:
                continue
            rows.append(_rate_row(f"B={colour} {line}: FOLLOWER", foll))
            rows.append(_rate_row(f"B={colour} {line}: control ", ctrl))
    print_table(rows, HEAD)
    print()

    print("== THE FOUR NAMED PAIRS (line constraint applied where the operator named it)")
    named = [
        ("green->green CCW, both INNER", "counterclockwise", "green", "green", "inner", "inner"),
        ("red->red CW, both INNER", "clockwise", "red", "red", "inner", "inner"),
        ("red->green CCW (any line)", "counterclockwise", "red", "green", None, None),
        ("green->red CW (any line)", "clockwise", "green", "red", None, None),
    ]
    rows = []
    tests = []
    for label, d, ca, cb, la, lb in named:
        sel = [
            b
            for a, b in pairs
            if a.direction == d
            and a.colour == ca
            and b.colour == cb
            and (la is None or a.line == la)
            and (lb is None or b.line == lb)
        ]
        ctrl = [r for r in clean if r.colour == cb and r.direction == d and (lb is None or r.line == lb)]
        rows.append(_rate_row(label, sel))
        rows.append(_rate_row("   matched CLEAN control (same B colour+line+dir, no predecessor)", ctrl))
        tests.append((label, sel, ctrl))
    print_table(rows, HEAD)
    for label, sel, ctrl in tests:
        _fisher(label, sel, ctrl)
    print()

    # ---- Geometry of the reposition, printed for every pair group so the
    # feasibility question can be answered without a second run.
    print("== REPOSITION GEOMETRY for cross-corner followers")
    print("   'cross needed' = how far B's commit put the chassis on the WRONG side (negative commit_lateral)")
    geo = []
    for key in sorted(g2, key=lambda k: -len(g2[k])):
        d, ca, cb = key
        members = g2[key]
        crossers = [m for m in members if m.commit_lateral_m < 0]
        rng = sorted(m.commit_range_m for m in members)
        spd = sorted(m.commit_speed_mps for m in members if m.commit_speed_mps is not None)
        need = sorted(-m.commit_lateral_m for m in crossers)
        geo.append(
            [
                f"{d[:3].upper()} {ca}->{cb}",
                len(members),
                f"{100 * len(crossers) / len(members):5.1f}%",
                f"{rng[len(rng) // 2]:.2f}" if rng else "--",
                f"{need[len(need) // 2]:.2f}" if need else "--",
                f"{spd[len(spd) // 2]:.2f}" if spd else "--",
            ]
        )
    ctrl_cross = [r for r in control if r.commit_lateral_m < 0]
    for label, members in (("ALL cross-corner followers", [b for _, b in pairs]), ("CLEAN CONTROL (no pred)", clean)):
        cr = [m for m in members if m.commit_lateral_m < 0]
        rng2 = sorted(m.commit_range_m for m in members)
        need2 = sorted(-m.commit_lateral_m for m in cr)
        geo.append(
            [
                label,
                len(members),
                f"{100 * len(cr) / len(members):5.1f}%" if members else "--",
                f"{rng2[len(rng2) // 2]:.2f}" if rng2 else "--",
                f"{need2[len(need2) // 2]:.2f}" if need2 else "--",
                "--",
            ]
        )
    geo.append(
        [
            "CONTROL: no cross-corner pred",
            len(control),
            f"{100 * len(ctrl_cross) / len(control):5.1f}%" if control else "--",
            f"{sorted(r.commit_range_m for r in control)[len(control) // 2]:.2f}" if control else "--",
            f"{sorted(-r.commit_lateral_m for r in ctrl_cross)[len(ctrl_cross) // 2]:.2f}" if ctrl_cross else "--",
            "--",
        ]
    )
    print_table(
        geo,
        ["group", "n", "on wrong side at commit", "commit range p50 m", "cross needed p50 m", "commit speed p50 m/s"],
    )
    # The mechanism that IS known to decide a pass is where the chassis stands
    # when the router engages, so test THAT on followers vs the clean control
    # even when the verdict itself does not move.
    try:
        from scipy.stats import fisher_exact  # noqa: PLC0415

        fo = [b for _, b in pairs]
        a1 = sum(1 for m in fo if m.commit_lateral_m < 0)
        b1 = sum(1 for m in clean if m.commit_lateral_m < 0)
        if fo and clean:
            odds, p = fisher_exact([[a1, len(fo) - a1], [b1, len(clean) - b1]])
            print(
                f"   FISHER wrong-side-at-commit, followers vs clean control: "
                f"{a1}/{len(fo)} vs {b1}/{len(clean)}, odds={odds:.2f}  p={p:.3f}"
            )
    except ImportError:
        pass
    print()

    # POSITIVE CONTROL. A null is only worth reading if the same query can see
    # an effect that is already known to be there. diag_bag_exec_failures found
    # that a pass which must CROSS wins 34.1% against 91.3% for one that only
    # holds its side; if that separation does not reappear here, the null above
    # is a dead pipeline rather than a finding.
    print("== POSITIVE CONTROL: had-to-cross vs already-on-the-legal-side at commit")
    wanted = [r for r in placeable_recs if r.verdict != "routing"]
    crossers = [r for r in wanted if r.commit_lateral_m < 0]
    holders = [r for r in wanted if r.commit_lateral_m >= 0]
    print_table(
        [
            _rate_row("commanded right, HAD TO CROSS", crossers),
            _rate_row("commanded right, already legal", holders),
        ],
        HEAD,
    )
    _fisher("crossers vs holders (known-present effect)", crossers, holders)
    print()
    print("CAVEAT: the line label comes from the BELIEVED position, whose error exceeds the")
    print("0.20 m gap between the two lines; that mixing shrinks any true difference.")


if __name__ == "__main__":
    main()
