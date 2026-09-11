r"""Is the INNER division line measurably worse, and at which step does it break?

Reported from the track on 2026-09-11: *"le cuesta cuando tiene que esquivar por
el lado interno"*. This splits the SHIPPED three-way verdict of
``diag_bag_pass_side.py`` -- ``ROUTING`` / ``EXECUTION`` / ``ok`` -- by which
division line the pillar stands on, using the lattice snap already written for
``diag_bag_pass_geometry.classify_lattice``. Nothing about the classification is
reinvented here; only the split is new.

BOUND ON THE INPUT. The line a pillar is assigned to comes from the BELIEVED
position, which on hardware sits 0.15-0.25 m from the real pillar while the two
lines are only 0.20 m apart. So the inner/outer label is noisy by construction,
and that noise is CONSERVATIVE: it mixes the two populations and can only pull
their rates together, never apart. An effect that survives it is real; a null
could be the mixing.

THE ROOM NUMBER. For an execution failure it matters whether the pass was badly
steered or geometrically impossible, so the free lateral margin is computed from
the SNAPPED line (not the believed position, which carries the map error) as

    room = wall_gap - pillar_half - chassis_width

with ``wall_gap`` the nominal distance from the division line to the wall on the
side the rule demands: 0.40 m or 0.60 m in the 1.00 m corridor.

CONTROL. The pooled row must reproduce ``diag_bag_pass_side.py``'s own tally on
the same bags; it is printed so a wrong path cannot masquerade as a null.

Usage::

    pixi run -e dev python scripts/bag/diag_bag_pass_side_inner_outer.py \
        data/live/runs/run_20260911_*
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401,E402  (imported first: models <-> enums cycle)
from scripts.bag.diag_bag_pass_geometry import classify_lattice  # noqa: E402
from scripts.bag.diag_bag_pass_side import Pass, _load, _passes  # noqa: E402
from scripts.common.bag_io import create_bags_parser, settled_direction  # noqa: E402
from scripts.common.tables import print_table  # noqa: E402
from shared.config.constants.robot import RobotSpecs  # noqa: E402
from shared.config.constants.track import CorridorDimensions, TrafficSignSpecs  # noqa: E402
from shared.domain.enums import Axis, Direction, Section  # noqa: E402
from src.config.tuning_helpers import get_tuning, tuning_with_overrides  # noqa: E402
from src.navigation.planning.sign_router.routing import pass_side_lateral_axis  # noqa: E402

CORRIDOR_W = CorridorDimensions.OBSTACLES_WIDTH
PILLAR_HALF = TrafficSignSpecs.WIDTH / 2
CHASSIS_W = RobotSpecs.WIDTH

# Which world axis carries the corridor WIDTH in each lattice section, and which
# way along that axis points at the INNER wall. Mirrors classify_lattice's own
# (section, width, depth) table, restated as an axis so the routing rule -- which
# is expressed as an Axis and a multiplier -- can be compared against it.
SECTION_WIDTH_AXIS: dict[str, tuple[Axis, int]] = {
    "south": (Axis.Y, +1),
    "north": (Axis.Y, -1),
    "west": (Axis.X, +1),
    "east": (Axis.X, -1),
}


def _verdict(p: Pass) -> str:
    """The SHIPPED three-way verdict, character for character as it ships."""
    if p.commanded < 0:
        return "routing"
    if p.achieved < 0:
        return "execution"
    return "ok"


def _room_m(p: Pass, section: str, is_inner: bool, direction: Direction | None) -> tuple[float, bool] | None:
    """Free margin on the side the rule demanded, and whether that side is INNER.

    The second element is the operator's own phrase made measurable: *"esquivar
    por el lado interno"* is about which SIDE OF THE CORRIDOR the chassis was
    sent to, which is not the same question as which line the pillar stands on.
    A pillar on the outer line passed on the inner side has 0.60 m of lane; the
    same pillar passed on the outer side has 0.40 m.

    Returns None when the rule cannot be evaluated (unknown colour/direction) or
    when the routing rule's axis disagrees with the lattice section's width axis
    -- which happens when the robot's corridor and the pillar's section differ at
    a corner, and where a width computed from the wrong axis would be nonsense.
    """
    try:
        corridor = Section.from_string(str(p.corridor))
    except ValueError:
        return None
    rule = pass_side_lateral_axis(corridor, p.colour, direction)
    if rule is None:
        return None
    axis, want = rule
    entry = SECTION_WIDTH_AXIS.get(section)
    if entry is None or entry[0] is not axis:
        return None
    _, toward_inner = entry
    # Distance from the pillar's line to the OUTER wall: that is exactly what
    # the division-line constants measure.
    from_outer = CorridorDimensions.DIVISION_INNER if is_inner else CorridorDimensions.DIVISION_OUTER
    legal_side_is_inner = want == toward_inner
    wall_gap = (CORRIDOR_W - from_outer) if legal_side_is_inner else from_outer
    return wall_gap - PILLAR_HALF - CHASSIS_W, legal_side_is_inner


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE")
    args = parser.parse_args()

    overrides = dict(pair.split("=", 1) for pair in args.set)
    tuning = tuning_with_overrides(overrides, group="sign_discovery") if overrides else get_tuning(None)

    # (pass, which line the pillar stands on, verdict, room, which SIDE it was
    # commanded to pass on)
    records: list[tuple[Pass, str | None, str, float | None, str | None]] = []
    skipped: list[str] = []
    runs = 0
    unplaceable = 0

    for bag in args.bag_dirs:
        try:
            rows, frames, scans = _load(Path(bag))
        except (RuntimeError, OSError, ValueError) as exc:
            skipped.append(f"{Path(bag).name} ({type(exc).__name__})")
            continue
        runs += 1
        run = Path(bag).name.replace("run_", "")
        direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
        passes, peak = _passes(run, rows, frames, scans, tuning)
        print(f"  {run}: {len(passes):3d} passes, direction={direction}, peak believed signs={peak}")
        for p in passes:
            cell = classify_lattice(p.sign_x, p.sign_y)
            if cell is None:
                unplaceable += 1
                records.append((p, None, _verdict(p), None, None))
                continue
            section, is_inner = cell
            room = _room_m(p, section, is_inner, direction)
            records.append(
                (
                    p,
                    "inner" if is_inner else "outer",
                    _verdict(p),
                    None if room is None else room[0],
                    None if room is None else ("inner" if room[1] else "outer"),
                )
            )

    if skipped:
        print(f"== SKIPPED {len(skipped)} unreadable bag(s): {', '.join(skipped[:6])}")
    if not records:
        print("No sign passes reconstructed from these bags.")
        return

    print()
    print(f"== {len(records)} passes over {runs} runs")
    print(f"   {unplaceable} believed positions matched NO legal lattice point (kept only in the CONTROL row)")
    print()

    def tally(members: list[Pass]) -> tuple[int, int, int]:
        vs = [_verdict(p) for p in members]
        return vs.count("routing"), vs.count("execution"), vs.count("ok")

    def row(label: str, members: list[Pass]) -> list:
        r, e, o = tally(members)
        n = r + e + o
        if n == 0:
            return [label, 0, "--", "--", "--", "--"]
        return [
            label,
            n,
            f"{r:3d}  {100 * r / n:5.1f}%",
            f"{e:3d}  {100 * e / n:5.1f}%",
            f"{o:3d}  {100 * o / n:5.1f}%",
            f"{100 * (r + e) / n:5.1f}%",
        ]

    everything = [r[0] for r in records]
    inner = [r[0] for r in records if r[1] == "inner"]
    outer = [r[0] for r in records if r[1] == "outer"]
    side_in = [r[0] for r in records if r[4] == "inner"]
    side_out = [r[0] for r in records if r[4] == "outer"]

    print("== THREE-WAY VERDICT BY DIVISION LINE  (the verdicts are diag_bag_pass_side's own)")
    print_table(
        [
            row("CONTROL: all passes", everything),
            row("INNER line", inner),
            row("OUTER line", outer),
        ],
        ["population", "n", "routing", "execution", "ok", "fail rate"],
    )
    print()
    print("== THE SAME VERDICTS BY THE SIDE THE ROBOT WAS COMMANDED TO PASS ON")
    print("   (the operator's phrase: 'esquivar por el lado interno' names the SIDE, not the line)")
    print_table(
        [
            row("CONTROL: all passes", everything),
            row("commanded INNER side of the pillar", side_in),
            row("commanded OUTER side of the pillar", side_out),
        ],
        ["population", "n", "routing", "execution", "ok", "fail rate"],
    )
    print()

    # Fisher's exact on the 2x2 that the operator's report predicts: inner vs
    # outer, failed vs ok. Reported whatever it says -- a p above 0.05 at this n
    # means the cells cannot separate the two, which is a real answer.
    try:
        from scipy.stats import fisher_exact  # noqa: PLC0415

        def fails(members: list[Pass]) -> tuple[int, int]:
            r, e, o = tally(members)
            return r + e, o

        i_f, i_o = fails(inner)
        o_f, o_o = fails(outer)
        if min(len(inner), len(outer)) > 0:
            odds, p_val = fisher_exact([[i_f, i_o], [o_f, o_o]])
            print(f"== FISHER EXACT, inner vs outer on fail/ok: odds={odds:.2f}  p={p_val:.3f}")
            ri, ei, _ = tally(inner)
            ro, eo, _ = tally(outer)
            if (ri + ei) > 0 and (ro + eo) > 0:
                _, p_step = fisher_exact([[ri, ei], [ro, eo]])
                print(f"   and among FAILURES only, routing vs execution:            p={p_step:.3f}")
        if min(len(side_in), len(side_out)) > 0:
            si_f, si_o = fails(side_in)
            so_f, so_o = fails(side_out)
            odds2, p2 = fisher_exact([[si_f, si_o], [so_f, so_o]])
            print(f"== FISHER EXACT, commanded inner SIDE vs outer SIDE on fail/ok: odds={odds2:.2f}  p={p2:.3f}")
            rsi, esi, _ = tally(side_in)
            rso, eso, _ = tally(side_out)
            if (rsi + esi) > 0 and (rso + eso) > 0:
                _, p2s = fisher_exact([[rsi, esi], [rso, eso]])
                print(f"   and among FAILURES only, routing vs execution:                p={p2s:.3f}")
    except ImportError:
        print("== scipy unavailable; no significance test")
    print()

    # The room the chassis actually had, for the step that is failing.
    print("== LATERAL ROOM ON THE COMMANDED SIDE  (snapped line, not believed position)")
    print(f"   corridor {CORRIDOR_W:.2f} m, pillar half {PILLAR_HALF:.3f} m, chassis {CHASSIS_W:.3f} m")
    room_rows = []
    for label, members in (
        ("all passes", records),
        ("INNER line", [r for r in records if r[1] == "inner"]),
        ("OUTER line", [r for r in records if r[1] == "outer"]),
        ("INNER, execution failures", [r for r in records if r[1] == "inner" and r[2] == "execution"]),
        ("OUTER, execution failures", [r for r in records if r[1] == "outer" and r[2] == "execution"]),
        ("INNER, ok", [r for r in records if r[1] == "inner" and r[2] == "ok"]),
        ("OUTER, ok", [r for r in records if r[1] == "outer" and r[2] == "ok"]),
        ("commanded INNER side", [r for r in records if r[4] == "inner"]),
        ("commanded OUTER side", [r for r in records if r[4] == "outer"]),
        ("cmd INNER side, exec failures", [r for r in records if r[4] == "inner" and r[2] == "execution"]),
        ("cmd OUTER side, exec failures", [r for r in records if r[4] == "outer" and r[2] == "execution"]),
    ):
        rooms = [r[3] for r in members if r[3] is not None]
        tight = sum(1 for v in rooms if v < 0.10)
        room_rows.append(
            [
                label,
                len(members),
                len(rooms),
                f"{sum(rooms) / len(rooms):6.3f}" if rooms else "--",
                f"{min(rooms):6.3f}" if rooms else "--",
                f"{tight}" if rooms else "--",
            ]
        )
    print_table(room_rows, ["population", "n", "room known", "mean room m", "min room m", "room < 0.10 m"])
    print()

    # Was the chassis already on the wrong side when the router engaged? This is
    # the separator diag_bag_exec_failures found on the same corpus, split the
    # new way, so the step verdict can be read without re-running that script.
    print("== HAD TO CROSS AT COMMIT  (commanded correctly, chassis on the wrong side)")
    cross_rows = []
    for label, members in (
        ("all passes", records),
        ("INNER line", [r for r in records if r[1] == "inner"]),
        ("OUTER line", [r for r in records if r[1] == "outer"]),
        ("commanded INNER side", [r for r in records if r[4] == "inner"]),
        ("commanded OUTER side", [r for r in records if r[4] == "outer"]),
    ):
        wanted = [r[0] for r in members if r[0].commanded >= 0]
        if not wanted:
            cross_rows.append([label, 0, "--", "--"])
            continue
        crossers = [p for p in wanted if p.commit_lateral_m < 0]
        won = sum(1 for p in crossers if p.achieved >= 0)
        cross_rows.append(
            [
                label,
                len(wanted),
                f"{len(crossers):3d}  {100 * len(crossers) / len(wanted):5.1f}%",
                f"{won}/{len(crossers)}" if crossers else "--",
            ]
        )
    print_table(cross_rows, ["population", "commanded right", "had to cross", "crossings won"])
    print()
    print("CAVEAT: the inner/outer label comes from the BELIEVED position, whose error")
    print("(0.15-0.25 m) exceeds the 0.20 m gap between the two lines. The mixing that")
    print("causes is conservative -- it shrinks any true difference, never invents one.")


if __name__ == "__main__":
    main()
