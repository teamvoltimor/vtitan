r"""Which sign-PAIR cell across a corner actually cost the 2026-09-12 rounds?

On 2026-09-12 the operator ran 31 OBSTACLES rounds on the competition track
(1.00 m corridor) and NONE reached 3/3; the same car had reached 3/3 seventeen
times that morning on the practice track. The operator named four geometries he
believes carry the failures, all of them a PAIR of pillars straddling a corner
with one on the INNER division line and one on the OUTER:

    green->red   CLOCKWISE          red->green  COUNTERCLOCKWISE
    green->green COUNTERCLOCKWISE   red->red    CLOCKWISE

``diag_bag_pass_pairs.py`` already tests exactly that hypothesis and REFUTED it
on the 2026-09-11 corpus. It is re-run alongside this script rather than
rewritten; what this one adds is everything that verdict does not carry:

* **corner vs straight PER PILLAR.** The sign grid rows sit at depth 1.0, 1.5
  and 2.0 along a section whose straight spans exactly 1.0..2.0, so a pillar is
  never *inside* a corner -- it is at the straight's ENTRY (just after a
  corner), its MIDDLE, or its EXIT (just before the next one). Which of the
  three it is depends on the travel sense, so the label is computed from the
  direction and the section's own depth axis, not from the raw coordinate.
* **the achieved CLEARANCE**, so a legal-but-touching pass is not scored the
  same as a clean one. ``clearance = |lateral| - pillar_half - chassis_half``;
  under 0.030 m is the graze band settled on 2026-09-12, at or under zero is
  contact.
* **a pair-level GEOMETRIC demand.** For every kept pair the lane translation
  the rule asks for between A and B is computed from the SNAPPED lattice (the
  believed position carries the map error, the snap does not), together with
  the along-track distance the corner actually offers, and turned into a
  demanded radius ``R = L^2 / (4*dLat)`` for an S-curve. That is compared
  against the speed curve ``R = 0.053 + 1.86|v|`` the chassis really achieves.
* **a matched CONTROL GROUP**, passed with ``--control``: the practice-track
  rounds of the same morning that DID finish 3/3. Without it a failure rate has
  nothing to be high relative to.

TRAPS OBSERVED, because each has bitten this repo:
* ``wrong_side_pass_count`` is NOT read -- it judges from the believed layout
  and has read 24 where the truth was 7.
* ``sign_deform_magnitude_m`` is NOT read -- ``sign_lane_suppress_deform``
  ships TRUE, so the deformation is computed and discarded.
* the verdict, the lattice snap and the router replay are imported from the
  scripts that ship them; only the joins are new.

MEASURED 2026-09-12 over the 31 competition rounds (197 committed passes) with
the 18 practice rounds of the same morning that finished 3/3 as control (90
passes). The sign map is in far better shape than the 2026-09-11 corpus: 0.0% of
believed positions are unplaceable on the lattice, against 56.3% then, and no run
believes in more than the physical maximum of 8 pillars.

THE HEADLINE IS A CORRECTION TO THE TOOL, not to the car. ``diag_bag_pass_side``
keys its verdict on ``d.current_corridor`` -- the ROBOT's corridor -- which the
fork here reproduces 197/197, while its own docstring says the SIGN's corridor is
the one to use. The router's per-sign label agrees with the independent lattice
section on 287/287 passes, so the arbiter is unambiguous, and under it:

    routing errors, competition:  21  ->  1
    routing errors, practice:     22  ->  0

42 of 43 "the planner COMMANDED the illegal side" verdicts are the perpendicular
axis, not a planner fault, and they land at the ENTRY row because that is where
robot and pillar corridors differ. The 2026-09-12 note "5 of 24 passes commanded
the WRONG side, 4 of 5 RED" is from the same substitution and does not survive.

WHAT REMAINS, all on the corrected verdict:

* THE PAIR HYPOTHESIS IS REFUTED AGAIN, and by the control's own sign: a B that
  follows a pass across a corner fails 15.1% (n=73) against 30.6% (n=36) for a B
  with no recent predecessor, and 8.0% for one preceded inside its own section.
* the four named cells collapse to n=2, 3, 2 and 2 once the inner/outer
  constraint the operator stated is applied, and carry 1, 0, 0 and 0 wrong-side
  passes respectively. Only under the UNCORRECTED verdict do they read 100%.
* the BAND CHANGE is not the discriminator either: CHANGE 13.5% (n=37) against
  HOLD inner->inner 20.0% (n=30). The demanded S-curve radius across a corner is
  2.76 m p50 against 0.35 m available -- comfortable by 8x. The corner shear is
  real geometry and is NOT what these passes die of.
* the one pass-level cell that leans is the SLOT: the first grid row AFTER a
  corner fails 20.5% (n=73) against 10.0% (n=90) for the last row before one
  (odds 2.33, p=0.076), and 36.1% vs 23.1% in the control. Same sign in both
  groups, significant in neither. Its artefact control holds: keyed on the raw
  coordinate with the direction dropped the same split reads odds 1.74, p=0.269.
* THE LOCUS IS TRACKING. Commit range is 0.65 m on a lost pass against 0.82 m on
  a clean one and the ENTRY row is detected no later than the EXIT row (0.75 vs
  0.81), so it is not perception. The plan is the same either way -- asked 0.243
  m on the lost passes against 0.236 m on the clean ones -- and the chassis
  delivers 0.099 m against 0.243 m. 71.4% of lost passes ran under a latched
  manoeuvre against 34.5% of clean ones.
* THE CORRIDOR WIDTH SHOWS UP IN ONE PLACE: on the 1.00 m competition corridor
  the chassis leaves a corner already 0.225 m into the WRONG band (p50) and owes
  0.421 m against 0.350 m of arc; on the narrower practice corridor the same row
  is entered at +0.149 m, on the legal side.

NOT ESTABLISHED: "contact" here is a BELIEVED clearance against a believed pillar
position carrying 0.15-0.25 m of error, so the 69/197 contact count is an
over-read of unknown size and no bag says whether a pillar was displaced. Nor can
a bag distinguish an operator stop from a failure, and most of these rounds were
stopped by hand.

Usage::

    VTITAN_HARDWARE_PROFILE='270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm' \
    PYTHONPATH=. pixi run -e dev python \
        scripts/bag/diag_bag_pair_crossing_2026_09_12.py --jobs 8 \
        --control ../../data/live/runs/run_20260912_074544 \
        ../../data/live/runs/run_20260912_09*
"""

from __future__ import annotations

import math
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums
from shared.config.constants.robot import RobotSpecs
from shared.config.constants.track import CorridorDimensions, TrafficSignSpecs
from shared.domain.enums import Direction

from scripts.bag.diag_bag_exec_failures import reachable_lateral_m, turn_radius_at
from scripts.bag.diag_bag_pass_geometry import classify_lattice
from scripts.bag.diag_bag_pass_pairs import CCW_NEXT, CW_NEXT
from scripts.bag.diag_bag_pass_side import _load, _passes
from scripts.common.bag_io import create_bags_parser, settled_direction
from scripts.common.tables import print_table
from src.config.tuning_helpers import get_tuning

GRAZE_M = 0.030
"""Clearance under this is a graze: settled 2026-09-12, all 8 sub-30 mm grazes
were tracking failures rather than plan failures."""

PILLAR_HALF = TrafficSignSpecs.WIDTH / 2
CHASSIS_HALF = RobotSpecs.WIDTH / 2
DEPTHS = (
    TrafficSignSpecs.GRID_DEPTH_NEAR,
    TrafficSignSpecs.GRID_DEPTH_MIDDLE,
    TrafficSignSpecs.GRID_DEPTH_FAR,
)
CORNER_ARC_M = math.pi / 2 * (CorridorDimensions.OBSTACLES_WIDTH / 2)
"""Quarter turn on the corridor CENTRELINE, which sits half a corridor from
each wall. 0.785 m at 1.00 m width."""

# Depth axis sense per section, travelling COUNTERCLOCKWISE. South is driven
# +x and its depth IS x, east is driven +y and its depth is y; north and west
# are driven against their depth axis. Clockwise is the exact negation.
CCW_DEPTH_RISES = {"south": True, "east": True, "north": False, "west": False}


def depth_of(section: str, x: float, y: float) -> float:
    """The along-corridor coordinate, in ``classify_lattice``'s own convention."""
    return x if section in ("south", "north") else y


def slot_of(section: str, x: float, y: float, direction: str) -> tuple[float, str]:
    """Snap the pillar to a grid row and name it relative to TRAVEL.

    ENTRY = the row just after the corner the car came out of, EXIT = the row
    just before the corner it is about to take, MIDDLE = neither. The operator's
    "at a corner" is EXIT-then-ENTRY across the turn, which only this label can
    separate from a pass in open corridor.
    """
    depth = depth_of(section, x, y)
    snapped = min(DEPTHS, key=lambda d: abs(d - depth))
    rises = CCW_DEPTH_RISES[section]
    if direction != "counterclockwise":
        rises = not rises
    if snapped == TrafficSignSpecs.GRID_DEPTH_MIDDLE:
        return snapped, "middle"
    is_far = snapped == TrafficSignSpecs.GRID_DEPTH_FAR
    return snapped, "exit" if is_far == rises else "entry"


def dist_to_straight_end(snapped: float, at_exit: bool) -> float:
    """How much straight is left between this row and the corner ahead/behind."""
    near, far = TrafficSignSpecs.GRID_DEPTH_NEAR, TrafficSignSpecs.GRID_DEPTH_FAR
    span = far - near
    off = snapped - near  # 0.0 at NEAR, span at FAR
    return (span - off) if at_exit else off


@dataclass
class Rec:
    """One committed pillar, as passed, with everything the cells are keyed on."""

    run: str
    group: str
    laps: int
    direction: str
    order: int
    colour: str
    corridor: str
    section: str | None
    line: str | None
    slot: str | None
    snapped_depth: float | None
    verdict: str
    verdict_shipped: str
    verdict_robot_corridor: str
    verdict_sign_corridor: str
    verdict_lattice: str
    router_label_agrees: bool
    outcome: str
    clearance_m: float
    asked_m: float
    achieved_m: float
    commit_range_m: float
    commit_lateral_m: float
    commit_speed_mps: float | None
    needed_m: float
    available_m: float
    maneuver: bool
    man_types: Counter[str] = field(default_factory=Counter)
    opposes: int = 0
    sign_x: float = 0.0
    sign_y: float = 0.0


def _verdict(p) -> str:
    if p.commanded < 0:
        return "routing"
    if p.achieved < 0:
        return "execution"
    return "ok"


def _outcome(verdict: str, clearance: float) -> str:
    """Outcome ranked by what it costs, worst first.

    WRONG-SIDE outranks everything: per the operator it ENDS the round, it is
    not a deduction. Contact and graze only describe a pass that was legal.
    """
    if verdict == "routing":
        return "wrong-side(routed)"
    if verdict == "execution":
        return "wrong-side(tracked)"
    if clearance <= 0.0:
        return "contact"
    if clearance < GRAZE_M:
        return "graze"
    return "clean"


def _dual_verdicts(rows, frames, scans, tuning) -> list[tuple[str, str, str, bool]]:
    """Re-judge every pass under BOTH corridors: the ROBOT's and the SIGN's own.

    This exists because ``diag_bag_pass_side._passes`` evaluates the rule with
    ``d.current_corridor`` -- the ROBOT's corridor -- while the router keys its
    deformation off ``self._sign_corridors[index]``, the SIGN's. That script's
    own docstring names the substitution as the trap that "moves the answer", so
    which corridor the shipped verdict actually uses has to be MEASURED: a
    pillar at the ENTRY row is by construction the first object of a corridor
    the robot has not entered yet, so a verdict keyed on the robot's corridor
    would manufacture routing errors exactly there and nowhere else.

    The loop is a deliberate fork of ``_passes``: same replay, same detections,
    same commitment bookkeeping, only the rule's corridor differs between the
    two verdicts returned. The ROBOT column must reproduce ``_passes``'s own
    tally, and that agreement is printed -- if it does not, the fork is wrong
    and nothing built on it means anything.

    A THIRD verdict is needed because the first two are both interested parties.
    Judging with the SIGN's corridor is very nearly tautological: the router
    deforms along that very axis, so its own waypoint lands on the legal side of
    it by construction and a routing error becomes almost undetectable. Judging
    with the ROBOT's corridor tests something real but different -- whether the
    lane is legal where the CAR is -- and at a corner that is not the pillar's
    corridor. So the arbiter is the pillar's own LATTICE section, snapped from
    its believed position by ``classify_lattice``, which is geometry and owes
    nothing to the router's labelling. Returned last, with a flag saying whether
    the router's label AGREED with it.
    """
    import math as _math

    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from shared.domain.enums import Axis, Section
    from shared.domain.models import Pose, SignColor

    from scripts.bag.diag_localizer_guard_replay import _scan_to_ranges_angles
    from scripts.common.bag_io import decode_detections
    from scripts.common.stats import nearest_by_time
    from src.navigation.planning.sign_discovery import detection_to_observation
    from src.navigation.planning.sign_router import SignRouter
    from src.navigation.planning.sign_router.routing import pass_side_lateral_axis

    direction = settled_direction(rows) or Direction.COUNTERCLOCKWISE
    router = SignRouter(signs=[], direction=direction, discover=True, tuning=tuning)
    frame_i = 0
    scan_times = [t for t, _ in scans]
    best: dict[tuple[float, float], tuple] = {}
    order: list[tuple[float, float]] = []

    for rel, d in rows:
        if d.pose_x is None or d.pose_y is None or d.pose_yaw is None:
            continue
        pose = Pose(x=d.pose_x, y=d.pose_y, yaw=d.pose_yaw)
        ranges = angles = None
        if scan_times:
            ranges, angles = _scan_to_ranges_angles(
                deserialize_message(nearest_by_time(scans, scan_times, rel), LaserScan)
            )
        obs = []
        while frame_i < len(frames) and frames[frame_i][0] <= rel:
            obs.extend(
                o
                for det in decode_detections(frames[frame_i][1])
                if (o := detection_to_observation(det, pose, tuning, ranges, angles)) is not None
            )
            frame_i += 1
        if d.steer_target_x is None:
            continue
        deformed = router.deform_waypoint(
            (d.steer_target_x, d.steer_target_y),
            (d.pose_x, d.pose_y),
            d.pose_yaw,
            d.current_corridor,
            obs,
        )
        committed = router.committed_sign_position
        if committed is None:
            continue
        rng = _math.hypot(committed.x - d.pose_x, committed.y - d.pose_y)
        key = (round(committed.x, 1), round(committed.y, 1))
        if key not in best:
            order.append(key)
        elif best[key][0] <= rng:
            continue
        idx = next(
            (
                i
                for i, sgn in enumerate(router.signs)
                if abs(sgn.x - committed.x) < 1e-9 and abs(sgn.y - committed.y) < 1e-9
            ),
            None,
        )
        colour = SignColor.UNKNOWN if idx is None else router.signs[idx].color
        sign_corr = d.current_corridor if idx is None else router._sign_corridors[idx]
        cell = classify_lattice(committed.x, committed.y)
        lat_corr = sign_corr if cell is None else Section(cell[0])
        best[key] = (
            rng,
            pass_side_lateral_axis(d.current_corridor, colour, direction),
            pass_side_lateral_axis(sign_corr, colour, direction),
            pass_side_lateral_axis(lat_corr, colour, direction),
            committed,
            (d.pose_x, d.pose_y),
            deformed,
            Axis,
            sign_corr == lat_corr,
        )

    def judge(rule, sign_pos, robot, deformed, axis_enum) -> str:
        if rule is None:
            return "n/a"
        axis, want = rule
        i = 0 if axis is axis_enum.X else 1
        sign_axis = sign_pos.x if i == 0 else sign_pos.y
        if (1 if deformed[i] - sign_axis > 0 else -1) * want < 0:
            return "routing"
        if (1 if robot[i] - sign_axis > 0 else -1) * want < 0:
            return "execution"
        return "ok"

    out: list[tuple[str, str, str, bool]] = []
    for key in order:
        _rng, rrule, srule, lrule, sign_pos, robot, deformed, axis_enum, agreed = best[key]
        out.append(
            (
                judge(rrule, sign_pos, robot, deformed, axis_enum),
                judge(srule, sign_pos, robot, deformed, axis_enum),
                judge(lrule, sign_pos, robot, deformed, axis_enum),
                agreed,
            )
        )
    return out


def _one_bag(arg: tuple[str, str]) -> tuple[str, list[Rec], str | None]:
    bag, group = arg
    try:
        rows, frames, scans = _load(Path(bag))
    except (RuntimeError, OSError, ValueError) as exc:
        return Path(bag).name, [], type(exc).__name__
    tuning = get_tuning(None)
    run = Path(bag).name.replace("run_", "")
    direction = str(settled_direction(rows) or Direction.COUNTERCLOCKWISE)
    laps = max((d.laps_completed for _, d in rows), default=0)
    passes, _peak = _passes(run, rows, frames, scans, tuning)
    duals = _dual_verdicts(rows, frames, scans, tuning)
    out: list[Rec] = []
    for i, p in enumerate(passes):
        cell = classify_lattice(p.sign_x, p.sign_y)
        section = None if cell is None else cell[0]
        line = None if cell is None else ("inner" if cell[1] else "outer")
        snapped = slot = None
        if section is not None:
            snapped, slot = slot_of(section, p.sign_x, p.sign_y, direction)
        shipped = _verdict(p)
        # The outcome is scored on the ARBITER verdict, not the shipped one.
        # ``_dual_verdicts`` shows the shipped verdict is keyed on the ROBOT's
        # corridor, which at a corner is not the pillar's, and that substitution
        # manufactures routing errors at the ENTRY row and nowhere else. The
        # shipped verdict is kept alongside so the swap is auditable.
        verdict = duals[i][2] if i < len(duals) and duals[i][2] != "n/a" else shipped
        clearance = p.lateral_m - PILLAR_HALF - CHASSIS_HALF
        out.append(
            Rec(
                run=run,
                group=group,
                laps=laps,
                direction=direction,
                order=i,
                colour=str(p.colour),
                corridor=p.corridor.split(".")[-1].lower(),
                section=section,
                line=line,
                slot=slot,
                snapped_depth=snapped,
                verdict=verdict,
                verdict_shipped=shipped,
                verdict_robot_corridor=duals[i][0] if i < len(duals) else "n/a",
                verdict_sign_corridor=duals[i][1] if i < len(duals) else "n/a",
                verdict_lattice=duals[i][2] if i < len(duals) else "n/a",
                router_label_agrees=duals[i][3] if i < len(duals) else True,
                outcome=_outcome(verdict, clearance),
                clearance_m=clearance,
                asked_m=p.commanded_m,
                achieved_m=p.lateral_m,
                commit_range_m=p.commit_range_m,
                commit_lateral_m=p.commit_lateral_m,
                commit_speed_mps=p.commit_speed_mps,
                needed_m=max(0.0, -p.commit_lateral_m),
                available_m=reachable_lateral_m(p.commit_range_m, turn_radius_at(p.commit_speed_mps)),
                maneuver=p.maneuver_during_pass,
                man_types=p.maneuver_ticks,
                opposes=p.manoeuvre_opposes,
                sign_x=p.sign_x,
                sign_y=p.sign_y,
            )
        )
    return Path(bag).name, out, None


HEAD = ["population", "n", "wrong-ROUTED", "wrong-TRACKED", "contact", "graze", "clean", "FAIL%"]


def _row(label: str, members: list[Rec]) -> list:
    n = len(members)
    if n == 0:
        return [label, 0, "--", "--", "--", "--", "--", "--"]
    c = Counter(m.outcome for m in members)
    fail = c["wrong-side(routed)"] + c["wrong-side(tracked)"]
    return [
        label,
        n,
        c["wrong-side(routed)"],
        c["wrong-side(tracked)"],
        c["contact"],
        c["graze"],
        c["clean"],
        f"{100 * fail / n:5.1f}%",
    ]


def _p50(vals: list) -> float | None:
    v = sorted(x for x in vals if x is not None)
    return None if not v else v[len(v) // 2]


def _fmt(v: float | None, nd: int = 3) -> str:
    return "--" if v is None else f"{v:.{nd}f}"


def _fisher(label: str, a: list[Rec], b: list[Rec]) -> None:
    """Fisher exact on wrong-side/not for two populations, reported whatever it says."""
    try:
        from scipy.stats import fisher_exact
    except ImportError:
        print(f"   {label}: scipy unavailable")
        return
    if not a or not b:
        print(f"   {label}: empty cell, no test")
        return
    af = sum(1 for m in a if m.outcome.startswith("wrong-side"))
    bf = sum(1 for m in b if m.outcome.startswith("wrong-side"))
    odds, p = fisher_exact([[af, len(a) - af], [bf, len(b) - bf]])
    print(f"   FISHER {label}: {af}/{len(a)} vs {bf}/{len(b)} wrong-side, odds={odds:.2f}  p={p:.4f}")


def _chain(recs: list[Rec], want_same: bool) -> list[tuple[Rec, Rec]]:
    """Consecutive placeable passes, either sharing a section or straddling one corner."""
    by_run: dict[str, list[Rec]] = {}
    for r in recs:
        by_run.setdefault(r.run, []).append(r)
    out: list[tuple[Rec, Rec]] = []
    for v in by_run.values():
        placeable = [r for r in sorted(v, key=lambda r: r.order) if r.section is not None]
        for a, b in zip(placeable, placeable[1:]):
            nxt = CCW_NEXT if a.direction == "counterclockwise" else CW_NEXT
            if want_same:
                if a.section == b.section:
                    out.append((a, b))
            elif a.section != b.section and nxt[a.section] == b.section:
                out.append((a, b))
    return out


def _lane_width_from_outer(line: str) -> float:
    """Where the DIVISION LINE sits, measured from the outer wall."""
    return CorridorDimensions.DIVISION_OUTER if line == "outer" else CorridorDimensions.DIVISION_INNER


def pair_geometry(a: Rec, b: Rec) -> tuple[float, float, float]:
    """Lane translation demanded across the corner, distance offered, radius.

    Both lanes are expressed as a distance from the OUTER wall of their own
    section, which is what makes them comparable across a 90 degree turn: the
    corridor is the same width on both sides of it. The magnitude of the offset
    from the division line is the one the router actually ASKED for; its sign is
    taken away from the nearer wall, which bounds the translation from BELOW
    (the true lane can only be further from the division line, never nearer).
    """
    half = CorridorDimensions.OBSTACLES_WIDTH / 2
    wa = _lane_width_from_outer(a.line or "inner")
    wb = _lane_width_from_outer(b.line or "inner")
    lane_a = wa + (a.asked_m if wa <= half else -a.asked_m)
    lane_b = wb + (b.asked_m if wb <= half else -b.asked_m)
    d_lat = abs(lane_b - lane_a)
    along = (
        dist_to_straight_end(a.snapped_depth or TrafficSignSpecs.GRID_DEPTH_MIDDLE, at_exit=True)
        + CORNER_ARC_M
        + dist_to_straight_end(b.snapped_depth or TrafficSignSpecs.GRID_DEPTH_MIDDLE, at_exit=False)
    )
    radius = float("inf") if d_lat < 1e-6 else along * along / (4.0 * d_lat)
    return d_lat, along, radius


def main() -> None:
    parser = create_bags_parser(__doc__)
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument(
        "--control",
        action="append",
        default=[],
        metavar="BAG",
        help="baseline bag (repeatable); reported as a separate group",
    )
    parser.add_argument("--per-pass", action="store_true", help="dump the full per-pass table")
    args = parser.parse_args()

    jobs = [(str(b), "STUDY") for b in args.bag_dirs] + [(str(b), "CONTROL") for b in args.control]
    recs: list[Rec] = []
    skipped: list[str] = []
    runs: Counter[str] = Counter()
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        futs = {pool.submit(_one_bag, j): j for j in jobs}
        for fut in as_completed(futs):
            name, got, err = fut.result()
            if err is not None:
                skipped.append(f"{name}({err})")
                continue
            runs[futs[fut][1]] += 1
            recs.extend(got)

    print(f"== {len(recs)} passes; runs: {dict(runs)}; skipped {len(skipped)}")
    if skipped:
        print(f"   skipped: {', '.join(skipped[:8])}")
    if not recs:
        print("No sign passes reconstructed -- nothing to say.")
        return
    unplaceable = sum(1 for r in recs if r.section is None)
    print(f"   unplaceable on the lattice: {unplaceable} ({100 * unplaceable / len(recs):.1f}%)")
    print()

    study = [r for r in recs if r.group == "STUDY"]
    control = [r for r in recs if r.group == "CONTROL"]

    print("== CONTROL 0: pooled outcome, study vs control")
    print("   Outcomes below are scored on the ARBITER verdict (the pillar's own lattice")
    print("   section). The SHIPPED verdict is printed underneath so the difference is")
    print("   auditable -- see the corridor table further down for why they differ.")
    print_table([_row("STUDY (competition)", study), _row("CONTROL (practice 3/3)", control)], HEAD)
    for grp, pool in (("STUDY", study), ("CONTROL", control)):
        r = sum(1 for m in pool if m.verdict_shipped == "routing")
        e = sum(1 for m in pool if m.verdict_shipped == "execution")
        print(
            f"   SHIPPED diag_bag_pass_side verdict on {grp}: routing {r}, execution {e}, "
            f"ok {len(pool) - r - e}  (n={len(pool)})"
        )
    print()

    if args.per_pass:
        print("== PER-PASS TABLE")
        ph = [
            "run", "grp", "dir", "#", "colour", "section", "line", "slot",
            "routed", "tracked", "asked m", "achieved m", "clear m", "commit m",
            "needed m", "avail m", "manoeuvre", "outcome",
        ]
        prows = []
        for r in sorted(recs, key=lambda r: (r.group, r.run, r.order)):
            prows.append([
                r.run, r.group[:3], r.direction[:3].upper(), r.order, r.colour,
                r.section or "--", r.line or "--", r.slot or "--",
                "WRONG" if r.verdict == "routing" else "ok",
                "WRONG" if r.verdict == "execution" else "ok",
                f"{r.asked_m:.3f}", f"{r.achieved_m:.3f}", f"{r.clearance_m:+.3f}",
                f"{r.commit_range_m:.2f}", f"{r.needed_m:.2f}", f"{r.available_m:.2f}",
                ",".join(sorted(r.man_types)) or "-", r.outcome,
            ])
        print_table(prows, ph)
        print()

    # ---- 1. Per-pass cells: does the SLOT (corner-adjacent vs mid-straight) matter?
    print("== PASS-LEVEL: slot x line x direction  (is a corner-adjacent pillar worse?)")
    g: dict[tuple, list[Rec]] = {}
    for r in study:
        if r.slot is None:
            continue
        g.setdefault((r.direction[:3].upper(), r.slot, r.line), []).append(r)
    print_table([_row(f"{d} {s:6s} {ln}", v) for (d, s, ln), v in sorted(g.items(), key=lambda kv: -len(kv[1]))], HEAD)
    print()
    print("== PASS-LEVEL: slot pooled over line and direction, BOTH GROUPS")
    srows = []
    slot_pop: dict[tuple[str, str], list[Rec]] = {}
    for r in recs:
        if r.slot is not None:
            slot_pop.setdefault((r.group, r.slot), []).append(r)
    for grp in ("STUDY", "CONTROL"):
        for slot in ("entry", "middle", "exit"):
            srows.append(_row(f"{grp:8s} {slot}", slot_pop.get((grp, slot), [])))
    print_table(srows, HEAD)
    _fisher(
        "STUDY entry vs exit",
        slot_pop.get(("STUDY", "entry"), []),
        slot_pop.get(("STUDY", "exit"), []),
    )
    _fisher(
        "CONTROL entry vs exit",
        slot_pop.get(("CONTROL", "entry"), []),
        slot_pop.get(("CONTROL", "exit"), []),
    )
    print()

    print("== WHY the slot separates: perception and commitment budget by slot")
    wrows = []
    for grp in ("STUDY", "CONTROL"):
        for slot in ("entry", "middle", "exit"):
            sel = slot_pop.get((grp, slot), [])
            if not sel:
                wrows.append([f"{grp} {slot}", 0, "--", "--", "--", "--", "--", "--"])
                continue
            wrows.append([
                f"{grp} {slot}", len(sel),
                _fmt(_p50([r.commit_range_m for r in sel]), 2),
                f"{100 * sum(1 for r in sel if r.commit_lateral_m < 0) / len(sel):5.1f}%",
                _fmt(_p50([r.needed_m for r in sel]), 2),
                _fmt(_p50([r.available_m for r in sel]), 2),
                f"{100 * sum(1 for r in sel if r.needed_m > r.available_m) / len(sel):5.1f}%",
                f"{100 * sum(1 for r in sel if r.maneuver) / len(sel):5.1f}%",
            ])
    print_table(
        wrows,
        ["population", "n", "commit range p50", "wrong AT COMMIT", "needed p50", "avail p50", "NO ROOM", "manoeuvre"],
    )
    print()

    # ---- 2. The pair matrix.
    pairs = _chain(study, want_same=False)
    same = _chain(study, want_same=True)
    followers = {id(b) for _, b in pairs}
    same_followers = {id(b) for _, b in same}
    placeable = [r for r in study if r.section is not None]
    clean_ctl = [r for r in placeable if id(r) not in followers and id(r) not in same_followers]

    print("== ARTEFACT CONTROL: the SAME cells keyed on RAW coordinates, direction ignored")
    print("   ENTRY/EXIT is a TRAVEL label: one physical grid row is ENTRY clockwise and EXIT")
    print("   counterclockwise. If the effect were a coordinate or map-error artefact it would")
    print("   stay put when the direction is dropped; if it is about travel it must DISSOLVE.")
    rawd: dict[str, list[Rec]] = {}
    for r in study:
        if r.snapped_depth is not None:
            rawd.setdefault(f"raw depth {r.snapped_depth:.1f}, direction ignored", []).append(r)
    print_table([_row(k, v) for k, v in sorted(rawd.items())], HEAD)
    _fisher(
        "raw NEAR(1.0) vs raw FAR(2.0), direction ignored",
        [r for r in study if r.snapped_depth == TrafficSignSpecs.GRID_DEPTH_NEAR],
        [r for r in study if r.snapped_depth == TrafficSignSpecs.GRID_DEPTH_FAR],
    )
    for d in ("clockwise", "counterclockwise"):
        _fisher(
            f"{d[:3].upper()} entry vs exit (travel label, within one direction)",
            [r for r in study if r.direction == d and r.slot == "entry"],
            [r for r in study if r.direction == d and r.slot == "exit"],
        )
    print()

    print("== WHICH CORRIDOR DOES THE SHIPPED VERDICT USE? (the trap diag_bag_pass_side names)")
    print("   The router deforms with the SIGN's corridor. If the verdict is keyed on the")
    print("   ROBOT's, every pillar the robot has not yet drawn level with is judged on the")
    print("   wrong axis -- and that is precisely the ENTRY row.")
    agree_robot = sum(1 for r in study if r.verdict == r.verdict_robot_corridor)
    agree_sign = sum(1 for r in study if r.verdict == r.verdict_sign_corridor)
    print(f"   shipped verdict reproduced by the ROBOT-corridor fork: {agree_robot}/{len(study)}")
    print(f"   shipped verdict reproduced by the SIGN-corridor fork:  {agree_sign}/{len(study)}")
    vrows = []
    for grp, pool in (("STUDY", study), ("CONTROL", control)):
        for slot in ("entry", "middle", "exit"):
            sel = [r for r in pool if r.slot == slot]
            if not sel:
                vrows.append([f"{grp} {slot}", 0, "--", "--", "--", "--"])
                continue
            vrows.append([
                f"{grp} {slot}",
                len(sel),
                sum(1 for r in sel if r.verdict_robot_corridor == "routing"),
                sum(1 for r in sel if r.verdict_sign_corridor == "routing"),
                sum(1 for r in sel if r.verdict_robot_corridor == "execution"),
                sum(1 for r in sel if r.verdict_sign_corridor == "execution"),
            ])
    print_table(
        vrows,
        [
            "population",
            "n",
            "routing (ROBOT corr)",
            "routing (SIGN corr)",
            "exec (ROBOT corr)",
            "exec (SIGN corr)",
        ],
    )
    print()

    print("== THE ARBITER: the pillar's own LATTICE section, which owes nothing to the router")
    lrows = []
    for grp, pool in (("STUDY", study), ("CONTROL", control)):
        for slot in ("entry", "middle", "exit"):
            sel = [r for r in pool if r.slot == slot]
            if not sel:
                lrows.append([f"{grp} {slot}", 0, "--", "--", "--", "--"])
                continue
            rout = sum(1 for r in sel if r.verdict_lattice == "routing")
            exe = sum(1 for r in sel if r.verdict_lattice == "execution")
            lrows.append([
                f"{grp} {slot}",
                len(sel),
                f"{100 * sum(1 for r in sel if not r.router_label_agrees) / len(sel):5.1f}%",
                rout,
                exe,
                f"{100 * (rout + exe) / len(sel):5.1f}%",
            ])
    print_table(
        lrows,
        ["population", "n", "router MISLABELLED the pillar", "routing", "execution", "FAIL%"],
    )

    def _lfail(members: list[Rec]) -> list[Rec]:
        return [r for r in members if r.verdict_lattice in ("routing", "execution")]

    for grp, pool in (("STUDY", study), ("CONTROL", control)):
        ent = [r for r in pool if r.slot == "entry"]
        ext = [r for r in pool if r.slot == "exit"]
        try:
            from scipy.stats import fisher_exact

            af, bf = len(_lfail(ent)), len(_lfail(ext))
            if ent and ext:
                odds, pv = fisher_exact([[af, len(ent) - af], [bf, len(ext) - bf]])
                print(
                    f"   FISHER {grp} entry vs exit ON THE LATTICE VERDICT: "
                    f"{af}/{len(ent)} vs {bf}/{len(ext)}, odds={odds:.2f}  p={pv:.4f}"
                )
        except ImportError:
            print("   scipy unavailable")
    mis = [r for r in study if not r.router_label_agrees]
    ok = [r for r in study if r.router_label_agrees]
    try:
        from scipy.stats import fisher_exact

        af, bf = len(_lfail(mis)), len(_lfail(ok))
        if mis and ok:
            odds, pv = fisher_exact([[af, len(mis) - af], [bf, len(ok) - bf]])
            print(
                f"   FISHER STUDY router-MISLABELLED vs correctly labelled: "
                f"{af}/{len(mis)} vs {bf}/{len(ok)}, odds={odds:.2f}  p={pv:.4f}"
            )
    except ImportError:
        pass
    print()

    print("== CORRIDOR ASSIGNMENT: is the ROBOT's believed corridor the PILLAR's own section?")
    print("   The pass-side rule is evaluated per corridor, and the two legitimately differ")
    print("   at a corner. A pillar at the ENTRY row is the first object of a NEW corridor,")
    print("   so this is where a stale corridor can mirror the rule and command the wrong side.")
    crows = []
    for grp, pool in (("STUDY", study), ("CONTROL", control)):
        for slot in ("entry", "middle", "exit"):
            sel = [r for r in pool if r.slot == slot]
            if not sel:
                crows.append([f"{grp} {slot}", 0, "--", "--", "--"])
                continue
            mismatch = [r for r in sel if r.corridor != r.section]
            routed = [r for r in sel if r.outcome == "wrong-side(routed)"]
            routed_mismatch = [r for r in routed if r.corridor != r.section]
            crows.append([
                f"{grp} {slot}", len(sel),
                f"{100 * len(mismatch) / len(sel):5.1f}%",
                f"{len(routed)}",
                "--" if not routed else f"{100 * len(routed_mismatch) / len(routed):5.1f}%",
            ])
    print_table(
        crows,
        ["population", "n", "robot corridor != pillar section", "routing errors", "of those, corridor mismatched"],
    )
    _fisher(
        "STUDY corridor MISMATCH vs match",
        [r for r in study if r.slot is not None and r.corridor != r.section],
        [r for r in study if r.slot is not None and r.corridor == r.section],
    )
    print()

    print("== THE MECHANISM: how much lateral the slot ARRIVES owing")
    mrows = []
    for grp, pool in (("STUDY", study), ("CONTROL", control)):
        for slot in ("entry", "middle", "exit"):
            sel = [r for r in pool if r.slot == slot]
            if not sel:
                mrows.append([f"{grp} {slot}", 0, "--", "--", "--", "--"])
                continue
            owing = [r for r in sel if r.commit_lateral_m < 0]
            mrows.append([
                f"{grp} {slot}", len(sel),
                f"{100 * len(owing) / len(sel):5.1f}%",
                _fmt(_p50([r.needed_m for r in owing]), 3),
                _fmt(_p50([r.commit_lateral_m for r in sel]), 3),
                _fmt(_p50([r.commit_range_m for r in sel]), 2),
            ])
    print_table(
        mrows,
        ["population", "n", "arrives on WRONG side", "owed p50 m (of those)", "commit lateral p50 m", "commit range p50"],
    )
    print()

    print(f"== PAIRS: {len(pairs)} straddle one corner, {len(same)} share a section")
    print_table(
        [
            _row("B after a CORNER crossing", [b for _, b in pairs]),
            _row("B after a SAME-SECTION pillar", [b for _, b in same]),
            _row("B with no recent predecessor", clean_ctl),
        ],
        HEAD,
    )
    print()

    print("== PAIR MATRIX: (A colour -> B colour) x direction x (A line -> B line), corner crossings")
    gm: dict[tuple, list[Rec]] = {}
    gpair: dict[tuple, list[tuple[Rec, Rec]]] = {}
    for a, b in pairs:
        key = (a.direction[:3].upper(), a.colour, b.colour, a.line, b.line)
        gm.setdefault(key, []).append(b)
        gpair.setdefault(key, []).append((a, b))
    print_table(
        [
            _row(f"{d} {ca}->{cb} {la}->{lb}", v)
            for (d, ca, cb, la, lb), v in sorted(gm.items(), key=lambda kv: -len(kv[1]))
        ],
        HEAD,
    )
    print()

    print("== PAIR MATRIX collapsed: colour pair x direction x band CHANGE vs HOLD")
    gb: dict[tuple, list[Rec]] = {}
    for a, b in pairs:
        band = "CHANGE" if a.line != b.line else f"HOLD {a.line}"
        gb.setdefault((a.direction[:3].upper(), f"{a.colour}->{b.colour}", band), []).append(b)
    print_table(
        [_row(f"{d} {cp:13s} {band}", v) for (d, cp, band), v in sorted(gb.items(), key=lambda kv: -len(kv[1]))],
        HEAD,
    )
    print()

    print("== RANKED BY COST: cells ordered by number of ROUND-ENDING wrong-side passes")
    cost = []
    for key, v in gm.items():
        d, ca, cb, la, lb = key
        wrong = sum(1 for m in v if m.outcome.startswith("wrong-side"))
        if wrong == 0:
            continue
        geos = [pair_geometry(a, b) for a, b in gpair[key]]
        rads = [r for _, _, r in geos if math.isfinite(r)]
        cost.append([
            f"{d} {ca}->{cb} {la}->{lb}", len(v), wrong, f"{100 * wrong / len(v):5.1f}%",
            _fmt(_p50([dl for dl, _, _ in geos])), _fmt(_p50(rads)),
            _fmt(_p50([m.needed_m for m in v]), 2), _fmt(_p50([m.available_m for m in v]), 2),
            _fmt(_p50([m.commit_range_m for m in v]), 2),
        ])
    cost.sort(key=lambda r: (-r[2], -r[1]))
    print_table(
        cost,
        ["cell", "n", "wrong", "wrong%", "dLat p50 m", "R demanded p50 m", "needed p50", "avail p50", "commit p50 m"],
    )
    print()

    # ---- 3. The four named cells.
    print("== THE OPERATOR'S FOUR NAMED CELLS")
    named = [
        ("green->red CW, one INNER one OUTER", "clockwise", "green", "red", True),
        ("red->green CCW, one INNER one OUTER", "counterclockwise", "red", "green", True),
        ("green->green CCW (any line)", "counterclockwise", "green", "green", False),
        ("red->red CW (any line)", "clockwise", "red", "red", False),
    ]
    rows = []
    for label, d, ca, cb, need_band_change in named:
        sel = [
            b
            for a, b in pairs
            if a.direction == d
            and a.colour == ca
            and b.colour == cb
            and (a.line != b.line if need_band_change else True)
        ]
        rows.append(_row(label, sel))
        rows.append(
            _row(
                "   matched control (same B colour+dir, no predecessor)",
                [r for r in clean_ctl if r.direction == d and r.colour == cb],
            )
        )
        if need_band_change:
            rows.append(
                _row(
                    "   same colours, band HELD or CHANGED",
                    [b for a, b in pairs if a.direction == d and a.colour == ca and b.colour == cb],
                )
            )
    print_table(rows, HEAD)
    print()

    # ---- 4. Where is it decided: detect / commit / track.
    print("== FAILURE LOCUS: detect (range), commit (side at commit), track (had room and missed)")
    loc_rows = []
    for label, sel in (
        ("STUDY clean", [r for r in study if r.outcome == "clean"]),
        ("STUDY graze/contact", [r for r in study if r.outcome in ("graze", "contact")]),
        ("STUDY wrong-side ROUTED", [r for r in study if r.outcome == "wrong-side(routed)"]),
        ("STUDY wrong-side TRACKED", [r for r in study if r.outcome == "wrong-side(tracked)"]),
        ("CONTROL clean", [r for r in control if r.outcome == "clean"]),
        ("CONTROL wrong-side (both)", [r for r in control if r.outcome.startswith("wrong-side")]),
    ):
        if not sel:
            loc_rows.append([label, 0, "--", "--", "--", "--", "--", "--"])
            continue
        wrong_at_commit = sum(1 for r in sel if r.commit_lateral_m < 0)
        no_room = sum(1 for r in sel if r.needed_m > r.available_m)
        man = sum(1 for r in sel if r.maneuver)
        loc_rows.append([
            label, len(sel),
            _fmt(_p50([r.commit_range_m for r in sel]), 2),
            f"{100 * wrong_at_commit / len(sel):5.1f}%",
            f"{100 * no_room / len(sel):5.1f}%",
            f"{100 * man / len(sel):5.1f}%",
            _fmt(_p50([r.asked_m for r in sel])),
            _fmt(_p50([r.achieved_m for r in sel])),
        ])
    print_table(
        loc_rows,
        ["population", "n", "commit range p50", "wrong AT COMMIT", "NO ROOM", "manoeuvre", "asked p50", "achieved p50"],
    )
    print()

    print("== TRACKING SHORTFALL: asked vs achieved, by outcome (plan quality vs chassis quality)")
    tr = []
    for label, sel in (
        ("clean", [r for r in study if r.outcome == "clean"]),
        ("graze", [r for r in study if r.outcome == "graze"]),
        ("contact", [r for r in study if r.outcome == "contact"]),
        ("wrong-side TRACKED", [r for r in study if r.outcome == "wrong-side(tracked)"]),
        ("CONTROL clean", [r for r in control if r.outcome == "clean"]),
    ):
        if not sel:
            tr.append([label, 0, "--", "--", "--"])
            continue
        tr.append([
            label, len(sel),
            _fmt(_p50([r.asked_m for r in sel])),
            _fmt(_p50([r.achieved_m for r in sel])),
            _fmt(_p50([r.achieved_m - r.asked_m for r in sel])),
        ])
    print_table(tr, ["outcome", "n", "asked p50 m", "achieved p50 m", "delta p50 m"])
    print()

    # ---- 5. The geometric discriminator.
    print("== GEOMETRIC DISCRIMINATOR: what the corner crossing demands vs what the chassis has")
    gd: dict[str, list[tuple[Rec, Rec]]] = {}
    for a, b in pairs:
        band = "band CHANGE" if a.line != b.line else f"band HOLD {a.line}->{b.line}"
        gd.setdefault(band, []).append((a, b))
    grows = []
    for band, members in sorted(gd.items(), key=lambda kv: -len(kv[1])):
        geos = [pair_geometry(a, b) for a, b in members]
        rads = [r for _, _, r in geos if math.isfinite(r)]
        bs = [b for _, b in members]
        fails = sum(1 for b in bs if b.outcome.startswith("wrong-side"))
        grows.append([
            band, len(members),
            _fmt(_p50([dl for dl, _, _ in geos])),
            _fmt(_p50([ln for _, ln, _ in geos]), 2),
            _fmt(_p50(rads)),
            _fmt(_p50([turn_radius_at(b.commit_speed_mps) for b in bs])),
            f"{100 * fails / len(bs):5.1f}%",
        ])
    print_table(
        grows,
        ["pair class", "n", "lane shift p50 m", "along-track p50 m", "R demanded p50 m", "R available p50 m", "FAIL%"],
    )
    print("   R demanded is an S-curve bound, L^2/(4*dLat); R available is 0.053+1.86|v|.")
    print("   A demand BELOW the available radius is comfortable; ABOVE it is impossible.")
    print()

    print("== SLOT PAIRING: does the pair really sit AT the corner (A exit -> B entry)?")
    gp: dict[str, list[Rec]] = {}
    for a, b in pairs:
        gp.setdefault(f"A {a.slot} -> B {b.slot}", []).append(b)
    print_table([_row(k, v) for k, v in sorted(gp.items(), key=lambda kv: -len(kv[1]))], HEAD)
    print()

    print("== PER-RUN ROLL-UP: does a run's wrong-side count track its lap count?")
    by_run: dict[str, list[Rec]] = {}
    for r in recs:
        by_run.setdefault(r.run, []).append(r)
    rrows = []
    for run in sorted(by_run):
        v = by_run[run]
        wrong = sum(1 for m in v if m.outcome.startswith("wrong-side"))
        entry_wrong = sum(1 for m in v if m.slot == "entry" and m.outcome.startswith("wrong-side"))
        rrows.append([
            run, v[0].group[:3], v[0].direction[:3].upper(), v[0].laps, len(v), wrong,
            entry_wrong, sum(1 for m in v if m.outcome == "contact"),
        ])
    print_table(rrows, ["run", "grp", "dir", "laps", "passes", "wrong-side", "of which ENTRY", "contact"])
    for lo, hi, label in ((0, 0, "runs with 0 laps"), (1, 2, "runs with 1-2 laps"), (3, 9, "runs with 3 laps")):
        sel = [m for v in by_run.values() for m in v if lo <= v[0].laps <= hi]
        if sel:
            n_runs = len({m.run for m in sel})
            wrong = sum(1 for m in sel if m.outcome.startswith("wrong-side"))
            print(f"   {label}: {n_runs} runs, {len(sel)} passes, {wrong} wrong-side ({100 * wrong / len(sel):.1f}%),"
                  f" {len(sel) / n_runs:.1f} passes/run")
    print()

    print("== POSITIVE CONTROL (known-present effect): had to cross vs already legal at commit")
    cross = [r for r in study if r.verdict != "routing" and r.commit_lateral_m < 0]
    hold = [r for r in study if r.verdict != "routing" and r.commit_lateral_m >= 0]
    print_table([_row("HAD TO CROSS at commit", cross), _row("already on the legal side", hold)], HEAD)
    print()
    print("CAVEAT: line and slot come from the BELIEVED position snapped to the lattice;")
    print("the believed error exceeds the 0.20 m gap between the two division lines, so")
    print("the band label is noisy and that noise can only SHRINK a true difference.")


if __name__ == "__main__":
    main()
