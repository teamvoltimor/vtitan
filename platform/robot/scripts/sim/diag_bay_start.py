"""Probe the IN-BAY start — the legal start no corpus scenario exercises.

The rules allow two starts: inside the parking lot, or parallel to it in the
same section. **Every one of the 256 corpus scenarios uses the second.** Verified
here as well as in the corpus: the along-corridor offset between start and bay is
0.0 in every scenario, the start sits on the corridor centreline and the bay
against the outer wall, so the two differ only in the across-corridor coordinate.

That makes the in-bay start invisible to every sweep in the repo, which is why it
went unnoticed that a robot placed there **never moves** (8/8 probed, dist=0.00m,
stuck): forward reads 0.05-0.19m against a parking fin, below the gate that
authorises the initial creep, and there is no rear sensing to reverse on
(``compute_rear_clearance`` fails open, there is no rear slot).

This script places the robot in the pocket and reports what it does. It ALWAYS
runs the parallel start as a control on the same scenarios -- the in-bay
diagnosis previously shipped a wrong cause precisely because the working case was
not run through the same probe, and ``not_yet_stepped`` at tick 1 turned out to
be snapshot lag rather than a state. Read the two arms side by side.

The bay pose is derived, not hardcoded: the lot's two magenta fins stand
perpendicular to the outer wall at ``parking_lot.block1_position`` and
``block2_position``, so their midpoint is the pocket centre. Heading is taken
from the scenario's own start, which is already parallel to the wall.

Usage::

    PYTHONPATH="." pixi run -e dev python scripts/sim/diag_bay_start.py --limit 8
    PYTHONPATH="." pixi run -e dev python scripts/sim/diag_bay_start.py --corpus --limit 32
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CompetitionSpecs, ParkingLotSpecs, RobotSpecs  # noqa: E402
from shared.domain.models import ScenarioMetadata  # noqa: E402

from scripts.common.diag_base import print_pool_progress, resolve_jobs, run_pool  # noqa: E402
from scripts.common.sim_defaults import CORPUS_DIR  # noqa: E402
from src.config.tuning_helpers import tuning_with_overrides  # noqa: E402
from src.navigation.track_geometry import parking_bay_centre  # noqa: E402
from src.navigation.utils import wrap_angle  # noqa: E402
from src.simulation.scenario_simulator import ScenarioSimulator  # noqa: E402

if TYPE_CHECKING:
    from collections.abc import Sequence

    from src.navigation.ports import LidarScan
    from src.simulation.kinematics import AckermannState

_COMMITTED_DIR = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "scenarios" / "obstacles"
# Seconds of continuous wall contact allowed before the run is judged a real
# failure rather than a chassis still working itself clear. Only meaningful
# with solid walls; the escape being measured IS a sustained scrape.
_CONTACT_GRACE_S = 8.0
# Below this separation the bay and the parallel start coincide on the axis
# perpendicular to the wall, leaving "out of the bay" without a direction.
_DEGENERATE_AXIS_M = 1e-6


@dataclass(frozen=True, slots=True)
class BayStartRow:
    """One scenario's outcome from one start, as produced by ``_run_case``."""

    id: str
    skipped: bool
    moved: bool = False
    fx: float = 0.0
    fy: float = 0.0
    dyaw_deg: float = 0.0
    net_m: float = 0.0
    exit_m: float | None = None
    bex: int = 0
    rev_ticks: int = 0
    fwd_ticks: int = 0
    rev_m: float = 0.0
    flips: int = 0
    dist: float = 0.0
    laps: int = 0
    collided: bool = False
    stuck: bool = False
    timed_out: bool = False
    pass_side: bool = False


def bay_outward_axis(
    centre: tuple[float, float],
    start_xy: tuple[float, float],
    start_yaw: float,
) -> tuple[float, float] | None:
    """Unit vector from the pocket towards the corridor, perpendicular to the wall.

    Derived, not hardcoded: the scenario's own (parallel) start sits on the
    corridor centreline and the bay against the outer wall, so the bay-to-start
    vector points out of the pocket. Its along-corridor part is 0.0 in every
    corpus scenario, but it is projected out anyway rather than trusted -- the
    axis this measurement is taken on must be the one perpendicular to the wall
    even if a future generator offsets the start along the corridor.

    Returns ``None`` when the two coincide on that axis, which would leave the
    direction undefined; the caller then reports no exit verdict rather than a
    guessed one.
    """
    vx, vy = start_xy[0] - centre[0], start_xy[1] - centre[1]
    along = vx * math.cos(start_yaw) + vy * math.sin(start_yaw)
    px, py = vx - along * math.cos(start_yaw), vy - along * math.sin(start_yaw)
    norm = math.hypot(px, py)
    if norm < _DEGENERATE_AXIS_M:
        return None
    return px / norm, py / norm


def bay_exit_clearance(
    pose: tuple[float, float, float],
    centre: tuple[float, float],
    outward: tuple[float, float],
) -> float:
    """Metres by which the whole chassis footprint clears the pocket mouth.

    The two fins stand ``ParkingLotSpecs.LENGTH`` (0.20 m) out from the outer
    wall, and the pocket centre sits ``WALL_OFFSET`` (0.10 m) from it, so the
    mouth is 0.10 m outboard of where the chassis is placed. The verdict is
    taken on the footprint CORNER nearest the wall, at the final heading, not on
    the centre: a chassis that has driven its nose out while its tail is still
    between the fins has not left the bay, and a centre-only test would call it
    out. Negative means still (partly) in the pocket.
    """
    x, y, yaw = pose
    hx, hy = math.cos(yaw), math.sin(yaw)
    half_l, half_w = RobotSpecs.LENGTH / 2.0, RobotSpecs.WIDTH / 2.0
    nearest = min(
        ParkingLotSpecs.WALL_OFFSET
        + (x + sl * half_l * hx + sw * half_w * -hy - centre[0]) * outward[0]
        + (y + sl * half_l * hy + sw * half_w * hx - centre[1]) * outward[1]
        for sl in (-1.0, 1.0)
        for sw in (-1.0, 1.0)
    )
    return nearest - ParkingLotSpecs.LENGTH


def _run_case(payload: tuple[str, bool, int, dict[str, float], bool, bool, bool, float]) -> BayStartRow:
    """Run one scenario from one start. Returns a row, never raises on outcome."""
    path_str, in_bay, laps, changes, known_start, solid_walls, slide, scrub = payload
    path = Path(path_str)
    raw = json.loads(path.read_text())

    start = raw["starting_conditions"]
    centre = parking_bay_centre(raw)
    # Captured before the in-bay branch overwrites it: the parallel start is the
    # reference the outward axis is derived from, and it must be the ORIGINAL
    # one on both arms so the two report the exit on the same axis.
    parallel_xy = (start["position"]["x"], start["position"]["y"])
    moved = False
    if in_bay:
        if centre is None:
            return BayStartRow(id=raw["scenario_id"], skipped=True)
        # Heading is left alone: the scenario's own start is already parallel to
        # the outer wall, which is the only orientation the 0.20m-deep pocket
        # admits for a 0.194m-wide chassis.
        start["position"]["x"], start["position"]["y"] = centre
        moved = True

    meta = ScenarioMetadata.model_validate(raw)
    sim = ScenarioSimulator(
        meta,
        num_laps=laps,
        tuning=tuning_with_overrides(changes),
        seed=raw["scenario_id"],
        blind=True,
        known_start=known_start,
        solid_walls=solid_walls,
        slide_on_contact=slide,
        scrub_yaw_gain=scrub,
    )
    # Wall contact is legal on OBSTACLES and not on Open, so a scraping escape is
    # only a real result on this challenge -- which is also the only one with a
    # parking lot to start in. With solid walls the chassis is stopped by the
    # wall instead of passing through it, and `allowed_step` then limits the TURN
    # while keeping the translation, which is what lets a cornered chassis peel
    # away. Without it the run simply stalls short of contact, which is what
    # every earlier bay-start number here measured.
    # Tracked over the whole run, not read off the final pose: a run that leaves
    # the pocket, drives its laps and then PARKS is back inside the bay at the
    # end, and a final-pose test scores that -- the best possible outcome -- as
    # never having left. Observed on scenario 0 of the committed set: 3 laps,
    # 26 m driven, final clearance -0.00 m.
    outward = bay_outward_axis(centre, parallel_xy, start["yaw"]) if centre else None
    best_exit_m: float | None = None

    def _observe(state: AckermannState, _scan: LidarScan) -> None:
        nonlocal best_exit_m
        if centre is None or outward is None:
            return
        clearance = bay_exit_clearance((state.x, state.y, state.yaw), centre, outward)
        best_exit_m = clearance if best_exit_m is None else max(best_exit_m, clearance)

    result = sim.run(contact_grace_s=_CONTACT_GRACE_S if solid_walls else None, on_step=_observe)

    # Where it STOPPED, not just how far it went. A distance alone cannot tell
    # "never left the pocket" from "drove out, crossed the corridor and stalled
    # facing the far wall" -- and those want opposite fixes. Yaw is reported
    # against the scenario's own start heading, which is parallel to the outer
    # wall, so ~90 deg means the chassis is broadside to the corridor it is
    # meant to be driving down.
    fx, fy, fyaw = result.final_pose
    sx, sy = start["position"]["x"], start["position"]["y"]
    return BayStartRow(
        id=raw["scenario_id"],
        skipped=False,
        moved=moved,
        fx=fx,
        fy=fy,
        dyaw_deg=math.degrees(abs(wrap_angle(fyaw - start["yaw"]))),
        net_m=math.hypot(fx - sx, fy - sy),
        exit_m=best_exit_m,
        bex=sim.bay_exit_ticks,
        rev_ticks=sim.bay_exit.legs[0],
        fwd_ticks=sim.bay_exit.legs[1],
        rev_m=sim.bay_exit.legs[2],
        flips=sim.bay_exit.open_flips,
        dist=result.distance_m,
        laps=result.laps_completed,
        collided=result.collided,
        stuck=result.stuck,
        timed_out=result.timed_out,
        pass_side=result.pass_side_violation,
    )


def _summarise(name: str, rows: Sequence[BayStartRow]) -> None:
    """One line per scenario, then the aggregate that actually decides it."""
    live = [r for r in rows if not r.skipped]
    if not live:
        print(f"  {name}: no scenarios with a parking lot")
        return

    print(f"\n=== {name} ===")
    print(
        f"| {'#':>4} | {'dist m':>7} | {'net m':>6} | {'exit m':>6} | {'bex':>5} | {'rev':>5} | {'fwd':>5} | "
        f"{'rev m':>6} | {'flip':>5} | {'dyaw':>6} | "
        f"{'end x':>6} | {'end y':>6} | {'laps':>4} | {'coll':>4} | {'stuck':>5} | {'t/o':>3} | {'pass':>4} |"
    )
    print(
        f"|{'-' * 6}|{'-' * 9}|{'-' * 8}|{'-' * 8}|{'-' * 7}|{'-' * 7}|{'-' * 7}|{'-' * 8}|"
        f"{'-' * 8}|{'-' * 8}|{'-' * 8}|{'-' * 6}|{'-' * 6}|{'-' * 7}|{'-' * 5}|{'-' * 6}|"
    )
    for r in sorted(live, key=lambda x: int(x.id)):
        exit_cell = f"{r.exit_m:>6.2f}" if r.exit_m is not None else f"{'--':>6}"
        print(
            f"| {r.id:>4} | {r.dist:>7.2f} | {r.net_m:>6.2f} | {exit_cell} | {r.bex:>5} | "
            f"{r.rev_ticks:>5} | {r.fwd_ticks:>5} | {r.rev_m:>6.3f} | {r.flips:>5} | "
            f"{r.dyaw_deg:>6.1f} | {r.fx:>6.2f} | {r.fy:>6.2f} | {r.laps:>4} | "
            f"{'Y' if r.collided else '.':>4} | {'Y' if r.stuck else '.':>5} | "
            f"{'Y' if r.timed_out else '.':>3} | {'Y' if r.pass_side else '.':>4} |"
        )

    n = len(live)
    dists = [r.dist for r in live]
    immobile = sum(1 for d in dists if d < 0.01)
    # The headline for the in-bay arm. Laps and distance both answer "how well
    # did the run go afterwards"; this answers the prior question the pocket
    # actually poses, and a run can clear the bay and then stall or collide
    # without that making the exit itself a failure. The margin is the run's
    # BEST clearance, so it says how far out it got, not where it ended.
    clear = [r for r in live if r.exit_m is not None]
    if clear:
        out = [r for r in clear if r.exit_m > 0.0]  # type: ignore[operator]
        margins = sorted(r.exit_m for r in clear)  # type: ignore[misc]
        print(
            f"  OUT OF BAY {len(out)}/{len(clear)}  "
            f"clearance min {margins[0]:.2f} / median {margins[len(margins) // 2]:.2f} / max {margins[-1]:.2f} m"
        )
    print(
        f"  n={n}  immobile(<1cm)={immobile}  "
        f"dist min {min(dists):.2f} / median {sorted(dists)[n // 2]:.2f} / max {max(dists):.2f}  "
        f"laps>=1 {sum(1 for r in live if r.laps >= 1)}  "
        f"collided {sum(1 for r in live if r.collided)}  "
        f"stuck {sum(1 for r in live if r.stuck)}"
    )


def main() -> None:
    """Run both starts over the same scenarios and print them side by side."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", type=int, default=8, help="Scenario count (0 = all).")
    parser.add_argument("--laps", type=int, default=CompetitionSpecs.OPEN_CHALLENGE_LAPS)
    parser.add_argument("--jobs", type=int, default=0, help="Workers; 0 picks cores minus a couple.")
    parser.add_argument("--corpus", action="store_true", help=f"use {CORPUS_DIR} instead of the committed set")
    parser.add_argument("--scenarios-dir", default=None)
    parser.add_argument(
        "--hold-steer",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_HOLD_STEER (0/1, in-bay arm only). Holds the forward leg's "
        "steering through the reverse instead of centring it, so the servo can actually "
        "reach the commanded angle within the pocket's 7.5 cm of stroke.",
    )
    parser.add_argument(
        "--cycle",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_CYCLE (0/1, in-bay arm only). Alternating steered-forward-arc "
        "and STRAIGHT reverse, repeated until clear. Asymmetric legs accumulate outward "
        "displacement without needing wall contact, which a constant-|steer| shuffle cannot.",
    )
    parser.add_argument(
        "--arc-steer",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_ARC_STEER_NORM (with --cycle). Full lock is a 17 mm turn radius "
        "-- a pivot, not a translation; ~0.5 is 42 deg and ~0.21 m.",
    )
    parser.add_argument(
        "--forward-dist",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_FORWARD_M (with --cycle): how far the forward arc runs before the reverse leg.",
    )
    parser.add_argument(
        "--cycle-rev-steer",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_CYCLE_REVERSE_STEER_NORM (with --cycle): steering on the reverse "
        "leg, applied OPPOSITE to the arc. 0 backs straight (keeps the heading the arc won); "
        "non-zero is the three-point turn, adding rotation on both legs at the cost of a full "
        "servo swing between them.",
    )
    parser.add_argument(
        "--latch-direction",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_LATCH_DIRECTION (0/1, in-bay arm only). Decides the open "
        "side once, on the first tick, rather than re-reading two rays that stop "
        "pointing across the pocket as soon as the chassis rotates.",
    )
    parser.add_argument(
        "--latch-reverse",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_LATCH_REVERSE (0/1, in-bay arm only). Makes the reverse leg "
        "a one-shot, so the forward turn is held instead of the gate flipping back and "
        "forth across its own threshold.",
    )
    parser.add_argument(
        "--max-frames",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_MAX_FRAMES (in-bay arm only). Ticks the bay-exit maneuver may "
        "hold control before handing over to CoreNavigator; 0 = forever (shipped). The "
        "maneuver currently never yields, so the escape ladder has never run from the pocket.",
    )
    parser.add_argument(
        "--solid-walls",
        action="store_true",
        help="make walls physically stop the chassis instead of ending the run on contact. "
        "Touching the outer wall is legal on OBSTACLES (not on Open), and `allowed_step` then "
        "caps the TURN while keeping the translation, so a cornered chassis can scrape and peel "
        "away. Without this the probe stalls short of contact and the wall can never help.",
    )
    parser.add_argument(
        "--slide",
        action="store_true",
        help="let a blocked translation slide ALONG the contacted surface instead of being "
        "scaled to nothing. Without it the chassis advances 0.125 mm per tick at 20 deg of "
        "incidence where a rubbing one gains 7.05 mm. Applies to both arms; every "
        "contact-dependent baseline in the repo was measured WITHOUT it.",
    )
    parser.add_argument(
        "--scrub",
        type=float,
        default=0.0,
        help="chassis yaw per radian of wheel turn while STATIONARY, modelling the servo "
        "scrubbing the tyres in place. The kinematics scale yaw with speed, so a stopped "
        "chassis cannot rotate at all -- yet the servo has 35-70x the torque needed to "
        "scrub a wheel. DELIBERATELY OPTIMISTIC: applied in the helpful direction with no "
        "friction threshold, so it bounds the benefit rather than modelling it.",
    )
    parser.add_argument(
        "--parallel-only",
        action="store_true",
        help="skip the in-bay arm (control alone, to confirm the probe is inert)",
    )
    parser.add_argument(
        "--steer",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_STEER_NORM over these values (in-bay arm only). "
        "Full lock spins about the chassis centre; the pocket needs translation.",
    )
    parser.add_argument(
        "--reverse",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_REVERSE_M over these values (in-bay arm only).",
    )
    parser.add_argument(
        "--rev-steer",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_REVERSE_STEER_NORM (in-bay arm only). 0 backs straight, "
        "which buys room ahead but nothing on the axis the pocket opens on.",
    )
    parser.add_argument(
        "--known-start",
        action="store_true",
        help="also run each in-bay arm with the believed pose seeded from truth. "
        "DIAGNOSTIC ONLY -- separates a failed exit maneuver from a good exit "
        "handing over to a plan built for a centreline the robot is not on.",
    )
    args = parser.parse_args()

    directory = Path(args.scenarios_dir) if args.scenarios_dir else (CORPUS_DIR if args.corpus else _COMMITTED_DIR)
    paths = sorted(directory.glob("*_metadata.json"))
    if not paths:
        parser.error(f"no *_metadata.json under {directory}")
    if args.limit:
        paths = paths[: args.limit]

    jobs = resolve_jobs(args.jobs)
    print(f"{len(paths)} scenarios from {directory}, {jobs} workers, {args.laps} laps")

    # The control always runs at shipped tuning: it exists to show the probe is
    # inert on a normal start, which a swept control could not.
    arms: list[tuple[str, bool, dict[str, float], bool]] = [("parallel (control)", False, {}, False)]
    if not args.parallel_only:
        axes = (
            ("BAY_EXIT_STEER_NORM", "steer", args.steer),
            ("BAY_EXIT_REVERSE_M", "rev-dist", args.reverse),
            ("BAY_EXIT_REVERSE_STEER_NORM", "rev-steer", args.rev_steer),
            ("BAY_EXIT_MAX_FRAMES", "max-frames", args.max_frames),
            ("BAY_EXIT_HOLD_STEER", "hold-steer", args.hold_steer),
            ("BAY_EXIT_LATCH_REVERSE", "latch", args.latch_reverse),
            ("BAY_EXIT_LATCH_DIRECTION", "latch-dir", args.latch_direction),
            ("BAY_EXIT_CYCLE", "cycle", args.cycle),
            ("BAY_EXIT_ARC_STEER_NORM", "arc", args.arc_steer),
            ("BAY_EXIT_FORWARD_M", "fwd", args.forward_dist),
            ("BAY_EXIT_CYCLE_REVERSE_STEER_NORM", "back-steer", args.cycle_rev_steer),
        )
        combos: list[dict[str, float]] = [{}]
        labels: list[str] = [""]
        for field, short, values in axes:
            if not values:
                continue
            combos, labels = (
                [{**c, field: v} for c in combos for v in values],
                [f"{lbl} {short} {v:g}".strip() for lbl in labels for v in values],
            )
        for changes, label in zip(combos, labels, strict=True):
            arms.append((f"IN-BAY{' ' + label if label else ''}", True, changes, False))
            if args.known_start:
                # Diagnostic only -- seeds the believed pose from truth, which
                # the hardware cannot do. Isolates "the exit maneuver failed"
                # from "the exit worked and the plan was 0.4 m off", since blind
                # assumes a centreline start the in-bay robot is not at.
                arms.append((f"IN-BAY{' ' + label if label else ''} +known_start", True, changes, True))

    for name, in_bay, changes, known in arms:
        payloads = [
            (str(p), in_bay, args.laps, changes, known, args.solid_walls, args.slide, args.scrub) for p in paths
        ]
        rows = run_pool(_run_case, payloads, jobs, on_result=print_pool_progress(name))
        _summarise(name, rows)


if __name__ == "__main__":
    main()
