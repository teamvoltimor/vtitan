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
from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CompetitionSpecs  # noqa: E402
from shared.config.navigation_tuning import NavigationTuning  # noqa: E402
from shared.domain.models import ScenarioMetadata  # noqa: E402

from scripts.common.diag_base import print_pool_progress, resolve_jobs, run_pool  # noqa: E402
from scripts.common.sim_defaults import CORPUS_DIR  # noqa: E402
from src.navigation.utils import wrap_angle  # noqa: E402
from src.simulation.scenario_simulator import ScenarioSimulator  # noqa: E402

if TYPE_CHECKING:
    from collections.abc import Sequence

_COMMITTED_DIR = Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "scenarios" / "obstacles"
# Seconds of continuous wall contact allowed before the run is judged a real
# failure rather than a chassis still working itself clear. Only meaningful
# with solid walls; the escape being measured IS a sustained scrape.
_CONTACT_GRACE_S = 8.0


def bay_centre(meta: dict[str, Any]) -> tuple[float, float] | None:
    """Pocket centre: the midpoint of the two fins bounding the lot.

    Returns ``None`` for a scenario without a parking lot, which is not an
    error -- the generator only builds one when ``has_parking_lot`` is set.
    """
    lot = meta.get("parking_lot")
    if not lot or not meta.get("has_parking_lot"):
        return None
    b1, b2 = lot["block1_position"], lot["block2_position"]
    return (b1["x"] + b2["x"]) / 2.0, (b1["y"] + b2["y"]) / 2.0


def _tuning_with(changes: dict[str, float]) -> NavigationTuning:
    """Shipped tuning with only the named ``corridor_follower`` fields moved.

    Rebuilds the one group and swaps it in, the same way ``diag_open_ab`` does:
    ``NavigationTuning`` is a frozen dataclass of pydantic groups, so the group
    revalidates itself and the aggregate is replaced rather than mutated. An
    unknown field is rejected by the group's own model rather than silently
    ignored.

    Deliberately NOT routed through ``_from_mapping``/``--tuning``, which resets
    every group the mapping omits to code defaults -- a partial override there
    silently discards the entire shipped profile.
    """
    base = NavigationTuning.load_default()
    if not changes:
        return base
    group = base.corridor_follower
    for field in changes:
        if not hasattr(group, field):
            message = f"no such corridor_follower field: {field}"
            raise ValueError(message)
    updated = group.model_validate({**group.model_dump(), **changes})
    return replace(base, corridor_follower=updated)


def _run_case(payload: tuple[str, bool, int, dict[str, float], bool, bool]) -> dict[str, object]:
    """Run one scenario from one start. Returns a row, never raises on outcome."""
    path_str, in_bay, laps, changes, known_start, solid_walls = payload
    path = Path(path_str)
    raw = json.loads(path.read_text())

    start = raw["starting_conditions"]
    moved = False
    if in_bay:
        centre = bay_centre(raw)
        if centre is None:
            return {"id": raw["scenario_id"], "skipped": True}
        # Heading is left alone: the scenario's own start is already parallel to
        # the outer wall, which is the only orientation the 0.20m-deep pocket
        # admits for a 0.194m-wide chassis.
        start["position"]["x"], start["position"]["y"] = centre
        moved = True

    meta = ScenarioMetadata.model_validate(raw)
    sim = ScenarioSimulator(
        meta,
        num_laps=laps,
        tuning=_tuning_with(changes),
        seed=raw["scenario_id"],
        blind=True,
        known_start=known_start,
        solid_walls=solid_walls,
    )
    # Wall contact is legal on OBSTACLES and not on Open, so a scraping escape is
    # only a real result on this challenge -- which is also the only one with a
    # parking lot to start in. With solid walls the chassis is stopped by the
    # wall instead of passing through it, and `allowed_step` then limits the TURN
    # while keeping the translation, which is what lets a cornered chassis peel
    # away. Without it the run simply stalls short of contact, which is what
    # every earlier bay-start number here measured.
    result = sim.run(contact_grace_s=_CONTACT_GRACE_S if solid_walls else None)

    # Where it STOPPED, not just how far it went. A distance alone cannot tell
    # "never left the pocket" from "drove out, crossed the corridor and stalled
    # facing the far wall" -- and those want opposite fixes. Yaw is reported
    # against the scenario's own start heading, which is parallel to the outer
    # wall, so ~90 deg means the chassis is broadside to the corridor it is
    # meant to be driving down.
    fx, fy, fyaw = result.final_pose
    sx, sy = start["position"]["x"], start["position"]["y"]
    return {
        "id": raw["scenario_id"],
        "skipped": False,
        "moved": moved,
        "fx": fx,
        "fy": fy,
        "dyaw_deg": math.degrees(abs(wrap_angle(fyaw - start["yaw"]))),
        "net_m": math.hypot(fx - sx, fy - sy),
        "dist": result.distance_m,
        "laps": result.laps_completed,
        "collided": result.collided,
        "stuck": result.stuck,
        "timed_out": result.timed_out,
        "pass_side": result.pass_side_violation,
    }


def _summarise(name: str, rows: Sequence[dict[str, object]]) -> None:
    """One line per scenario, then the aggregate that actually decides it."""
    live = [r for r in rows if not r["skipped"]]
    if not live:
        print(f"  {name}: no scenarios with a parking lot")
        return

    print(f"\n=== {name} ===")
    print(
        f"| {'#':>4} | {'dist m':>7} | {'net m':>6} | {'dyaw':>6} | {'end x':>6} | {'end y':>6} | "
        f"{'laps':>4} | {'coll':>4} | {'stuck':>5} | {'t/o':>3} | {'pass':>4} |"
    )
    print(
        f"|{'-' * 6}|{'-' * 9}|{'-' * 8}|{'-' * 8}|{'-' * 8}|{'-' * 8}|"
        f"{'-' * 6}|{'-' * 6}|{'-' * 7}|{'-' * 5}|{'-' * 6}|"
    )
    for r in sorted(live, key=lambda x: int(x["id"])):  # type: ignore[arg-type]
        print(
            f"| {r['id']:>4} | {r['dist']:>7.2f} | {r['net_m']:>6.2f} | {r['dyaw_deg']:>6.1f} | "
            f"{r['fx']:>6.2f} | {r['fy']:>6.2f} | {r['laps']:>4} | "
            f"{'Y' if r['collided'] else '.':>4} | {'Y' if r['stuck'] else '.':>5} | "
            f"{'Y' if r['timed_out'] else '.':>3} | {'Y' if r['pass_side'] else '.':>4} |"
        )

    n = len(live)
    dists = [float(r["dist"]) for r in live]  # type: ignore[arg-type]
    immobile = sum(1 for d in dists if d < 0.01)
    print(
        f"  n={n}  immobile(<1cm)={immobile}  "
        f"dist min {min(dists):.2f} / median {sorted(dists)[n // 2]:.2f} / max {max(dists):.2f}  "
        f"laps>=1 {sum(1 for r in live if int(r['laps']) >= 1)}  "  # type: ignore[arg-type]
        f"collided {sum(1 for r in live if r['collided'])}  "
        f"stuck {sum(1 for r in live if r['stuck'])}"
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
        payloads = [(str(p), in_bay, args.laps, changes, known, args.solid_walls) for p in paths]
        rows = run_pool(_run_case, payloads, jobs, on_result=print_pool_progress(name))
        _summarise(name, rows)


if __name__ == "__main__":
    main()
