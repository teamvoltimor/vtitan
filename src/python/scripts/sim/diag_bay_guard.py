"""What number does the bay-exit guard actually veto on, at the first tick of each leg?

The in-bay exit is at 0/16 and the ratchet burns hundreds of legs for
millimetres of net progress, which says legs are dying almost as soon as they
start. Three separate attempts to name the term responsible -- the leg speed,
the inertia/coast term, the mirrored reverse lock -- were each refuted by an
A/B, because each one guessed at which quantity in ``_guarded_command`` was
binding instead of reading it. This prints it.

The instrument deliberately duplicates NO geometry. ``_predicted_gap`` is
wrapped on the live ``BayExit`` instance, and within one tick the guard calls it
in a fixed order:

* ``step_m != 0`` -> the ``gap`` at the reachable pose, the value tested against
  the margin;
* ``step_m == 0`` immediately after -> ``held``, the overlap-recovery reference;
* ``step_m == 0`` with no preceding non-zero call -> the one-off
  ``_guard_min_gap`` seed on the leg's first guarded tick.

So every number reported here came out of the shipped code path rather than a
re-derivation of it, which is the failure mode the refuted attempts shared.
``_begin_leg`` is wrapped alongside it to cut the tick stream into legs; the
``_guard_flips`` delta across that call separates a leg the CLEARANCE guard
ended from one ``BAY_EXIT_LEG_MAX_S`` timed out.

Single-process on purpose: the wrappers close over recording state, which a
``ProcessPoolExecutor`` worker would not carry back.

Usage (from ``src``)::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \\
    PYTHONPATH=".;../shared/src" pixi run python scripts/sim/diag_bay_guard.py --limit 4
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CompetitionSpecs  # noqa: E402
from shared.domain.models import ScenarioMetadata  # noqa: E402

from scripts.common.sim_defaults import CORPUS_DIR  # noqa: E402
from scripts.sim.diag_bay_start import (  # noqa: E402
    _COMMITTED_DIR,
    _CONTACT_GRACE_S,
    bay_exit_clearance,
    bay_outward_axis,
)
from src.config.tuning_helpers import tuning_with_overrides  # noqa: E402
from src.navigation.track_geometry import parking_bay_centre  # noqa: E402
from src.navigation.utils import wrap_angle  # noqa: E402
from src.simulation.scenario_simulator import ScenarioSimulator  # noqa: E402

if TYPE_CHECKING:
    from collections.abc import Sequence


@dataclass
class GuardTick:
    """One guarded tick, as the guard itself computed it."""

    leg: int
    is_reverse: bool
    leg_tick: int
    dr_out: float
    dr_yaw: float
    dr_along: float
    reach: float
    gap: float
    held: float


@dataclass
class LegRecord:
    """One leg: the ticks it drove and how it ended."""

    index: int
    is_reverse: bool
    ticks: list[GuardTick] = field(default_factory=list)
    ended_by: str = "run-end"


def _instrument(bay_exit: object, margin: float) -> list[LegRecord]:
    """Wrap ``_predicted_gap``/``_begin_leg`` on one instance; return the growing log."""
    legs: list[LegRecord] = [LegRecord(index=0, is_reverse=False)]
    pending: dict[str, float] = {}
    pending_label: dict[str, object] = {}
    raw_gap = bay_exit._predicted_gap  # noqa: SLF001 - probing internals is this script's purpose
    raw_begin = bay_exit._begin_leg  # noqa: SLF001

    def gap_spy(step_m: float, wheel_norm: float, tuning: object) -> float:
        if pending_label:
            leg = pending_label["leg"]
            leg.ended_by = "clearance" if bay_exit._guard_flips > pending_label["flips"] else "leg-max"  # noqa: SLF001
            pending_label.clear()
        value = raw_gap(step_m, wheel_norm, tuning)
        if step_m != 0.0:
            # The tested value. Its `held` partner arrives on the next call.
            pending.clear()
            pending["reach"] = step_m
            pending["gap"] = value
            pending["out"] = bay_exit._dr_out  # noqa: SLF001
            pending["yaw"] = bay_exit._dr_yaw  # noqa: SLF001
            pending["along"] = bay_exit._dr_along  # noqa: SLF001
            pending["leg_tick"] = float(bay_exit._leg_ticks)  # noqa: SLF001
        elif "gap" in pending:
            legs[-1].ticks.append(
                GuardTick(
                    leg=legs[-1].index,
                    is_reverse=bool(bay_exit._leg_is_reverse),  # noqa: SLF001
                    leg_tick=int(pending["leg_tick"]),
                    dr_out=pending["out"],
                    dr_yaw=pending["yaw"],
                    dr_along=pending["along"],
                    reach=pending["reach"],
                    gap=pending["gap"],
                    held=value,
                )
            )
            pending.clear()
        return value

    def begin_spy(**kwargs: object) -> None:
        # `_guard_flips` rises only on the clearance veto, so its delta names
        # which of the two exits ended this leg without the script re-testing
        # `gap <= margin` itself -- but the increment happens AFTER this call
        # returns, so reading it here labels every leg `leg-max`. Resolved on
        # the next tick instead, which is the first moment the counter is true.
        pending_label.clear()
        pending_label["leg"] = legs[-1]
        pending_label["flips"] = bay_exit._guard_flips  # noqa: SLF001
        raw_begin(**kwargs)
        legs.append(LegRecord(index=legs[-1].index + 1, is_reverse=bool(kwargs["is_reverse"])))

    bay_exit._predicted_gap = gap_spy  # type: ignore[assignment]  # noqa: SLF001
    bay_exit._begin_leg = begin_spy  # type: ignore[assignment]  # noqa: SLF001
    _ = margin
    return legs


def _run_one(
    path: Path,
    args: argparse.Namespace,
    changes: dict[str, float | bool],
    sim_changes: dict[str, float | bool] | None = None,
) -> tuple[str, list[LegRecord], float, object, float | None, tuple[float, float]] | None:
    """Place the chassis in the pocket, run, and hand back the guard's own log.

    The exit verdict is the TRUE footprint clearance of the pocket mouth, taken
    over the whole run rather than off the final pose -- the dead-reckoned
    ``_dr_out`` the guard steers on is a belief, and judging the manoeuvre by
    the number it is itself computing would be circular.
    """
    raw = json.loads(path.read_text())
    centre = parking_bay_centre(raw)
    if centre is None:
        return None
    start = raw["starting_conditions"]
    parallel_xy = (start["position"]["x"], start["position"]["y"])
    start["position"]["x"] = centre[0]
    start["position"]["y"] = centre[1]

    tuning = tuning_with_overrides(changes)
    if sim_changes:
        # The PHYSICS arm, not a follower knob: an arm that moves the chassis
        # model and one that moves the manoeuvre are different claims, so they
        # are carried apart rather than merged into one override dict.
        tuning = tuning_with_overrides(sim_changes, group="simulation", base=tuning)
    if args.min_turn_radius is not None:
        tuning = tuning_with_overrides({"MIN_TURN_RADIUS_M": args.min_turn_radius}, group="simulation", base=tuning)
    if args.no_progress_window > 0.0:
        tuning = tuning_with_overrides(
            {"NO_PROGRESS_WINDOW_S": args.no_progress_window}, group="simulation", base=tuning
        )
    sim = ScenarioSimulator(
        ScenarioMetadata.model_validate(raw),
        num_laps=args.laps,
        tuning=tuning,
        seed=raw["scenario_id"],
        blind=True,
        solid_walls=args.solid_walls,
        slide_on_contact=args.slide,
    )
    margin = tuning.corridor_follower.BAY_EXIT_CLEARANCE_MARGIN_M
    legs = _instrument(sim.bay_exit, margin)

    outward = bay_outward_axis(centre, parallel_xy, start["yaw"])
    best_exit: float | None = None
    # TRUE outward displacement of the chassis centre on the same axis the
    # guard's `_dr_out` estimates. Same quantity, one measured and one believed,
    # so the ratio is the dead reckoning's error rather than an inference from
    # two different measures.
    true_out = 0.0

    # Per-tick TRUE state on the same three axes the dead reckoning carries, so
    # the drift can be attributed to one of them rather than read off the single
    # aggregate `out`. Paired at the same instant: `_dr_*` is sampled inside the
    # observer, not from the guard log, because a guarded tick and an observed
    # step are not the same clock.
    drift: list[tuple[float, float, float, float, float, float]] = []
    along_axis = (math.cos(start["yaw"]), math.sin(start["yaw"]))

    def _observe(state: object, _scan: object) -> None:
        nonlocal best_exit, true_out
        if outward is None:
            return
        clearance = bay_exit_clearance((state.x, state.y, state.yaw), centre, outward)
        best_exit = clearance if best_exit is None else max(best_exit, clearance)
        dx, dy = state.x - centre[0], state.y - centre[1]
        out_now = dx * outward[0] + dy * outward[1]
        true_out = max(true_out, out_now)
        drift.append(
            (
                out_now,
                sim.bay_exit._dr_out,  # noqa: SLF001
                dx * along_axis[0] + dy * along_axis[1],
                sim.bay_exit._dr_along,  # noqa: SLF001
                wrap_angle(state.yaw - start["yaw"]),
                sim.bay_exit._dr_yaw,  # noqa: SLF001
            )
        )

    result = sim.run(contact_grace_s=_CONTACT_GRACE_S if args.solid_walls else None, on_step=_observe)
    # The manoeuvre's OWN release test, in the units it releases on. Passing
    # `yaw_rad` from the simulator makes this reachable at all -- it read 0 for
    # every sim run before 2026-09-10 -- so whether the sim can reach
    # BAY_EXIT_TARGET_YAW_DEG is what says that plumbing is inert or not.
    print(
        f"  rotation reached {sim.bay_exit.rotation_deg:.2f} deg of "
        f"{tuning.corridor_follower.BAY_EXIT_TARGET_YAW_DEG:.0f} target",
        flush=True,
    )
    believed = max((t.dr_out for leg in legs for t in leg.ticks), default=0.0)
    if drift:
        # Signed error, not its magnitude: a term that oscillates about the
        # truth and one that walks away from it need different fixes, and an
        # absolute error cannot tell them apart.
        for label, true_i, dr_i, scale in (("out", 0, 1, 1.0), ("along", 2, 3, 1.0), ("yaw", 4, 5, 180.0 / math.pi)):
            errs = [(row[dr_i] - row[true_i]) * scale for row in drift]
            trues = [row[true_i] * scale for row in drift]
            print(
                f"  drift {label:<6} true span {min(trues):+.4f}..{max(trues):+.4f}  "
                f"dr-true: median {statistics.median(errs):+.4f} final {errs[-1]:+.4f} "
                f"worst {max(errs, key=abs):+.4f}",
                flush=True,
            )
    return raw["scenario_id"], legs, margin, result, best_exit, (true_out, believed)


def _report(
    scenario: str,
    legs: Sequence[LegRecord],
    margin: float,
    result: object,
    best_exit: float | None,
    out_pair: tuple[float, float],
    *,
    detail: bool,
) -> tuple[int, int, list[float], float | None, list[str], tuple[float, float]]:
    """Print one scenario's legs; return ``(legs, one_tick_legs, slacks, best exit, end reasons)``."""
    driven = [leg for leg in legs if leg.ticks]
    one_tick = [leg for leg in driven if len(leg.ticks) == 1]
    # The slack the leg ACTUALLY ran down to, not the one it started with: a
    # first-tick reading answers "was the leg dead on arrival", and once that is
    # 0% the live question is how much of the margin a leg leaves unspent when
    # its TIME bound cuts it.
    slacks = [min(x.gap for x in leg.ticks) - margin for leg in driven]
    ends = [leg.ended_by for leg in driven]
    if detail:
        print(
            f"\n{scenario}: {len(legs)} legs ({len(driven)} with a guarded tick), "
            f"{len(one_tick)} died on their FIRST tick, margin={margin:.4f} m, "
            f"run={getattr(result, 'sim_time_s', float('nan')):.1f}s sim "
            f"collided={getattr(result, 'collided', None)} timed_out={getattr(result, 'timed_out', None)} "
            f"surface={getattr(result, 'terminal_surface', None)} "
            f"contacts={getattr(result, 'contact_count', None)} laps={getattr(result, 'laps_completed', None)}",
            flush=True,
        )
        print(
            f"  {'leg':<5}{'dir':<5}{'ticks':<7}{'ended':<11}{'gap[0]':<10}{'gap[-1]':<10}{'gap_min':<10}"
            f"{'slack_min':<11}{'out':<9}{'yaw_deg':<9}{'along'}",
            flush=True,
        )
        for leg in driven[:12]:
            t, last = leg.ticks[0], leg.ticks[-1]
            gap_min = min(x.gap for x in leg.ticks)
            print(
                f"  {leg.index:<5}{'rev' if leg.is_reverse else 'fwd':<5}{len(leg.ticks):<7}{leg.ended_by:<11}"
                f"{t.gap:<10.4f}{last.gap:<10.4f}{gap_min:<10.4f}{gap_min - margin:<+11.4f}"
                f"{last.dr_out:<9.4f}{math.degrees(last.dr_yaw):<9.2f}{last.dr_along:.4f}",
                flush=True,
            )
        if len(driven) > 12:
            print(f"  ... {len(driven) - 12} more legs", flush=True)
    return len(driven), len(one_tick), slacks, best_exit, ends, out_pair


def main() -> None:
    """Run the probe over a few in-bay scenarios and summarise what binds."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", type=int, default=4, help="Scenario count (0 = all).")
    parser.add_argument("--laps", type=int, default=CompetitionSpecs.OPEN_CHALLENGE_LAPS)
    parser.add_argument("--corpus", action="store_true", help=f"use {CORPUS_DIR} instead of the committed set")
    parser.add_argument("--scenarios-dir", default=None)
    parser.add_argument(
        "--solid-walls",
        action="store_true",
        default=True,
        help="walls stop the chassis instead of ending the run. On by default here: the "
        "ratchet's whole mechanism is the wall clipping the yaw, so without it there is no "
        "manoeuvre to instrument.",
    )
    parser.add_argument("--no-solid-walls", dest="solid_walls", action="store_false")
    parser.add_argument("--slide", action="store_true", default=True, help="let a blocked translation slide.")
    parser.add_argument("--no-slide", dest="slide", action="store_false")
    parser.add_argument(
        "--no-progress-window",
        type=float,
        default=180.0,
        help="seconds of under-0.08 m displacement tolerated; the shipped 30 s ends the run "
        "mid-manoeuvre, which is a harness bound rather than a rule (9.4 gives three minutes).",
    )
    parser.add_argument(
        "--min-turn-radius",
        type=float,
        default=None,
        help="override simulation MIN_TURN_RADIUS_M. Since 966b2b36 the dead reckoning floors "
        "the curvature exactly as the physics does, so this no longer decouples the two models "
        "-- it now asks the cleaner question of what the manoeuvre does at a radius the chassis "
        "does not have, which is the physics every pre-72e7172b bay-exit result was measured on.",
    )
    parser.add_argument(
        "--measured-yaw",
        action="store_true",
        help="add an arm with BAY_EXIT_DR_USES_MEASURED_YAW on, which replaces the guard's "
        "integrated-and-clamped yaw with the one `_track_rotation` already measures.",
    )
    parser.add_argument(
        "--tracks-speed",
        action="store_true",
        help="add an arm with simulation.MIN_TURN_RADIUS_TRACKS_SPEED on. The bay creeps at "
        "<= 0.10 m/s, inside the range the curve was measured over, so this is the one arm the "
        "curve legitimately answers -- the corridor at 0.26-0.50 m/s is extrapolation.",
    )
    parser.add_argument(
        "--only",
        default=None,
        help="keep only arms whose name contains this, so ONE arm remains and the per-leg "
        "table prints. Reading aid; the shipped baseline is what any verdict needs.",
    )
    parser.add_argument(
        "--mirror",
        action="store_true",
        help="add an arm with BAY_EXIT_GUARD_MIRRORS_REVERSE on. Held lock and mirrored lock "
        "retrace one another only at the model radius: the docstring's ratchet arithmetic says "
        "holding accumulates NO rotation against the measured 0.29 m floor while mirroring "
        "accumulates it at ANY radius. The lever's refutation predates 966b2b36, so it was "
        "measured on a dead reckoning that over-read outward travel 31x.",
    )
    parser.add_argument(
        "--leg-max",
        type=float,
        nargs="*",
        help="sweep BAY_EXIT_LEG_MAX_S (seconds). Its own comment calls it a backstop for a "
        "STALLED wheel -- the guard is meant to end a healthy leg -- but measured here it ends "
        "100%% of them, so it is the primary bound by accident. One arm per value; the shipped "
        "value is always run first as the baseline.",
    )
    args = parser.parse_args()

    directory = Path(args.scenarios_dir) if args.scenarios_dir else (CORPUS_DIR if args.corpus else _COMMITTED_DIR)
    paths = sorted(directory.glob("*_metadata.json"))
    if not paths:
        parser.error(f"no *_metadata.json under {directory}")
    if args.limit:
        paths = paths[: args.limit]
    print(
        f"{len(paths)} scenarios from {directory}; solid_walls={args.solid_walls} "
        f"slide={args.slide} no_progress_window={args.no_progress_window}",
        flush=True,
    )

    arms: list[tuple[str, dict[str, float | bool], dict[str, float | bool]]] = [("shipped", {}, {})]
    arms += [(f"leg_max {v:g}s", {"BAY_EXIT_LEG_MAX_S": v}, {}) for v in (args.leg_max or [])]
    if args.mirror:
        arms.append(("mirror reverse", {"BAY_EXIT_GUARD_MIRRORS_REVERSE": True}, {}))
    if args.measured_yaw:
        arms.append(("measured yaw", {"BAY_EXIT_DR_USES_MEASURED_YAW": True}, {}))
    if args.mirror and args.measured_yaw:
        # The pair is the point: mirroring is what produces rotation, the
        # measured yaw is what lets the guard see it. Each alone is a term of a
        # mechanism, so both singles are run too rather than inferred.
        arms.append(
            (
                "mirror+measured",
                {"BAY_EXIT_GUARD_MIRRORS_REVERSE": True, "BAY_EXIT_DR_USES_MEASURED_YAW": True},
                {},
            )
        )
    if args.tracks_speed:
        # The bay creeps at <= 0.10 m/s, INSIDE the range the speed curve was
        # measured over, so this is the one arm the curve legitimately answers.
        # Paired with the levers because the question is whether the manoeuvre
        # works on the corrected physics, not whether either alone helps.
        arms.append(("tracks speed", {}, {"MIN_TURN_RADIUS_TRACKS_SPEED": True}))
        if args.mirror:
            arms.append(
                (
                    "tracks+mirror",
                    {"BAY_EXIT_GUARD_MIRRORS_REVERSE": True},
                    {"MIN_TURN_RADIUS_TRACKS_SPEED": True},
                )
            )

    if args.only:
        arms = [arm for arm in arms if args.only in arm[0]] or arms
    detail = len(arms) == 1

    for name, changes, sim_changes in arms:
        total_legs = 0
        total_one_tick = 0
        all_slacks: list[float] = []
        exits: list[float] = []
        ended_by: list[str] = []
        out_pairs: list[tuple[float, float]] = []
        for path in paths:
            run = _run_one(path, args, changes, sim_changes)
            if run is None:
                continue
            legs, one_tick, slacks, best_exit, ends, out_pair = _report(*run, detail=detail)
            ended_by.extend(ends)
            out_pairs.append(out_pair)
            total_legs += legs
            total_one_tick += one_tick
            all_slacks.extend(slacks)
            if best_exit is not None:
                exits.append(best_exit)
        if not all_slacks:
            print(f"\n{name}: no guarded ticks recorded", flush=True)
            continue
        ordered = sorted(all_slacks)
        # `out` is the verdict: metres by which the footprint cleared the pocket
        # mouth at its best moment. Positive on a scenario means that scenario
        # got out; the ratio is the arm's score.
        got_out = sum(1 for e in exits if e > 0.0)
        ended = dict(Counter(e for e in ended_by if e != "run-end"))
        print(
            f"\n{name:<16} legs {total_legs:<5} tick-1 deaths {total_one_tick:<5} "
            f"out {got_out}/{len(exits)}  best exit max {max(exits):+.4f} median "
            f"{statistics.median(exits):+.4f} m  ended {ended}",
            flush=True,
        )
        print(
            f"{'':<16} lowest slack reached: min {ordered[0]:+.4f}  "
            f"median {statistics.median(ordered):+.4f}  max {ordered[-1]:+.4f} m; "
            f"legs that reached the margin {sum(1 for x in all_slacks if x <= 0)}",
            flush=True,
        )
        if out_pairs:
            true_m = statistics.median(t for t, _ in out_pairs)
            believed_m = statistics.median(b for _, b in out_pairs)
            print(
                f"{'':<16} outward travel: BELIEVED {believed_m:.4f} m vs TRUE {true_m:.4f} m "
                f"({believed_m / true_m:.1f}x over-read)"
                if true_m > 1e-6
                else f"{'':<16} outward travel: BELIEVED {believed_m:.4f} m vs TRUE {true_m:.4f} m",
                flush=True,
            )


if __name__ == "__main__":
    main()
