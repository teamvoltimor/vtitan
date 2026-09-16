r"""A/B the SlotSignMap's colour gate: publication floor, vote margin, and thaw.

The production map (``sign_slot_map.SlotSignMap``) publishes a cell once its
summed evidence clears ``slot_min_evidence`` (0.75) and colours it by the argmax
of that cell's votes. One confident detection is enough for both. When the
router then commits to the slot, ``set_committed`` FREEZES it, and
``_refresh_colour`` returns early forever after -- so a colour published from a
single flipped vote is the colour the robot routes on for the rest of the round.

That is a deliberate design, not an oversight: the freeze is what took committed
colour flips to zero (adr:0058). The question this script exists to answer is
whether the freeze has to start at the FIRST vote, or can wait until the cell's
evidence is worth freezing.

THREE ARMS, and they are not interchangeable:

* ``ev=X`` raises the publication floor. Costs anticipation -- the sign is
  published later, and the 2026-09-11 budget put 62% of the missing 1.4 m of
  anticipation in perception already.
* ``margin=M`` requires the winning colour to beat the runner-up by M before the
  cell has a colour at all. Refuses to guess rather than delaying.
* ``thaw=X`` leaves publication untouched and narrows the FREEZE: a committed
  slot keeps refreshing its colour while its cell still weighs less than X.
  Anticipation is unchanged; only a weakly-evidenced commitment stays
  correctable.

Arms combine: ``thaw=1.5,margin=0.5``.

``ctl`` is the control and should be run in every session. It sets the floor to
an impossible value so NOTHING publishes, and it must therefore fail everywhere.
An arm list without it cannot tell "this knob does nothing" from "this patch is
not on the path" -- and that distinction has already cost this repo a night:
the first version of this A/B patched ``sign_discovery.ObservedSignMap``, which
the production router no longer uses, and returned byte-identical output that
read as a clean refutation.

MEASURED 2026-09-15 on the committed 16, fail = collision, wrong-side pass, or
under 3 laps::

    base                 7   (0000 0003 0004 0008 0009 0013 0014)
    ctl                 16   control fails everywhere, as it must
    ev=1.5               9   fixes 0000 0013, breaks 0006 0007 0010 0012
    ev=2.0              10   the same, plus 0011
    margin=0.5           6   fixes 0013, breaks nothing
    thaw=1.5             7   identical to base
    thaw=3.0             5   fixes 0000 0013, breaks nothing
    thaw=1.5,margin=0.5  6   fixes 0013, breaks nothing

CAVEAT, and it is why those numbers are quoted as history rather than as a
recommendation: every one of them was measured while the simulator's LIDAR
occlusion band sat 180 degrees from where the chassis puts it, against a corpus
baseline of 28 failing tests. The band was corrected the following day and the
baseline moved to 22 -- and two of the three scenarios ``thaw`` fixed (0000,
0013) are among the ones the band correction fixed by another route. Re-run
before believing the table: the arm may now be redundant, or it may be attacking
what is left.

Comparison is by SET, not by count. Two arms that both fail seven scenarios are
not the same arm, and a table of counts cannot say so.

WHY THIS PATCHES CLASSES instead of setting a flag, which the repo otherwise
avoids: ``diag_sign_router_flag_ab.py`` already A/Bs any BOOLEAN
``SignRouterParams`` field, and is the right tool the moment a knob exists.
``thaw`` does not exist -- there is no ``slot_colour_thaw_evidence`` key -- and
``margin`` is not a field either. Measuring first is what decides whether the
key is worth adding. If ``thaw`` survives a re-run on the corrected baseline it
should ship as a real TOML key (0.0 meaning today's behaviour) and be measured
from then on by the flag harness, not by this one.

Usage::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        pixi run -e dev python scripts/sim/diag_sign_slot_ab.py \
        --arm base --arm ctl --arm thaw=3.0 --jobs 14
"""

from __future__ import annotations

import argparse
import logging
import sys
from dataclasses import dataclass, replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import shared.domain.enums  # noqa: F401  (imported first: models <-> enums cycle)

from scripts.common.diag_base import Verdict, resolve_jobs, run_pool, verdict
from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS

logging.disable(logging.CRITICAL)

_CONTROL_EVIDENCE = 1e9
"""The control arm's publication floor: unreachable, so nothing is ever published."""


@dataclass(frozen=True, slots=True)
class Arm:
    """One parsed arm. ``None`` means "leave production's value alone"."""

    name: str
    min_evidence: float | None = None
    margin: float = 0.0
    thaw: float | None = None


def parse_arm(spec: str) -> Arm:
    """Parse ``base`` / ``ctl`` / a comma-separated ``key=value`` list."""
    if spec == "base":
        return Arm(spec)
    if spec == "ctl":
        return Arm(spec, min_evidence=_CONTROL_EVIDENCE)
    arm = Arm(spec)
    for part in spec.split(","):
        key, _, raw = part.partition("=")
        match key:
            case "ev":
                arm = replace(arm, min_evidence=float(raw))
            case "margin":
                arm = replace(arm, margin=float(raw))
            case "thaw":
                arm = replace(arm, thaw=float(raw))
            case _:
                msg = f"unknown arm knob {key!r} in {spec!r}; expected ev, margin or thaw"
                raise SystemExit(msg)
    return arm


_ACTIVE = Arm("base")
"""The arm the patched hooks below consult, re-set per case.

A process pool reuses its workers across payloads, so patching the classes per
arm would STACK one arm's patch on the next one's run. The hooks are installed
once and read this instead, which keeps a worker's behaviour a function of the
case it was handed rather than of the cases it happened to run before.
"""

_INSTALLED = False


def _install_hooks() -> None:
    """Wrap the three production seams, once per worker process."""
    global _INSTALLED
    if _INSTALLED:
        return
    from shared.domain.models import SignColor  # noqa: PLC0415

    import src.navigation.planning.sign_slot_map as sm  # noqa: PLC0415

    original_init = sm.SlotSignMap.__init__
    original_colour = sm._CellEvidence.colour  # noqa: SLF001 - this IS the seam under measurement

    def init(self, *args, **kwargs) -> None:  # noqa: ANN001, ANN002, ANN003
        original_init(self, *args, **kwargs)
        if _ACTIVE.min_evidence is not None:
            self._min_evidence = _ACTIVE.min_evidence

    def refresh_colour(self, slot) -> None:  # noqa: ANN001
        # Production returns unconditionally while the slot is frozen. The thaw
        # arm keeps refreshing for as long as the cell's evidence is still light.
        if slot.frozen and (_ACTIVE.thaw is None or self._cells[slot.cell].weight >= _ACTIVE.thaw):
            return
        slot.colour = self._cells[slot.cell].colour
        slot.hits = self._cells[slot.cell].hits

    def colour(self) -> SignColor:  # noqa: ANN001
        if not _ACTIVE.margin or not self.votes:
            return original_colour.fget(self)
        ranked = sorted(self.votes.values(), reverse=True)
        runner_up = ranked[1] if len(ranked) > 1 else 0.0
        # UNKNOWN, not the argmax: a cell whose two colours sit within the
        # margin has not decided, and publication already refuses UNKNOWN.
        return original_colour.fget(self) if (ranked[0] - runner_up) >= _ACTIVE.margin else SignColor.UNKNOWN

    sm.SlotSignMap.__init__ = init
    sm.SlotSignMap._refresh_colour = refresh_colour  # noqa: SLF001
    sm._CellEvidence.colour = property(colour)  # noqa: SLF001
    _INSTALLED = True


def run_case(payload: tuple[str, int, int]) -> tuple[str, str, str, int, float]:
    """Run one (arm, scenario index) case. Module-level for ProcessPoolExecutor."""
    global _ACTIVE
    spec, index, max_steps = payload
    _ACTIVE = parse_arm(spec)
    _install_hooks()

    from src.simulation.scenario_catalog import all_obstacles_demo_scenarios  # noqa: PLC0415
    from src.simulation.scenario_simulator import ScenarioSimulator  # noqa: PLC0415

    scenario = all_obstacles_demo_scenarios()[index]
    result = ScenarioSimulator(
        scenario.metadata,
        num_laps=scenario.laps,
        seed=scenario.seed,
        emit_vision_detections=True,
    ).run(max_steps=max_steps)
    return spec, scenario.label, str(verdict(result)), result.laps_completed, round(result.sim_time_s, 1)


def _short(label: str) -> str:
    """``go_obstacles_0004[East/clockwise]`` as ``0004``, which is how every table here names it."""
    return label.rsplit("_", 1)[-1].partition("[")[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--arm",
        action="append",
        help="Arm spec, repeatable: base, ctl, or ev=X / margin=M / thaw=X joined by commas.",
    )
    parser.add_argument(
        "--scenario",
        type=int,
        action="append",
        help="Fixture index to run, repeatable. Reproduces exactly the case the full run reports under that number.",
    )
    parser.add_argument("--jobs", type=int, default=0, help="Workers; 0 picks cores minus a couple.")
    parser.add_argument("--max-steps", type=int, default=OBSTACLES_MAX_STEPS)
    args = parser.parse_args()

    arms: list[str] = args.arm or ["base", "ctl", "thaw=3.0"]
    for spec in arms:
        parse_arm(spec)  # fail on a typo before spending the cores
    if "ctl" not in arms:
        print("WARNING: no ctl arm. A null result will not be distinguishable from a dead patch.\n")

    from src.simulation.scenario_catalog import all_obstacles_demo_scenarios  # noqa: PLC0415

    indices = args.scenario or list(range(len(all_obstacles_demo_scenarios())))
    count = len(indices)
    payloads = [(spec, index, args.max_steps) for spec in arms for index in indices]
    rows = run_pool(run_case, payloads, resolve_jobs(args.jobs))

    failing: dict[str, set[str]] = {spec: set() for spec in arms}
    for spec, label, name, _laps, _time in rows:
        if name not in (Verdict.OK, Verdict.OVER_TIME):
            failing[spec].add(label)

    baseline = failing[arms[0]]
    print(f"\n{count} scenarios, fail = collision, wrong-side pass, reverse run or short of target laps")
    for spec in arms:
        print(f"\n  {spec:24} {len(failing[spec]):>2} failing  {' '.join(sorted(map(_short, failing[spec])))}")
        if spec == arms[0]:
            continue
        fixes = sorted(map(_short, baseline - failing[spec]))
        breaks = sorted(map(_short, failing[spec] - baseline))
        print(f"  {'':24} vs {arms[0]}: fixes {' '.join(fixes) or '-'}   breaks {' '.join(breaks) or '-'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
