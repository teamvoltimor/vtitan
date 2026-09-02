"""Probe: is the blind obstacles loss a WRONG layout or a LATE one?

`diag_sign_sweep.py blind-source` attributes the whole blind shortfall to the
estimated corridor layout rather than to sign discovery. That leaves two very
different causes, with two different fixes:

* **Wrong.** The estimator settles on widths that are not the true ones, so the
  path is permanently off and no amount of runway helps. Fix belongs in
  ``corridor_estimator``.
* **Late.** A blind run starts believing every corridor is narrow and corrects
  from LIDAR as it drives, so the opening stretch is driven on a path built for
  a layout the robot is standing on but has not measured yet. If the first sign
  arrives before convergence, the run is lost to timing, not accuracy — and the
  fix is about ordering (delay commitment, or route conservatively until
  settled), not about the estimator's precision.

Reports, per fixture: the final belief vs truth, the step at which the belief
last changed, and the step the run ended. A collision BEFORE the last belief
change is a "late" failure; one after it with a correct belief is neither, and
points back at the sign geometry itself.

Usage (from ``platform/robot``, PYTHONPATH=.)::

    python scripts/sim/diag_blind_layout.py
"""

from __future__ import annotations

import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from src.navigation import corridor_estimator
from src.navigation.corridor_estimator import section_from_heading
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

_WIDTH_MATCH_TOL_M = 0.01
"""Belief matches truth within this. The two legal widths are 0.40 m apart, so
anything this side of a centimetre is the same classification."""


@dataclass(slots=True)
class _ProbeState:
    """Mutable counters closed over by ``probe()``'s tracking callbacks."""

    step: int = 0
    last_change: int = -1
    changes: int = 0
    here_ok: bool | None = None
    here_seen: bool | None = None
    calls: int = 0
    valid: int = 0


@dataclass(frozen=True, slots=True)
class LayoutProbeResult:
    """One blind fixture's outcome, reduced to what the probe reports on."""

    label: str
    wrong_corridors: list[str]
    seen: set[str]
    votes: dict[str, list[int]]
    calls: int
    valid: int
    here_ok: bool | None
    here_seen: bool | None
    last_change: int
    changes: int
    end_step: int
    collided: bool
    laps: int


def probe(index: int) -> LayoutProbeResult:
    """Run one blind fixture, tracking when its layout belief settled."""
    scenario = all_obstacles_demo_scenarios()[index]
    sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True)

    original_update = sim._update_layout_belief  # noqa: SLF001 - a probe, by design
    estimator = sim._width_estimator  # noqa: SLF001
    true_widths = sim._true_geometry.to_widths_dict()  # noqa: SLF001
    state = _ProbeState()

    # Count how often a scan yields a usable width at all. A corridor that is
    # never measured is either short of time (few calls) or having its readings
    # rejected (many calls, few valid) — different problems, different fixes.
    original_measure = corridor_estimator.measure_corridor_width

    def counted_measure(*a: Any, **kw: Any) -> Any:
        state.calls += 1
        m = original_measure(*a, **kw)
        if m is not None:
            state.valid += 1
        return m

    corridor_estimator.measure_corridor_width = counted_measure

    def tracked() -> bool:
        state.step += 1
        changed = original_update()
        if changed:
            state.last_change = state.step
            state.changes += 1
        # Snapshot the belief about the corridor the robot is in RIGHT NOW.
        # Whatever it believes about corridors it has not reached yet is not a
        # cause of anything -- a run that dies at step 124 has never seen three
        # of the four, so scoring the final belief as a whole would blame the
        # estimator for the collision's own consequences.
        pose = sim.gateway.get_current_pose()
        if pose is not None and estimator is not None:
            here = section_from_heading(pose.yaw, sim._direction)  # noqa: SLF001
            state.here_ok = abs(estimator.widths[here] - true_widths[here]) <= _WIDTH_MATCH_TOL_M
            state.here_seen = here in estimator.observed_sections
        return changed

    sim._update_layout_belief = tracked  # type: ignore[method-assign]  # noqa: SLF001

    try:
        result = sim.run(max_steps=OBSTACLES_MAX_STEPS)
    finally:
        corridor_estimator.measure_corridor_width = original_measure

    believed = sim.believed_widths or {}
    wrong = [s.value for s in true_widths if abs(believed.get(s, 0.0) - true_widths[s]) > _WIDTH_MATCH_TOL_M]
    seen = {s.value for s in (estimator.observed_sections if estimator else set())}
    # Votes banked per corridor: [narrow, wide]. Distinguishes "no time to
    # measure" (few votes) from "measured and got it wrong" (many, wrong way).
    votes = {s.value: list(v) for s, v in estimator._votes.items() if sum(v)} if estimator else {}  # noqa: SLF001

    return LayoutProbeResult(
        label=scenario.label,
        wrong_corridors=wrong,
        seen=seen,
        votes=votes,
        calls=state.calls,
        valid=state.valid,
        here_ok=state.here_ok,
        here_seen=state.here_seen,
        last_change=state.last_change,
        changes=state.changes,
        end_step=result.steps,
        collided=result.collided,
        laps=result.laps_completed,
    )


def main() -> None:
    """Probe every obstacles fixture blind and attribute each failure."""
    verdicts: Counter = Counter()
    for i in range(len(all_obstacles_demo_scenarios())):
        r = probe(i)
        if not r.collided:
            verdict = "no collision"
        elif r.here_ok is False and r.here_seen is False:
            verdict = "WRONG+UNMEASURED (still on the narrow default)"
        elif r.here_ok is False:
            verdict = "WRONG (measured this corridor and got it wrong)"
        else:
            verdict = "belief here correct, still collided"
        verdicts[verdict] += 1
        print(
            f"{r.label:<34} end={r.end_step:>4} changes={r.changes:>2} laps={r.laps} "
            f"here_ok={r.here_ok} here_measured={r.here_seen} "
            f"scans={r.calls} valid={r.valid} votes={r.votes} -> {verdict}",
            flush=True,
        )
    print("\nSUMMARY " + "  ".join(f"{k}={v}" for k, v in verdicts.most_common()))


if __name__ == "__main__":
    main()
