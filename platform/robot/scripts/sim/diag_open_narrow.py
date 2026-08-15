"""Why the Open Challenge narrow-corridor runs started colliding.

Bisected to ``668e40a``, whose only simulation-side change was clamping the
integrated speed to the measured 0.156 m/s drivetrain ceiling. That is
counterintuitive — driving slower should make control easier — so this isolates
the interaction rather than assuming one.

The suspicion is a speed-dependent steering law. ``WaypointController`` is a
P-controller on bearing error with a rate limit (``MAX_STEERING_RATE`` rad/s).
The rate limit is per *second*, so at lower speed the steering winds up further
per metre travelled, and the same gain carves a tighter arc. Combined with
counter-phase 4WS (``8eb3c38``, half the effective wheelbase) and the raised
steering limit (``9ce0514``, 0.5236 -> 1.2253 rad), the commanded curvature at a
given bearing error is far higher than the gain was ever fitted to.

Sweeps the two knobs that would confirm it — the speed ceiling and
``STEER_KP`` — over the 8 symmetric-narrow starts the test battery uses.

2026-08-15 update: repurposed. The narrow-corridor NAVIGATION bug is fixed
(``5c0e16f``), and what remains is a pure time budget, which this now measures
per hardware profile. Default run overrides nothing, so
``VTITAN_HARDWARE_PROFILE`` decides the speed ceiling and steering limit:

    python scripts/sim/diag_open_narrow.py                          # base
    VTITAN_HARDWARE_PROFILE=fastwide python scripts/sim/diag_open_narrow.py

Measured over the 8 narrow starts, 3 laps against the 180 s limit:

===============  ======  =========  =========
speed ceiling    pass    3 laps?    slowest
===============  ======  =========  =========
0.156 (base)     0/8     no (2)     200.0 s
0.170            0/8     yes        194.8 s
0.185            8/8     yes        179.8 s
0.234 (fastwide) 8/8     yes        140.8 s
===============  ======  =========  =========

The path is 26.07 m over 3 laps, so the limit demands a 0.145 m/s AVERAGE. The
base ceiling is 0.156 -- only 7% above that average -- and the car actually
sustains ~79% of its ceiling once corners are priced in. **The base car cannot
finish this course in time at any control quality**, and steering range does not
enter into it: the ``wideonly`` profile (85 deg wheels, base speed) is
indistinguishable from base at 0/8 and 200.0 s.

2026-08-03 update: root-caused and fixed structurally --
``WaypointController.compute_steering`` no longer uses ``steer_kp`` at all
(replaced with curvature-based pure pursuit off the real chassis geometry, see
``docs/internal/audits/2026-08-03-realtrack-control-instability-findings.md``).
The ``steer_kp`` arm of this sweep is now inert -- every case in it behaves
identically regardless of the value swept, since nothing reads it anymore.
Kept for the speed-ceiling arm, which is still a live question.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/sim/diag_open_narrow.py
"""

from __future__ import annotations

import argparse
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
from itertools import product
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CompetitionSpecs, CorridorDimensions, RobotSpecs
from shared.config.hardware_profile import active_profiles
from shared.config.navigation_tuning import NavigationTuning
from shared.domain.enums import Direction, Section

from src.simulation.kinematics import AckermannKinematics
from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.scenario_simulator import ScenarioSimulator

_N_LAPS = CompetitionSpecs.OPEN_CHALLENGE_LAPS
_NARROW_MM = int(CorridorDimensions.NARROW * 1000)
_STARTS = list(product(Section, Direction))

# Sweep parameters
_SPEED_SWEEP_MPS = (0.156, 0.25, 0.35, 0.50)
_SHIPPED_STEER_KP = 1.5
_SHIPPED_MAX_STEER_RATE = 2.0
_REAL_MAX_SPEED_MPS = 0.156
_STEER_KP_SWEEP = (1.2, 1.0, 0.8, 0.6, 0.4)
_STEER_RATE_SWEEP = (1.0, 4.0)


@dataclass(frozen=True, slots=True)
class Case:
    """One (max speed, steering gain, steering rate) point."""

    max_speed: float
    steer_kp: float
    max_steer_rate: float

    @property
    def label(self) -> str:
        """One-line description of this sweep point."""
        return f"max_speed {self.max_speed:.3f}  steer_kp {self.steer_kp:.2f}  rate {self.max_steer_rate:.1f}"


def _run(args: tuple[Case | None, int]) -> tuple[bool, int, float]:
    """Run one narrow start. ``case is None`` runs the ACTIVE profile untouched.

    The ``None`` arm is the one that matters for a hardware-profile question:
    it overrides nothing, so ``RobotSpecs`` supplies the speed ceiling and the
    steering limit exactly as ``VTITAN_HARDWARE_PROFILE`` resolved them. Passing
    a ``Case`` instead only reaches the SIMULATED car -- the navigator reads
    ``RobotSpecs.MAX_STEERING_ANGLE`` as a default argument, evaluated at import,
    so a steering limit injected here would let the car turn harder than the
    planner ever asks it to and read as a flat knob.
    """
    case, index = args
    section, direction = _STARTS[index]
    meta = build_open_metadata(uniform_widths(_NARROW_MM), section, direction)

    tuning = None
    kinematics = None
    if case is not None:
        base = NavigationTuning()
        # model_copy, not dataclasses.replace: NavigationTuning is a dataclass
        # but its GROUPS are frozen pydantic models, so replace() raises
        # TypeError on them. This script did exactly that and failed on every
        # case before running a single scenario.
        tuning = replace(
            base,
            pursuit=base.pursuit.model_copy(
                update={"STEER_KP": case.steer_kp, "MAX_STEERING_RATE": case.max_steer_rate}
            ),
        )
        kinematics = AckermannKinematics(max_speed_mps=case.max_speed, max_steer_rate=case.max_steer_rate)

    result = ScenarioSimulator(meta, num_laps=_N_LAPS, tuning=tuning, kinematics=kinematics).run()
    within = result.success and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
    return within, result.laps_completed, result.sim_time_s


def _report(label: str, results: list[tuple[bool, int, float]]) -> None:
    passed = sum(1 for within, _, _ in results if within)
    worst = min(laps for _, laps, _ in results)
    slowest = max(t for _, _, t in results)
    print(f"{label:<48} pass {passed}/{len(_STARTS)}  min_laps {worst}  slowest {slowest:5.1f}s", flush=True)


def main() -> None:
    """Run the 8 narrow starts, on the active profile or over the speed sweep."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--speed-sweep",
        action="store_true",
        help="sweep the SIMULATED car's speed ceiling instead of running the active profile",
    )
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()

    profiles = active_profiles()
    shipped = f"profile {','.join(profiles)}" if profiles else "profile <base>"
    print(
        f"{shipped}: max_speed {RobotSpecs.MAX_SPEED_MPS:.3f} m/s, "
        f"max wheel angle {math.degrees(RobotSpecs.MAX_STEERING_ANGLE):.1f} deg, "
        f"limit {CompetitionSpecs.ROUND_TIME_LIMIT_S:.0f} s, {_N_LAPS} laps"
    )

    cases: list[Case | None] = [None]
    if args.speed_sweep:
        # Simulated car only -- see _run. Useful for asking how much of the
        # shortfall is raw speed, but it does NOT model a wider steering range.
        cases = [Case(s, _SHIPPED_STEER_KP, _SHIPPED_MAX_STEER_RATE) for s in _SPEED_SWEEP_MPS]

    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for case in cases:
            results = list(pool.map(_run, [(case, i) for i in range(len(_STARTS))]))
            _report(case.label if case else shipped, results)


if __name__ == "__main__":
    main()
