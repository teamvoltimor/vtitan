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

import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, replace
from itertools import product
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CompetitionSpecs, CorridorDimensions
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


def _run(args: tuple[Case, int]) -> tuple[bool, int, float]:
    case, index = args
    section, direction = _STARTS[index]
    meta = build_open_metadata(uniform_widths(_NARROW_MM), section, direction)
    base = NavigationTuning()
    tuning = replace(
        base,
        pursuit=replace(base.pursuit, STEER_KP=case.steer_kp, MAX_STEERING_RATE=case.max_steer_rate),
    )
    result = ScenarioSimulator(
        meta,
        num_laps=_N_LAPS,
        tuning=tuning,
        kinematics=AckermannKinematics(
            max_speed_mps=case.max_speed,
            max_steer_rate=case.max_steer_rate,
        ),
    ).run()
    within = result.success and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
    return within, result.laps_completed, result.sim_time_s


def main() -> None:
    """Sweep speed ceiling and steering gain over the 8 narrow starts."""
    cases = [
        # Speed ceiling, at the shipped gain — does restoring headroom fix it?
        *(Case(s, _SHIPPED_STEER_KP, _SHIPPED_MAX_STEER_RATE) for s in _SPEED_SWEEP_MPS),
        # Steering gain, at the real 0.156 m/s ceiling — does softening fix it?
        *(Case(_REAL_MAX_SPEED_MPS, k, _SHIPPED_MAX_STEER_RATE) for k in _STEER_KP_SWEEP),
        # Steering rate, at the real ceiling and shipped gain.
        *(Case(_REAL_MAX_SPEED_MPS, _SHIPPED_STEER_KP, r) for r in _STEER_RATE_SWEEP),
    ]
    with ProcessPoolExecutor(max_workers=8) as pool:
        for case in cases:
            results = list(pool.map(_run, [(case, i) for i in range(len(_STARTS))]))
            passed = sum(1 for within, _, _ in results if within)
            worst = min(laps for _, laps, _ in results)
            slowest = max(t for _, _, t in results)
            print(
                f"{case.label:<48} pass {passed}/{len(_STARTS)}  min_laps {worst}  slowest {slowest:5.1f}s",
                flush=True,
            )


if __name__ == "__main__":
    main()
