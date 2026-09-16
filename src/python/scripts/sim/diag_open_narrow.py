"""Can the Open Challenge narrow corridors be driven inside the round limit?

Runs the 8 symmetric-narrow starts the test battery uses and scores each on the
only thing that counts: 3 laps within ``ROUND_TIME_LIMIT_S``.

The narrow-corridor NAVIGATION bug this script was written for is fixed
(``5c0e16f``); what remains is a pure time budget, so this now measures that per
hardware profile. The script overrides nothing, so ``VTITAN_HARDWARE_PROFILE``
decides the speed ceiling and steering limit -- and it is MANDATORY, since the
base config no longer declares a motor or a servo:

    VTITAN_HARDWARE_PROFILE=180deg-injora-14kg,generic-motor-1500rpm \
        python scripts/sim/diag_open_narrow.py
    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \
        python scripts/sim/diag_open_narrow.py

Scored over the 8 narrow starts, 3 laps against ``ROUND_TIME_LIMIT_S``. The
path is long enough over 3 laps that the limit demands an average well above the
slowest profile's ceiling, and that motor cannot finish the course in time at
any control quality. Steering range does not enter into it: swapping only the
servo is indistinguishable from the slower build, confirmed scenario for
scenario over the full 128-scenario blind sweep. Before the tuning moved to
physical units the servo appeared to be a real lever; it was only ever rescaling
normalised steering constants. See ``adr:0087-test-methodology``.

History worth keeping, because it explains what this script does NOT measure:
the original suspicion was a speed-dependent steering law, since
``MAX_STEERING_RATE`` is per *second*, so a slower car winds the steering
further per metre and carves a tighter arc. That was root-caused and fixed
structurally -- ``WaypointController.compute_steering`` no longer reads
``steer_kp`` at all, having moved to curvature-based pure pursuit off the real
chassis geometry (``adr:0052-pursuit-target-selection``). Any ``steer_kp`` value
passed here is therefore inert.

That per-second rate limit is still live in the other direction, and is the
thing to check first if a FASTER profile ever starts clipping walls: the same
commanded curvature carves a wider arc the quicker you go, and none of the
absolute-unit tuning (arc radius, lookahead) rescales with the profile.
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

_NARROW_MM = int(CorridorDimensions.NARROW * 1000)
_STARTS = list(product(Section, Direction))

# Sweep parameters. Read from the config rather than restated: _SHIPPED_STEER_KP
# was a hardcoded 1.5 against a shipped 1.2, and _REAL_MAX_SPEED_MPS a hardcoded
# 0.156 that would have gone on reading 0.156 under a hardware profile that
# raises the ceiling -- i.e. the sweep would have silently mislabelled which car
# it measured.
_SHIPPED = NavigationTuning.load_default()
_SHIPPED_STEER_KP = _SHIPPED.pursuit.steer_kp
_SHIPPED_MAX_STEER_RATE = _SHIPPED.pursuit.max_steering_rate
_SPEED_SWEEP_MPS = (RobotSpecs.MAX_SPEED_MPS, 0.25, 0.35, 0.50)


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
        base = NavigationTuning.load_default()
        # model_copy, not dataclasses.replace: NavigationTuning is a dataclass
        # but its GROUPS are frozen pydantic models, so replace() raises
        # TypeError on them. This script did exactly that and failed on every
        # case before running a single scenario.
        tuning = replace(
            base,
            pursuit=base.pursuit.model_copy(
                update={"steer_kp": case.steer_kp, "max_steering_rate": case.max_steer_rate}
            ),
        )
        kinematics = AckermannKinematics(max_speed_mps=case.max_speed, max_steer_rate=case.max_steer_rate)

    result = ScenarioSimulator(meta, num_laps=CompetitionSpecs.OPEN_CHALLENGE_LAPS, tuning=tuning, kinematics=kinematics).run()
    within = result.success and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
    return within, result.laps_completed, result.sim_time_s


def _report(label: str, results: list[tuple[bool, int, float]]) -> None:
    passed = sum(1 for within, _, _ in results if within)
    worst = min(laps for _, laps, _ in results)
    slowest = max(t for _, _, t in results)
    print(f"{label:<48} pass {passed}/{len(_STARTS)}  min_laps {worst}  slowest {slowest:5.1f}s", flush=True)


def main() -> int:
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
        f"limit {CompetitionSpecs.ROUND_TIME_LIMIT_S:.0f} s, {CompetitionSpecs.OPEN_CHALLENGE_LAPS} laps"
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
