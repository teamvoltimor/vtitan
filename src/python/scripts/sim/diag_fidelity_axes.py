r"""The SIMULATOR half of the sim-divergence audit: the same axes, on a simulated round.

Read this next to ``scripts/bag/diag_bag_fidelity_axes.py``, which measures a
recorded hardware round. Both import their definitions from
``scripts/common/fidelity_axes.py`` and print the same line shapes, so the two
outputs diff cleanly and the diff is about the robot.

WHAT THE PAIR IS FOR. Every tuning verdict here is scored in this simulator, so
each of its idealisations silently weights every A/B. The measured divergence,
worst first (see ``adr:0086-simulator-realism``):

* the LIDAR occlusion band sat on the WRONG SIDE of the car, blinding the
  forward diagonals instead of the rear wedge (correcting it is a new baseline,
  not a fix);
* ``vision_frame_miss_rate`` shipped at 0.79, which is pessimistic against the
  share of ticks that really carry a fresh USABLE observation -- recalibrated to
  0.30 on 2026-09-16, where the simulator returns a detection on 29.6% of polls
  against a hardware mean of 29.6% of nav ticks;
* the sim rotates less per escape, because ``allowed_step`` scales yaw but never
  INDUCES it from contact, while the real chassis reverses as if on a tight
  radius;
* displacing a pillar is terminal here, and on hardware a round can score its
  laps with several pillars pushed.

Two of those are pessimistic and the rest optimistic. That mix is the whole
reason to measure rather than assume: an idealisation you assume is in your
favour can be costing laps.

THE VISION AXIS IS NOT DEFINED HERE, on purpose. It lives in
``scripts/common/fidelity_axes.py`` alongside the LIDAR bands, because until
2026-09-16 this half and the bag half computed it two different ways at once:
the bag half string-matched the raw payload (counting detections production
discards) and divided by nav ticks, while this half counted gateway polls. Both
shares printed in the same line shape and neither said which denominator it
used. They now share :class:`VisionFreshness`, which carries its denominator
with it, and both print it.

ONE DELIBERATE DIFFERENCE FROM THE CORPUS. ``test_obstacles_challenge_sim.py``
constructs the simulator with ``emit_vision_detections`` at its default False,
which makes the camera always-empty and resolves sign colours from scenario
ground truth. That is a reasonable simplification for a lap-scoring battery and
a useless one for a fidelity probe -- there is no vision to compare. So this
runs with the emulator ON by default. ``--no-vision`` reproduces the corpus
construction exactly, and in that mode the vision block truthfully reads zero.

WHAT IS DELIBERATELY NOT COMPARABLE. The simulator never emits a non-finite
range -- ``SimulatedHardwareGateway`` substitutes ``LIDAR_MAX_RANGE`` for a
miss, exactly as the production gateway does to a real scan. So "finite" here
means "returned something nearer than max range", while on the bag side it means
"the driver returned a number at all". Both are the honest reading of "the
sensor saw something" for their own source; the shares are comparable, the
definitions are not identical, and pretending otherwise is how you conclude the
C1 is clean.

Usage::

    VTITAN_HARDWARE_PROFILE=... python scripts/sim/diag_fidelity_axes.py go_obstacles_0005 go_obstacles_0004
    VTITAN_HARDWARE_PROFILE=... python scripts/sim/diag_fidelity_axes.py --list
"""

from __future__ import annotations

import argparse
import math
import time
from typing import TYPE_CHECKING, Any

import numpy as np
from shared.config.constants.robot import RobotSpecs

from scripts.common.fidelity_axes import (
    SUB_FLOOR_M,
    SectorCensus,
    VisionFreshness,
    contiguous_episodes,
    format_committed,
    format_escapes,
    format_steering,
    format_vision_freshness,
)
from scripts.common.stats import percentile
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios
from src.simulation.scenario_simulator import ScenarioSimulator

if TYPE_CHECKING:
    from src.simulation.scenario_catalog import NamedScenario

_CONTROL_DT = 0.05
_SCAN_STRIDE = 5
_ESCAPE_STEER_FLOOR = 0.3
_MAX_STEPS = 6000


class _Trace:
    """Per-tick record of the axes, collected from ``on_step``.

    Parallel lists rather than a dataclass per tick: a 6000-step run allocating
    one object per tick is measurably slower than the navigation it is meant to
    be observing, and this runs over a whole fixture set.
    """

    def __init__(self) -> None:
        self.maneuver: list[object] = []
        self.committed: list[bool] = []
        self.cmd_speed: list[float | None] = []
        self.steer: list[float | None] = []
        self.maneuver_steer: list[float | None] = []
        self.yaw: list[float] = []
        self.speed: list[float] = []
        self.census = SectorCensus()

    def __len__(self) -> int:
        return len(self.yaw)


def _run(scenario: NamedScenario, *, emit_vision: bool) -> None:
    sim = ScenarioSimulator(
        scenario.metadata,
        num_laps=scenario.laps,
        seed=scenario.seed,
        emit_vision_detections=emit_vision,
    )
    navigator = sim.navigator
    gateway = sim.gateway
    trace = _Trace()
    vision = {"calls": 0, "with_detection": 0}

    original = gateway.get_vision_detections

    def counted(*a: Any, **k: Any) -> Any:
        result = original(*a, **k)
        vision["calls"] += 1
        vision["with_detection"] += bool(result)
        return result

    gateway.get_vision_detections = counted

    def on_step(state: Any, scan: Any) -> None:
        snap = navigator.debug_snapshot
        trace.maneuver.append(snap.active_maneuver_type)
        trace.committed.append(snap.committed_sign_x_m is not None)
        trace.cmd_speed.append(snap.commanded_speed_mps)
        trace.steer.append(snap.commanded_steering_norm)
        trace.maneuver_steer.append(snap.maneuver_steering)
        trace.yaw.append(state.yaw)
        trace.speed.append(state.v)
        if len(trace) % _SCAN_STRIDE == 0:
            ranges = np.asarray(scan.ranges_m, dtype=float)
            bearings = np.degrees(np.asarray(scan.angles_rad))
            returned = ranges < RobotSpecs.LIDAR_MAX_RANGE - 1e-6
            trace.census.add(bearings, ranges, finite=returned)

    started = time.time()
    result = sim.run(max_steps=_MAX_STEPS, on_step=on_step)
    ticks = len(trace)
    duration = ticks * _CONTROL_DT
    print(
        f"\n=== {scenario.label}  laps={result.laps_completed}/{result.target_laps} "
        f"collided={result.collided} pass_side={result.pass_side_violation} stuck={result.stuck} "
        f"timeout={result.timed_out} steps={result.steps} contacts={result.contact_count} "
        f"({time.time() - started:.0f}s wall)"
    )
    if ticks == 0:
        print("  no ticks -- nothing to compare")
        return

    yaw = np.unwrap(trace.yaw)
    episodes = contiguous_episodes([m is not None for m in trace.maneuver])
    kinds: dict[str, int] = {}
    for a, _b in episodes:
        key = str(trace.maneuver[a])
        kinds[key] = kinds.get(key, 0) + 1
    for line in format_escapes(
        kinds=kinds,
        durations_s=[(b - a + 1) * _CONTROL_DT for a, b in episodes],
        yaw_deltas_deg=[abs(math.degrees(yaw[b] - yaw[a])) for a, b in episodes],
        short_episodes=sum(1 for a, b in episodes if b - a + 1 <= 2),
    ):
        print(line)

    turn_rates = [
        abs(math.degrees(yaw[i + 1] - yaw[i])) / _CONTROL_DT
        for a, b in episodes
        for i in range(a, b)
        if trace.maneuver_steer[i] is not None and abs(trace.maneuver_steer[i]) >= _ESCAPE_STEER_FLOOR
    ]
    print(
        f"  escape ticks |steer|>={_ESCAPE_STEER_FLOOR}: n={len(turn_rates)} "
        f"yaw rate deg/s p50={percentile(turn_rates, 0.5):.1f} p90={percentile(turn_rates, 0.9):.1f}"
    )

    cruise = [i for i in range(ticks) if trace.committed[i] and trace.maneuver[i] is None]
    print(
        format_committed(
            committed_ticks=len(cruise),
            total_ticks=ticks,
            commanded_mps=[trace.cmd_speed[i] for i in cruise if trace.cmd_speed[i] is not None],
            achieved_mps=[abs(trace.speed[i]) for i in cruise],
        )
    )

    steer = [trace.steer[i] for i in range(ticks) if trace.steer[i] is not None and trace.maneuver[i] is None]
    for line in format_steering(steer, duration):
        print(line)

    for line in format_vision_freshness(
        VisionFreshness(
            hits=vision["with_detection"],
            opportunities=vision["calls"],
            denominator="gateway polls",
        ),
        extra=(
            f"nav {1 / _CONTROL_DT:.0f} Hz over {ticks} ticks; no funnel to print, because the emulator's "
            f"detections are BUILT to pass the production gate -- a poll returning anything is a kept observation"
        ),
    ):
        print(line)
    print(f"{trace.census.format()}   (robot frame, {trace.census.scans} scans; 'finite' = nearer than max range)")
    print(f"  sub-floor share is rays under {SUB_FLOOR_M} m, which the sector filter drops on both sides")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("labels", nargs="*", help="Scenario labels or unique prefixes (default: the first fixture)")
    parser.add_argument("--list", action="store_true", help="Print the available scenario labels and exit")
    parser.add_argument(
        "--no-vision",
        action="store_true",
        help="Run with emit_vision_detections=False, as the pytest corpus does (the vision block then reads zero)",
    )
    args = parser.parse_args()

    scenarios = all_obstacles_demo_scenarios()
    if args.list:
        for scenario in scenarios:
            print(scenario.label)
        return 0

    wanted = args.labels or [scenarios[0].label]
    for label in wanted:
        matches = [s for s in scenarios if s.label == label or s.label.startswith(label)]
        if not matches:
            print(f"!!! no scenario matching {label!r} -- run with --list")
            continue
        if len(matches) > 1:
            print(f"!!! {label!r} matches {len(matches)}: {', '.join(s.label for s in matches)}")
            continue
        _run(matches[0], emit_vision=not args.no_vision)

    print(
        "\nDiff these blocks against scripts/bag/diag_bag_fidelity_axes.py on a real round. 'finite'\n"
        "means 'nearer than max range' here and 'the driver returned a number' there -- the shares\n"
        "compare, the definitions do not. Do not assume which side an idealisation favours.\n"
        "The vision share is over GATEWAY POLLS here and over NAV TICKS on the bag, which on hardware\n"
        "are 81.3% of ticks; both halves name their denominator rather than silently sharing a line."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
