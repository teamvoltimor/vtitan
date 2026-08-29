"""Single-scenario headless CLI: runs one ``ScenarioSimulator`` pass and prints
its ``SimResult`` as one line of JSON to stdout.

This is the machine-readable counterpart to the ``scripts/sim/diag_*.py``
family, which are all interactive investigation tools that print
human-formatted text and hardcode which corpus they load. Built specifically
as the subprocess contract for ``platform/robot-go``'s
``internal/sim/scenario`` orchestrator (see
``docs/internal/plans/go-migration-plan.md``'s "Simulation" section): the Go
orchestrator shells out to this script once per scenario rather than
reimplementing the kinematics/collision math in Go, which stays Python until
profiling shows the interpreter loop -- not the already-vectorized numpy
raycast -- is the actual bottleneck.

Usage (from ``platform/robot``, with ``PYTHONPATH=.``)::

    python scripts/sim/run_scenario.py tests/fixtures/scenarios/obstacles/scenario_0000_metadata.json
    python scripts/sim/run_scenario.py path/to/scenario_metadata.json --laps 3 --seed 7 --sighted

Exit code is 0 whenever the simulator produced a result, even a failed one
(collision, timeout, pass-side violation, stuck) -- those are DATA describing
how the run ended, not a tool error. Non-zero means a result could not be
produced at all (bad metadata file, unhandled exception); a traceback goes to
stderr and stdout stays empty, so a caller can tell "the robot failed" apart
from "the harness failed" by checking the exit code before parsing stdout.
"""

from __future__ import annotations

import argparse
import json
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import CompetitionSpecs
from shared.domain.enums import ScenarioType
from shared.domain.models import ScenarioMetadata

from scripts.common.sim_defaults import OBSTACLES_MAX_STEPS
from src.simulation.scenario_result import SimResult
from src.simulation.scenario_simulator import ScenarioSimulator

_EXIT_OK = 0
_EXIT_ERROR = 1


def _default_laps(metadata: ScenarioMetadata) -> int:
    """The competition lap target for this scenario's challenge, absent ``--laps``."""
    if metadata.challenge_type == ScenarioType.OPEN:
        return CompetitionSpecs.OPEN_CHALLENGE_LAPS
    return CompetitionSpecs.OBSTACLE_CHALLENGE_LAPS


def _result_payload(metadata_path: Path, result: SimResult) -> dict[str, object]:
    """Flatten a ``SimResult`` into the JSON object this CLI prints.

    Every field here is read directly off ``SimResult`` (or its ``success``/
    ``over_time`` properties) -- nothing invented beyond what the simulator
    already produces, so a Go-side ``Result`` struct can be a direct mirror.
    """
    return {
        "scenario": str(metadata_path),
        "target_laps": result.target_laps,
        "laps_completed": result.laps_completed,
        "collided": result.collided,
        "timed_out": result.timed_out,
        "stuck": result.stuck,
        "pass_side_violation": result.pass_side_violation,
        "pass_side_violation_signs": result.pass_side_violation_signs,
        "parked": result.parked,
        "success": result.success,
        "over_time": result.over_time,
        "steps": result.steps,
        "sim_time_s": result.sim_time_s,
        "distance_m": result.distance_m,
        "max_speed_mps": result.max_speed_mps,
        "avg_speed_mps": result.avg_speed_mps,
        "min_lidar_range_m": result.min_lidar_range_m,
        "collision_xy": list(result.collision_xy) if result.collision_xy is not None else None,
        "final_pose": list(result.final_pose),
        "contact_count": result.contact_count,
        "contact_time_s": result.contact_time_s,
        "terminal_surface": result.terminal_surface.value,
        "lap_step_indices": result.lap_step_indices,
    }


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("metadata", type=Path, help="Path to a *_metadata.json scenario fixture")
    parser.add_argument("--laps", type=int, default=None, help="Override the target lap count")
    parser.add_argument("--seed", type=int, default=0, help="RNG seed for sensor noise / LIDAR sampling")
    parser.add_argument("--max-steps", type=int, default=OBSTACLES_MAX_STEPS, help="Control-tick safety budget")
    parser.add_argument(
        "--sighted",
        dest="blind",
        action="store_false",
        default=True,
        help="Hand the robot the true layout/direction/signs (control condition). "
        "Default is blind, the competition condition.",
    )
    parser.add_argument(
        "--no-park",
        dest="park",
        action="store_false",
        default=True,
        help="Skip the parking maneuver; score laps only.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Run one scenario and print its result as JSON. Returns the process exit code."""
    args = _parse_args(argv)
    try:
        metadata = ScenarioMetadata.model_validate(json.loads(args.metadata.read_text()))
        laps = args.laps if args.laps is not None else _default_laps(metadata)
        result = ScenarioSimulator(
            metadata,
            num_laps=laps,
            seed=args.seed,
            blind=args.blind,
            park=args.park,
        ).run(max_steps=args.max_steps)
    except Exception:  # noqa: BLE001 - deliberately broad: any failure here is a harness error, reported on stderr
        traceback.print_exc()
        return _EXIT_ERROR

    print(json.dumps(_result_payload(args.metadata, result)))
    return _EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
