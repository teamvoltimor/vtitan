"""What does real state estimation cost the navigator?

Every closed-loop test drives the navigator on ground-truth pose, so
``LidarLocalizer`` — the position source the real robot actually navigates on —
is exercised by no closed-loop test at all. This runs the same scenarios both
ways and reports the difference, which is the honest measure of how much of the
current pass rate depends on perfect odometry.

Reports, per configuration:

* Open Challenge 3-lap solvability across all 8 narrow starts and 8 wide starts.
* Obstacles fixtures (collisions / laps).
* Position error of the estimate against ground truth.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    python scripts/diag_localization.py open
    python scripts/diag_localization.py obstacles
    python scripts/diag_localization.py blind
    python scripts/diag_localization.py perturbed --sweep all

``perturbed`` goes furthest: blind to the track *and* unsure of its own pose,
which is the only configuration that matches what the robot faces on the mat.
"""

from __future__ import annotations

import argparse
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from enum import StrEnum
from itertools import product
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from shared.config.constants import CompetitionSpecs, CorridorDimensions
from shared.config.enums import Direction, Section

from src.navigation.track_geometry import corridor_widths_from_metadata
from src.simulation.scenario_simulator import ScenarioSimulator
from src.simulation.simulated_hardware_gateway import SensorErrors
from src.simulation.scenario_builder import build_open_metadata, uniform_widths
from src.simulation.scenario_catalog import all_obstacles_demo_scenarios, all_test_scenarios

class DiagMode(StrEnum):
    """Diagnostic comparison modes."""

    OPEN = "open"
    OBSTACLES = "obstacles"
    OPEN_FIXTURES = "open-fixtures"
    BLIND = "blind"
    PERTURBED = "perturbed"


_N_LAPS = CompetitionSpecs.OPEN_CHALLENGE_LAPS
_NARROW_MM = int(CorridorDimensions.NARROW * 1000)
_WIDE_MM = int(CorridorDimensions.WIDE * 1000)
_STARTS = list(product(Section, Direction))
_OBSTACLES_MAX_STEPS = 6000
_WIDTH_MATCH_TOLERANCE_M = 1e-6
_OPEN_NAME_WIDTH = 6
_ERROR_FORMAT = "5.1f"
_LABEL_WIDTH = 34
_LAPS_TARGET = 3
_ERROR_FORMAT_1F = ".1f"
_PLACEMENT_ERROR_2CM = 0.02
_PLACEMENT_ERROR_5CM = 0.05
_PLACEMENT_ERROR_10CM = 0.10
_PLACEMENT_ERROR_20CM = 0.20
_YAW_BIAS_2DEG = 2
_YAW_BIAS_5DEG = 5
_YAW_BIAS_10DEG = 10
_IMU_DRIFT_0_1 = 0.1
_IMU_DRIFT_0_25 = 0.25
_IMU_DRIFT_0_5 = 0.5
_IMU_DRIFT_1_0 = 1.0
_IMU_DRIFT_FINE_0_01 = 0.01
_IMU_DRIFT_FINE_0_02 = 0.02
_IMU_DRIFT_FINE_0_03 = 0.03
_IMU_DRIFT_FINE_0_05 = 0.05
_IMU_DRIFT_FINE_0_07 = 0.07
_GYRO_SCALE_0_1PCT = 0.001
_GYRO_SCALE_0_25PCT = 0.0025
_GYRO_SCALE_0_5PCT = 0.005
_GYRO_SCALE_1_0PCT = 0.01
_GYRO_SCALE_2_0PCT = 0.02
_IMU_NOISE_0_5DEG = 0.5
_IMU_NOISE_1_0DEG = 1.0
_IMU_NOISE_2_0DEG = 2.0


@dataclass(frozen=True, slots=True)
class _OpenResult:
    """Result from running an Open Challenge start position."""

    within_time: bool
    laps_completed: int
    collided: bool
    peak_pos_err: float


def _run_open(args: tuple[int, int, bool]) -> _OpenResult:
    index, width_mm, localize = args
    section, direction = _STARTS[index]
    meta = build_open_metadata(uniform_widths(width_mm), section, direction)
    sim = ScenarioSimulator(meta, num_laps=_N_LAPS, use_lidar_localization=localize)
    peak = [0.0]

    def on_step(_state: object, _scan: object) -> None:
        peak[0] = max(peak[0], sim.gateway.position_error_m)

    result = sim.run(on_step=on_step)
    within = result.success and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
    return _OpenResult(
        within_time=within,
        laps_completed=result.laps_completed,
        collided=result.collided,
        peak_pos_err=peak[0],
    )


def _run_obstacles(args: tuple[int, bool]) -> tuple[bool, int, float]:
    index, localize = args
    scenario = all_obstacles_demo_scenarios()[index]
    sim = ScenarioSimulator(
        scenario.metadata,
        num_laps=scenario.laps,
        seed=scenario.seed,
        use_lidar_localization=localize,
    )
    peak = [0.0]

    def on_step(_state: object, _scan: object) -> None:
        peak[0] = max(peak[0], sim.gateway.position_error_m)

    result = sim.run(max_steps=_OBSTACLES_MAX_STEPS, on_step=on_step)
    return result.collided, result.laps_completed, peak[0]


def report_open(workers: int) -> None:
    """Open Challenge 3-lap solvability, perfect pose vs LIDAR-estimated pose."""
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for width_mm, name in ((_NARROW_MM, "narrow"), (_WIDE_MM, "wide")):
            for localize in (False, True):
                jobs = [(i, width_mm, localize) for i in range(len(_STARTS))]
                results = list(pool.map(_run_open, jobs))
                passed = sum(1 for r in results if r.within_time)
                collided = sum(1 for r in results if r.collided)
                worst_laps = min(r.laps_completed for r in results)
                peak = max(r.peak_pos_err for r in results)
                pose = "LIDAR-estimated" if localize else "ground truth   "
                print(
                    f"OPEN {name:<{_OPEN_NAME_WIDTH}} pose={pose}  pass {passed}/{len(_STARTS)}  "
                    f"collided {collided}/{len(_STARTS)}  min_laps {worst_laps}  "
                    f"peak_pos_err {peak * 100:{_ERROR_FORMAT}}cm",
                    flush=True,
                )


def report_obstacles(workers: int) -> None:
    """Obstacles fixtures, perfect pose vs LIDAR-estimated pose."""
    count = len(all_obstacles_demo_scenarios())
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for localize in (False, True):
            results = list(pool.map(_run_obstacles, [(i, localize) for i in range(count)]))
            collisions = sum(1 for c, _, _ in results if c)
            laps1 = sum(1 for _, laps, _ in results if laps >= 1)
            laps3 = sum(1 for _, laps, _ in results if laps >= _N_LAPS)
            peak = max(p for _, _, p in results)
            pose = "LIDAR-estimated" if localize else "ground truth   "
            print(
                f"OBSTACLES pose={pose}  collisions {collisions}/{count}  "
                f"laps>=1 {laps1}/{count}  laps>=3 {laps3}/{count}  "
                f"peak_pos_err {peak * 100:{_ERROR_FORMAT}}cm",
                flush=True,
            )


@dataclass(frozen=True, slots=True)
class _BlindResult:
    """Result from running a blind fixture without layout knowledge."""

    label: str
    within_time: bool
    laps_completed: int
    collided: bool
    layout_ok: bool
    peak_pos_err: float


def _run_blind(args: tuple[int]) -> _BlindResult:
    """One fixture with the layout withheld — LIDAR + IMU only."""
    (index,) = args
    scenario = all_test_scenarios()[index]
    true_geometry = corridor_widths_from_metadata(scenario.metadata)
    sim = ScenarioSimulator(scenario.metadata, num_laps=scenario.laps, seed=scenario.seed, blind=True)
    peak = [0.0]

    def on_step(_state: object, _scan: object) -> None:
        peak[0] = max(peak[0], sim.gateway.position_error_m)

    result = sim.run(on_step=on_step)
    believed = sim.believed_widths or {}
    layout_ok = all(abs(believed.get(s, -1) - w) < _WIDTH_MATCH_TOLERANCE_M for s, w in true_geometry.to_widths_dict().items())
    within = result.success and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
    return _BlindResult(
        label=scenario.label,
        within_time=within,
        laps_completed=result.laps_completed,
        collided=result.collided,
        layout_ok=layout_ok,
        peak_pos_err=peak[0],
    )


def report_blind(workers: int) -> None:
    """The 28 fixtures with no layout knowledge at all."""
    count = len(all_test_scenarios())
    with ProcessPoolExecutor(max_workers=workers) as pool:
        results = list(pool.map(_run_blind, [(i,) for i in range(count)]))
    passed = sum(1 for r in results if r.within_time)
    collided = sum(1 for r in results if r.collided)
    layout = sum(1 for r in results if r.layout_ok)
    peak = max(r.peak_pos_err for r in results)
    print(
        f"BLIND  pass {passed}/{count}  collided {collided}/{count}  "
        f"layout_learned {layout}/{count}  peak_pos_err {peak * 100:{_ERROR_FORMAT}}cm",
        flush=True,
    )
    for r in results:
        if not (r.within_time and r.layout_ok):
            print(
                f"    {r.label:<{_LABEL_WIDTH}} pass={r.within_time} laps={r.laps_completed}/{_LAPS_TARGET} collided={r.collided} layout_ok={r.layout_ok} pos_err={r.peak_pos_err * 100:{_ERROR_FORMAT_1F}}cm",
                flush=True,
            )


@dataclass(frozen=True, slots=True)
class _PerturbedRun:
    """One blind fixture run under a given sensor-error configuration."""

    label: str
    passed: bool
    laps: int
    collided: bool
    layout_ok: bool
    peak_pos_err: float
    final_pos_err: float
    peak_heading_err: float


def _run_perturbed(args: tuple[int, SensorErrors]) -> _PerturbedRun:
    """One blind fixture, with the robot also unsure where it is and where it points."""
    index, errors = args
    scenario = all_test_scenarios()[index]
    true_geometry = corridor_widths_from_metadata(scenario.metadata)
    sim = ScenarioSimulator(
        scenario.metadata,
        num_laps=scenario.laps,
        seed=scenario.seed,
        blind=True,
        sensor_errors=errors,
    )
    peak_pos = [0.0]
    peak_yaw = [0.0]

    def on_step(_state: object, _scan: object) -> None:
        peak_pos[0] = max(peak_pos[0], sim.gateway.position_error_m)
        peak_yaw[0] = max(peak_yaw[0], abs(sim.gateway.heading_error_rad))

    result = sim.run(on_step=on_step)
    believed = sim.believed_widths or {}
    layout_ok = all(abs(believed.get(s, -1) - w) < _WIDTH_MATCH_TOLERANCE_M for s, w in true_widths.items())
    return _PerturbedRun(
        label=scenario.label,
        passed=result.success and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S,
        laps=result.laps_completed,
        collided=result.collided,
        layout_ok=layout_ok,
        peak_pos_err=peak_pos[0],
        final_pos_err=sim.gateway.position_error_m,
        peak_heading_err=peak_yaw[0],
    )


def _deg(degrees: float) -> float:
    return math.radians(degrees)


# One axis at a time, so a drop is attributable. The combined rows are the ones
# that matter for a hardware prediction -- the errors are simultaneous on the mat.
_SWEEPS: dict[str, list[tuple[str, SensorErrors]]] = {
    "placement": [
        ("exact placement", SensorErrors()),
        ("placement 2cm", SensorErrors(start_pos_error_m=_PLACEMENT_ERROR_2CM)),
        ("placement 5cm", SensorErrors(start_pos_error_m=_PLACEMENT_ERROR_5CM)),
        ("placement 10cm", SensorErrors(start_pos_error_m=_PLACEMENT_ERROR_10CM)),
        ("placement 20cm", SensorErrors(start_pos_error_m=_PLACEMENT_ERROR_20CM)),
    ],
    "heading": [
        ("exact heading", SensorErrors()),
        ("yaw bias 2deg", SensorErrors(yaw_bias_rad=_deg(_YAW_BIAS_2DEG))),
        ("yaw bias 5deg", SensorErrors(yaw_bias_rad=_deg(_YAW_BIAS_5DEG))),
        ("yaw bias 10deg", SensorErrors(yaw_bias_rad=_deg(_YAW_BIAS_10DEG))),
    ],
    "drift": [
        ("perfect IMU", SensorErrors()),
        ("drift 0.1deg/s", SensorErrors(imu_drift_rad_per_s=_deg(_IMU_DRIFT_0_1))),
        ("drift 0.25deg/s", SensorErrors(imu_drift_rad_per_s=_deg(_IMU_DRIFT_0_25))),
        ("drift 0.5deg/s", SensorErrors(imu_drift_rad_per_s=_deg(_IMU_DRIFT_0_5))),
        ("drift 1.0deg/s", SensorErrors(imu_drift_rad_per_s=_deg(_IMU_DRIFT_1_0))),
    ],
    # 0.1 deg/s already scores 6/28, so the usable ceiling is somewhere below
    # it and the coarse ladder never sampled that range. These are the values
    # a BNO085 spec sheet actually lives at.
    "drift-fine": [
        ("perfect IMU", SensorErrors()),
        ("drift 0.01deg/s", SensorErrors(imu_drift_rad_per_s=_deg(_IMU_DRIFT_FINE_0_01))),
        ("drift 0.02deg/s", SensorErrors(imu_drift_rad_per_s=_deg(_IMU_DRIFT_FINE_0_02))),
        ("drift 0.03deg/s", SensorErrors(imu_drift_rad_per_s=_deg(_IMU_DRIFT_FINE_0_03))),
        ("drift 0.05deg/s", SensorErrors(imu_drift_rad_per_s=_deg(_IMU_DRIFT_FINE_0_05))),
        ("drift 0.07deg/s", SensorErrors(imu_drift_rad_per_s=_deg(_IMU_DRIFT_FINE_0_07))),
        ("drift 0.1deg/s", SensorErrors(imu_drift_rad_per_s=_deg(_IMU_DRIFT_0_1))),
    ],
    # Gyro scale-factor error: accumulates per degree turned, not per second.
    # Three laps is 12 corners of 90 degrees, so >1080 deg of deliberate
    # rotation is banked before steering corrections. Reported BNO08x behaviour
    # is ~1-2 deg per full revolution (~0.3-0.6%), which is anecdotal rather
    # than a datasheet figure -- hence the spread either side of it.
    "scale": [
        ("perfect gyro", SensorErrors()),
        ("scale 0.1%", SensorErrors(gyro_scale_error=_GYRO_SCALE_0_1PCT)),
        ("scale 0.25%", SensorErrors(gyro_scale_error=_GYRO_SCALE_0_25PCT)),
        ("scale 0.5%", SensorErrors(gyro_scale_error=_GYRO_SCALE_0_5PCT)),
        ("scale 1.0%", SensorErrors(gyro_scale_error=_GYRO_SCALE_1_0PCT)),
        ("scale 2.0%", SensorErrors(gyro_scale_error=_GYRO_SCALE_2_0PCT)),
    ],
    "noise": [
        ("clean IMU", SensorErrors()),
        ("yaw noise 0.5deg", SensorErrors(imu_noise_rad=_deg(_IMU_NOISE_0_5DEG))),
        ("yaw noise 1deg", SensorErrors(imu_noise_rad=_deg(_IMU_NOISE_1_0DEG))),
        ("yaw noise 2deg", SensorErrors(imu_noise_rad=_deg(_IMU_NOISE_2_0DEG))),
    ],
    # Anchored on the BNO085 in UART-RVC mode: 6-axis fusion, so the drift term
    # is the datasheet's 0.5 deg/min (0.0083 deg/s) rather than a guess, and the
    # bias term is placement error, since RVC yaw is relative to power-on with
    # no absolute reference to correct it.
    "combined": [
        ("ideal", SensorErrors()),
        (
            "bno085 at spec",
            SensorErrors(
                start_pos_error_m=0.02,
                yaw_bias_rad=_deg(2),
                imu_drift_rad_per_s=_deg(0.0083),
                gyro_scale_error=0.0025,
                imu_noise_rad=_deg(0.5),
            ),
        ),
        (
            "realistic",
            SensorErrors(
                start_pos_error_m=0.05,
                yaw_bias_rad=_deg(3),
                imu_drift_rad_per_s=_deg(0.02),
                gyro_scale_error=0.005,
                imu_noise_rad=_deg(1.0),
            ),
        ),
        (
            "pessimistic",
            SensorErrors(
                start_pos_error_m=0.10,
                yaw_bias_rad=_deg(5),
                imu_drift_rad_per_s=_deg(0.05),
                gyro_scale_error=0.01,
                imu_noise_rad=_deg(2.0),
            ),
        ),
    ],
}


def report_perturbed(workers: int, sweep: str, verbose: bool) -> None:
    """Blind navigation when the robot is also unsure of its own pose.

    ``blind`` withholds the track. This withholds the robot's own starting pose
    and lets its heading drift, which is the harder of the two for blind mode:
    width readings are attributed to a corridor by heading, so yaw error can
    file a measurement under the wrong corridor entirely.
    """
    count = len(all_test_scenarios())
    names = list(_SWEEPS) if sweep == "all" else [sweep]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for name in names:
            print(f"\n== {name.upper()} ==  ({count} blind Open Challenge fixtures)", flush=True)
            print(
                f"{'configuration':<22} {'pass':>7} {'layout':>7} {'collide':>8} "
                f"{'peak_pos':>9} {'final_pos':>10} {'peak_yaw':>9}",
                flush=True,
            )
            for label, errors in _SWEEPS[name]:
                runs = list(pool.map(_run_perturbed, [(i, errors) for i in range(count)]))
                passed = sum(1 for r in runs if r.passed)
                layout = sum(1 for r in runs if r.layout_ok)
                collided = sum(1 for r in runs if r.collided)
                print(
                    f"{label:<22} {f'{passed}/{count}':>7} {f'{layout}/{count}':>7} "
                    f"{f'{collided}/{count}':>8} "
                    f"{max(r.peak_pos_err for r in runs) * 100:8.1f}cm "
                    f"{max(r.final_pos_err for r in runs) * 100:9.1f}cm "
                    f"{math.degrees(max(r.peak_heading_err for r in runs)):8.1f}d",
                    flush=True,
                )
                if verbose:
                    for r in runs:
                        if not (r.passed and r.layout_ok):
                            print(
                                f"     {r.label:<32} pass={r.passed} laps={r.laps}/3 "
                                f"collided={r.collided} layout_ok={r.layout_ok} "
                                f"pos_err={r.peak_pos_err * 100:.1f}cm",
                                flush=True,
                            )


@dataclass(frozen=True, slots=True)
class _OpenFixtureResult:
    """Result from running an Open Challenge fixture."""

    label: str
    within_time: bool
    laps_completed: int
    collided: bool
    peak_pos_err: float


def _run_open_fixture(args: tuple[int, bool]) -> _OpenFixtureResult:
    index, localize = args
    scenario = all_test_scenarios()[index]
    sim = ScenarioSimulator(
        scenario.metadata,
        num_laps=scenario.laps,
        seed=scenario.seed,
        use_lidar_localization=localize,
    )
    peak = [0.0]

    def on_step(_state: object, _scan: object) -> None:
        peak[0] = max(peak[0], sim.gateway.position_error_m)

    result = sim.run(on_step=on_step)
    within = result.success and result.sim_time_s <= CompetitionSpecs.ROUND_TIME_LIMIT_S
    return _OpenFixtureResult(
        label=scenario.label,
        within_time=within,
        laps_completed=result.laps_completed,
        collided=result.collided,
        peak_pos_err=peak[0],
    )


def report_open_fixtures(workers: int) -> None:
    """The 28 Go-generated Open Challenge fixtures — the centre-wall variations.

    These are the layouts ``sim:navigate:visualize --challenge open`` steps
    through, and no test drives them: ``test_scenario_catalog.py`` only asserts
    there are 28 and that label lookup works, while the closed-loop Open tests
    build their own symmetric/mixed layouts instead. So this is the only place
    the generated wall variations are actually run.
    """
    count = len(all_test_scenarios())
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for localize in (False, True):
            results = list(pool.map(_run_open_fixture, [(i, localize) for i in range(count)]))
            passed = sum(1 for r in results if r.within_time)
            collided = sum(1 for r in results if r.collided)
            peak = max(r.peak_pos_err for r in results)
            pose = "LIDAR-estimated" if localize else "ground truth   "
            print(
                f"OPEN-FIXTURES pose={pose}  pass {passed}/{count}  "
                f"collided {collided}/{count}  peak_pos_err {peak * 100:5.1f}cm",
                flush=True,
            )
            for r in results:
                if not r.within_time:
                    print(
                        f"    FAIL {r.label:<34} laps={r.laps_completed}/3 collided={r.collided} pos_err={r.peak_pos_err * 100:.1f}cm",
                        flush=True,
                    )


def main() -> None:
    """Run the requested comparison."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=[m.value for m in DiagMode])
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument(
        "--sweep",
        choices=[*_SWEEPS, "all"],
        default="combined",
        help="Which error axis to sweep in 'perturbed' mode (default: combined).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="List the individual failing fixtures under each configuration.",
    )
    args = parser.parse_args()
    mode = DiagMode(args.mode)
    if mode == DiagMode.OPEN:
        report_open(args.workers)
    elif mode == DiagMode.OPEN_FIXTURES:
        report_open_fixtures(args.workers)
    elif mode == DiagMode.BLIND:
        report_blind(args.workers)
    elif mode == DiagMode.PERTURBED:
        report_perturbed(args.workers, args.sweep, args.verbose)
    else:
        report_obstacles(args.workers)


if __name__ == "__main__":
    main()
