"""Measure how far ``AckermannKinematics`` is from the chassis it claims to model.

Every Open Challenge sweep is scored against a simulator whose motion model was
written from first principles and never checked against a bag. This asks the
three questions that model can be wrong about, using a recorded run as ground
truth:

  * ``--yaw`` -- does the chassis achieve the yaw rate the model predicts from
    the commanded wheel angle? Bucketed by steering magnitude, because the
    model's ``tan(steer)`` term means a scalar error and a large-angle
    saturation look identical at any single angle. Reports the wheel angle the
    chassis *behaved* as if it had, which is the readable form of the answer.
  * ``--speed`` -- does the drivetrain deliver the commanded speed, and does it
    get there the way ``max_accel`` says? Steady-state gain, then the step
    response behind it (achieved acceleration and a first-order time constant).
  * ``--replay`` -- drive the real command stream through the actual
    :class:`AckermannKinematics` and compare integrated yaw against the IMU.
    This is the only section that tests the model as assembled, slew limit and
    all, rather than one term of it; it also solves for the ``rear_steer_ratio``
    that would make total yaw match.

What this deliberately does NOT measure: the servo slew rate
(``MAX_STEERING_RATE``). ``/motor/steering_position`` on a servo chassis is the
command echoed back, not a position sensor, so a bag contains no evidence about
how fast the wheels actually moved. That constant needs a bench test.

Ground truth is the IMU quaternion, differentiated. Speed is the encoder, which
became trustworthy once ``counts_per_rev`` was corrected on 2026-08-29 -- on
older bags the drivetrain figures here read ~1.4x high.

Usage:
    pixi run -e dev python scripts/bag/diag_bag_sim_fidelity.py RUN_DIR
    pixi run -e dev python scripts/bag/diag_bag_sim_fidelity.py RUN_DIR --yaw
    pixi run -e dev python scripts/bag/diag_bag_sim_fidelity.py RUN_DIR --replay
"""

from __future__ import annotations

import math
import sys
from itertools import pairwise
from pathlib import Path
from typing import TYPE_CHECKING, NamedTuple

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import RobotSpecs

from scripts.common.bag_io import create_bag_parser, read_motion_streams
from scripts.common.stats import median, nearest_by_time, percentile
from scripts.common.tables import print_table
from src.navigation.utils import wrap_angle
from src.simulation.kinematics import AckermannKinematics, AckermannState

if TYPE_CHECKING:
    from collections.abc import Sequence

    from scripts.common.bag_io import MotionStreams

_YAW_WINDOW_S = 0.20
"""Differentiation window for the IMU yaw. Long enough that quaternion noise
averages out, short enough that a corner is not smeared into the straight."""
_PAIR_TOL_S = 0.15
_MIN_SPEED_MPS = 0.05
"""Below this the chassis is not really moving and yaw per metre is undefined."""
_MIN_STEER_DEG = 2.0

_STEER_BUCKET_EDGES_DEG = (2.0, 5.0, 10.0, 20.0, 30.0, 45.0, 90.0)
_MIN_BUCKET = 30
"""Fewest samples a bucket needs before its median says anything."""

_SETTLE_S = 0.75
"""How long a commanded speed must hold before the drivetrain is judged settled."""
_STEP_MIN_MPS = 0.05
"""Smallest command change worth reading a step response out of."""
_STEP_WINDOW_S = 2.0
_RISE_LOW = 0.1
_RISE_HIGH = 0.9
_TAU_FRACTION = 0.632
"""A first-order lag reaches this fraction of its step in one time constant."""

_RATIO_SOLVE_ITERATIONS = 8

_SAME_COMMAND_MPS = 1e-6
"""Commands are floats off a wire; anything closer than this is the same value."""
_COMMAND_BUCKET_MPS = 1e-3
"""Width of a commanded-speed bucket in the steady-state table."""
_MIN_STEP_SAMPLES = 3
"""Fewest achieved-speed samples a step needs before a rise can be read from it."""


class YawSample(NamedTuple):
    """One IMU window: what was asked of the chassis, and what it did."""

    t: float
    cmd_steer_rad: float
    speed_mps: float
    achieved_yaw_rate: float


def _yaw_samples(streams: MotionStreams) -> list[YawSample]:
    """Pair each IMU differentiation window with the command and speed in force.

    Speed comes from the encoder rather than the command so that a drivetrain
    that undershoots does not get charged to the steering geometry -- the two
    failures are separable and this section is only about the second.
    """
    imu = streams.imu_yaw_rad
    if not imu:
        return []
    speeds = streams.drive_speed_mps()
    steer_times = [t for t, _ in streams.cmd_steer_rad]
    speed_times = [t for t, _ in speeds]

    step = max(1, int(len(imu) * _YAW_WINDOW_S / max(imu[-1][0], 1e-6)))
    out: list[YawSample] = []
    for i in range(len(imu) - step):
        t_a, yaw_a = imu[i]
        t_b, yaw_b = imu[i + step]
        dt = t_b - t_a
        if not 0.5 * _YAW_WINDOW_S <= dt <= 2.0 * _YAW_WINDOW_S:
            continue
        mid = 0.5 * (t_a + t_b)
        steer = nearest_by_time(streams.cmd_steer_rad, steer_times, mid, tolerance=_PAIR_TOL_S)
        speed = nearest_by_time(speeds, speed_times, mid, tolerance=_PAIR_TOL_S)
        if steer is None or speed is None:
            continue
        if abs(speed) < _MIN_SPEED_MPS or abs(math.degrees(steer)) < _MIN_STEER_DEG:
            continue
        out.append(YawSample(mid, steer, speed, wrap_angle(yaw_b - yaw_a) / dt))
    return out


def _model_yaw_rate(speed: float, steer_rad: float, turn_reference_len: float) -> float:
    """The yaw rate ``AckermannKinematics`` integrates for this command."""
    return speed / turn_reference_len * math.tan(steer_rad)


def _print_yaw(samples: Sequence[YawSample], turn_reference_len: float) -> None:
    """Achieved vs modelled yaw rate, bucketed by commanded steering magnitude.

    Bucketing is the point. A single pooled ratio cannot tell a wrong
    ``rear_steer_ratio`` (a constant factor at every angle) from a chassis that
    simply cannot deliver what ``tan()`` promises near full lock (a ratio that
    collapses as the angle grows). The ``as-if deg`` column inverts the model:
    the wheel angle that WOULD have produced the observed yaw at the modelled
    geometry.
    """
    print(f"\n--- yaw gain (n={len(samples)}, turn reference {turn_reference_len:.4f} m) ---")
    print("  modelled = speed / L_eff * tan(commanded wheel angle), the sim's own formula\n")
    rows = []
    for low, high in pairwise(_STEER_BUCKET_EDGES_DEG):
        bucket = [s for s in samples if low <= abs(math.degrees(s.cmd_steer_rad)) < high]
        if len(bucket) < _MIN_BUCKET:
            continue
        cmd_deg = median([abs(math.degrees(s.cmd_steer_rad)) for s in bucket])
        speed = median([abs(s.speed_mps) for s in bucket])
        achieved = median([abs(s.achieved_yaw_rate) for s in bucket])
        modelled = median([abs(_model_yaw_rate(s.speed_mps, s.cmd_steer_rad, turn_reference_len)) for s in bucket])
        ratio = achieved / modelled if modelled else math.nan
        # Invert the model at the bucket's median speed: what angle explains the
        # yaw actually seen? atan keeps this finite where the ratio does not.
        as_if = math.degrees(math.atan(achieved * turn_reference_len / speed)) if speed else math.nan
        rows.append([
            f"{low:.0f}-{high:.0f}",
            len(bucket),
            f"{cmd_deg:.1f}",
            f"{speed:.3f}",
            f"{achieved:.2f}",
            f"{modelled:.2f}",
            f"{ratio:.2f}",
            f"{as_if:.1f}",
        ])
    print_table(
        rows,
        ["cmd deg", "n", "med cmd", "speed", "achieved", "modelled", "ratio", "as-if deg"],
    )
    if not rows:
        print("  no bucket reached the minimum sample count")


def _steady_speed_pairs(streams: MotionStreams) -> list[tuple[float, float]]:
    """``(commanded, achieved)`` for ticks where the command has held steady.

    Transients are excluded rather than averaged in: a bag is mostly
    acceleration, and including it would report the drivetrain's lag as if it
    were a steady-state gain error.
    """
    cmds = streams.cmd_speed_mps
    achieved = streams.drive_speed_mps()
    achieved_times = [t for t, _ in achieved]
    out: list[tuple[float, float]] = []
    hold_since = cmds[0][0] if cmds else 0.0
    for i, (t, cmd) in enumerate(cmds):
        if i and abs(cmd - cmds[i - 1][1]) > _SAME_COMMAND_MPS:
            hold_since = t
            continue
        if t - hold_since < _SETTLE_S or abs(cmd) < _MIN_SPEED_MPS:
            continue
        got = nearest_by_time(achieved, achieved_times, t, tolerance=_PAIR_TOL_S)
        if got is not None:
            out.append((cmd, got))
    return out


class StepResponse(NamedTuple):
    """One commanded speed step and what the drivetrain did with it."""

    magnitude: float
    accel_mps2: float | None
    tau_s: float | None


def _step_responses(streams: MotionStreams) -> list[StepResponse]:
    """Rise rate and first-order time constant for each commanded speed step-up.

    Only step-UPS are read. A step down is dominated by rolling resistance and
    the H-bridge's coast behaviour, which ``max_accel`` -- a single symmetric
    clamp -- does not claim to model either way.
    """
    cmds = streams.cmd_speed_mps
    achieved = streams.drive_speed_mps()
    achieved_times = [t for t, _ in achieved]
    out: list[StepResponse] = []
    for i in range(1, len(cmds)):
        t, cmd = cmds[i]
        prev = cmds[i - 1][1]
        if cmd - prev < _STEP_MIN_MPS:
            continue
        start = nearest_by_time(achieved, achieved_times, t, tolerance=_PAIR_TOL_S)
        if start is None:
            continue
        window = [(ts, v) for ts, v in achieved if t <= ts <= t + _STEP_WINDOW_S]
        # A step is only readable if the command survives long enough to settle;
        # the navigator re-commands at 20 Hz, so most "steps" are immediately
        # superseded and must be dropped rather than read as a failure to track.
        held = next((ts for ts, c in cmds[i + 1 :] if abs(c - cmd) > _SAME_COMMAND_MPS), t + _STEP_WINDOW_S)
        window = [(ts, v) for ts, v in window if ts <= held]
        if len(window) < _MIN_STEP_SAMPLES:
            continue
        span = cmd - start
        t_low = next((ts for ts, v in window if v >= start + _RISE_LOW * span), None)
        t_high = next((ts for ts, v in window if v >= start + _RISE_HIGH * span), None)
        t_tau = next((ts for ts, v in window if v >= start + _TAU_FRACTION * span), None)
        accel = (_RISE_HIGH - _RISE_LOW) * span / (t_high - t_low) if t_low and t_high and t_high > t_low else None
        out.append(StepResponse(span, accel, (t_tau - t) if t_tau else None))
    return out


def _print_speed(streams: MotionStreams) -> None:
    """Steady-state drivetrain gain, then the transient the sim models as a clamp."""
    pairs = _steady_speed_pairs(streams)
    print(f"\n--- drivetrain (n={len(pairs)} settled ticks) ---")
    if not pairs:
        print("  no settled ticks -- the command never held still long enough")
    else:
        ratios = [got / cmd for cmd, got in pairs if cmd]
        print(f"  achieved/commanded  med={median(ratios):.3f}  p10={percentile(ratios, 0.1):.3f}  p90={percentile(ratios, 0.9):.3f}")
        rows = []
        for cmd in sorted({round(c, 3) for c, _ in pairs}):
            bucket = [got for c, got in pairs if abs(c - cmd) < _COMMAND_BUCKET_MPS]
            if len(bucket) < _MIN_BUCKET:
                continue
            rows.append([f"{cmd:.3f}", len(bucket), f"{median(bucket):.3f}", f"{median(bucket) / cmd:.3f}"])
        print_table(rows, ["commanded", "n", "med achieved", "ratio"])

    steps = [s for s in _step_responses(streams) if s.accel_mps2 is not None]
    print(f"\n--- step response (n={len(steps)} readable step-ups) ---")
    if not steps:
        print("  no commanded step survived long enough to read a rise from")
        return
    accels = [s.accel_mps2 for s in steps if s.accel_mps2 is not None]
    taus = [s.tau_s for s in steps if s.tau_s is not None]
    print(f"  achieved accel   med={median(accels):.2f}  p90={percentile(accels, 0.9):.2f} m/s^2")
    print(f"  sim max_accel    {RobotSpecs.MAX_ACCEL_MPS2:.2f} m/s^2")
    if taus:
        print(f"  time to {_TAU_FRACTION:.0%} of step  med={median(taus):.3f} s")
    # Which model the drivetrain wants is readable here and nowhere else: under a
    # constant-acceleration clamp the accel column is flat and tau grows with the
    # step; under a first-order lag tau is flat and accel grows with the step.
    print_table(
        [[f"{s.magnitude:.3f}", f"{s.accel_mps2:.2f}", f"{s.tau_s:.3f}" if s.tau_s else "-"] for s in sorted(steps, key=lambda s: s.magnitude)],
        ["step m/s", "accel m/s^2", "tau s"],
    )


def _replay(streams: MotionStreams, kinematics: AckermannKinematics) -> tuple[float, float, float]:
    """Integrate the recorded command stream through the sim's own model.

    Returns ``(simulated_abs_yaw, measured_abs_yaw, final_heading_error_rad)``.

    Driven with the MEASURED speed, not the commanded one, so drivetrain error
    stays in the ``--speed`` section instead of leaking into the yaw verdict.
    Total *absolute* yaw is the comparison rather than final heading because
    left and right errors cancel in a closed lap and would report a badly wrong
    model as a good one.
    """
    speeds = streams.drive_speed_mps()
    speed_times = [t for t, _ in speeds]
    imu = streams.imu_yaw_rad
    imu_times = [t for t, _ in imu]

    state = AckermannState(x=0.0, y=0.0, yaw=imu[0][1] if imu else 0.0)
    sim_abs = 0.0
    measured_abs = 0.0
    prev_measured = imu[0][1] if imu else 0.0
    max_steer = RobotSpecs.MAX_STEERING_ANGLE
    for i in range(1, len(streams.cmd_steer_rad)):
        t, steer = streams.cmd_steer_rad[i]
        dt = t - streams.cmd_steer_rad[i - 1][0]
        if not 0.0 < dt < 1.0:
            continue
        speed = nearest_by_time(speeds, speed_times, t, tolerance=_PAIR_TOL_S)
        measured = nearest_by_time(imu, imu_times, t, tolerance=_PAIR_TOL_S)
        if speed is None or measured is None:
            continue
        before = state.yaw
        state = kinematics.step(state, speed, steer / max_steer, dt)
        sim_abs += abs(wrap_angle(state.yaw - before))
        measured_abs += abs(wrap_angle(measured - prev_measured))
        prev_measured = measured
    return sim_abs, measured_abs, wrap_angle(state.yaw - prev_measured)


def _print_replay(streams: MotionStreams) -> None:
    """Run the sim's model over the real commands, then solve for the gain that fits.

    ``yaw_gain`` is the free parameter rather than ``rear_steer_ratio`` because
    the rear axle is known to steer counter-phase -- the geometry is not in
    doubt, the zero-slip assumption on top of it is. The two are
    indistinguishable from a bag (both scale the yaw rate linearly), so which
    one to move is a question about the chassis, not about the data.

    That linearity is what makes the solve converge in a handful of passes:
    each one rescales the gain by however far the total yaw still misses.
    """
    print("\n--- open-loop replay of the recorded commands through AckermannKinematics ---")
    shipped = AckermannKinematics()
    sim_abs, measured_abs, heading_err = _replay(streams, shipped)
    if measured_abs <= 0.0:
        print("  no measured yaw in this bag")
        return
    print(f"  shipped: yaw_gain {RobotSpecs.YAW_GAIN:.2f}, rear_steer_ratio {RobotSpecs.REAR_STEER_RATIO:.2f}, speed_tau {RobotSpecs.SPEED_RESPONSE_TAU_S:.2f}s")
    print(f"  total yaw travelled: sim {math.degrees(sim_abs):8.0f} deg   measured {math.degrees(measured_abs):8.0f} deg   sim/real {sim_abs / measured_abs:.2f}x")
    print(f"  final heading error: {math.degrees(heading_err):+.0f} deg")

    gain = RobotSpecs.YAW_GAIN
    for _ in range(_RATIO_SOLVE_ITERATIONS):
        sim_abs, measured_abs, _ = _replay(streams, AckermannKinematics(yaw_gain=gain))
        if not sim_abs:
            break
        gain *= measured_abs / sim_abs
    sim_abs, measured_abs, heading_err = _replay(streams, AckermannKinematics(yaw_gain=gain))
    print(f"\n  best-fit yaw_gain {gain:.2f}  ->  sim/real {sim_abs / measured_abs:.2f}x, final heading error {math.degrees(heading_err):+.0f} deg")
    print("  A gain below 1.0 is tyre slip and linkage compliance the zero-slip model")
    print("  has no term for. It is specific to these tyres on this surface, so it is")
    print("  re-measured after a tyre, weight or mat change -- not a universal constant.")


def main() -> None:
    """Parse CLI args and print whichever fidelity sections were asked for."""
    parser = create_bag_parser(
        "Compare a recorded run against the simulator's AckermannKinematics: yaw gain "
        "vs commanded steering, drivetrain tracking, and an open-loop replay.",
    )
    parser.add_argument("--yaw", action="store_true", help="achieved vs modelled yaw rate, bucketed by steering angle")
    parser.add_argument("--speed", action="store_true", help="drivetrain steady-state gain and step response")
    parser.add_argument("--replay", action="store_true", help="integrate the real commands through the sim's model")
    args = parser.parse_args()

    everything = not (args.yaw or args.speed or args.replay)
    streams = read_motion_streams(args.bag_dir)
    print(
        f"{args.bag_dir.name}: {len(streams.imu_yaw_rad)} imu, {len(streams.cmd_steer_rad)} cmd, "
        f"{len(streams.drive_speed_dps)} encoder samples",
    )

    if everything or args.yaw:
        _print_yaw(_yaw_samples(streams), RobotSpecs.WHEELBASE / (1.0 + abs(RobotSpecs.REAR_STEER_RATIO)))
    if everything or args.speed:
        _print_speed(streams)
    if everything or args.replay:
        _print_replay(streams)


if __name__ == "__main__":
    main()
