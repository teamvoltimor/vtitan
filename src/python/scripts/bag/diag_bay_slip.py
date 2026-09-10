r"""What turn radius does the chassis ACHIEVE while ratcheting out of the pocket?

Every bay-exit A/B run in the simulator is currently scoring the simulator. The
mirrored-reverse arm looked like an 11x win until the same run with
``--no-slide`` collapsed it to below the held lock, which put ~95% of the
result in the slide-on-contact resolver rather than in the manoeuvre. So the
next number has to come from hardware, and this is it.

The quantity is the EFFECTIVE TURN RADIUS, ``|ds| / |dpsi|``: how much wheel
travel the chassis spent per radian it actually turned. Against it stands the
model's radius, the plain bicycle term floored by
``RobotSpecs.MIN_TURN_RADIUS_M`` exactly as ``AckermannKinematics`` floors it.
A ratio below 1 means the chassis rotated TIGHTER than free-space kinematics
permits.

That was first read as the wall, and it is mostly NOT. THE FLOOR IS NOT A
CONSTANT: measured in free space at full lock, the achieved radius rises with
SPEED and then saturates --

    mean speed m/s   0.025  0.079  0.132  0.168  0.227  0.270  0.324
    R achieved m     0.105  0.202  0.298  0.391  0.412  0.445  0.429

-- so ``min(0.43, 0.055 + 2.0 * v)`` describes it and the shipped constant
0.29 m is simply the value at ONE speed, 0.118 m/s. The pocket adds a further
1.3-1.7x on top of that, and only THAT part is the wall.

Three corrections are baked into those numbers, each of which moved them by
more than the effect being measured. Every one was found by testing the
INSTRUMENT, not the chassis.

1. MATCH THE OPERATING POINT. The pocket is also where the wheel sits at full
   lock and crawls, so an unmatched pooled ratio cannot tell the surroundings
   from the speed. Hence ``--control-phase`` and the paired table.
2. TAKE A NET DELTA OVER A WINDOW. ``|yaw|`` summed at the IMU's ~166 Hz
   ACCUMULATES noise where a signed sum cancels it, and reported the corridor
   at creep as 0.088 m against the converged 0.16-0.20. The result is stable
   from ``--window-s`` 0.02 to 0.20 and only the raw rate disagrees; a reading
   that moves with its own window is not converged.
3. INTEGRATE THE SPEED SERIES. Holding one sample across the window credits a
   stop-start creep with the speed it happened to open on, which is exactly the
   low-speed end of the curve. See ``_travel_between``.

Why the IMU and not the pose: ``quaternion_yaw``'s docstring says ``pose_yaw``
is localizer-fused and damped, and MEASURED here on one time base it sees 0.66x
the rotation the IMU does -- so a radius taken against it is ~1.5x overstated.
The shipped 0.29 m was taken against it (see ``kinematics.py``).

Why speed and not odometry distance: ``travelled_m`` is SIGNED and a ratchet
alternates, so it cancels -- the recorded reason a previous bay measurement read
1 cm of progress across 539 legs.

Usage (from ``src``)::

    VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm \\
    PYTHONPATH=".;../shared/src" pixi run -e dev python \\
        scripts/bag/diag_bay_slip.py ../../data/live/runs/run_202609*
"""

from __future__ import annotations

import math
import statistics
import sys
from bisect import bisect_left
from itertools import pairwise
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import RobotSpecs

from scripts.common.bag_io import (
    create_bags_parser,
    load_nav_debug_rows,
    read_motion_streams,
)
from scripts.common.tables import print_table

BAY_PHASE = "bay_exit"

_MIN_TRAVEL_M = 2e-4
"""Wheel travel below which a sample carries no radius at all.

0.2 mm. Dividing a stalled tick's travel by its yaw noise manufactures radii of
millimetres, and 68% of bay ticks are stalled -- measured, the wheel moves only
for ~0.5 s after each reversal. Those ticks are not evidence about geometry."""

_MIN_YAW_RAD = math.radians(0.05)
"""Yaw below which the ratio is noise over noise, so the sample is dropped."""

_EFFECTIVE_WHEELBASE_M = RobotSpecs.WHEELBASE / (1.0 + RobotSpecs.REAR_STEER_RATIO)
"""Same counter-phase wheelbase ``bay_exit`` and the kinematics both turn about."""


def _sample_at(series: list[tuple[float, float]], t: float) -> float | None:
    """Value of a ``(time, value)`` series at ``t``, held from the last sample.

    Held rather than interpolated: ``cmd_steer_rad`` is a step command and the
    phase stream is a state, so a linear blend between two samples would invent
    an angle that was never asked for.
    """
    if not series:
        return None
    i = bisect_left(series, (t, -math.inf))
    if i == 0:
        return None
    return series[i - 1][1]


def _model_radius_m(wheel_rad: float) -> float:
    """Turn radius free-space kinematics allows at this wheel angle.

    Floored by ``MIN_TURN_RADIUS_M``, which is the measured chassis saturation
    (`72e7172b`) and the thing the achieved radius is being tested against.
    """
    if abs(wheel_rad) < 1e-6:
        return math.inf
    return max(RobotSpecs.MIN_TURN_RADIUS_M, _EFFECTIVE_WHEELBASE_M / abs(math.tan(wheel_rad)))


def _phase_windows(rows: list[tuple[float, object]], phase: str) -> list[tuple[float, float]]:
    """Contiguous elapsed-time spans the run spent in ``phase``.

    A list rather than one span because the CONTROL phase is re-entered many
    times per run, and taking its first and last sample would swallow every
    other phase in between -- including the bay exit this is a control for.
    """
    spans: list[tuple[float, float]] = []
    start: float | None = None
    previous = 0.0
    for t, snapshot in rows:
        if getattr(snapshot, "phase", None) == phase:
            if start is None:
                start = t
        elif start is not None:
            spans.append((start, previous))
            start = None
        previous = t
    if start is not None:
        spans.append((start, previous))
    return [(a, b) for a, b in spans if b > a]


_SETTLED_TOLERANCE_RAD = math.radians(5.0)
"""How far the command may have moved over the settle window and still count.

NOT zero: pure pursuit rewrites the angle every control tick, so an exact-match
test admits nothing at all. What matters physically is whether the command is
MOVING FAST -- the servo slews at a finite rate and its feedback is the command
echoed back, so a fast-moving command means the wheel is somewhere behind it and
the achieved radius reads wider than the lock implies."""


def _command_settled(commands: list[tuple[float, float]], t: float, settled_s: float) -> bool:
    """Whether the steering command stayed within a few degrees of its value at ``t``."""
    now = _sample_at(commands, t)
    if now is None:
        return False
    return all(
        abs(v - now) <= _SETTLED_TOLERANCE_RAD for a, v in commands if t - settled_s <= a <= t
    )


def _travel_between(speeds: list[tuple[float, float]], t0: float, t1: float) -> float:
    """Wheel distance covered over ``[t0, t1]``, integrating the speed series.

    Holding one sample across the window instead biases exactly where it hurts:
    the bay creeps in stop-start bursts -- 88% of its ticks have the wheel
    stopped -- so a window opened on a moving sample credits the whole window
    with that speed and the low-speed end of the curve is invented.
    """
    total = 0.0
    for (a, v), (b, _) in pairwise(speeds):
        lo, hi = max(a, t0), min(b, t1)
        if hi > lo:
            total += abs(v) * (hi - lo)
    return total


def _windows(series: list[tuple[float, float]], window_s: float) -> list[tuple[int, int]]:
    """Index pairs spanning at least ``window_s`` of ``series``, non-overlapping.

    Zero or less gives back consecutive pairs, i.e. the raw sample rate.
    """
    if window_s <= 0.0:
        return [(i, i + 1) for i in range(len(series) - 1)]
    out: list[tuple[int, int]] = []
    start = 0
    for i in range(1, len(series)):
        if series[i][0] - series[start][0] >= window_s:
            out.append((start, i))
            start = i
    return out


def _in_any(spans: list[tuple[float, float]], t0: float, t1: float) -> bool:
    """Whether the whole interval falls inside one of ``spans``."""
    return any(a <= t0 and t1 <= b for a, b in spans)


def _run_samples(
    bag_dir: Path, phase: str = BAY_PHASE, window_s: float = 0.0, settled_s: float = 0.0
) -> tuple[list[tuple[float, float, float, float, float, float]], float, float, float, tuple[float, float, float]]:
    """Samples plus the STALLED-tick yaw control.

    Returns ``(samples, stalled_yaw_rad, stalled_s, driven_s)`` for one IMU
    interval inside ``phase``.

    The control exists because the radius sums ``|yaw|``, and an absolute value
    ACCUMULATES sensor noise where a signed sum would cancel it -- so a
    sufficiently noisy IMU manufactures rotation and reports an arbitrarily
    tight radius. Ticks whose wheel did not move are the same sensor over the
    same interval with the true rotation near zero, which makes them a direct
    read of that noise floor rather than an assumption about it.
    """
    rows, _ = load_nav_debug_rows(bag_dir)
    spans = _phase_windows(rows, phase)
    if not spans:
        return [], 0.0, 0.0, 0.0, (0.0, 0.0, 0.0)

    streams = read_motion_streams(bag_dir)
    speeds = streams.drive_speed_mps()
    posed = [(t, s.pose_yaw) for t, s in rows if isinstance(getattr(s, "pose_yaw", None), (int, float))]
    samples: list[tuple[float, ...]] = []
    stalled_yaw = 0.0
    stalled_s = 0.0
    driven_s = 0.0
    yaw = streams.imu_yaw_rad
    for i0, i1 in _windows(yaw, window_s):
        t0, y0 = yaw[i0]
        t1, y1 = yaw[i1]
        if not _in_any(spans, t0, t1):
            continue
        dt = t1 - t0
        if dt <= 0.0:
            continue
        speed = _sample_at(speeds, t0)
        wheel = _sample_at(streams.cmd_steer_rad, t0)
        if speed is None or wheel is None:
            continue
        # The servo SLEWS, and `steer_pos_deg` is the command echoed back
        # (`MotionStreams`), so the real wheel angle during a transient is not
        # recorded anywhere. A window opened just after a steering change is
        # therefore driving at LESS lock than the command says, which inflates
        # the radius. `--settled-s` drops those; if the curve moves when it is
        # raised, the curve was reading servo lag rather than the chassis.
        if settled_s > 0.0 and not _command_settled(streams.cmd_steer_rad, t0, settled_s):
            continue
        travel = _travel_between(speeds, t0, t1)
        # NET delta across the window, not the sum of the sub-steps inside it.
        # |yaw| summed at the IMU's ~166 Hz accumulates noise instead of
        # cancelling it, which understates the radius by ~19% measured against
        # the same windows scored net. The window is what makes the two differ.
        step = (y1 - y0 + math.pi) % (2.0 * math.pi) - math.pi
        if travel < _MIN_TRAVEL_M:
            stalled_yaw += abs(step)
            stalled_s += dt
            continue
        if abs(step) < _MIN_YAW_RAD:
            continue
        driven_s += dt
        samples.append((travel, abs(step), _model_radius_m(wheel), abs(wheel), abs(speed)))

    # The 0.29 m floor in `kinematics.py` was measured against POSE yaw, and
    # `quaternion_yaw` says that signal is damped. Testing that claim needs ONE
    # TIME BASE for both: the pose stream runs at ~11 Hz against the IMU's ~166,
    # so scoring a pose delta against an IMU interval's travel divides a long
    # window's rotation by a short window's distance and invents a radius.
    # Here both are NET deltas over the same pose-to-pose window, which also
    # keeps |yaw| from accumulating noise at different rates on the two signals.
    pose_travel = pose_rot = pose_imu_rot = 0.0
    for (t0, p0), (t1, p1) in pairwise(posed):
        if not _in_any(spans, t0, t1) or t1 <= t0:
            continue
        i0, i1 = _sample_at(yaw, t0), _sample_at(yaw, t1)
        if i0 is None or i1 is None:
            continue
        travelled = _travel_between(speeds, t0, t1)
        if travelled < _MIN_TRAVEL_M:
            continue
        pose_travel += travelled
        pose_rot += abs((p1 - p0 + math.pi) % (2.0 * math.pi) - math.pi)
        pose_imu_rot += abs((i1 - i0 + math.pi) % (2.0 * math.pi) - math.pi)

    return samples, stalled_yaw, stalled_s, driven_s, (pose_travel, pose_rot, pose_imu_rot)


_LOCK_EDGES_DEG = (10.0, 20.0, 30.0)
"""Commanded |wheel angle| buckets. Full lock on this chassis is ~35 deg."""

_SPEED_EDGES_MPS = (0.10, 0.20, 0.30, 0.40)
"""Wheel-speed buckets. Bay creep is commanded at 0.10 m/s.

Resolved this finely because the achieved radius turned out to depend on SPEED
and not only on the surroundings: at full lock the corridor itself reaches
0.108 m at creep, well inside the 0.29 m floor, so the floor is a statement
about the speed it was measured at rather than about the chassis."""


def _bucket(value: float, edges: tuple[float, ...]) -> int:
    """Index of the bucket ``value`` falls in, ``len(edges)`` for the top one."""
    return sum(1 for edge in edges if value >= edge)


def _cell(sample: tuple[float, ...]) -> tuple[int, int]:
    """``(lock bucket, speed bucket)`` a sample belongs to."""
    return _bucket(math.degrees(sample[3]), _LOCK_EDGES_DEG), _bucket(sample[4], _SPEED_EDGES_MPS)


def _paired_table(
    pocket: list[tuple[float, ...]],
    corridor: list[tuple[float, ...]],
) -> None:
    """Achieved radius IN the pocket against OUT of it, at matched lock and speed.

    The whole objection this answers: a tighter radius in the pocket proves
    nothing about the wall if the pocket is also where the wheel sits at full
    lock and crawls. Comparing only WITHIN a (lock, speed) cell removes both,
    so what is left is the difference the surroundings make. Cells either side
    is not enough on its own -- a cell with three intervals is noise -- so the
    interval count is printed and thin cells are meant to be ignored.
    """
    cells = sorted({_cell(x) for x in pocket} & {_cell(x) for x in corridor})
    lock_names = [f"<{_LOCK_EDGES_DEG[0]:.0f}"] + [f"{e:.0f}+" for e in _LOCK_EDGES_DEG]
    speed_names = [f"<{_SPEED_EDGES_MPS[0]:.2f}"] + [f"{e:.2f}+" for e in _SPEED_EDGES_MPS]
    rows = []
    for lock, speed in cells:
        inside = [x for x in pocket if _cell(x) == (lock, speed)]
        outside = [x for x in corridor if _cell(x) == (lock, speed)]
        r_in = sum(x[0] for x in inside) / sum(x[1] for x in inside)
        r_out = sum(x[0] for x in outside) / sum(x[1] for x in outside)
        rows.append(
            (
                lock_names[lock],
                speed_names[speed],
                len(inside),
                len(outside),
                round(r_in, 4),
                round(r_out, 4),
                round(r_out / r_in, 2) if r_in > 0.0 else None,
            )
        )
    print("\nPAIRED, matched on commanded lock and wheel speed:", flush=True)
    print_table(
        rows,
        ("lock deg", "speed m/s", "n pocket", "n corridor", "R pocket m", "R corridor m", "out/in"),
    )


_FIT_LOCK_DEG = 30.0
"""Lock above which the geometric term is irrelevant and only the floor binds.

At 30 degrees the bicycle radius is already ~2.7 cm on this chassis, two orders
under anything measured, so every metre of these samples is the saturation
being measured rather than the geometry."""


def _fit_table(corridor: list[tuple[float, ...]]) -> None:
    """Achieved radius against MEAN speed, free space, full lock.

    This is the curve `MIN_TURN_RADIUS_M` should be, and it is taken in the
    CORRIDOR rather than the pocket on purpose: the kinematics floor is a
    free-space statement, and whatever the wall adds belongs to the contact
    model. Mean speed per bin rather than the bin edge, because the bins are
    not uniformly filled and a fit through edges is a fit through fiction.
    """
    full = [x for x in corridor if math.degrees(x[3]) >= _FIT_LOCK_DEG]
    if not full:
        return
    edges = (0.05, 0.10, 0.15, 0.20, 0.25, 0.30)
    rows = []
    for lo, hi in zip((0.0, *edges), (*edges, math.inf), strict=True):
        binned = [x for x in full if lo <= x[4] < hi]
        if len(binned) < 20:
            continue
        travel = sum(x[0] for x in binned)
        turned = sum(x[1] for x in binned)
        rows.append(
            (
                f"{lo:.2f}-{hi:.2f}" if math.isfinite(hi) else f"{lo:.2f}+",
                len(binned),
                round(sum(x[4] for x in binned) / len(binned), 4),
                round(travel / turned, 4),
            )
        )
    if not rows:
        return
    print(
        f"\nFREE-SPACE FLOOR vs SPEED (control phase, lock >= {_FIT_LOCK_DEG:.0f} deg). "
        f"Shipped MIN_TURN_RADIUS_M is a CONSTANT {RobotSpecs.MIN_TURN_RADIUS_M:.3f} m:",
        flush=True,
    )
    print_table(rows, ("speed bin m/s", "n", "mean speed", "R achieved m"))


def main() -> None:
    """Report achieved-vs-model turn radius in the pocket, per run and pooled."""
    parser = create_bags_parser(__doc__ or "")
    parser.add_argument(
        "--control-phase",
        default="normal_drive",
        help="phase to compare the pocket against, at matched lock and speed. The objection "
        "this answers is that the pocket's tighter radius could be full lock and creep rather "
        "than the wall; matching on both leaves only the surroundings.",
    )
    parser.add_argument(
        "--window-s",
        type=float,
        default=0.05,
        help="aggregate IMU samples into windows this long and take the NET yaw across each. "
        "0 uses the raw ~166 Hz rate, where summing |yaw| accumulates noise rather than "
        "cancelling it and understates the radius by ~19%%.",
    )
    parser.add_argument(
        "--settled-s",
        type=float,
        default=0.0,
        help="require the steering COMMAND to have been unchanged this long before the window. "
        "The servo slews and its feedback is the command echoed back, so a window taken during "
        "a transient runs at less lock than commanded and reads too wide.",
    )
    args = parser.parse_args()

    headers = ("run", "ticks", "travel m", "turned deg", "R_eff m", "R_eff/R_model")
    table: list[tuple[object, ...]] = []
    pooled: list[tuple[float, ...]] = []
    control: list[tuple[float, ...]] = []
    noise_yaw = 0.0
    noise_s = 0.0
    driven_s = 0.0
    pose_travel = 0.0
    pose_yaw = 0.0
    pose_imu_yaw = 0.0
    for bag_dir in args.bag_dirs:
        try:
            samples, stalled_yaw, stalled_s, run_driven_s, pose_trio = _run_samples(bag_dir, window_s=args.window_s)
        except (OSError, ValueError, RuntimeError) as exc:
            print(f"!! {bag_dir.name}: {exc}", flush=True)
            continue
        if not samples:
            continue
        pooled.extend(samples)
        control.extend(_run_samples(bag_dir, args.control_phase, args.window_s, args.settled_s)[0])
        noise_yaw += stalled_yaw
        noise_s += stalled_s
        driven_s += run_driven_s
        pose_travel += pose_trio[0]
        pose_yaw += pose_trio[1]
        pose_imu_yaw += pose_trio[2]
        # Radius is a RATIO of sums, not a mean of ratios: a per-tick radius is
        # dominated by the shortest ticks, and the manoeuvre's question is how
        # much travel it spent for how much rotation over the whole leg.
        travel = sum(s[0] for s in samples)
        turned = sum(s[1] for s in samples)
        ratios = [s[0] / s[1] / s[2] for s in samples if math.isfinite(s[2])]
        table.append(
            (
                bag_dir.name.removeprefix("run_"),
                len(samples),
                round(travel, 3),
                round(math.degrees(turned), 1),
                round(travel / turned, 4),
                round(statistics.median(ratios), 3) if ratios else None,
            )
        )

    if not table:
        print("no bag reached the bay_exit phase with usable motion", flush=True)
        return
    print_table(table, headers)

    travel = sum(s[0] for s in pooled)
    turned = sum(s[1] for s in pooled)
    ratios = sorted(s[0] / s[1] / s[2] for s in pooled if math.isfinite(s[2]))
    radii = sorted(s[0] / s[1] for s in pooled)
    print(
        f"\npooled {len(pooled)} intervals over {len(table)} runs: "
        f"{travel:.2f} m of wheel travel for {math.degrees(turned):.0f} deg",
        flush=True,
    )
    print(
        f"  ACHIEVED radius m: p10 {radii[len(radii) // 10]:.4f}  "
        f"median {statistics.median(radii):.4f}  p90 {radii[len(radii) * 9 // 10]:.4f}  "
        f"(pooled {travel / turned:.4f})",
        flush=True,
    )
    print(f"  MODEL floor is {RobotSpecs.MIN_TURN_RADIUS_M:.4f} m", flush=True)
    # The 0.29 m floor was measured against POSE yaw (see `kinematics.py`), and
    # `quaternion_yaw` says that signal is damped and understates the achieved
    # yaw rate by roughly half. Understating yaw OVERSTATES radius, so the same
    # intervals scored both ways say how much of the floor is that artefact
    # rather than physics. Same samples, not a second population.
    if pose_travel > 0.0 and pose_yaw > 0.0 and pose_imu_yaw > 0.0:
        print(
            f"  ON THE POSE TIME BASE: R from pose yaw {pose_travel / pose_yaw:.4f} m vs "
            f"R from IMU yaw {pose_travel / pose_imu_yaw:.4f} m over the SAME windows "
            f"({pose_yaw / pose_imu_yaw:.2f}x the rotation)",
            flush=True,
        )
    if noise_s > 0.0:
        # The same |yaw| sum over ticks whose wheel did not move. Whatever this
        # rate is, the driven ticks carry it too, so it bounds how much of the
        # measured rotation is sensor noise rather than chassis motion.
        print(
            f"  NOISE CONTROL: stalled ticks accumulate "
            f"{math.degrees(noise_yaw / noise_s):.2f} deg/s of |yaw| over {noise_s:.0f} s "
            f"vs {math.degrees(turned) / driven_s:.2f} deg/s over {driven_s:.0f} s DRIVEN "
            f"-- noise is {noise_yaw / noise_s / (turned / driven_s):.1%} of the measured rate",
            flush=True,
        )
    if ratios:
        tighter = sum(1 for r in ratios if r < 1.0) / len(ratios)
        print(
            f"  ACHIEVED / MODEL: p10 {ratios[len(ratios) // 10]:.3f}  "
            f"median {statistics.median(ratios):.3f}  p90 {ratios[len(ratios) * 9 // 10]:.3f}; "
            f"TIGHTER THAN THE MODEL ALLOWS on {tighter:.0%} of intervals",
            flush=True,
        )

    if control:
        _paired_table(pooled, control)
        _fit_table(control)


if __name__ == "__main__":
    main()
