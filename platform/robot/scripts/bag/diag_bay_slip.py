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

That was first read as the wall, and it is mostly NOT. The paired table is
what settles it: matched on commanded lock AND wheel speed, the CORRIDOR at
full lock reaches 0.088 m at creep, 0.160 at 0.10+ m/s and 0.198 at 0.20+ --
never the 0.29 m floor, anywhere. So the achieved radius is a strong function
of SPEED, as tyre scrub would predict, and a constant floor is wrong at the low
end. The pocket adds a further 1.35-1.60x on top, which IS the wall, and is the
smaller half of the effect. Reporting the pooled ratio alone credits the wall
with both.

Both control phases exist for that reason: the pocket is also where the wheel
sits at full lock and crawls, so an unmatched comparison cannot tell the
surroundings from the operating point.

Why the IMU and not the pose: ``quaternion_yaw``'s docstring is explicit that
``pose_yaw`` is localizer-fused and damped and understates the achieved yaw rate
by roughly half. Half is the whole size of the effect being measured here.

Why speed and not odometry distance: ``travelled_m`` is SIGNED and a ratchet
alternates, so it cancels -- the recorded reason a previous bay measurement read
1 cm of progress across 539 legs. Integrating ``|speed| * dt`` counts the travel
each leg actually spent.

Usage (from ``platform/robot``)::

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


def _in_any(spans: list[tuple[float, float]], t0: float, t1: float) -> bool:
    """Whether the whole interval falls inside one of ``spans``."""
    return any(a <= t0 and t1 <= b for a, b in spans)


def _run_samples(
    bag_dir: Path, phase: str = BAY_PHASE
) -> tuple[list[tuple[float, float, float, float, float]], float, float, float]:
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
        return [], 0.0, 0.0, 0.0

    streams = read_motion_streams(bag_dir)
    speeds = streams.drive_speed_mps()
    samples: list[tuple[float, float, float]] = []
    stalled_yaw = 0.0
    stalled_s = 0.0
    driven_s = 0.0
    yaw = streams.imu_yaw_rad
    for (t0, y0), (t1, y1) in pairwise(yaw):
        if not _in_any(spans, t0, t1):
            continue
        dt = t1 - t0
        if dt <= 0.0:
            continue
        speed = _sample_at(speeds, t0)
        wheel = _sample_at(streams.cmd_steer_rad, t0)
        if speed is None or wheel is None:
            continue
        travel = abs(speed) * dt
        step = (y1 - y0 + math.pi) % (2.0 * math.pi) - math.pi
        if travel < _MIN_TRAVEL_M:
            stalled_yaw += abs(step)
            stalled_s += dt
            continue
        if abs(step) < _MIN_YAW_RAD:
            continue
        driven_s += dt
        samples.append((travel, abs(step), _model_radius_m(wheel), abs(wheel), abs(speed)))
    return samples, stalled_yaw, stalled_s, driven_s


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


def _cell(sample: tuple[float, float, float, float, float]) -> tuple[int, int]:
    """``(lock bucket, speed bucket)`` a sample belongs to."""
    return _bucket(math.degrees(sample[3]), _LOCK_EDGES_DEG), _bucket(sample[4], _SPEED_EDGES_MPS)


def _paired_table(
    pocket: list[tuple[float, float, float, float, float]],
    corridor: list[tuple[float, float, float, float, float]],
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
    args = parser.parse_args()

    headers = ("run", "ticks", "travel m", "turned deg", "R_eff m", "R_eff/R_model")
    table: list[tuple[object, ...]] = []
    pooled: list[tuple[float, float, float, float, float]] = []
    control: list[tuple[float, float, float, float, float]] = []
    noise_yaw = 0.0
    noise_s = 0.0
    driven_s = 0.0
    for bag_dir in args.bag_dirs:
        try:
            samples, stalled_yaw, stalled_s, run_driven_s = _run_samples(bag_dir)
        except (OSError, ValueError, RuntimeError) as exc:
            print(f"!! {bag_dir.name}: {exc}", flush=True)
            continue
        if not samples:
            continue
        pooled.extend(samples)
        control.extend(_run_samples(bag_dir, args.control_phase)[0])
        noise_yaw += stalled_yaw
        noise_s += stalled_s
        driven_s += run_driven_s
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


if __name__ == "__main__":
    main()
