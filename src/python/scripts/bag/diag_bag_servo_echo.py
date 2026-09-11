r"""Is ``/motor/steering_position`` a sensor, or the command echoed back?

``MAX_STEERING_RATE = 1.2 rad/s`` (``src/config/navigation/motion/pursuit.toml``)
has never been measured, and it owns ~2.5 s per bay-exit reversal because a
lock-to-lock swing is budgeted as ``ceil(swing_rad / (rate / CONTROL_HZ))``
ticks of commanded ZERO. The obvious way to measure it is the feedback topic.

This script refuses to hand over a number until the topic has earned it. It
runs the SAME step-response measurement on two topics at once:

* ``/motor/steering_position`` -- the candidate. On the servo chassis
  ``ServoDriver.get_steering_position()`` returns ``self._position``, the last
  commanded angle, so the prediction is a PERFECT echo: zero lag, zero
  residual, and an apparent slew rate limited only by the publish cadence.
* ``/motor/drive_speed`` -- the CONTROL. That one is encoder-derived, i.e. a
  real measurement of a real plant, so it MUST show lag and residual. If the
  control comes back looking like a perfect echo too, the method is broken and
  neither column means anything.

Reported per topic:

* best-fit ``feedback = a*command + b`` with R^2 and residual RMS, evaluated at
  every integer sample lag, so the lag that minimises residual is read off
  rather than assumed;
* time from a commanded step to 90% of the new feedback value;
* the maximum observed slew of the feedback series, in the command's own units,
  which is the number ``MAX_STEERING_RATE`` would have to match.

Sign/units: ``/ackermann_cmd.drive.steering_angle`` is the WHEEL angle in rad;
``/motor/steering_position`` is published as ``get_steering_position() *
linkage_ratio`` in degrees. The fit solves for the scale, so no conversion is
assumed. ``/motor/drive_speed`` is DEG/S at the wheel, converted on the 7 cm
wheel before the fit (``bag_drive_speed_is_deg_per_sec``).

Usage::

    pixi run -e dev python scripts/bag/diag_bag_servo_echo.py \
        ../../data/live/runs/run_20260911_1523*
"""

from __future__ import annotations

import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from shared.config.constants import RobotSpecs

from scripts.common.bag_io import create_bags_parser, read_motion_streams

_SHIPPED_MAX_STEERING_RATE = 1.2  # rad/s, the constant under test
_CONTROL_HZ = 20.0
_LOCK_TO_LOCK_RAD = 2 * math.radians(85.0)  # RobotSpecs.MAX_WHEEL_ANGLE_DEG, the bay swing


def _zoh(command: list[tuple[float, float]], t: float) -> float | None:
    """Value of a zero-order-held command series at time ``t`` (last sample <= t)."""
    lo, hi, found = 0, len(command) - 1, None
    while lo <= hi:
        mid = (lo + hi) // 2
        if command[mid][0] <= t:
            found = command[mid][1]
            lo = mid + 1
        else:
            hi = mid - 1
    return found


def _fit(pairs: list[tuple[float, float]]) -> tuple[float, float, float, float]:
    """Least-squares ``y = a*x + b``; returns (a, b, r2, residual RMS)."""
    n = len(pairs)
    if n < 3:
        return (math.nan, math.nan, math.nan, math.nan)
    sx = sum(x for x, _ in pairs)
    sy = sum(y for _, y in pairs)
    sxx = sum(x * x for x, _ in pairs)
    sxy = sum(x * y for x, y in pairs)
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return (math.nan, math.nan, math.nan, math.nan)
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    resid = [y - (a * x + b) for x, y in pairs]
    ss_res = sum(r * r for r in resid)
    ymean = sy / n
    ss_tot = sum((y - ymean) ** 2 for _, y in pairs)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else math.nan
    return (a, b, r2, math.sqrt(ss_res / n))


def _lag_scan(
    command: list[tuple[float, float]], feedback: list[tuple[float, float]], max_lag: int
) -> list[tuple[int, float, float, float]]:
    """For each integer sample lag, fit feedback[i] against command at t - lag*dt."""
    if len(feedback) < 10:
        return []
    dts = [b[0] - a[0] for a, b in zip(feedback, feedback[1:], strict=False)]
    dt = statistics.median(dts) if dts else 0.05
    out = []
    for lag in range(-max_lag, max_lag + 1):
        pairs = []
        for t, y in feedback:
            x = _zoh(command, t - lag * dt)
            if x is not None:
                pairs.append((x, y))
        a, _b, r2, rms = _fit(pairs)
        out.append((lag, a, r2, rms))
    return out


def _step_responses(
    command: list[tuple[float, float]],
    feedback: list[tuple[float, float]],
    min_step: float,
    scale: float,
    window_s: float,
) -> tuple[list[float], list[float], int]:
    """Time from a commanded step to 90% of the new level, and the slew it implies."""
    times: list[float] = []
    slews: list[float] = []
    unusable = 0
    for (t0, v0), (t1, v1) in zip(command, command[1:], strict=False):
        _ = t0
        if abs(v1 - v0) < min_step:
            continue
        target = v1 * scale
        start = None
        for t, y in feedback:
            if t <= t1:
                start = (t, y)
            else:
                break
        if start is None:
            unusable += 1
            continue
        span = target - start[1]
        if abs(span) < 1e-9:
            unusable += 1
            continue
        hit = None
        for t, y in feedback:
            if t <= t1 or t > t1 + window_s:
                continue
            if (y - start[1]) / span >= 0.9:
                hit = t
                break
        if hit is None:
            unusable += 1
            continue
        dt = max(hit - t1, 1e-6)
        times.append(dt)
        slews.append(abs(span) / dt)
    return times, slews, unusable


def _raw_slew(feedback: list[tuple[float, float]]) -> tuple[float, float]:
    """Max and 99th-percentile |dy/dt| of consecutive feedback samples."""
    rates = []
    for (ta, ya), (tb, yb) in zip(feedback, feedback[1:], strict=False):
        dt = tb - ta
        if dt <= 1e-6:
            continue
        rates.append(abs(yb - ya) / dt)
    if not rates:
        return (math.nan, math.nan)
    rates.sort()
    return (rates[-1], rates[int(0.99 * (len(rates) - 1))])


def _q(values: list[float]) -> str:
    if not values:
        return "n=0"
    values = sorted(values)
    p = lambda f: values[min(len(values) - 1, int(f * (len(values) - 1)))]  # noqa: E731
    return f"n={len(values)} min={values[0]:.3f} p50={p(0.5):.3f} p90={p(0.9):.3f} max={values[-1]:.3f}"



def _command_slew_and_coast(bag_dir: Path) -> tuple[list[float], float, list[float], list[float]]:
    """Per-tick command slew (rad/s) from /ackermann_cmd, plus coast during commanded zero.

    The rate limiter in ``WaypointController.compute_steering`` caps the command
    at ``max_steering_rate * dt``. If the shipped 1.2 rad/s is live on the wire
    the per-tick slew MUST clip at exactly 1.2 -- that is the control: a known
    value the measurement has to reproduce, or the series is not what it says.

    Coast: runs of commanded speed == 0 lasting >= 0.2 s, with the encoder
    distance (|deg/s| integrated on the wheel rim) accumulated over the run.
    """
    streams = read_motion_streams(bag_dir)
    slews: list[float] = []
    for (ta, va), (tb, vb) in zip(streams.cmd_steer_rad, streams.cmd_steer_rad[1:], strict=False):
        dt = tb - ta
        if 1e-3 < dt < 0.5:
            slews.append(abs(vb - va) / dt)
    at_cap = sum(1 for s in slews if s > _SHIPPED_MAX_STEERING_RATE * 0.98) / len(slews) if slews else math.nan

    scale = math.radians(1.0) * RobotSpecs.WHEEL_RADIUS
    speed = streams.drive_speed_mps()
    coast_mm: list[float] = []
    zero_dur: list[float] = []
    _ = scale
    run_start = None
    for (ta, va), (tb, _vb) in zip(streams.cmd_speed_mps, streams.cmd_speed_mps[1:], strict=False):
        if abs(va) < 1e-6:
            if run_start is None:
                run_start = ta
            continue
        if run_start is not None and ta - run_start >= 0.2:
            dist = 0.0
            for (t0, s0), (t1, _s1) in zip(speed, speed[1:], strict=False):
                if run_start <= t0 <= ta:
                    dist += abs(s0) * (t1 - t0)
            coast_mm.append(dist * 1000.0)
            zero_dur.append(ta - run_start)
        run_start = None
        _ = tb
    return slews, at_cap, coast_mm, zero_dur


def main() -> None:
    parser = create_bags_parser(__doc__, formatter_class=__import__("argparse").RawDescriptionHelpFormatter)
    parser.add_argument("--min-step-rad", type=float, default=0.20, help="commanded wheel-angle jump counted as a step")
    parser.add_argument("--min-step-mps", type=float, default=0.08, help="commanded speed jump counted as a step")
    parser.add_argument("--max-lag", type=int, default=6, help="sample lags scanned either side of zero")
    parser.add_argument("--window", type=float, default=2.0, help="seconds tracked after a step")
    args = parser.parse_args()

    all_steer_times: list[float] = []
    all_steer_slew: list[float] = []
    all_speed_times: list[float] = []

    for bag_dir in args.bag_dirs:
        streams = read_motion_streams(Path(bag_dir))
        name = Path(bag_dir).name
        print(f"\n=== {name} ===")
        print(
            f"samples: ackermann_cmd={len(streams.cmd_steer_rad)} "
            f"steering_position={len(streams.steer_pos_deg)} drive_speed={len(streams.drive_speed_dps)}"
        )
        if not streams.cmd_steer_rad:
            print("  no /ackermann_cmd -- nothing measurable in this bag")
            continue

        # ---- CANDIDATE: steering_position (published in degrees) ----
        steer_fb = streams.steer_pos_deg
        if steer_fb:
            scan = _lag_scan(streams.cmd_steer_rad, steer_fb, args.max_lag)
            if scan:
                best = min(scan, key=lambda r: (math.inf if math.isnan(r[3]) else r[3]))
                print("  [CANDIDATE] /motor/steering_position vs commanded wheel angle")
                print(f"    best lag {best[0]:+d} samples  slope={best[1]:.4f} deg/rad  R2={best[2]:.6f}  rms={best[3]:.4f} deg")
                for lag, a, r2, rms in scan:
                    if abs(lag) <= 2:
                        print(f"      lag {lag:+d}: slope={a:8.4f}  R2={r2:9.6f}  rms={rms:7.4f} deg")
            uniq = len({round(v, 6) for _, v in steer_fb})
            print(f"    distinct feedback values: {uniq} of {len(steer_fb)} samples")
            mx, p99 = _raw_slew(steer_fb)
            print(f"    raw |d/dt| of feedback: max={mx:.1f} deg/s ({math.radians(mx):.2f} rad/s)  p99={math.radians(p99):.2f} rad/s")
            times, slews, bad = _step_responses(
                streams.cmd_steer_rad, steer_fb, args.min_step_rad, math.degrees(1.0), args.window
            )
            all_steer_times += times
            all_steer_slew += [math.radians(s) for s in slews]
            print(f"    step->90% time (s): {_q(times)}   unusable={bad}")
        else:
            print("  [CANDIDATE] /motor/steering_position ABSENT from this bag")

        # ---- CONTROL: drive_speed, encoder-derived, must NOT look like an echo ----
        speed_fb = streams.drive_speed_mps()
        if speed_fb:
            scan = _lag_scan(streams.cmd_speed_mps, speed_fb, args.max_lag)
            if scan:
                best = min(scan, key=lambda r: (math.inf if math.isnan(r[3]) else r[3]))
                print("  [CONTROL ] /motor/drive_speed (encoder, m/s) vs commanded speed")
                print(f"    best lag {best[0]:+d} samples  slope={best[1]:.4f}  R2={best[2]:.6f}  rms={best[3]:.4f} m/s")
                for lag, a, r2, rms in scan:
                    if abs(lag) <= 2:
                        print(f"      lag {lag:+d}: slope={a:8.4f}  R2={r2:9.6f}  rms={rms:7.4f} m/s")
            times, _slews, bad = _step_responses(streams.cmd_speed_mps, speed_fb, args.min_step_mps, 1.0, args.window)
            all_speed_times += times
            print(f"    step->90% time (s): {_q(times)}   unusable={bad}")
        else:
            print("  [CONTROL ] /motor/drive_speed ABSENT -- the null above is UNREADABLE")

        slews, at_cap, coast_mm, zero_dur = _command_slew_and_coast(Path(bag_dir))
        print("  [COMMAND ] per-tick slew of /ackermann_cmd steering (rad/s)")
        print(f"    {_q(slews)}   share at/above the shipped 1.2 cap: {at_cap * 100:.1f}%")
        print(f"  [COAST   ] commanded-zero runs >= 0.2 s: {len(coast_mm)}")
        print(f"    duration (s): {_q(zero_dur)}")
        print(f"    encoder distance during them (mm): {_q(coast_mm)}")

    print("\n=== verdict ===")
    print(f"  CANDIDATE steering step->90% (s): {_q(all_steer_times)}")
    print(f"  CANDIDATE implied slew    (rad/s): {_q(all_steer_slew)}")
    print(f"  CONTROL   speed    step->90% (s): {_q(all_speed_times)}")
    if all_steer_slew:
        med = statistics.median(all_steer_slew)
        ticks_shipped = math.ceil(_LOCK_TO_LOCK_RAD / (_SHIPPED_MAX_STEERING_RATE / _CONTROL_HZ))
        ticks_meas = math.ceil(_LOCK_TO_LOCK_RAD / (med / _CONTROL_HZ)) if med > 0 else -1
        print(
            f"  lock-to-lock {_LOCK_TO_LOCK_RAD:.2f} rad budget: shipped {_SHIPPED_MAX_STEERING_RATE} rad/s "
            f"-> {ticks_shipped} ticks ({ticks_shipped / _CONTROL_HZ:.2f} s); "
            f"candidate {med:.2f} rad/s -> {ticks_meas} ticks ({ticks_meas / _CONTROL_HZ:.2f} s)"
        )
    print(
        "  READ THIS BEFORE THE NUMBER: if the CANDIDATE fits at lag 0 with R2~1 and rms~0 while the\n"
        "  CONTROL needs several samples and carries residual, the candidate is the command echoed back\n"
        "  and its 'slew rate' is the publish cadence, NOT the servo. No bag can then measure the servo."
    )


if __name__ == "__main__":
    main()
