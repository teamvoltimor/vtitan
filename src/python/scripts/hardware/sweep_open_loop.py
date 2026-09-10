#!/usr/bin/env python3
"""Open-loop hardware characterization: raw H-bridge duty sweep, full servo
range, and start/stop dynamics (step response).

Unlike test_motors.py, which commands /ackermann_cmd over ROS2 and goes
through ackermann_motor_node's closed-loop PID, this drives the BTS7960 and
servo objects DIRECTLY at raw duty fractions / absolute angles -- no ROS2, no
PID, no m/s. That matters specifically because the closed loop's
target_wheel_rpm math depends on encoder.toml's counts_per_rev/max_rpm, which
are calibrated against the RETIRED motor and are not trustworthy for a new
one (see EncoderConfig.max_duty's docstring). This script answers "what does
raw duty X actually produce" without assuming that calibration at all --
exactly what's needed to characterize a new motor before recalibrating it.

Run ON the Pi Zero directly (needs exclusive GPIO/PWM access -- the same
pins ackermann_motor_node's driver holds). Preferred: `task
robot:sweep-open-loop -- [args]`, which stops vtitan-pi-zero.service first
and restarts it afterward (even on failure/Ctrl-C, via a shell trap) so the
service is never left down by an interrupted run. Manual equivalent:
    sudo systemctl stop vtitan-pi-zero.service
    cd src
    PYTHONPATH="." ~/.pixi/bin/pixi run -e dev sweep-open-loop
    sudo systemctl start vtitan-pi-zero.service

Loads .env itself (load_dotenv() below) rather than relying on the caller
to `source .env` first -- the exact class of bug fixed in `097ab6cf` for
run-lidar's RobotSpecs read (Config()/EncoderConfig.load()/ServoConfig() below
need VTITAN_HARDWARE_PROFILE etc. from it).

The script itself ALSO checks whether vtitan-pi-zero.service is active and
refuses to run if so -- a safety net for anyone invoking the raw pixi task
or this file directly, bypassing the Taskfile wrapper above, rather than
fighting the service for the same sysfs/GPIO handles (see
docs/bts7960-ibt2-wiring.md for what shared PWM state produces if two
processes write to it at once).

Usage:
    python3 scripts/hardware/sweep_open_loop.py                        # full sweep, asks to confirm
    python3 scripts/hardware/sweep_open_loop.py --yes
    python3 scripts/hardware/sweep_open_loop.py --skip-drive            # servo sweep only
    python3 scripts/hardware/sweep_open_loop.py --skip-servo            # drive sweep only
    python3 scripts/hardware/sweep_open_loop.py --duty-fractions 0.1,0.2,0.3,0.5,0.7,1.0
    python3 scripts/hardware/sweep_open_loop.py --servo-steps 9         # -max..+max in 9 steps

Start/stop dynamics (0 -> duty -> stop, both directions, runs by default
alongside the duty sweep unless --skip-step): samples the encoder
continuously through the ramp-up AND the post-stop coast-down (not just a
discarded-then-averaged steady-state window like sweep_drive() above), and
reports rise time (10/50/90% of steady-state), an R^2 linearity check on the
10-90% rise window, and settling time back under 10% after stop_drive().
Also prints a live rpm/m/s readout every --step-progress-s during the hold
and coast, so a long --step-hold-s isn't silent.
    python3 scripts/hardware/sweep_open_loop.py --skip-servo --duty-fractions 1.0   # duty sweep is a single point; step response still runs
    python3 scripts/hardware/sweep_open_loop.py --skip-servo --step-duty 0.5 --step-hold-s 4 --step-coast-s 4
    python3 scripts/hardware/sweep_open_loop.py --skip-servo --skip-reverse --step-duty 1.0 --step-hold-s 60 --step-progress-s 2
    python3 scripts/hardware/sweep_open_loop.py --skip-servo --skip-step   # duty sweep only, no step response
"""

from __future__ import annotations

import argparse
import statistics
import subprocess
import sys
import time

from dotenv import load_dotenv

# Must run before any src.hardware.motors.* import: those transitively import
# shared.config.constants.RobotSpecs, which reads VTITAN_HARDWARE_PROFILE at
# MODULE IMPORT TIME (a top-level statement in shared/config/constants/
# _shared.py, not inside a function) -- the same class of bug fixed in
# `097ab6cf` for run-lidar. Calling load_dotenv() any later is too late.
load_dotenv()

from src.hardware.motors.bts7960 import Driver as Bts7960Driver  # noqa: E402 - see load_dotenv() note above
from src.hardware.motors.config import Config  # noqa: E402
from src.hardware.motors.encoder import EncoderConfig, QuadratureEncoder  # noqa: E402
from src.hardware.motors.encoder.calibration import DEFAULT_WHEEL_DIAMETER_M  # noqa: E402
from src.hardware.motors.servo import Driver as ServoDriver  # noqa: E402
from src.hardware.motors.servo.config import ServoConfig  # noqa: E402

SERVICE_NAME = "vtitan-pi-zero.service"
SETTLE_S = 0.5
"""Pause after commanding a new duty/angle, before sampling -- lets the PWM
carrier and any mechanical inertia settle before the sample window starts."""


def _service_is_active() -> bool:
    result = subprocess.run(
        ["systemctl", "is-active", SERVICE_NAME],  # noqa: S607 - fixed, no user input
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout.strip() == "active"


def rpm_to_mps(rpm: float) -> float:
    """Wheel rpm -> linear m/s, using the same wheel diameter the encoder/odometry use."""
    return abs(rpm) / 60.0 * 3.14159265358979 * DEFAULT_WHEEL_DIAMETER_M


def sweep_drive(
    driver: Bts7960Driver,
    encoder: QuadratureEncoder,
    fractions: list[float],
    hold_s: float,
    reverse: bool,
) -> list[tuple[float, float, float]]:
    """Command each duty fraction in turn, sample the encoder, report mean rpm/m/s.

    Returns a list of (fraction, mean_rpm, mean_mps) tuples. Zero is skipped
    on purpose -- it is not a meaningful duty to hold, and stop_drive() is
    called between every step regardless.
    """
    direction = "REVERSE" if reverse else "FORWARD"
    results = []
    for frac in fractions:
        if frac <= 0:
            continue
        if reverse:
            driver.run_drive_reverse(frac * 100.0)
        else:
            driver.run_drive_forward(frac * 100.0)
        time.sleep(SETTLE_S)

        samples = []
        deadline = time.monotonic() + hold_s
        last = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            dt = now - last
            last = now
            if dt > 0:
                samples.append(encoder.get_rpm())
            time.sleep(0.05)

        driver.stop_drive()
        time.sleep(0.5)  # coast to a stop before the next step

        mean_rpm = statistics.fmean(samples) if samples else 0.0
        mean_mps = rpm_to_mps(mean_rpm)
        print(
            f"  [{direction}] duty={frac:.2f} -> mean {mean_rpm:+.1f} rpm "
            f"({mean_mps:.4f} m/s), n={len(samples)}",
        )
        results.append((frac, mean_rpm, mean_mps))
    return results


def _linear_fit_r2(points: list[tuple[float, float]]) -> float:
    """Least-squares R^2 of a straight-line fit to (t, rpm) points.

    Pure stdlib, no numpy dependency -- this is a hardware probe script, and
    every other one in this directory sticks to `statistics` only. Returns
    0.0 for fewer than 2 points (can't fit a line) rather than raising.
    """
    n = len(points)
    if n < 2:
        return 0.0
    ts = [t for t, _ in points]
    rpms = [r for _, r in points]
    t_mean = statistics.fmean(ts)
    r_mean = statistics.fmean(rpms)
    ss_t = sum((t - t_mean) ** 2 for t in ts)
    if ss_t == 0:
        return 0.0
    slope = sum((t - t_mean) * (r - r_mean) for t, r in zip(ts, rpms, strict=True)) / ss_t
    intercept = r_mean - slope * t_mean
    ss_res = sum((r - (slope * t + intercept)) ** 2 for t, r in points)
    ss_tot = sum((r - r_mean) ** 2 for r in rpms)
    if ss_tot == 0:
        return 1.0
    return 1.0 - ss_res / ss_tot


def _time_to_threshold(samples: list[tuple[float, float]], steady_abs: float, frac: float, after_t: float = 0.0) -> float | None:
    """First timestamp (relative to `after_t`) where |rpm| crosses `frac` of `steady_abs`.

    Returns None if the threshold is never crossed in the given samples.
    """
    target = steady_abs * frac
    for t, rpm in samples:
        if t < after_t:
            continue
        if abs(rpm) >= target:
            return t - after_t
    return None


def step_response(
    driver: Bts7960Driver,
    encoder: QuadratureEncoder,
    duty: float,
    hold_s: float,
    coast_s: float,
    reverse: bool,
    progress_interval_s: float = 1.0,
) -> list[tuple[float, float]]:
    """Command a single duty step, sample continuously through the ramp-up AND
    the stop/coast-down, and report rise/settling times + a linearity check.

    Unlike sweep_drive() (which discards the ramp-up and reports only the
    steady-state mean per duty level), this KEEPS the full time series so the
    start/stop dynamics are visible -- how long 0 -> steady-state actually
    takes, whether that ramp is linear (R^2 of a straight-line fit), and how
    long the coast-down after stop_drive() takes to settle back near zero.

    Prints a live rpm/m/s readout every `progress_interval_s` during both the
    hold and the coast-down -- a multi-minute --step-hold-s would otherwise
    sit silent with no sign it's still running.

    Returns the full (t, rpm) sample list, t=0 at the moment the duty command
    was issued, so a caller can plot/inspect it directly.
    """
    direction = "REVERSE" if reverse else "FORWARD"
    print(f"\n=== Step response [{direction}]: duty={duty:.2f}, hold={hold_s}s, coast={coast_s}s ===")

    samples: list[tuple[float, float]] = []
    t0 = time.monotonic()
    next_progress = progress_interval_s
    if reverse:
        driver.run_drive_reverse(duty * 100.0)
    else:
        driver.run_drive_forward(duty * 100.0)

    while (t := time.monotonic() - t0) < hold_s:
        rpm = encoder.get_rpm()
        samples.append((t, rpm))
        if t >= next_progress:
            print(f"  t={t:5.1f}s [hold]  {rpm:+7.1f} rpm  ({rpm_to_mps(rpm):.4f} m/s)")
            next_progress += progress_interval_s
        time.sleep(0.02)

    t_stop = time.monotonic() - t0
    driver.stop_drive()
    print(f"  t={t_stop:5.1f}s [stop_drive() issued]")

    next_progress = t_stop + progress_interval_s
    coast_deadline = time.monotonic() + coast_s
    while time.monotonic() < coast_deadline:
        t = time.monotonic() - t0
        rpm = encoder.get_rpm()
        samples.append((t, rpm))
        if t >= next_progress:
            print(f"  t={t:5.1f}s [coast] {rpm:+7.1f} rpm  ({rpm_to_mps(rpm):.4f} m/s)")
            next_progress += progress_interval_s
        time.sleep(0.02)

    # Steady-state estimate: mean of the last 20% of the hold window, right
    # before the stop command -- the most settled part of the ramp.
    steady_window = [(t, r) for t, r in samples if t_stop * 0.8 <= t < t_stop]
    steady_rpm = statistics.fmean(r for _, r in steady_window) if steady_window else 0.0
    steady_abs = abs(steady_rpm)

    rise_10 = _time_to_threshold(samples, steady_abs, 0.10)
    rise_50 = _time_to_threshold(samples, steady_abs, 0.50)
    rise_90 = _time_to_threshold(samples, steady_abs, 0.90)

    # Linearity of the rise: fit only the 10%-90% window, not the flat
    # steady-state tail that would drag R^2 toward "not linear" for a
    # response that ramps linearly and then plateaus (expected, not a fault).
    rise_window = [(t, r) for t, r in samples if rise_10 is not None and rise_90 is not None and rise_10 <= t <= rise_90]
    rise_r2 = _linear_fit_r2(rise_window)

    settle_10 = _time_to_threshold(samples, steady_abs, 0.10, after_t=t_stop)

    print(f"  steady-state: {steady_rpm:+.1f} rpm ({rpm_to_mps(steady_rpm):.4f} m/s)")
    print(
        f"  rise time: 10%={_fmt_s(rise_10)}, 50%={_fmt_s(rise_50)}, 90%={_fmt_s(rise_90)} "
        f"(from command onset)",
    )
    print(f"  rise linearity (10-90% window): R^2={rise_r2:.3f} ({'linear' if rise_r2 >= 0.95 else 'NOT linear'})")
    print(f"  settling time after stop_drive(): {_fmt_s(settle_10)} to fall back under 10% of steady-state")
    return samples


def _fmt_s(value: float | None) -> str:
    return "never" if value is None else f"{value * 1000:.0f}ms"


def sweep_servo(driver: ServoDriver, servo_config: ServoConfig, steps: int) -> None:
    """Sweep the servo across its full commanded range in `steps`, centre to centre.

    No encoder feedback exists for the servo -- this is a range-of-motion/
    binding check, not a speed measurement. Reports the driver's own
    get_steering_position() read-back (last commanded value, not sensed).
    """
    max_angle = servo_config.range_deg / 2.0
    print(f"\n=== Servo sweep: -{max_angle:.1f} deg .. +{max_angle:.1f} deg, {steps} steps ===")
    angles = [-max_angle + 2 * max_angle * i / (steps - 1) for i in range(steps)]
    for angle in [0.0, *angles, 0.0]:
        driver.move_steering_to(angle)
        time.sleep(SETTLE_S + 0.3)
        readback = driver.get_steering_position()
        print(f"  commanded={angle:+.1f} deg -> readback={readback:+.1f} deg")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--skip-drive", action="store_true")
    parser.add_argument("--skip-servo", action="store_true")
    parser.add_argument("--skip-reverse", action="store_true", help="Drive sweep: forward only")
    parser.add_argument(
        "--duty-fractions",
        type=str,
        default="0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0",
        help="Comma-separated duty fractions (0-1) to sweep, ascending",
    )
    parser.add_argument("--hold-s", type=float, default=2.0, help="Seconds to hold + sample each duty step")
    parser.add_argument("--servo-steps", type=int, default=5, help="Number of servo positions across its full range")
    parser.add_argument("--yes", "-y", action="store_true", help="Skip the drive-test confirmation prompt")
    parser.add_argument(
        "--skip-step",
        action="store_true",
        help="Skip the start/stop dynamics (step response) measurement",
    )
    parser.add_argument(
        "--step-duty",
        type=float,
        default=1.0,
        help="Duty fraction (0-1) for the step-response measurement (default 1.0 = full duty, 0 -> max)",
    )
    parser.add_argument("--step-hold-s", type=float, default=3.0, help="Seconds to hold the step duty before stopping")
    parser.add_argument(
        "--step-coast-s",
        type=float,
        default=3.0,
        help="Seconds to keep sampling after stop_drive(), to capture the coast-down",
    )
    parser.add_argument(
        "--step-progress-s",
        type=float,
        default=1.0,
        help="Live rpm/m/s readout interval during a step-response hold/coast (for long --step-hold-s runs)",
    )
    args = parser.parse_args()

    if _service_is_active():
        print(
            f"ERROR: {SERVICE_NAME} is active and already holds these GPIO/PWM lines. "
            f"Stop it first: sudo systemctl stop {SERVICE_NAME}",
            file=sys.stderr,
        )
        raise SystemExit(1)

    fractions = sorted({float(v) for v in args.duty_fractions.split(",")})

    if not args.skip_servo:
        servo_config = ServoConfig()
        servo = ServoDriver(servo_config)
        servo.connect()
        try:
            sweep_servo(servo, servo_config, args.servo_steps)
        finally:
            servo.disconnect()

    if not args.skip_drive:
        if not args.yes:
            reply = input(
                "\nThe drive sweep will spin the wheels across increasing duty, up to 100%. "
                "Confirm the robot is on a stand / wheels are clear, then type 'yes' to continue: ",
            )
            if reply.strip().lower() != "yes":
                print("Drive sweep skipped (not confirmed).")
                return

        config = Config()  # type: ignore[call-arg]
        encoder_config = EncoderConfig.load()
        drive = Bts7960Driver(invert=config.drive.reversed)
        encoder = QuadratureEncoder(
            pin_a=encoder_config.pin_a,
            pin_b=encoder_config.pin_b,
            counts_per_rev=encoder_config.counts_per_rev,
            wheel_diameter_m=DEFAULT_WHEEL_DIAMETER_M,
            invert=config.drive.encoder_reversed,
        )
        drive.connect()
        encoder.connect()
        try:
            print(f"\n=== Drive duty sweep: {fractions} ===")
            print("(reported m/s assumes counts_per_rev/wheel_diameter are correct; rpm is the raw ground truth)")
            forward_results = sweep_drive(drive, encoder, fractions, args.hold_s, reverse=False)
            reverse_results = []
            if not args.skip_reverse:
                reverse_results = sweep_drive(drive, encoder, fractions, args.hold_s, reverse=True)

            print("\n=== Summary ===")
            for frac, rpm, mps in forward_results:
                print(f"  FORWARD duty={frac:.2f}: {rpm:+.1f} rpm, {mps:.4f} m/s")
            for frac, rpm, mps in reverse_results:
                print(f"  REVERSE duty={frac:.2f}: {rpm:+.1f} rpm, {mps:.4f} m/s")

            if not args.skip_step:
                step_response(
                    drive, encoder, args.step_duty, args.step_hold_s, args.step_coast_s,
                    reverse=False, progress_interval_s=args.step_progress_s,
                )
                if not args.skip_reverse:
                    step_response(
                        drive, encoder, args.step_duty, args.step_hold_s, args.step_coast_s,
                        reverse=True, progress_interval_s=args.step_progress_s,
                    )
        finally:
            drive.stop_drive()
            drive.disconnect()
            encoder.disconnect()


if __name__ == "__main__":
    main()
