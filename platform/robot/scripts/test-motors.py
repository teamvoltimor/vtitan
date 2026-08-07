#!/usr/bin/env python3
"""Hardware smoke test for ackermann_motor_node: steering servo, then drive motor.

Publishes real AckermannDriveStamped commands to /ackermann_cmd and checks the
resulting /motor/steering_position and /motor/drive_speed feedback -- the same
command -> feedback round trip verify-zero-integration.sh checks once, but
exercised through a fuller range of motion with pass/fail reporting.

Run ON Pi 5 (needs ROS2 discovery to the Pi Zero's ackermann_motor_node over
USB/WiFi, same as verify-zero-integration.sh).

The steering test only moves the servo -- safe with the robot sitting still.
The drive test spins the wheels and requires a typed confirmation (or --yes)
before it runs; make sure the robot is on a stand / wheels clear before
confirming.

Usage:
    python3 scripts/test-motors.py                  # steering + drive (asks to confirm drive)
    python3 scripts/test-motors.py --skip-drive      # steering only
    python3 scripts/test-motors.py --yes             # skip the drive confirmation prompt
    python3 scripts/test-motors.py --drive-speed 0.2 --drive-duration-s 2.0
    python3 scripts/test-motors.py --find-min-speed --yes   # step up from low speed, report first that moves it
"""

from __future__ import annotations

import argparse
import math
import statistics
import time

import rclpy
from _motor_hold import publish_hold
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from std_msgs.msg import Float32

STEERING_TOLERANCE_DEG = 1.0
DRIVE_MOVING_THRESHOLD_DEG_S = 5.0
SETTLE_TIMEOUT_S = 2.0


def _wait_for_condition(
    node: Node,
    latest: dict,
    key: str,
    predicate,
    timeout_s: float = SETTLE_TIMEOUT_S,
) -> float | None:
    """Spin, continuously draining the feedback stream, until `predicate(value)`
    is true or timeout_s elapses.

    The feedback topic streams continuously regardless of commands, so the
    subscription queue can already hold stale (pre-command) samples the
    instant we publish. Grabbing the *first* new message risks reporting that
    stale value instead of where the actuator actually ended up -- keep
    spinning until the reading actually satisfies the predicate, or give up
    at timeout and report the last value seen either way.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        value = latest.get(key)
        if value is not None and predicate(value):
            return value
    return latest.get(key)


def _wait_for_value(node: Node, latest: dict, key: str, target: float, tolerance: float) -> float | None:
    return _wait_for_condition(node, latest, key, lambda v: abs(v - target) <= tolerance)


def run_steering_test(node: Node, pub, latest: dict, angle_deg: float) -> bool:
    print(f"\n=== Steering test (+-{angle_deg} deg) ===")
    passed = True
    for label, deg in [("LEFT", angle_deg), ("CENTER", 0.0), ("RIGHT", -angle_deg), ("CENTER", 0.0)]:
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = math.radians(deg)
        msg.drive.speed = 0.0
        pub.publish(msg)
        feedback = _wait_for_value(node, latest, "steering", deg, STEERING_TOLERANCE_DEG)
        ok = feedback is not None and abs(feedback - deg) <= STEERING_TOLERANCE_DEG
        status = "PASS" if ok else "FAIL"
        print(f"  [{status}] {label}: commanded {deg:+.1f}deg -> feedback {feedback}")
        passed &= ok
        time.sleep(0.3)
    return passed


def run_drive_test(
    node: Node,
    pub,
    latest: dict,
    speed_mps: float,
    duration_s: float,
    samples: list[tuple[float, float]],
    spinup_s: float = 1.0,
) -> bool:
    print(f"\n=== Drive test ({speed_mps} m/s for {duration_s}s) ===")
    passed = True

    # ackermann_motor_node has a 1s command watchdog that auto-stops the drive
    # if it stops seeing fresh commands -- republish throughout the hold like
    # a real controller would, instead of one-shot-then-sleep, so the
    # watchdog never fires mid-test.
    move_msg = AckermannDriveStamped()
    move_msg.drive.steering_angle = 0.0
    move_msg.drive.speed = speed_mps
    samples.clear()
    hold_start = time.monotonic()
    publish_count = publish_hold(node, pub, move_msg, duration_s)
    print(f"  (published {publish_count} drive commands during the hold)")

    # Average the steady-state portion instead of a single end-of-hold sample:
    # one snapshot lands at an arbitrary point on a noisy signal, which made
    # run-to-run numbers vary wildly and useless for comparing directions.
    # Drop the spin-up window so the acceleration ramp doesn't drag the mean.
    steady = [v for t, v in samples if t - hold_start >= spinup_s]
    if steady:
        mean = statistics.fmean(steady)
        stdev = statistics.pstdev(steady) if len(steady) > 1 else 0.0
        print(
            f"  steady-state (after {spinup_s}s spin-up, n={len(steady)}): "
            f"mean {mean:.1f} deg/s, min {min(steady):.1f}, max {max(steady):.1f}, stdev {stdev:.1f}",
        )
        moving_feedback = mean
    else:
        print(f"  WARNING: no feedback samples after the {spinup_s}s spin-up window")
        moving_feedback = latest.get("drive")

    moving_ok = moving_feedback is not None and abs(moving_feedback) >= DRIVE_MOVING_THRESHOLD_DEG_S
    print(f"  [{'PASS' if moving_ok else 'FAIL'}] moving: commanded {speed_mps} m/s -> {moving_feedback} deg/s")
    passed &= moving_ok

    stop_msg = AckermannDriveStamped()
    stop_msg.drive.steering_angle = 0.0
    stop_msg.drive.speed = 0.0
    pub.publish(stop_msg)
    stopped_feedback = _wait_for_condition(node, latest, "drive", lambda v: abs(v) < DRIVE_MOVING_THRESHOLD_DEG_S)
    stopped_ok = stopped_feedback is not None and abs(stopped_feedback) < DRIVE_MOVING_THRESHOLD_DEG_S
    print(f"  [{'PASS' if stopped_ok else 'FAIL'}] stop: commanded 0.0 m/s -> feedback {stopped_feedback} deg/s")
    passed &= stopped_ok

    return passed


def run_min_speed_test(node: Node, pub, latest: dict, candidates: list[float], hold_s: float) -> float | None:
    """Step through `candidates` (ascending) and report the first that moves the drive motor.

    Tracks the peak |feedback| seen during each hold instead of a single
    end-of-hold snapshot -- a speed right at the deadband threshold can
    produce a brief flicker of movement rather than a steady reading.
    """
    print(f"\n=== Minimum drive speed detection (candidates: {candidates}) ===")
    found = None
    for speed in candidates:
        move_msg = AckermannDriveStamped()
        move_msg.drive.steering_angle = 0.0
        move_msg.drive.speed = speed
        peak = 0.0

        def _track_peak() -> None:
            nonlocal peak
            value = latest.get("drive")
            if value is not None:
                peak = max(peak, abs(value))

        publish_hold(node, pub, move_msg, hold_s, on_spin=_track_peak)

        moved = peak >= DRIVE_MOVING_THRESHOLD_DEG_S
        print(f"  {speed:+.3f} m/s -> peak feedback {peak:.1f} deg/s [{'MOVED' if moved else 'no movement'}]")
        if moved and found is None:
            found = speed

        stop_msg = AckermannDriveStamped()
        stop_msg.drive.steering_angle = 0.0
        stop_msg.drive.speed = 0.0
        pub.publish(stop_msg)
        _wait_for_condition(node, latest, "drive", lambda v: abs(v) < DRIVE_MOVING_THRESHOLD_DEG_S, timeout_s=1.5)
        time.sleep(0.3)

        if found is not None:
            break

    if found is not None:
        print(f"\nMinimum speed that produced movement: {found} m/s")
    else:
        print("\nNo candidate speed produced movement -- deadband is above the highest candidate tested.")
    return found


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--steering-angle-deg", type=float, default=15.0)
    parser.add_argument("--skip-drive", action="store_true", help="Only run the steering test")
    parser.add_argument("--drive-speed", type=float, default=0.1, help="Drive test speed in m/s (kept low by default)")
    parser.add_argument("--drive-duration-s", type=float, default=1.5)
    parser.add_argument("--yes", "-y", action="store_true", help="Skip the drive test confirmation prompt")
    parser.add_argument(
        "--find-min-speed",
        action="store_true",
        help="Instead of the fixed-speed drive test, step through --min-speed-candidates "
        "(ascending) and report the first that actually moves the drive motor",
    )
    parser.add_argument(
        "--min-speed-candidates",
        type=str,
        default="0.02,0.04,0.06,0.08,0.1,0.15,0.2,0.3",
        help="Comma-separated m/s values to try, ascending, for --find-min-speed",
    )
    parser.add_argument("--min-speed-hold-s", type=float, default=1.0, help="Hold duration per candidate speed")
    parser.add_argument(
        "--spinup-s",
        type=float,
        default=1.0,
        help="Seconds of the drive hold to discard as acceleration ramp before averaging steady-state feedback",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("test_motors_probe")
    latest: dict[str, float] = {}
    drive_samples: list[tuple[float, float]] = []

    def _on_drive(msg: Float32) -> None:
        latest["drive"] = msg.data
        drive_samples.append((time.monotonic(), msg.data))

    node.create_subscription(Float32, "/motor/steering_position", lambda m: latest.__setitem__("steering", m.data), 10)
    node.create_subscription(Float32, "/motor/drive_speed", _on_drive, 10)
    pub = node.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
    time.sleep(0.5)  # let discovery/matching settle before the first publish

    steering_passed = run_steering_test(node, pub, latest, args.steering_angle_deg)

    drive_passed = True
    if not args.skip_drive:
        if not args.yes:
            reply = input(
                "\nThe drive test will spin the wheels. Confirm the robot is on a stand / "
                "wheels are clear, then type 'yes' to continue: ",
            )
            if reply.strip().lower() != "yes":
                print("Drive test skipped (not confirmed).")
                args.skip_drive = True
        if not args.skip_drive:
            if args.find_min_speed:
                candidates = [float(v) for v in args.min_speed_candidates.split(",")]
                min_speed = run_min_speed_test(node, pub, latest, candidates, args.min_speed_hold_s)
                drive_passed = min_speed is not None
            else:
                drive_passed = run_drive_test(
                    node,
                    pub,
                    latest,
                    args.drive_speed,
                    args.drive_duration_s,
                    drive_samples,
                    args.spinup_s,
                )

    node.destroy_node()
    rclpy.shutdown()

    print("\n==================================")
    print(f"Steering: {'PASS' if steering_passed else 'FAIL'}")
    if not args.skip_drive:
        print(f"Drive:    {'PASS' if drive_passed else 'FAIL'}")
    print("==================================")

    raise SystemExit(0 if (steering_passed and drive_passed) else 1)


if __name__ == "__main__":
    main()
