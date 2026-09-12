#!/usr/bin/env python3
"""Measure the servo's real slew rate by bisection, using only the naked eye.

``MAX_STEERING_RATE`` ships at 1.2 rad/s and has never been measured. It is not
cosmetic: ``bay_exit`` budgets its servo standstill from it as
``ceil(swing / (rate / CONTROL_HZ))``, which at 1.2 rad/s is 50 ticks of
commanded ZERO after every leg change. Confirmed on track -- the commanded-zero
runs in the bay last p50 2.551 / 2.556 / 2.552 s against the 2.50 s the constant
predicts -- and at 8-14 reversals that is most of the manoeuvre.

    rate (rad/s)   ticks   per reversal   9 reversals
    1.2 (shipped)     50       2.50 s        22.2 s
    3.0               20       1.00 s         9.0 s
    5.0               12       0.60 s         5.4 s

**The bags cannot settle it.** ``/motor/steering_position`` is an ECHO of the
command: fitted against ``/ackermann_cmd`` it gives slope 57.2958 (= 180/pi) at
lag 0 with R^2 = 1.000000, and ``ServoDriver.get_steering_position`` says so in
its own docstring. The gyro gives only a lower bound (>= ~1.27 rad/s at p90),
because the lock-to-lock swings happen in the bay at ~0 speed where there is no
yaw to read.

THE METHOD, and it needs no high-speed camera and no extra hardware. Drive the
wheel to one lock, command the OTHER lock, hold for ``t``, then command the first
lock again. Then ask a question the naked eye answers reliably:

    did the wheel REACH the far stop, or did it turn back part-way?

If it reached, ``t`` is at least the slew time. If it turned back early, ``t`` is
less. Bisect on ``t`` and the boundary IS the lock-to-lock slew time -- which is
exactly the swing ``bay_exit`` budgets, so the number lands in the same units the
constant is used in, with no scaling assumption in between.

Judging "did it arrive" is far easier than judging a duration, and easier still
because a ``t`` above the boundary gives a visible DWELL at the stop before the
wheel reverses. That dwell appearing is the signal.

**Run it with the wheel LOADED against the mat, not in the air.** Unloaded reads
optimistic -- tyre scrub binds the linkage, and the bay's swings happen with the
chassis's weight on the wheel.

**Watch the ANGLE too.** ``robot.toml`` flags ``max_wheel_angle_deg = 85.0`` as
NOT bench-verified, its predecessor was wrong by 1.28x, and it multiplies
straight into the same budget. If the wheel visibly stops short of the commanded
angle, say so -- that is the other measurement this trip is for.

Steering only. The drive motor is never touched, so this is safe with the robot
sitting on the mat.

Usage (on the Pi 5, which has ROS2 discovery to the Pi Zero's motor node)::

    python3 scripts/hardware/diag_servo_slew.py              # interactive bisection
    python3 scripts/hardware/diag_servo_slew.py --ladder     # fixed descending ladder
    python3 scripts/hardware/diag_servo_slew.py --angle-deg 60
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from shared.config.constants import RobotSpecs

QOS_ACKERMANN_CMD = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    deadline=Duration(nanoseconds=200_000_000),
)
"""Restated rather than imported from ``src.ros2.qos``: this script runs from
``scripts/`` on the Pi without the repo root on ``sys.path``, and a hardware tool
that cannot start is worth less than a duplicated four-line profile. It must stay
identical to the navigator's -- a deadline mismatch would silently drop commands,
which is the failure this whole measurement was chasing."""

_COMMAND_HZ = 20.0
"""Republish rate, matching ``ControlParams.CONTROL_HZ``. The topic is a STREAM
on this robot, not a latched setpoint."""

_SETTLE_S = 1.5
"""Time parked at the starting lock before a trial, so every trial begins from
the same place and the wheel is stationary rather than still arriving."""

_RECOVER_S = 3.0
"""Rest at CENTRE between trials. Generous on purpose: a trial that starts from
a half-way position measures nothing, and the gap is what lets an observer tell
one trial from the next."""

_REVERSAL_S = 1.6
"""Time given to the reversal in step 3, long enough that it is unmistakably a
turn BACK rather than the tail of the dash."""

_LO_S, _HI_S = 0.10, 3.00
"""Bracket for the bisection. 3.00 s clears the shipped 1.2 rad/s (which would
need 2.47 s lock-to-lock) and 0.10 s is below any plausible servo, so the
boundary is inside by construction."""

_ROUNDS = 6
"""Bisection steps. Six halvings of a 2.9 s bracket land at ~45 ms, which is
under one control tick at 20 Hz -- finer than the budget can use."""


class _Servo:
    def __init__(self) -> None:
        rclpy.init()
        self._node = rclpy.create_node("diag_servo_slew")
        # The SAME profile the navigator publishes with. A default profile is
        # not obviously wrong here, but "obviously" is what this session keeps
        # paying for, and a deadline mismatch would silently drop commands.
        self._pub = self._node.create_publisher(AckermannDriveStamped, "/ackermann_cmd", QOS_ACKERMANN_CMD)

    def command(self, angle_deg: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.drive.steering_angle = math.radians(angle_deg)
        msg.drive.speed = 0.0
        self._pub.publish(msg)

    def hold(self, angle_deg: float, seconds: float) -> None:
        """Command ``angle_deg`` CONTINUOUSLY for ``seconds``, at the control rate.

        One message and a sleep is not how this topic is driven: the navigator
        republishes every control tick, and both the motor node's closed loop
        and its watchdog are built for a stream. Sending a single command and
        waiting produced "they barely moved at all" on the mat across every
        trial of the first three attempts at this measurement.
        """
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.command(angle_deg)
            time.sleep(1.0 / _COMMAND_HZ)

    def close(self) -> None:
        # Centre on EVERY exit path, Ctrl-C included: a servo left at full lock
        # against a loaded wheel sits there drawing stall current.
        self.command(0.0)
        time.sleep(0.2)
        self._node.destroy_node()
        rclpy.shutdown()


def _trial(servo: _Servo, angle_deg: float, hold_s: float) -> None:
    """One trial: settle at one lock, dash for the other, REVERSE, then park centre.

    Two things have to be true at once and the first two versions of this each
    got one of them, which is why it is written out here.

    The dash must end in a REVERSAL, not in a move to centre. Centre lies ON the
    path of the dash, so a short hold makes the wheel continue in the SAME
    direction and simply stop at 0 -- one smooth crossing with nothing to see.
    Reported from the mat: "around the seventh I saw it crossing again", which is
    exactly that, and it is why the signal has to be the wheel turning BACK
    toward the lock it came from.

    And trials must be separable, which parking at a lock destroys: every leg is
    then a 170 deg sweep and the timed dash looks like the return. So the trial
    ends by parking at CENTRE, well after the reversal has been seen.

    Sequence, and only step 2 is timed:

        1. go to -angle and settle
        2. GO: command +angle, hold ``hold_s``
        3. command -angle  <- the reversal; this is the measurement
        4. let it get back
        5. park at centre, which marks the end of the trial

    Above the slew time the wheel ARRIVES at +angle and DWELLS before step 3
    turns it round. Below it, the wheel turns round MID-SWEEP having never
    touched the stop.
    """
    servo.hold(-angle_deg, _SETTLE_S)
    print(f"\n  >>> GO  (hold {hold_s:.2f} s)", flush=True)
    servo.hold(+angle_deg, hold_s)
    servo.hold(-angle_deg, _REVERSAL_S)
    servo.hold(0.0, _RECOVER_S)


def _ask(hold_s: float) -> bool:
    while True:
        answer = input(f"      did it REACH the far stop at {hold_s:.2f} s? [y/n/r=repeat] ").strip().lower()
        if answer in ("y", "s", "si"):
            return True
        if answer == "n":
            return False
        if answer == "r":
            return None  # type: ignore[return-value]
        print("      answer y (reached), n (turned back early), or r (repeat this trial)")


def _report(swing_rad: float, lo: float, hi: float) -> None:
    print(f"\nBoundary bracketed: turned back at {lo:.2f} s, reached at {hi:.2f} s.")
    print(f"Lock-to-lock swing is {math.degrees(swing_rad):.1f} deg = {swing_rad:.3f} rad, so:")
    print(f"  rate is between {swing_rad / hi:.2f} and {swing_rad / lo:.2f} rad/s")
    print(f"  take {swing_rad / ((lo + hi) / 2):.2f} rad/s as the estimate")
    print("\nShipped MAX_STEERING_RATE is 1.2 rad/s. At 20 Hz the bay budgets")
    print(f"  ceil({swing_rad:.3f} / (rate / 20)) ticks of commanded ZERO per leg change:")
    for rate in (1.2, swing_rad / ((lo + hi) / 2)):
        ticks = math.ceil(swing_rad / (rate / 20.0))
        print(f"    {rate:4.2f} rad/s -> {ticks:3d} ticks = {ticks / 20.0:.2f} s per reversal")
    print("\nAlso report the ANGLE the wheel actually reached: max_wheel_angle_deg is")
    print("unverified and multiplies into this same swing.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__ or "", formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--angle-deg",
        type=float,
        default=RobotSpecs.MAX_WHEEL_ANGLE_DEG,
        help="Half-swing in road-wheel degrees; each trial dashes from -angle to +angle.",
    )
    parser.add_argument(
        "--ladder",
        action="store_true",
        help="Run a fixed descending ladder instead of bisecting, for when watching the wheel "
        "and answering prompts at once is awkward. Report afterwards which hold time was the "
        "smallest that still arrived.",
    )
    args = parser.parse_args()

    swing_rad = math.radians(2 * args.angle_deg)
    servo = _Servo()
    print(f"Lock-to-lock is {2 * args.angle_deg:.1f} deg. Wheel must be LOADED against the mat.")
    print("Watch the WHEEL, not the screen. The question each time is only:")
    print("      did it reach the far stop, or turn back part-way?")

    try:
        if args.ladder:
            for hold_s in (2.50, 1.80, 1.20, 0.90, 0.70, 0.55, 0.45, 0.35, 0.25):
                _trial(servo, args.angle_deg, hold_s)
                print(f"      ^ that was {hold_s:.2f} s", flush=True)
            print("\nReport the SMALLEST hold time that still reached the stop.")
            return 0

        lo, hi = _LO_S, _HI_S
        for round_i in range(_ROUNDS):
            mid = (lo + hi) / 2
            print(f"\n[{round_i + 1}/{_ROUNDS}]  bracket {lo:.2f}-{hi:.2f} s", flush=True)
            while True:
                _trial(servo, args.angle_deg, mid)
                answer = _ask(mid)
                if answer is not None:
                    break
            if answer:
                hi = mid
            else:
                lo = mid
        _report(swing_rad, lo, hi)
    except KeyboardInterrupt:
        print("\ninterrupted -- centring")
    finally:
        servo.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
