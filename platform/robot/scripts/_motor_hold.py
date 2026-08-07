"""Shared timing loop for holding an /ackermann_cmd publish against the watchdog.

ackermann_motor_node stops the drive motor ~1s after the last command it
sees, so any script that wants to hold a command for longer than that must
keep republishing throughout the hold instead of firing once and sleeping.
Every scripts/*.py hardware probe that drives the real motor
(calibrate-encoder.py, test-motors.py, reset-motors.py) needs this same loop;
this is the one copy.
"""

from __future__ import annotations

import time
from collections.abc import Callable

import rclpy
from rclpy.node import Node


def publish_hold(
    node: Node,
    pub,
    msg,
    duration_s: float,
    *,
    publish_interval_s: float = 0.1,
    spin_timeout_s: float = 0.02,
    on_spin: Callable[[], None] | None = None,
) -> int:
    """Republish `msg` on `pub` every `publish_interval_s`, spinning `node`
    every `spin_timeout_s`, until `duration_s` has elapsed.

    `publish_interval_s=0.0` publishes on every spin instead of a throttled
    schedule -- for a plain stop-and-coast hold rather than a rate-matched
    command stream. `on_spin`, if given, runs once after each spin (e.g. to
    poll feedback for a live peak/min-max while the hold is in progress).
    Returns the number of publishes made.
    """
    deadline = time.monotonic() + duration_s
    next_publish = 0.0
    count = 0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_publish:
            pub.publish(msg)
            count += 1
            next_publish = now + publish_interval_s
        rclpy.spin_once(node, timeout_sec=spin_timeout_s)
        if on_spin is not None:
            on_spin()
    return count
