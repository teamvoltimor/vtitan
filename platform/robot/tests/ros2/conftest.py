"""
ROS2 test fixtures and configuration.
"""

import os
import re
import sys
from pathlib import Path
from unittest import mock

import pytest
import rclpy

# Every ros_context fixture use calls rclpy.init()/shutdown() with no domain
# isolation, so under pytest-xdist all worker processes' DDS participants
# share ROS_DOMAIN_ID 0 and discover each other's dozens-of-nodes-per-test
# churn over multicast -- the actual cause behind the "graph discovery race"
# flakiness in test_button_node.py/test_state_machine_node.py/etc (wider
# retry timeouts didn't fix it because the bottleneck is the shared discovery
# domain, not any one node's discovery latency). Give each xdist worker its
# own domain so its participants are only ever visible to itself. Falls back
# to domain 0 outside xdist (a plain `pytest` run has only one process).
_worker_id = os.environ.get("PYTEST_XDIST_WORKER", "")
_worker_num = int(m.group()) if (m := re.search(r"\d+", _worker_id)) else 0
os.environ["ROS_DOMAIN_ID"] = str(1 + _worker_num % 200)

# Mock buildhat to avoid import errors when testing ROS2 nodes.
#
# Only the BuildHAT motor backend imports it, and only lazily -- see
# ackermann_motor_node._shared_build_hat, reached solely when
# STEERING_BACKEND/DRIVE_BACKEND select build_hat. The default backends
# (servo / dc_encoder) never touch it.
#
# No "buildhat.serinterface" shim any more: the button driver used to do
# `from buildhat.serinterface import threading` -- an accidental route to the
# stdlib that made an unrelated LEGO serial module a hard dependency of the
# button on the Pi Zero. It imports plain `threading` now, so mocking the
# submodule would only hide a reintroduction of that import.
sys.modules["buildhat"] = mock.MagicMock()

# Adafruit Blinka's "board" raises NotImplementedError at import on any host it
# cannot identify, which is every dev machine that is not the Pi. It is reached
# transitively -- the UART-RVC node imports the bno08x package, whose __init__
# pulls in the I2C driver, which imports board -- so the failure surfaces as an
# unrelated node refusing to construct. Mocked for the same reason buildhat is:
# these tests exercise node wiring, not the sensor.
for _blinka in ("board", "busio"):
    if _blinka not in sys.modules:
        try:
            __import__(_blinka)
        except (ImportError, NotImplementedError):
            sys.modules[_blinka] = mock.MagicMock()

# The real per-board node implementations (ackermann_motor_node, button_node,
# state_machine_node, ...) live across several ament_python packages under
# ros2_ws/src/vtitan_* (drivers/navigation/vision/state_machine/bringup),  # noqa: ERA001 -- prose, not commented-out code; ruff misreads this glob-like package path
# none of which are on PYTHONPATH — they're normally only importable after a
# colcon build. They're pure Python (ament_python, no compiled extensions), so
# adding each source dir directly lets tests import the exact deployed node
# code without needing a full ROS2 workspace build.
_ROS2_WS_SRC = Path(__file__).resolve().parents[2] / "ros2_ws" / "src"
for _pkg_dir in sorted(_ROS2_WS_SRC.glob("vtitan_*")):
    if str(_pkg_dir) not in sys.path:
        sys.path.insert(0, str(_pkg_dir))


@pytest.fixture()
def ros_context():
    """Initialize and cleanup ROS2 context for each test."""
    try:
        rclpy.init()
        yield
        rclpy.shutdown()
    except Exception as e:
        pytest.skip(f"ROS2 initialization failed: {e}")
