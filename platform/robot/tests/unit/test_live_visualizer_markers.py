"""Marker-construction tests for the live RViz visualizer.

Two unrelated hazards, both invisible to plain attribute access on the message
object: JSON ints reaching a float64 field (see below), and the hand-derived
quaternion that stands the wheel cylinders on their axles.

Regression test for JSON-int → ROS message float64 corruption in sign/parking markers.

Whole-number scenario-metadata coordinates (e.g. ``"x": 1``) parse from JSON as a Python
``int``, not ``float``. Assigning that ``int`` straight to a ``geometry_msgs/Point`` field
reads back fine in memory (Python doesn't enforce the annotation), but CDR serialization
onto the DDS wire reinterprets its bits as a float64 instead of converting the value,
silently collapsing it to a near-zero subnormal (~4e-323) — which showed up in RViz as
signs and parking blocks jumping onto a wall. This test forces an actual CDR
serialize/deserialize round trip, which is the only way to catch this class of bug: plain
attribute access on the in-memory message object does NOT reproduce it.
"""

from __future__ import annotations

import math

import pytest
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from shared.domain.enums import Section
from visualization_msgs.msg import Marker

from src.simulation.live_visualizer import (
    LiveScenarioVisualizer,
    _wheel_to_quaternion,
    init_rclpy_once,
)
from src.simulation.track_model import TrackModel


@pytest.fixture(autouse=True)
def _shutdown_rclpy_after_test():
    """Undo init_rclpy_once()'s process-lifetime init after each test here.

    init_rclpy_once() is correct for its real caller (a long-lived visualizer
    script that runs until process exit, so it never needs to shut back
    down) but leaves rclpy.ok() True for the rest of the pytest session if
    left as-is here -- breaking any later test/fixture elsewhere in the
    suite that calls rclpy.init() unconditionally, expecting a clean
    context (e.g. test_imu_bno08x_i2c_node.py's fixture, or
    track_navigator_node.main()).
    """
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _wide_track() -> TrackModel:
    return TrackModel({
        Section.NORTH: 1.0,
        Section.SOUTH: 1.0,
        Section.EAST: 1.0,
        Section.WEST: 1.0,
    })


def _rotate(q, v: tuple[float, float, float]) -> tuple[float, float, float]:
    """Apply a geometry_msgs Quaternion to a vector, via v + 2u x (u x v + w v)."""
    u = (q.x, q.y, q.z)
    t = tuple(2.0 * c for c in _cross(u, v))
    return tuple(v[i] + q.w * t[i] + _cross(u, t)[i] for i in range(3))


def _cross(a, b) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


@pytest.mark.parametrize("degrees", [0, 15, -15, 40, -55])
@pytest.mark.parametrize("roll_degrees", [0, 90, -90, 240])
def test_wheel_quaternion_lays_the_cylinder_on_a_steered_axle(degrees, roll_degrees):
    """The wheel orientation is a hand-composed Rz(steer) * Rx(90deg) * Rz(-roll).

    A Marker CYLINDER extrudes along its own +z, so an unrotated one stands on
    end like a bollard; the Rx(90deg) term lays it flat with its axle across the
    car, Rz(steer) steers it, and the last term spins it about that axle.
    Composed as a closed-form quaternion product rather than by multiplying
    three Quaternion objects, so the arithmetic is worth pinning: get a sign
    wrong and the wheels still render, just rolling sideways or steering
    backwards -- which reads as a chassis bug, not a drawing bug, exactly when
    you are using this view to diagnose one.

    Checked by rotating the cylinder's own axis and asserting where it lands,
    which is independent of how the quaternion was built. Rolling must NOT move
    the axle at all -- that is what makes it a roll rather than a wobble.
    """
    steer = math.radians(degrees)
    axle = _rotate(_wheel_to_quaternion(steer, math.radians(roll_degrees)), (0.0, 0.0, 1.0))

    # Rx(90deg) sends +z to -y; Rz(steer) then swings that to (sin, -cos, 0).
    assert axle[0] == pytest.approx(math.sin(steer), abs=1e-9)
    assert axle[1] == pytest.approx(-math.cos(steer), abs=1e-9)
    assert axle[2] == pytest.approx(0.0, abs=1e-9), "axle must stay level with the ground"

    # The wheel rolls perpendicular to its axle, and that heading is the steer.
    rolling = (math.cos(steer), math.sin(steer), 0.0)
    assert sum(a * r for a, r in zip(axle, rolling, strict=True)) == pytest.approx(0.0, abs=1e-9)


def test_positive_roll_spins_the_wheel_forwards():
    """Sign check: driving forward must not render as wheels spinning backwards.

    Traced on the spoke mark, since that is the only part of the wheel whose
    rotation is visible at all. At rest it lies along the wheel's forward
    diameter; a quarter turn of FORWARD roll has to carry that tip down and
    under the axle, not up and over it.
    """
    tip = _rotate(_wheel_to_quaternion(0.0, math.radians(90)), (1.0, 0.0, 0.0))

    assert tip[2] == pytest.approx(-1.0, abs=1e-9), "forward roll must carry the mark downward"
    assert tip[0] == pytest.approx(0.0, abs=1e-9)


def test_frame_ids_reach_the_wire_as_plain_strings():
    """TfFrames is a StrEnum, and marker frame_ids are assigned members directly.

    rclpy does not coerce on assignment, and this file already documents one
    case where an in-memory field looked right and serialized wrong, so the
    substitution is checked through an actual CDR round trip rather than by
    trusting that a str subclass behaves like str everywhere.
    """
    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name="test_frame_id_node")
    try:
        marker = visualizer._robot_model_markers(steer=0.0).markers[0]
        roundtripped = deserialize_message(serialize_message(marker), Marker)
        assert roundtripped.header.frame_id == "base_link"
        assert type(roundtripped.header.frame_id) is str
    finally:
        visualizer.destroy_node()


def test_sign_marker_survives_cdr_roundtrip_with_int_json_coords():
    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name="test_sign_marker_node")
    try:
        # Whole-number x/y, exactly as json.loads() would hand back from `"x": 1`.
        sign = {"x": 1, "y": 2, "color": "red"}
        marker = visualizer._sign_marker(0, sign)
        roundtripped = deserialize_message(serialize_message(marker), Marker)
        assert roundtripped.pose.position.x == 1.0
        assert roundtripped.pose.position.y == 2.0
    finally:
        visualizer.destroy_node()


def test_parking_block_marker_survives_cdr_roundtrip_with_int_json_coords():
    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name="test_parking_marker_node")
    try:
        block = {"x": 3, "y": 2}
        marker = visualizer._parking_block_marker(10, block, yaw=0.0)
        roundtripped = deserialize_message(serialize_message(marker), Marker)
        assert roundtripped.pose.position.x == 3.0
        assert roundtripped.pose.position.y == 2.0
    finally:
        visualizer.destroy_node()
