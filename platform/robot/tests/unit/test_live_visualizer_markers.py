"""Regression test for JSON-int → ROS message float64 corruption in sign/parking markers.

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

from rclpy.serialization import deserialize_message, serialize_message
from shared.config.enums import Section
from visualization_msgs.msg import Marker

from src.simulation.live_visualizer import LiveScenarioVisualizer, init_rclpy_once
from src.simulation.track_model import TrackModel


def _wide_track() -> TrackModel:
    return TrackModel({
        Section.NORTH: 1.0,
        Section.SOUTH: 1.0,
        Section.EAST: 1.0,
        Section.WEST: 1.0,
    })


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
