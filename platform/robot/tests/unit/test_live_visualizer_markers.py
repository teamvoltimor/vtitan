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
from shared.config.constants import TrafficSignSpecs
from shared.domain.enums import Section
from shared.domain.models import BlockPosition, ParkingLot, SignPosition
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


def _apply(offset, point):
    """Put a believed-frame point into map, the way RViz composes map->belief."""
    off_x, off_y, off_yaw = offset
    cos_o, sin_o = math.cos(off_yaw), math.sin(off_yaw)
    return (
        off_x + point[0] * cos_o - point[1] * sin_o,
        off_y + point[0] * sin_o + point[1] * cos_o,
    )


@pytest.mark.parametrize(
    ("true_yaw", "expected_rotation"),
    [(0.0, 0.0), (math.pi / 2, 90.0), (math.pi, 180.0), (-math.pi / 2, -90.0)],
)
def test_belief_frame_carries_the_section_relabelling_rotation(true_yaw, expected_rotation):
    """Blind always guesses SOUTH, so a start elsewhere rotates the whole plan.

    The four cases are the four sections: NORTH reads 180 deg, EAST 90, WEST
    -90, SOUTH 0. Drawing the plan in `map` instead of through this offset is
    what made go_obstacles_0002 (a WEST start) show a path square to the real
    track, with the sign estimates rotated to match.
    """
    init_rclpy_once()
    # abs(): rclpy rejects a node name containing '-', so -90 cannot go in as-is.
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name=f"test_belief_{abs(expected_rotation):.0f}")
    try:
        believed = (1.5, 0.25, 0.0)
        true = (0.25, 1.5, true_yaw)
        visualizer.set_belief_frame(believed, true)

        assert math.degrees(visualizer._belief_offset[2]) == pytest.approx(expected_rotation)
        # The defining property: the believed start must land on the true one.
        landed = _apply(visualizer._belief_offset, believed[:2])
        assert landed[0] == pytest.approx(true[0], abs=1e-9)
        assert landed[1] == pytest.approx(true[1], abs=1e-9)
    finally:
        visualizer.destroy_node()


class _FakeScan:
    """Minimal stand-in for a LidarScan — the visualizer only reads these two."""

    def __init__(self, angles_rad, ranges_m):
        self.angles_rad = angles_rad
        self.ranges_m = ranges_m


def _full_sweep(range_m: float = 2.0, count: int = 360):
    step = 2.0 * math.pi / count
    angles = [-math.pi + i * step for i in range(count)]
    return _FakeScan(angles, [range_m] * count)


def test_scan_masks_the_bearings_the_mount_cannot_see():
    """Drives the REAL publish path, not the builder in isolation.

    Testing _build_laserscan alone is what let a TypeError ship: nothing in
    this module had ever called publish() with a scan, so a bad call inside
    the scan branch was invisible to the whole suite while crashing on the
    first tick of any actual run.
    """
    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name="test_scan_mask")
    try:
        scan = _full_sweep()
        msg = visualizer._build_laserscan(scan, visualizer.get_clock().now().to_msg())
        sectors = visualizer._lidar_sectors
        by_degree = {
            round(math.degrees(a)): r for a, r in zip(scan.angles_rad, msg.ranges, strict=True)
        }

        # Straight back is masked edge to edge on this mount. The sweep spans
        # [-180, +180), so only one of the two ends is actually sampled.
        rear = [by_degree[d] for d in (180, -180) if d in by_degree]
        assert rear, "sweep should sample one of the two rear ends"
        assert all(math.isnan(r) for r in rear), "straight back must be blanked"
        assert math.isnan(by_degree[round(sectors.BLIND_WEDGE_RIGHT_MIN_DEG) + 5])
        assert math.isnan(by_degree[round(sectors.BLIND_WEDGE_LEFT_MAX_DEG) - 5])
        # Forward is untouched -- masking must not eat the useful sweep.
        for degree in (0, 45, -45, 90, -90):
            assert by_degree[degree] == pytest.approx(2.0), f"{degree} deg should survive"
    finally:
        visualizer.destroy_node()


def test_scan_mask_follows_the_config_not_a_literal():
    """Widen the wedges and more rays blank, with no code change.

    The point of reading lidar_sectors rather than restating the angles: if
    the mount changes, editing lidar_sectors.toml is the whole edit.
    """
    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name="test_scan_mask_config")
    try:
        stamp = visualizer.get_clock().now().to_msg()
        scan = _full_sweep()
        before = sum(math.isnan(r) for r in visualizer._build_laserscan(scan, stamp).ranges)

        # Same shape the real config uses, just reaching further forward.
        visualizer._lidar_sectors = visualizer._lidar_sectors.model_copy(
            update={"BLIND_WEDGE_LEFT_MAX_DEG": -90.0, "BLIND_WEDGE_RIGHT_MIN_DEG": 90.0},
        )
        after = sum(math.isnan(r) for r in visualizer._build_laserscan(scan, stamp).ranges)

        assert after > before, "widening the wedges must blank more rays"
    finally:
        visualizer.destroy_node()


def test_belief_frame_is_identity_for_a_sighted_run():
    """A correct belief must not move anything -- sighted runs are unaffected."""
    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name="test_belief_identity")
    try:
        pose = (1.5, 0.25, math.pi / 3)
        visualizer.set_belief_frame(pose, pose)

        assert visualizer._belief_offset == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
        assert _apply(visualizer._belief_offset, (2.0, 1.0)) == pytest.approx((2.0, 1.0), abs=1e-12)
    finally:
        visualizer.destroy_node()


def test_sign_marker_survives_cdr_roundtrip_with_int_json_coords():
    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name="test_sign_marker_node")
    try:
        # Whole-number x/y, exactly as json.loads() would hand back from `"x": 1`.
        # Validated rather than constructed, because validation is now the step
        # that does the int -> float conversion the marker builder used to do.
        sign = SignPosition.model_validate({"x": 1, "y": 2, "color": "red"})
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
        block = BlockPosition.model_validate({"x": 3, "y": 2})
        marker = visualizer._parking_block_marker(10, block, yaw=0.0)
        roundtripped = deserialize_message(serialize_message(marker), Marker)
        assert roundtripped.pose.position.x == 3.0
        assert roundtripped.pose.position.y == 2.0
    finally:
        visualizer.destroy_node()


def test_the_models_are_what_makes_the_int_coordinates_safe():
    """The guarantee the marker builders now lean on, asserted directly.

    Both builders dropped their float() casts when they started taking models
    instead of dicts. That is only correct if validation actually converts --
    if pydantic ever passed an int through, the CDR corruption above would come
    straight back, and it is silent.
    """
    sign = SignPosition.model_validate({"x": 1, "y": 2, "color": "red"})
    block = BlockPosition.model_validate({"x": 3, "y": 2})

    for value in (sign.x, sign.y, block.x, block.y):
        assert type(value) is float


def test_parking_lot_keeps_the_block_yaws():
    """simgen randomises these to 0 or pi/2; the model used to drop them.

    A lot with both blocks forced axis-aligned is not the bay the robot has to
    park in -- the blocks' orientation is what makes it a slot.
    """
    lot = ParkingLot.model_validate({
        "block1_position": {"x": 1, "y": 2},
        "block2_position": {"x": 3, "y": 2},
        "block1_yaw": math.pi / 2,
    })

    assert lot.block1_yaw == pytest.approx(math.pi / 2)
    assert lot.block2_yaw == 0.0, "absent yaw means axis-aligned, not missing"


def test_floor_marker_is_present_and_white():
    """set_track() publishes an opaque white floor plane before walls/markings.

    The floor is an unlit TRIANGLE_LIST box (constant colour at any view
    angle), so it carries vertices/per-vertex colours rather than a CUBE
    scale/pose.
    """
    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name="test_floor_plane")
    try:
        visualizer.set_track(_wide_track())
        floor = [m for m in visualizer._cached_track_markers.markers if m.ns == "floor"]
        assert len(floor) == 1
        marker = floor[0]
        assert marker.type == Marker.TRIANGLE_LIST
        # 12 triangles * 3 vertices for the box.
        assert len(marker.points) == 36
        # All vertices lie below z=0 so line markings render on top.
        assert all(p.z < 0.0 for p in marker.points)
        # Every vertex is opaque white.
        assert all(c.r == c.g == c.b == 1.0 and c.a == 1.0 for c in marker.colors)
    finally:
        visualizer.destroy_node()


def test_floor_markings_present_for_wide_track():
    """A wide track has four starting-square outlines, band/cell lines and corner lines."""
    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name="test_floor_markings")
    try:
        markers = visualizer._floor_marking_markers(_wide_track())
        by_ns: dict[str, list[Marker]] = {}
        for marker in markers:
            by_ns.setdefault(marker.ns, []).append(marker)

        floor = by_ns.get("floor_markings", [])
        assert len(floor) >= 4, "expected at least one marker per corridor"

        # Each wide corridor has: 1 outline + 2 band divisions + 1 cell division = 4.
        outlines = [m for m in floor if m.type == Marker.LINE_STRIP]
        assert len(outlines) == 4
        assert all(len(m.points) == 5 for m in outlines)  # 4 corners closed
    finally:
        visualizer.destroy_node()


def test_floor_markings_respect_narrow_corridor():
    """A narrow corridor drops one band division, leaving one division line instead of two."""
    init_rclpy_once()
    track = TrackModel({
        Section.NORTH: 0.6,
        Section.SOUTH: 0.6,
        Section.EAST: 0.6,
        Section.WEST: 0.6,
    })
    visualizer = LiveScenarioVisualizer(track, node_name="test_floor_narrow")
    try:
        markers = visualizer._floor_marking_markers(track)
        # Wide track has 4 outlines + 4*3 divisions + 8 corner lines = 24 markers.
        # Narrow track has 4 outlines + 4*2 divisions + 8 corner lines = 20 markers.
        assert len(markers) == 20
    finally:
        visualizer.destroy_node()


def test_sign_markers_are_cubes_not_cylinders():
    """Traffic signs are 5 cm × 5 cm × 10 cm rectangular prisms, not cylinders."""
    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name="test_sign_shape")
    try:
        sign = SignPosition.model_validate({"x": 1, "y": 2, "color": "red"})
        marker = visualizer._sign_marker(0, sign)
        assert marker.type == Marker.CUBE
        assert marker.scale.x == pytest.approx(TrafficSignSpecs.WIDTH)
        assert marker.scale.y == pytest.approx(TrafficSignSpecs.DEPTH)
        assert marker.scale.z == pytest.approx(TrafficSignSpecs.HEIGHT)
    finally:
        visualizer.destroy_node()


def test_sign_floor_markings_are_present():
    """Each sign has the 5 cm square and 85 mm placement circle drawn under it."""
    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name="test_sign_floor")
    try:
        sign = SignPosition.model_validate({"x": 1.5, "y": 2.5, "color": "green"})
        square, circle = visualizer._sign_floor_markers(0, sign)

        assert square.ns == "sign_floor_markings"
        assert square.type == Marker.LINE_STRIP
        assert len(square.points) == 5  # closed square
        square_half = TrafficSignSpecs.WIDTH / 2.0
        expected_corners = sorted([
            (sign.x + dx, sign.y + dy)
            for dx, dy in [
                (-square_half, -square_half),
                (square_half, -square_half),
                (square_half, square_half),
                (-square_half, square_half),
            ]
        ])
        actual_corners = sorted((p.x, p.y) for p in square.points[:4])
        for actual, expected in zip(actual_corners, expected_corners, strict=True):
            assert actual[0] == pytest.approx(expected[0])
            assert actual[1] == pytest.approx(expected[1])

        assert circle.ns == "sign_floor_markings"
        assert circle.type == Marker.LINE_STRIP
        assert len(circle.points) == 33  # 32 segments + closed
        radius = TrafficSignSpecs.PLACEMENT_CIRCLE_DIAMETER / 2.0
        for p in circle.points:
            assert math.hypot(p.x - sign.x, p.y - sign.y) == pytest.approx(radius, abs=1e-9)
    finally:
        visualizer.destroy_node()


def test_sign_estimate_marker_is_also_a_cube():
    """Believed signs use the same prism shape as ground-truth signs."""
    init_rclpy_once()
    visualizer = LiveScenarioVisualizer(_wide_track(), node_name="test_sign_estimate_shape")
    try:
        spec = SignPosition.model_validate({"x": 1, "y": 2, "color": "red"})
        marker = visualizer._sign_estimate_marker(0, spec)
        assert marker.type == Marker.CUBE
        assert marker.scale.x == pytest.approx(TrafficSignSpecs.WIDTH)
        assert marker.scale.y == pytest.approx(TrafficSignSpecs.DEPTH)
        assert marker.scale.z == pytest.approx(TrafficSignSpecs.HEIGHT)
    finally:
        visualizer.destroy_node()
