"""Tests for telemetry_bridge_node — the real node deployed on the

Raspberry Pi 5. No real backend/HTTP is touched: these tests exercise the
pure lidar/detection helpers and the /ui/telemetry_summary publisher against
directly-constructed sensor_msgs, not a live requests.Session.
"""

from __future__ import annotations

import json
import threading
import time
from unittest import mock

import pytest
import rclpy
from sensor_msgs.msg import Imu, LaserScan
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


@pytest.fixture()
def bridge_module():
    from vtitan_state_machine import telemetry_bridge_node as module

    return module


@pytest.fixture()
def bridge_node(bridge_module):
    node = bridge_module.TelemetryBridgeNode()
    yield node
    node.destroy_node()


def _detection(class_id: str, score: float, size_x: float, size_y: float) -> Detection2D:
    det = Detection2D()
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = class_id
    hyp.hypothesis.score = score
    det.results.append(hyp)
    det.bbox = BoundingBox2D(size_x=size_x, size_y=size_y)
    return det


def _window(center_idx: int, half_width: int, n: int) -> range:
    """Indices in ``[center_idx - half_width, center_idx + half_width)``, wrapped mod n."""
    return (i % n for i in range(center_idx - half_width, center_idx + half_width))


class TestLidarClearancesCm:
    """Ported from oled_display_node's now-deleted _lidar_callback tests -- same logic, relocated."""

    # angles = linspace(-pi, pi, n, endpoint=False) + _LIDAR_YAW_OFFSET_RAD
    # (180deg, the C1's mount offset) -- 0.5deg/index at n=720. Since the
    # offset is exactly pi, corrected_angle(i) = i * (2*pi/n) exactly: front
    # sits at index 0 (angle 0, wrapping through n-1), left at n/4 (+90deg),
    # right at 3n/4 (-90deg). The real sector is +/-30deg, i.e. +/-60 index --
    # a +/-62-index window fully covers it (with a hair of margin) so the
    # mean isn't diluted by the background value at the edges.
    def test_front_is_mean_over_the_forward_sector(self, bridge_module):
        n = 720
        ranges = [10.0] * n
        for i in _window(0, 62, n):
            ranges[i] = 0.5

        c = bridge_module._lidar_clearances(ranges)

        assert c.front_m * 100 == pytest.approx(50.0, abs=0.5)

    def test_left_is_mean_over_the_plus_90_sector(self, bridge_module):
        n = 720
        ranges = [10.0] * n
        for i in _window(n // 4, 62, n):
            ranges[i] = 1.5

        c = bridge_module._lidar_clearances(ranges)

        assert c.left_m * 100 == pytest.approx(150.0, abs=0.5)

    def test_right_is_mean_over_the_minus_90_sector(self, bridge_module):
        n = 720
        ranges = [10.0] * n
        for i in _window(3 * n // 4, 62, n):
            ranges[i] = 0.3

        c = bridge_module._lidar_clearances(ranges)

        assert c.right_m * 100 == pytest.approx(30.0, abs=0.5)

    def test_a_single_noisy_point_barely_moves_the_mean(self, bridge_module):
        """The old min-based approach let one stray point dominate a whole sector -- mean shouldn't."""
        n = 720
        ranges = [1.0] * n
        ranges[0] = 0.02  # one spurious near-range return, dead ahead

        c = bridge_module._lidar_clearances(ranges)

        assert c.front_m * 100 == pytest.approx(100.0, abs=5.0)

    def test_left_and_right_filter_self_detection_but_front_does_not(self, bridge_module):
        """Front must still register a genuine near-contact; sides discard chassis self-reflection."""
        n = 720
        ranges = [10.0] * n
        for i in _window(0, 62, n):
            ranges[i] = 0.02  # inside LIDAR_SELF_DETECTION_THRESHOLD (0.08m)
        for i in _window(n // 4, 62, n):
            ranges[i] = 0.02
        for i in _window(3 * n // 4, 62, n):
            ranges[i] = 0.02

        c = bridge_module._lidar_clearances(ranges)

        assert c.front_m * 100 == pytest.approx(2.0, abs=0.5)  # not filtered
        assert c.left_m == 0.0  # filtered out entirely -- no valid points left
        assert c.right_m == 0.0

    def test_empty_ranges_returns_zeros(self, bridge_module):
        c = bridge_module._lidar_clearances([])
        assert (c.front_m, c.left_m, c.right_m) == (0.0, 0.0, 0.0)

    def test_no_valid_points_in_a_window_defaults_to_zero(self, bridge_module):
        n = 720
        ranges = [0.0] * n  # every reading below the min-valid floor

        c = bridge_module._lidar_clearances(ranges)

        assert (c.front_m, c.left_m, c.right_m) == (0.0, 0.0, 0.0)


class TestBestDetection:
    """Ported from oled_display_node's now-deleted _detections_callback tests -- same logic, relocated."""

    def test_picks_confidence_times_area(self, bridge_module):
        """Neither the highest-confidence nor the largest box alone -- the product."""
        msg = Detection2DArray()
        # High confidence, tiny box: 0.95 * (5*5) = 23.75
        msg.detections.append(_detection("small_far_sign", 0.95, 5.0, 5.0))
        # Lower confidence, much larger box: 0.6 * (40*40) = 960
        msg.detections.append(_detection("large_near_sign", 0.6, 40.0, 40.0))

        assert bridge_module._best_detection(msg) == ("large_near_sign", 0.6)

    def test_skips_results_without_a_hypothesis(self, bridge_module):
        msg = Detection2DArray()
        empty = Detection2D()  # no results appended -- must not raise
        msg.detections.append(empty)

        assert bridge_module._best_detection(msg) is None


class TestPublishUiSummary:
    def test_publishes_defaults_with_no_data_yet(self, ros_context, bridge_node):
        published = []
        bridge_node._ui_summary_pub.publish = published.append

        bridge_node._publish_ui_summary()

        assert len(published) == 1
        data = json.loads(published[0].data)
        assert data == {
            "lidar_front_cm": 0.0,
            "lidar_left_cm": 0.0,
            "lidar_right_cm": 0.0,
            "gyro_yaw_deg": 0.0,
            "best_detection_class_id": None,
            "best_detection_confidence": None,
        }

    def test_publishes_cached_scan_and_imu_and_vision(self, ros_context, bridge_node):
        n = 720
        ranges = [10.0] * n
        for i in _window(0, 62, n):
            ranges[i] = 0.186
        scan = LaserScan()
        scan.ranges = ranges
        bridge_node._scan_callback(scan)

        imu = Imu()
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = 0.707
        imu.orientation.w = 0.707
        bridge_node._imu_callback(imu)

        vision = Detection2DArray()
        vision.detections.append(_detection("red_sign", 0.9, 10.0, 10.0))
        bridge_node._vision_callback(vision)

        published = []
        bridge_node._ui_summary_pub.publish = published.append
        bridge_node._publish_ui_summary()

        data = json.loads(published[0].data)
        assert data["lidar_front_cm"] == pytest.approx(18.6, abs=0.1)
        assert data["gyro_yaw_deg"] == pytest.approx(90.0, abs=1.0)
        assert data["best_detection_class_id"] == "red_sign"
        assert data["best_detection_confidence"] == pytest.approx(0.9)


class TestPublishTelemetryDoesNotBlock:
    """A slow/hung backend must never stall this node's own callback processing.

    _publish_telemetry now hands snapshots off to TelemetryIngestChannel's
    push_snapshot/push_topics, which are non-blocking by construction (each
    backed by a maxsize=1 keep-latest queue.Queue, drained by the channel's
    own background threads) -- so there's no in-flight tracking left at the
    bridge-node level the way the old HTTP POST path required. These tests
    stub out the channel itself (no real threads/grpc) and assert
    _publish_telemetry returns immediately and calls push_snapshot/
    push_topics exactly once each, with the freshly-built dataclasses,
    regardless of the channel's connection state.
    """

    def test_publish_telemetry_returns_immediately_and_pushes_snapshot_and_topics(self, ros_context, bridge_node):
        pushed_snapshots = []
        pushed_topics = []
        bridge_node._telemetry_channel.push_snapshot = pushed_snapshots.append
        bridge_node._telemetry_channel.push_topics = pushed_topics.append

        start = time.perf_counter()
        bridge_node._publish_telemetry()
        elapsed = time.perf_counter() - start

        assert elapsed < 0.2, "push_snapshot/push_topics must not block _publish_telemetry"
        assert len(pushed_snapshots) == 1
        assert len(pushed_topics) == 1
        assert pushed_snapshots[0].missionName == bridge_node._build_snapshot().missionName

    def test_publish_telemetry_still_pushes_when_channel_reports_disconnected(self, ros_context, bridge_node):
        """push_snapshot/push_topics never raise/block even while the channel is disconnected.

        Doesn't need a real disconnected TelemetryIngestChannel here --
        push_snapshot/push_topics are non-blocking by construction (see
        test_telemetry_ingest_channel.py's TestPushLatest), so plain mocks
        standing in for them are enough to pin _publish_telemetry's own
        contract: call both, once each, and return.
        """
        bridge_node._telemetry_channel.push_snapshot = mock.Mock()
        bridge_node._telemetry_channel.push_topics = mock.Mock()

        bridge_node._publish_telemetry()

        bridge_node._telemetry_channel.push_snapshot.assert_called_once()
        bridge_node._telemetry_channel.push_topics.assert_called_once()

    def test_second_ticks_data_replaces_the_first_via_keep_latest_queue(self, bridge_module):
        """Ported from the old bridge-node-level in-flight test -- now expressed against

        TelemetryIngestChannel's keep-latest queue directly (see
        test_telemetry_ingest_channel.py's TestPushLatest for the
        thorough coverage); this just confirms the bridge node's two push
        calls per tick land on that same drop-oldest-keep-latest queue.
        """
        from vtitan_state_machine.telemetry_ingest_channel import TelemetryIngestChannel

        channel = TelemetryIngestChannel(
            backend_target="localhost:9010",
            logger=mock.Mock(),
        )
        first = mock.Mock(name="first")
        second = mock.Mock(name="second")

        channel._push_latest(channel._snapshot_queue, first)
        channel._push_latest(channel._snapshot_queue, second)

        assert channel._snapshot_queue.qsize() == 1
        assert channel._snapshot_queue.get_nowait() is second


class TestUiSummaryRoundTripsWithOledNode:
    """RPi 5 (telemetry_bridge_node) -> RPi Zero (oled_display_node).

    Composes the real producer with the real consumer -- no DDS transport --
    so schema drift between the two boards fails a test instead of surfacing
    for the first time on the physical robot (the same class of bug
    test_cross_board_contracts.py exists to catch).
    """

    def test_summary_round_trips_into_oled_node_state(self, ros_context, bridge_node):
        import sys
        from unittest import mock

        for optional in ("board", "busio", "adafruit_ssd1306", "fcntl"):
            if optional not in sys.modules:
                try:
                    __import__(optional)
                except (ImportError, NotImplementedError):
                    sys.modules[optional] = mock.MagicMock()

        from vtitan_drivers.oled_display_node import OLEDDisplayNode

        n = 720
        ranges = [10.0] * n
        for i in _window(0, 62, n):
            ranges[i] = 0.186
        scan = LaserScan()
        scan.ranges = ranges
        bridge_node._scan_callback(scan)

        vision = Detection2DArray()
        vision.detections.append(_detection("red_sign", 0.9, 10.0, 10.0))
        bridge_node._vision_callback(vision)

        published = []
        bridge_node._ui_summary_pub.publish = published.append
        bridge_node._publish_ui_summary()
        summary_msg = published[0]

        oled_node = OLEDDisplayNode()
        try:
            oled_node._ui_summary_callback(summary_msg)

            assert oled_node.lidar_front == pytest.approx(18.6, abs=0.1)
            assert oled_node.best_detection == ("red_sign", pytest.approx(0.9))
        finally:
            oled_node.destroy_node()
