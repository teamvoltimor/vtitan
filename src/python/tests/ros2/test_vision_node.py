"""Mock-hardware tests for VisionNode — the real node deployed on the

Raspberry Pi 5 (registered console script, launched by rpi5_nodes.launch.py).
No real YOLO/Hailo model or camera is touched: create_detector is mocked so
these tests exercise the node's actual wiring, callback, and parameter-toggle
logic against a fake detector.

camera_source='direct' *capture* (actually opening Picamera2/rpicam-cli) is
out of scope here: it requires real camera hardware bindings that are
legitimately absent on a dev machine, the same reason the direct-capture path
is untested on sibling hardware nodes. The per-run video recording gating
(TestVideoRecordingGating below) IS covered, though -- it only depends on
camera_source='direct' being selected, not on a live capture; _start_direct_capture
itself is mocked out so no real camera is ever touched.
"""

from __future__ import annotations

import json
from unittest import mock

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, LaserScan
from shared.config.ros_topics import RosMessageType, RosTopicConfig
from shared.domain.models import Detection, SignColor
from std_msgs.msg import String

from tests.ros2.common_node_fixtures import wait_for_graph_entry, wait_for_subscriptions_info


@pytest.fixture()
def mock_detector():
    return mock.MagicMock(name="detector")


@pytest.fixture()
def vision_node_class(mock_detector):
    """Import VisionNode with create_detector mocked so no real model loads."""
    with mock.patch("src.ros2.vision.node.create_detector", return_value=mock_detector) as create_detector_mock:
        from src.ros2.vision.node import VisionNode

        yield VisionNode, create_detector_mock


def _image_msg(*, height: int = 2, width: int = 2, encoding: str = "rgb8") -> Image:
    msg = Image()
    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.data = bytes(height * width * 3)
    return msg


def _sign_detection(color: SignColor = SignColor.RED, confidence: float = 0.9) -> Detection:
    x1, y1, x2, y2 = 1.0, 2.0, 5.0, 6.0
    w, h = x2 - x1, y2 - y1
    return Detection(
        class_name=color,
        confidence=confidence,
        bbox=(x1, y1, x2, y2),
        x=(x1 + x2) / 2,
        y=(y1 + y2) / 2,
        width=w,
        height=h,
        area=w * h,
    )


class TestVisionNodeInit:
    def test_creates_detections_publisher_on_the_default_topic(self, ros_context, vision_node_class):
        VisionNode, _ = vision_node_class
        node = VisionNode()

        detections_topic = RosTopicConfig.load_default().sensors.vision_detections
        topics = wait_for_graph_entry(
            node,
            lambda: dict(node.get_publisher_names_and_types_by_node(node.get_name(), "")),
            detections_topic,
        )
        assert detections_topic in topics
        assert topics[detections_topic] == [RosMessageType.STRING]

        node.destroy_node()

    def test_annotated_publisher_absent_by_default(self, ros_context, vision_node_class):
        VisionNode, _ = vision_node_class
        node = VisionNode()

        assert node._annotated_publisher is None

        node.destroy_node()

    def test_raw_publisher_absent_by_default(self, ros_context, vision_node_class):
        VisionNode, _ = vision_node_class
        node = VisionNode()

        assert node._raw_publisher is None

        node.destroy_node()

    def test_annotated_publisher_created_when_param_enabled(self, ros_context, vision_node_class, monkeypatch):
        """No parameter_overrides plumbing exists on VisionNode's own __init__, so this
        drives the same Config() default VisionNode reads at declare_parameter time --
        env vars outrank the TOML file per HardwareBaseSettings' source order."""
        monkeypatch.setenv("VISION_NODE_PUBLISH_ANNOTATED", "true")
        VisionNode, _ = vision_node_class
        node = VisionNode()

        assert node._annotated_publisher is not None
        topics = dict(node.get_publisher_names_and_types_by_node(node.get_name(), ""))
        assert "/vision/image_annotated" in topics

        node.destroy_node()

    def test_raw_publisher_created_when_param_enabled(self, ros_context, vision_node_class, monkeypatch):
        monkeypatch.setenv("VISION_NODE_PUBLISH_RAW", "true")
        VisionNode, _ = vision_node_class
        node = VisionNode()

        assert node._raw_publisher is not None

        node.destroy_node()

    def test_topic_camera_source_subscribes_to_camera_topic(self, ros_context, vision_node_class):
        VisionNode, _ = vision_node_class
        node = VisionNode()

        subs = wait_for_subscriptions_info(node, RosTopicConfig.load_default().sensors.camera_image_raw)
        assert len(subs) == 1
        assert subs[0].topic_type == RosMessageType.IMAGE

        node.destroy_node()

    def test_detector_built_with_yolo_backend_by_default(self, ros_context, vision_node_class):
        VisionNode, create_detector_mock = vision_node_class
        node = VisionNode()

        assert create_detector_mock.call_args[0][0] == "yolo"

        node.destroy_node()

    def test_detector_built_with_hailo_backend_when_configured(self, ros_context, vision_node_class, monkeypatch):
        """Only the backend selection is exercised -- no real Hailo device is opened.

        _detection_threshold reads HailoConfig().min_confidence for this branch,
        which is a pure pydantic-settings TOML/env read with no hardware I/O.
        """
        monkeypatch.setenv("VISION_NODE_BACKEND", "hailo")
        VisionNode, create_detector_mock = vision_node_class
        node = VisionNode()

        assert create_detector_mock.call_args[0][0] == "hailo"

        node.destroy_node()

    def test_detector_enter_called_when_backend_is_a_context_manager(self, ros_context):
        cm_detector = mock.MagicMock(name="hailo_detector")
        with mock.patch("src.ros2.vision.node.create_detector", return_value=cm_detector):
            from src.ros2.vision.node import VisionNode

            node = VisionNode()

        cm_detector.__enter__.assert_called_once()
        node.destroy_node()


class TestVisionNodeImageCallback:
    def test_valid_rgb8_frame_runs_detection_and_publishes(self, ros_context, vision_node_class, mock_detector):
        VisionNode, _ = vision_node_class
        mock_detector.detect.return_value = [_sign_detection()]
        node = VisionNode()
        published = []
        node._publisher.publish = published.append

        node._image_callback(_image_msg())

        mock_detector.detect.assert_called_once()
        assert len(published) == 1
        payload = json.loads(published[0].data)
        assert len(payload) == 1
        assert payload[0]["class_name"] == str(SignColor.RED)
        assert payload[0]["confidence"] == pytest.approx(0.9)
        assert payload[0]["bbox"] == [1.0, 2.0, 5.0, 6.0]

        node.destroy_node()

    def test_no_detections_publishes_empty_list(self, ros_context, vision_node_class, mock_detector):
        VisionNode, _ = vision_node_class
        mock_detector.detect.return_value = []
        node = VisionNode()
        published = []
        node._publisher.publish = published.append

        node._image_callback(_image_msg())

        assert len(published) == 1
        assert json.loads(published[0].data) == []

        node.destroy_node()

    def test_bgr8_frame_is_converted_before_detection(self, ros_context, vision_node_class, mock_detector):
        VisionNode, _ = vision_node_class
        mock_detector.detect.return_value = []
        node = VisionNode()

        msg = _image_msg(height=1, width=1, encoding="bgr8")
        msg.data = bytes([10, 20, 30])  # BGR

        node._image_callback(msg)

        seen: np.ndarray = mock_detector.detect.call_args[0][0]
        assert seen[0, 0].tolist() == [30, 20, 10]  # converted to RGB

        node.destroy_node()

    def test_unsupported_encoding_is_skipped_without_raising(self, ros_context, vision_node_class, mock_detector):
        VisionNode, _ = vision_node_class
        node = VisionNode()
        published = []
        node._publisher.publish = published.append

        node._image_callback(_image_msg(encoding="mono8"))

        mock_detector.detect.assert_not_called()
        assert published == []

        node.destroy_node()

    def test_detector_exception_is_caught_and_does_not_raise(self, ros_context, vision_node_class, mock_detector):
        VisionNode, _ = vision_node_class
        mock_detector.detect.side_effect = RuntimeError("model exploded")
        node = VisionNode()
        published = []
        node._publisher.publish = published.append

        node._image_callback(_image_msg())  # must not raise

        assert published == []

        node.destroy_node()

    def test_detector_exception_of_an_unlisted_type_is_also_caught(
        self,
        ros_context,
        vision_node_class,
        mock_detector,
    ):
        """_process's second except clause is a catch-all beyond RuntimeError/ValueError/TypeError."""
        VisionNode, _ = vision_node_class
        mock_detector.detect.side_effect = KeyError("unexpected")
        node = VisionNode()
        published = []
        node._publisher.publish = published.append

        node._image_callback(_image_msg())  # must not raise

        assert published == []

        node.destroy_node()

    def test_publishes_annotated_frame_when_enabled(self, ros_context, vision_node_class, mock_detector, monkeypatch):
        monkeypatch.setenv("VISION_NODE_PUBLISH_ANNOTATED", "true")
        VisionNode, _ = vision_node_class
        mock_detector.detect.return_value = [_sign_detection()]
        node = VisionNode()
        published = []
        node._annotated_publisher.publish = published.append

        node._image_callback(_image_msg())

        assert len(published) == 1
        assert isinstance(published[0], Image)

        node.destroy_node()

    def test_publishes_raw_frame_when_enabled(self, ros_context, vision_node_class, mock_detector, monkeypatch):
        monkeypatch.setenv("VISION_NODE_PUBLISH_RAW", "true")
        VisionNode, _ = vision_node_class
        mock_detector.detect.return_value = []
        node = VisionNode()
        published = []
        node._raw_publisher.publish = published.append

        node._image_callback(_image_msg())

        assert len(published) == 1
        assert isinstance(published[0], Image)

        node.destroy_node()


class TestOnSetParameters:
    def test_enabling_publish_annotated_creates_the_publisher(self, ros_context, vision_node_class):
        VisionNode, _ = vision_node_class
        node = VisionNode()
        assert node._annotated_publisher is None

        result = node._on_set_parameters([Parameter("publish_annotated", Parameter.Type.BOOL, value=True)])

        assert result.successful
        assert node._publish_annotated is True
        assert node._annotated_publisher is not None

        node.destroy_node()

    def test_disabling_publish_annotated_destroys_the_publisher(self, ros_context, vision_node_class, monkeypatch):
        monkeypatch.setenv("VISION_NODE_PUBLISH_ANNOTATED", "true")
        VisionNode, _ = vision_node_class
        node = VisionNode()
        assert node._annotated_publisher is not None

        result = node._on_set_parameters([Parameter("publish_annotated", Parameter.Type.BOOL, value=False)])

        assert result.successful
        assert node._publish_annotated is False
        assert node._annotated_publisher is None

        node.destroy_node()

    def test_setting_debug_stream_fps_updates_the_min_interval(self, ros_context, vision_node_class):
        VisionNode, _ = vision_node_class
        node = VisionNode()

        node._on_set_parameters([Parameter("debug_stream_fps", Parameter.Type.DOUBLE, value=10.0)])

        assert node._annotated_min_interval == pytest.approx(0.1)

        node.destroy_node()

    def test_zero_debug_stream_fps_means_uncapped(self, ros_context, vision_node_class, monkeypatch):
        monkeypatch.setenv("VISION_NODE_DEBUG_STREAM_FPS", "10.0")
        VisionNode, _ = vision_node_class
        node = VisionNode()
        assert node._annotated_min_interval == pytest.approx(0.1)

        node._on_set_parameters([Parameter("debug_stream_fps", Parameter.Type.DOUBLE, value=0.0)])

        assert node._annotated_min_interval == 0.0

        node.destroy_node()

    def test_unrelated_parameter_is_ignored(self, ros_context, vision_node_class):
        VisionNode, _ = vision_node_class
        node = VisionNode()

        result = node._on_set_parameters([Parameter("model_path", Parameter.Type.STRING, value="other.pt")])

        assert result.successful

        node.destroy_node()


class TestVisionNodeDestroy:
    def test_destroy_exits_detector_context_manager(self, ros_context):
        cm_detector = mock.MagicMock(name="hailo_detector")
        with mock.patch("src.ros2.vision.node.create_detector", return_value=cm_detector):
            from src.ros2.vision.node import VisionNode

            node = VisionNode()

        node.destroy_node()

        cm_detector.__exit__.assert_called_once_with(None, None, None)

    def test_destroy_without_camera_does_not_raise(self, ros_context, vision_node_class):
        """camera_source='topic' never opens self._camera -- destroy must still be a no-op there."""
        VisionNode, _ = vision_node_class
        node = VisionNode()

        node.destroy_node()  # must not raise


@pytest.fixture()
def direct_node_class(mock_detector, monkeypatch):
    """VisionNode built for camera_source='direct' -- the only mode the per-run
    video recorder is ever active in -- with the real camera connection and the
    real VideoRecorder both mocked out. Real camera hardware bindings are
    legitimately absent on a dev machine (see the module docstring); the real
    VideoRecorder would spin up a genuine thread + cv2.VideoWriter per test,
    which the wiring tests below don't need and shouldn't pay for.
    """
    monkeypatch.setenv("VISION_NODE_CAMERA_SOURCE", "direct")
    with (
        mock.patch("src.ros2.vision.node.create_detector", return_value=mock_detector),
        mock.patch("src.ros2.vision.node.VisionNode._start_direct_capture"),
        mock.patch("src.ros2.vision.node.VideoRecorder") as recorder_cls,
    ):
        from src.ros2.vision.node import VisionNode

        yield VisionNode, recorder_cls


class TestVideoRecordingGating:
    """Per-run annotated video: RACING + a known run path from bag_recorder_node,
    arriving in any order -- see
    docs/internal/plans/2026-08-11-run-video-recording-colocated-with-mcap.md.
    Runs on both challenges (Obstacles already carries strictly more load on
    the same pipeline than Open Challenge ever will) -- see
    docs/internal/plans/2026-08-11-navigation-hud-overlay-and-open-challenge-recording.md.
    """

    def test_direct_mode_subscribes_to_every_gating_and_hud_topic(self, ros_context, direct_node_class):
        VisionNode, _ = direct_node_class
        node = VisionNode()
        topics = RosTopicConfig.load_default()

        assert len(wait_for_subscriptions_info(node, topics.state_machine.state)) == 1
        assert len(wait_for_subscriptions_info(node, topics.challenge_mode.active)) == 1
        assert len(wait_for_subscriptions_info(node, topics.bag_recorder.run_path)) == 1
        assert len(wait_for_subscriptions_info(node, topics.navigation.nav_debug)) == 1
        assert len(wait_for_subscriptions_info(node, topics.sensors.scan)) == 1

        node.destroy_node()

    def test_topic_mode_never_subscribes_to_gating_or_hud_topics(self, ros_context, vision_node_class):
        """camera_source='topic' (sim/test) has no race concept -- must not wire up
        state it will never act on."""
        VisionNode, _ = vision_node_class
        node = VisionNode()
        topics = RosTopicConfig.load_default()

        assert node.get_subscriptions_info_by_topic(topics.state_machine.state) == []
        assert node.get_subscriptions_info_by_topic(topics.challenge_mode.active) == []
        assert node.get_subscriptions_info_by_topic(topics.bag_recorder.run_path) == []
        assert node.get_subscriptions_info_by_topic(topics.navigation.nav_debug) == []
        assert node.get_subscriptions_info_by_topic(topics.sensors.scan) == []

        node.destroy_node()

    def test_starts_recording_once_racing_and_run_path_known(self, ros_context, direct_node_class, tmp_path):
        VisionNode, recorder_cls = direct_node_class
        node = VisionNode()
        recorder = recorder_cls.return_value
        recorder.is_recording = False

        node._on_run_path(String(data=str(tmp_path)))
        node._on_robot_state(String(data="racing"))
        node._poll_for_run_path_dir()  # directory already exists -- simulate the timer's first tick

        recorder.start.assert_called_once_with(tmp_path / "video.mp4")

        node.destroy_node()

    def test_open_challenge_also_starts_recording(self, ros_context, direct_node_class, tmp_path):
        """No longer Obstacles-only -- see the class docstring."""
        VisionNode, recorder_cls = direct_node_class
        node = VisionNode()
        recorder = recorder_cls.return_value
        recorder.is_recording = False

        node._on_challenge_mode_active(String(data="open"))
        node._on_run_path(String(data=str(tmp_path)))
        node._on_robot_state(String(data="racing"))
        node._poll_for_run_path_dir()

        recorder.start.assert_called_once_with(tmp_path / "video.mp4")

        node.destroy_node()

    def test_record_video_false_never_starts_recording(self, ros_context, mock_detector, monkeypatch):
        monkeypatch.setenv("VISION_NODE_CAMERA_SOURCE", "direct")
        monkeypatch.setenv("VISION_NODE_RECORD_VIDEO", "false")
        with (
            mock.patch("src.ros2.vision.node.create_detector", return_value=mock_detector),
            mock.patch("src.ros2.vision.node.VisionNode._start_direct_capture"),
            mock.patch("src.ros2.vision.node.VideoRecorder") as recorder_cls,
        ):
            from src.ros2.vision.node import VisionNode

            node = VisionNode()
        recorder = recorder_cls.return_value
        recorder.is_recording = False

        node._on_run_path(String(data="/some/path"))
        node._on_robot_state(String(data="racing"))

        recorder.start.assert_not_called()

        node.destroy_node()

    def test_directory_never_appearing_gives_up_without_starting(self, ros_context, direct_node_class, tmp_path):
        VisionNode, recorder_cls = direct_node_class
        node = VisionNode()
        recorder = recorder_cls.return_value
        recorder.is_recording = False

        missing = tmp_path / "never_created"
        node._on_run_path(String(data=str(missing)))
        node._on_robot_state(String(data="racing"))
        # Force the timeout branch on the very first poll instead of waiting
        # out the real _RUN_PATH_POLL_TIMEOUT_SEC.
        node._run_path_poll_deadline = 0.0
        node._poll_for_run_path_dir()

        recorder.start.assert_not_called()
        assert node._run_path_poll_timer is None  # gave up, not left armed forever

        node.destroy_node()

    def test_leaving_racing_stops_an_active_recording(self, ros_context, direct_node_class):
        VisionNode, recorder_cls = direct_node_class
        node = VisionNode()
        recorder = recorder_cls.return_value
        recorder.is_recording = True
        node._on_robot_state(String(data="racing"))

        node._on_robot_state(String(data="finished"))

        recorder.stop.assert_called_once()

        node.destroy_node()

    def test_challenge_flipping_mid_race_does_not_stop_recording(self, ros_context, direct_node_class):
        """Challenge is a HUD label now, not a gate -- see the class docstring."""
        VisionNode, recorder_cls = direct_node_class
        node = VisionNode()
        recorder = recorder_cls.return_value
        recorder.is_recording = True
        node._racing = True

        node._on_challenge_mode_active(String(data="open"))

        recorder.stop.assert_not_called()

        node.destroy_node()

    def test_destroy_node_stops_an_active_recording(self, ros_context, direct_node_class):
        VisionNode, recorder_cls = direct_node_class
        node = VisionNode()
        recorder = recorder_cls.return_value
        recorder.is_recording = True

        node.destroy_node()

        recorder.stop.assert_called_once()

    def test_annotate_runs_for_recording_even_when_publish_annotated_is_off(
        self, ros_context, direct_node_class, mock_detector,
    ):
        VisionNode, recorder_cls = direct_node_class
        mock_detector.detect.return_value = [_sign_detection()]
        node = VisionNode()
        recorder = recorder_cls.return_value
        recorder.is_recording = True

        with mock.patch("src.ros2.vision.node.annotate") as annotate_mock:
            annotate_mock.return_value = np.zeros((2, 2, 3), dtype=np.uint8)
            node._process(np.zeros((2, 2, 3), dtype=np.uint8))

        annotate_mock.assert_called_once()
        recorder.submit.assert_called_once()

        node.destroy_node()

    def test_neither_annotate_nor_submit_run_when_nothing_wants_annotated_frames(
        self, ros_context, direct_node_class, mock_detector,
    ):
        VisionNode, recorder_cls = direct_node_class
        mock_detector.detect.return_value = []
        node = VisionNode()
        recorder = recorder_cls.return_value
        recorder.is_recording = False

        with mock.patch("src.ros2.vision.node.annotate") as annotate_mock:
            node._process(np.zeros((2, 2, 3), dtype=np.uint8))

        annotate_mock.assert_not_called()
        recorder.submit.assert_not_called()

        node.destroy_node()


class TestHudTelemetryCaching:
    """/nav_debug and /scan feed the recorded video's HUD (src/vision/hud.py) --
    see docs/internal/plans/2026-08-11-navigation-hud-overlay-and-open-challenge-recording.md.
    Neither is a recording gate; both may still be None when a snapshot is built.
    """

    def test_nav_debug_is_cached_as_parsed_json(self, ros_context, direct_node_class):
        VisionNode, _ = direct_node_class
        node = VisionNode()

        node._on_nav_debug(String(data=json.dumps({"phase": "normal_drive", "laps_completed": 1})))

        assert node._nav_debug == {"phase": "normal_drive", "laps_completed": 1}

        node.destroy_node()

    def test_scan_is_cached(self, ros_context, direct_node_class):
        VisionNode, _ = direct_node_class
        node = VisionNode()
        scan = LaserScan(angle_min=-1.0, angle_increment=0.5, ranges=[1.0, 2.0, 3.0])

        node._on_scan(scan)

        assert node._scan is scan

        node.destroy_node()

    def test_snapshot_has_none_telemetry_before_either_topic_arrives(self, ros_context, direct_node_class):
        VisionNode, _ = direct_node_class
        node = VisionNode()

        snapshot = node._build_frame_snapshot(np.zeros((2, 2, 3), dtype=np.uint8))

        assert snapshot.nav_debug is None
        assert snapshot.scan_ranges is None
        assert snapshot.scan_angles is None
        assert snapshot.active_challenge is None

        node.destroy_node()

    def test_snapshot_carries_cached_nav_debug_and_challenge(self, ros_context, direct_node_class):
        VisionNode, _ = direct_node_class
        node = VisionNode()
        node._on_nav_debug(String(data=json.dumps({"phase": "normal_drive"})))
        node._on_challenge_mode_active(String(data="obstacles"))

        snapshot = node._build_frame_snapshot(np.zeros((2, 2, 3), dtype=np.uint8))

        assert snapshot.nav_debug == {"phase": "normal_drive"}
        assert snapshot.active_challenge == "obstacles"

        node.destroy_node()

    def test_snapshot_derives_scan_angles_from_angle_min_and_increment(self, ros_context, direct_node_class):
        VisionNode, _ = direct_node_class
        node = VisionNode()
        node._on_scan(LaserScan(angle_min=-1.0, angle_increment=0.5, ranges=[1.0, 2.0, 3.0]))

        snapshot = node._build_frame_snapshot(np.zeros((2, 2, 3), dtype=np.uint8))

        # Deliberately RAW /scan bearings, no mount-inversion/yaw-offset
        # correction applied here -- draw_radar (src/vision/hud.py) does that
        # itself now, config-driven from HudConfig.lidar_inverted/
        # lidar_yaw_offset_deg.
        assert snapshot.scan_ranges == [1.0, 2.0, 3.0]
        assert snapshot.scan_angles == pytest.approx([-1.0, -0.5, 0.0])

        node.destroy_node()
