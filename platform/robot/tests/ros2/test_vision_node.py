"""Mock-hardware tests for VisionNode — the real node deployed on the

Raspberry Pi 5 (registered console script, launched by rpi5_nodes.launch.py).
No real YOLO/Hailo model or camera is touched: create_detector is mocked so
these tests exercise the node's actual wiring, callback, and parameter-toggle
logic against a fake detector.

camera_source='direct' (Picamera2/rpicam-cli frame-grabbing) is out of scope
here: it requires real camera hardware bindings that are legitimately absent
on a dev machine, the same reason the direct-capture path is untested on
sibling hardware nodes.
"""

from __future__ import annotations

import json
from typing import TYPE_CHECKING
from unittest import mock

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from src.vision.detector import SignDetection, TrafficSignColor

if TYPE_CHECKING:
    import numpy as np


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


def _sign_detection(color: TrafficSignColor = TrafficSignColor.RED, confidence: float = 0.9) -> SignDetection:
    return SignDetection(color=color, bbox=(1.0, 2.0, 5.0, 6.0), confidence=confidence)


class TestVisionNodeInit:
    def test_creates_detections_publisher_on_the_default_topic(self, ros_context, vision_node_class):
        VisionNode, _ = vision_node_class
        node = VisionNode()

        topics = dict(node.get_publisher_names_and_types_by_node(node.get_name(), ""))
        assert "/vision/detections" in topics
        assert topics["/vision/detections"] == ["std_msgs/msg/String"]

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

        subs = node.get_subscriptions_info_by_topic("/camera/image_raw")
        assert len(subs) == 1
        assert subs[0].topic_type == "sensor_msgs/msg/Image"

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
        assert payload[0]["class_name"] == str(TrafficSignColor.RED)
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
