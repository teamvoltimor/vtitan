"""ROS2 Vision Node for YOLO detection.

Subscribes to camera images and publishes JSON detections using LocalYoloDetector.
"""

import json
import time
from contextlib import suppress
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from pydantic_settings import SettingsConfigDict
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from shared.config.ros_topics import RosTopicConfig
from shared.domain.enums import RobotState, ScenarioType
from std_msgs.msg import String

from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings
from src.ros2.vision.detection_payload_keys import (
    AREA_KEY,
    BBOX_KEY,
    CLASS_NAME_KEY,
    CONFIDENCE_KEY,
    HEIGHT_KEY,
    WIDTH_KEY,
    X_KEY,
    Y_KEY,
)
from src.vision import create_detector
from src.vision.overlay import annotate
from src.vision.video_recorder import VideoRecorder

if TYPE_CHECKING:
    from src.hardware.camera.base import Driver as CameraDriver


class Config(HardwareBaseSettings):
    """Fallback defaults for VisionNode's ROS2 parameters.

    Sourced from config/hardware/vision/node.toml. ``rpi5_nodes.launch.py``
    still overrides these at launch time via ROS2 parameters (e.g. to select
    the hailo backend and direct camera capture) -- this only changes what a
    node launched with no parameter overrides falls back to.
    """

    model_config = SettingsConfigDict(env_prefix="vision_node_", toml_file=CONFIG_DIR / "vision" / "node.toml")

    camera_topic: str = "/camera/image_raw"
    model_path: str = "yolov8n.pt"  # matches config/hardware/vision/node.toml
    backend: str = "yolo"  # 'yolo' or 'hailo'
    # 'direct' opens the camera in this process and feeds frames straight to
    # the model -- no sensor_msgs/Image on the wire, which is what a race
    # run wants. 'topic' keeps the subscription, for bag replay and sim.
    camera_source: str = "topic"  # 'topic' or 'direct'
    capture_fps: float = 15.0
    # Debug video, off by default: a race publishes detections and nothing
    # else. Both of these cost real bandwidth at speed.
    publish_annotated: bool = False
    annotated_topic: str = "/vision/image_annotated"
    publish_raw: bool = False
    # Caps the annotated stream's publish rate independent of capture_fps, so a
    # remote debug-toggle can also throttle bandwidth. 0 means uncapped.
    debug_stream_fps: float = 0.0
    # Per-run annotated video, written next to that run's mcap bag -- see
    # docs/internal/plans/2026-08-11-run-video-recording-colocated-with-mcap.md.
    # Only ever active in camera_source='direct' mode, gated on RACING and the
    # Obstacles Challenge (see _maybe_start_recording): Open Challenge never
    # has anything worth boxing, and this must never touch Open's behaviour.
    record_video: bool = True
    # Width of the recorded artifact; height is derived at runtime from the
    # actual captured frame's aspect ratio, never hardcoded.
    video_width: int = 640


# Matches state_machine_node's/telemetry_bridge_node's _QOS_TRANSIENT-style
# /system_status publishers: TRANSIENT_LOCAL so a late subscriber (the OLED,
# which restarts independently on the Pi Zero) gets this node's one startup
# publish instead of waiting for a periodic re-publish that never comes.
# BEST_EFFORT for the same reason those publishers are -- a RELIABLE writer
# blocks on a slow reader, which the OLED's own board has been measured doing
# for 30+ seconds under contention.
_QOS_SYSTEM_STATUS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

# Matches state_machine_node's /robot_state and /challenge_mode/active
# publishers, and bag_recorder_node's /bag_recorder/run_path -- all
# TRANSIENT_LOCAL + BEST_EFFORT, same rationale as _QOS_SYSTEM_STATUS above.
# A RELIABLE reader against any of these BEST_EFFORT writers is an
# incompatible QoS pair that DDS resolves by delivering nothing at all.
_QOS_LATCHED_STATE = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

# How long to poll for bag_recorder_node's run directory to actually appear
# on disk before giving up on video for this run. ros2 bag record creates its
# output directory itself, asynchronously, sometime after its subprocess
# starts -- rclpy init + discovery can take real wall-clock time. This node
# must never create that directory itself: doing so would make the bag
# process's own `-o <path>` call refuse to start, since rosbag2 requires the
# output directory not to already exist.
_RUN_PATH_POLL_INTERVAL_SEC = 0.1
_RUN_PATH_POLL_TIMEOUT_SEC = 3.0


class VisionNode(Node):
    """ROS2 node that runs YOLO detection on camera images."""

    def __init__(self) -> None:  # noqa: PLR0915 - constructor wires every subsystem together by design
        super().__init__("vision_detector")

        defaults = Config()
        topics = RosTopicConfig.load_default()
        self.declare_parameter("camera_topic", defaults.camera_topic)
        self.declare_parameter("detections_topic", topics.sensors.vision_detections)
        self.declare_parameter("model_path", defaults.model_path)
        self.declare_parameter("backend", defaults.backend)
        self.declare_parameter("camera_source", defaults.camera_source)
        self.declare_parameter("capture_fps", defaults.capture_fps)
        self.declare_parameter("publish_annotated", value=defaults.publish_annotated)
        self.declare_parameter("annotated_topic", defaults.annotated_topic)
        self.declare_parameter("publish_raw", value=defaults.publish_raw)
        self.declare_parameter("debug_stream_fps", defaults.debug_stream_fps)
        self.declare_parameter("record_video", value=defaults.record_video)
        self.declare_parameter("video_width", defaults.video_width)

        camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        backend = self.get_parameter("backend").get_parameter_value().string_value
        self._camera_source = self.get_parameter("camera_source").get_parameter_value().string_value
        capture_fps = self.get_parameter("capture_fps").get_parameter_value().double_value
        self._publish_annotated = self.get_parameter("publish_annotated").get_parameter_value().bool_value
        self._annotated_topic = self.get_parameter("annotated_topic").get_parameter_value().string_value
        self._publish_raw = self.get_parameter("publish_raw").get_parameter_value().bool_value
        debug_stream_fps = self.get_parameter("debug_stream_fps").get_parameter_value().double_value
        self._annotated_min_interval = 1.0 / debug_stream_fps if debug_stream_fps > 0 else 0.0
        self._last_annotated_pub_time = 0.0
        self._record_video = self.get_parameter("record_video").get_parameter_value().bool_value
        video_width = self.get_parameter("video_width").get_parameter_value().integer_value

        self.get_logger().info(f"Loading {backend.upper()} vision model from {model_path}...")

        from src.vision.detector import DEFAULT_CLASS_TO_COLOR, DetectorConfig  # noqa: PLC0415

        # Take the mapping from the detector rather than restating it: this copy
        # said (red, green, magenta), which is the dataset's stale order and the
        # opposite of what the model emits for red and green. It silently
        # inverts the WRO pass side on every obstacle.
        config = DetectorConfig(
            model_path=model_path,
            class_to_color=DEFAULT_CLASS_TO_COLOR,
            min_confidence=self._detection_threshold(backend),
        )
        detector = create_detector(backend, config)
        # Enter context manager for backends that hold hardware resources (Hailo).
        # For YOLO the __enter__ is a no-op; calling it unconditionally is safe.
        if hasattr(detector, "__enter__"):
            detector.__enter__()
        self.detector = detector
        self._publish_model_status(Path(model_path).name)

        self._publisher = self.create_publisher(String, detections_topic, 10)
        self._annotated_publisher = (
            self.create_publisher(Image, self._annotated_topic, 1) if self._publish_annotated else None
        )
        self._raw_publisher = self.create_publisher(Image, camera_topic, 1) if self._publish_raw else None
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Per-run annotated video, colocated with that run's mcap bag -- only
        # meaningful in direct-capture mode, since that's the only mode a real
        # race actually runs in. Cheap to construct even when never started.
        self._recorder = VideoRecorder(video_width=video_width, fps=capture_fps)
        self._racing = False
        self._active_challenge: ScenarioType | None = None
        self._run_path: str | None = None
        self._run_path_poll_timer = None
        self._run_path_poll_deadline = 0.0
        if self._camera_source == "direct":
            topics_state = topics.state_machine.state
            self.create_subscription(String, topics_state, self._on_robot_state, _QOS_LATCHED_STATE)
            self.create_subscription(
                String, topics.challenge_mode.active, self._on_challenge_mode_active, _QOS_LATCHED_STATE,
            )
            self.create_subscription(
                String, topics.bag_recorder.run_path, self._on_run_path, _QOS_LATCHED_STATE,
            )

        self._camera: CameraDriver | None = None
        self._subscription = None
        if self._camera_source == "direct":
            self._start_direct_capture(capture_fps)
            self.get_logger().info(
                f"Vision Node ready. Capturing directly at {capture_fps:g} fps, publishing to {detections_topic}"
                + (f" (+ annotated on {self._annotated_topic})" if self._publish_annotated else ""),
            )
        else:
            self._subscription = self.create_subscription(
                Image,
                camera_topic,
                self._image_callback,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                f"Vision Node ready. Subscribed to {camera_topic}, publishing to {detections_topic}",
            )

    def _publish_model_status(self, model_name: str) -> None:
        """Publish the real loaded model name to /system_status, once.

        The OLED's READY page used to show a hardcoded "yolov8n.hef" that had
        already drifted from the actually-deployed model (gmr.hef) -- there was
        no live source for this at all, just a string nobody updated when the
        model changed. state_machine_node and telemetry_bridge_node already
        both publish their own DiagnosticArray to this same topic and the OLED
        merges entries by name (see oled_display_node._diagnostics_callback),
        so a third publisher here costs nothing and can't disagree with the
        others -- it names one field ("VisionModel") that only this node ever
        sets.
        """
        topics = RosTopicConfig.load_default()
        pub = self.create_publisher(DiagnosticArray, topics.state_machine.system_status, _QOS_SYSTEM_STATUS)
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "VisionModel"
        status.level = DiagnosticStatus.OK
        status.message = model_name
        msg.status.append(status)
        pub.publish(msg)

    @staticmethod
    def _detection_threshold(backend: str) -> float:
        """Return the confidence floor detections must clear.

        For the Hailo backend this comes from HailoConfig, so HAILO_MIN_CONFIDENCE
        actually governs what reaches the navigator. It previously did not:
        HailoDetector filters on DetectorConfig.min_confidence, which nobody set,
        so the effective threshold was that dataclass's 0.25 default while the
        documented variable only fed a driver path the vision node never calls.
        """
        from src.vision.detector import DetectorConfig  # noqa: PLC0415

        if backend != "hailo":
            # model_path/class_to_color are always caller-supplied (see class
            # docstring) -- placeholders here since only min_confidence's
            # resolved TOML/env value is wanted.
            return DetectorConfig(model_path="", class_to_color={}).min_confidence
        from src.hardware.hailo.base import Config as HailoConfig  # noqa: PLC0415

        return HailoConfig().min_confidence

    def _start_direct_capture(self, capture_fps: float) -> None:
        """Open the camera in-process and drive detection from a timer.

        Prefers Picamera2 and falls back to the rpicam CLI, because picamera2
        is absent from every environment on the robot and cannot be installed
        into the pixi env its bindings would have to match.
        """
        # Distinct names per branch (not a shared alias reassigned in each) --
        # mypy treats a conditional import bound to the same name in both
        # branches as one incompatible reassignment, even though only one
        # branch's class is ever actually constructed. self._camera is typed
        # against the two backends' shared camera.base.Driver ABC instead, so
        # either concrete instance is a valid assignment.
        try:
            from src.hardware.camera.rpi.camera_module_3.driver import (  # noqa: PLC0415
                Config as PicamConfig,
                Driver as PicamDriver,
            )

            self._camera = PicamDriver(PicamConfig())
            backend = "picamera2"
        except ImportError:
            from src.hardware.camera.rpicam.driver import (  # noqa: PLC0415
                Config as RpicamConfig,
                Driver as RpicamDriver,
            )

            self._camera = RpicamDriver(RpicamConfig())
            backend = "rpicam-cli"

        self._camera.connect()
        size = self._camera.get_resolution()
        self.get_logger().info(
            f"Camera opened via {backend} at {size.width_px}x{size.height_px}, "
            f"rotation={size.rotation_deg}",
        )
        self._timer = self.create_timer(1.0 / max(capture_fps, 1.0), self._capture_once)

    def _capture_once(self) -> None:
        """Grab one frame and run the detection/publish path over it."""
        # Only ever scheduled by _start_direct_capture, right after self._camera
        # is set -- guaranteed non-None whenever this timer callback fires.
        assert self._camera is not None  # noqa: S101 - guaranteed by _start_direct_capture before scheduling
        try:
            frame = self._camera.capture_frame().frame
        except Exception as err:  # noqa: BLE001
            self.get_logger().error(f"Camera capture failed: {err}", throttle_duration_sec=5.0)
            return
        self._process(self._camera.to_rgb(frame))

    def _on_robot_state(self, msg: String) -> None:
        """Start/stop the per-run video recording.

        Same RACING transition track_navigator_node and bag_recorder_node
        already gate on.
        """
        was_racing = self._racing
        self._racing = msg.data.strip().lower() == RobotState.RACING.value
        if self._racing and not was_racing:
            self._maybe_start_recording()
        elif was_racing and not self._racing:
            self._stop_recording()

    def _on_challenge_mode_active(self, msg: String) -> None:
        """Cache the jumper-resolved challenge; video is Obstacles-only.

        Also the mid-race stop path: if the state machine cycles to a new
        round with a different challenge without this node restarting (the
        button alone can do that), a recording that's no longer valid for the
        new challenge must not keep running.
        """
        self._active_challenge = ScenarioType.from_string(msg.data)
        if not self._racing:
            return
        if self._active_challenge is ScenarioType.OBSTACLES:
            self._maybe_start_recording()
        else:
            self._stop_recording()

    def _on_run_path(self, msg: String) -> None:
        """Cache bag_recorder_node's chosen run directory for this race."""
        self._run_path = msg.data
        if self._racing:
            self._maybe_start_recording()

    def _maybe_start_recording(self) -> None:
        """Arm a poll for the bag run directory, if every gate is satisfied.

        Gates: direct capture (recording is meaningless against a topic-fed
        image stream), the operator's record_video toggle, the Obstacles
        Challenge specifically (see the Config docstring), and a run path
        having actually arrived from bag_recorder_node -- any of these can
        still be pending when RACING fires, since this node, bag_recorder_node
        and the challenge-mode resolution are three independent processes with
        no ordering guarantee between their /robot_state deliveries.
        """
        if not self._record_video or self._camera_source != "direct":
            return
        if self._recorder.is_recording or self._run_path_poll_timer is not None:
            return
        if self._active_challenge is not ScenarioType.OBSTACLES:
            return
        if self._run_path is None:
            return

        self._run_path_poll_deadline = time.monotonic() + _RUN_PATH_POLL_TIMEOUT_SEC
        self._run_path_poll_timer = self.create_timer(_RUN_PATH_POLL_INTERVAL_SEC, self._poll_for_run_path_dir)

    def _poll_for_run_path_dir(self) -> None:
        """Wait for bag_recorder_node's `ros2 bag record` to create its output directory.

        See _RUN_PATH_POLL_TIMEOUT_SEC's docstring for why this node must
        never create that directory itself.
        """
        assert self._run_path is not None  # noqa: S101 - only armed by _maybe_start_recording with a run path set
        path = Path(self._run_path)
        if path.is_dir():
            self._run_path_poll_timer.cancel()
            self._run_path_poll_timer = None
            video_path = path / "video.mp4"
            self._recorder.start(video_path)
            self.get_logger().info(f"Recording annotated video to {video_path}")
            return
        if time.monotonic() >= self._run_path_poll_deadline:
            self._run_path_poll_timer.cancel()
            self._run_path_poll_timer = None
            self.get_logger().warning(
                f"Bag run directory {path} never appeared within {_RUN_PATH_POLL_TIMEOUT_SEC}s "
                "- skipping video for this run",
            )

    def _stop_recording(self) -> None:
        if self._run_path_poll_timer is not None:
            self._run_path_poll_timer.cancel()
            self._run_path_poll_timer = None
        self._recorder.stop()

    def _image_callback(self, msg: Image) -> None:
        """Process incoming image and publish detections."""
        try:
            # Simple conversion for standard bgr8/rgb8
            if msg.encoding not in ["rgb8", "bgr8"]:
                self.get_logger().warning(
                    f"Unsupported image encoding: {msg.encoding}. Expected rgb8 or bgr8.",
                    throttle_duration_sec=5.0,
                )
                return

            img: np.ndarray = np.ndarray(
                shape=(msg.height, msg.width, 3),
                dtype=np.uint8,
                buffer=msg.data,
            )

            if msg.encoding == "bgr8":
                img = img[:, :, ::-1]  # Convert BGR to RGB

            self._process(img)

        except (RuntimeError, ValueError, TypeError) as e:
            self.get_logger().error(f"Error processing image: {type(e).__name__}: {e}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Unexpected error processing image: {e}")

    def _process(self, rgb: np.ndarray) -> None:
        """Detect on one RGB frame, publish detections and any debug video."""
        try:
            detections = self.detector.detect(rgb)

            data = []
            for d in detections:
                det = d.to_detection()
                data.append({
                    CLASS_NAME_KEY: det.class_name,
                    CONFIDENCE_KEY: det.confidence,
                    BBOX_KEY: det.bbox,
                    X_KEY: det.x,
                    Y_KEY: det.y,
                    WIDTH_KEY: det.width,
                    HEIGHT_KEY: det.height,
                    AREA_KEY: det.area,
                })

            out_msg = String()
            out_msg.data = json.dumps(data)
            self._publisher.publish(out_msg)

            if self._raw_publisher is not None:
                self._raw_publisher.publish(self._to_image_msg(rgb))
            # Computed once, shared by the live debug topic and the recorder --
            # neither is on during a race by default, so this costs nothing on
            # a normal Open Challenge round.
            if self._annotated_publisher is not None or self._recorder.is_recording:
                annotated = annotate(rgb, detections)
                if self._recorder.is_recording:
                    # Every frame, unthrottled -- the debug topic's rate cap
                    # below is for live bandwidth, not for what gets recorded.
                    # submit() never blocks: a slow encoder drops frames
                    # instead of stalling this (the Hailo inference) tick.
                    self._recorder.submit(annotated)
                if self._annotated_publisher is not None:
                    now = self.get_clock().now().nanoseconds / 1e9
                    due = now - self._last_annotated_pub_time >= self._annotated_min_interval
                    if self._annotated_min_interval <= 0 or due:
                        self._annotated_publisher.publish(self._to_image_msg(annotated))
                        self._last_annotated_pub_time = now

        except (RuntimeError, ValueError, TypeError) as e:
            self.get_logger().error(f"Error processing image: {type(e).__name__}: {e}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Unexpected error processing image: {e}")

    def _on_set_parameters(self, params: list[Parameter]) -> SetParametersResult:
        """Apply publish_annotated/debug_stream_fps changes without a restart.

        Backs the remote vision-debug toggle: telemetry_bridge_node forwards a
        SetVisionDebugParams command here via this node's standard
        set_parameters service, instead of requiring publish_annotated to be
        fixed at launch time.
        """
        for param in params:
            if param.name == "publish_annotated":
                self._publish_annotated = bool(param.value)
                if self._publish_annotated and self._annotated_publisher is None:
                    self._annotated_publisher = self.create_publisher(Image, self._annotated_topic, 1)
                elif not self._publish_annotated and self._annotated_publisher is not None:
                    self.destroy_publisher(self._annotated_publisher)
                    self._annotated_publisher = None
            elif param.name == "debug_stream_fps":
                fps = float(param.value)
                self._annotated_min_interval = 1.0 / fps if fps > 0 else 0.0
        return SetParametersResult(successful=True)

    def _to_image_msg(self, rgb: np.ndarray) -> Image:
        """Wrap an RGB array as a sensor_msgs/Image."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(rgb).tobytes()
        return msg

    def destroy_node(self) -> None:
        """Release the camera and detector, then tear down the node."""
        self._stop_recording()  # closes an in-flight video the same way _camera.close() below does the camera
        if self._camera is not None:
            with suppress(Exception):
                self._camera.close()
        if hasattr(self.detector, "__exit__"):
            self.detector.__exit__(None, None, None)
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the ROS2 vision node."""
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
