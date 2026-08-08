"""ROS2 Vision Node for YOLO detection.

Subscribes to camera images and publishes JSON detections using LocalYoloDetector.
"""

import json
from contextlib import suppress
from dataclasses import asdict

import numpy as np
import rclpy
from pydantic_settings import SettingsConfigDict
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from src.hardware.camera.base import Driver as CameraDriver
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
from src.vision.detector import DEFAULT_YOLO_MODEL_PATH
from src.vision.overlay import annotate


class Config(HardwareBaseSettings):
    """Fallback defaults for VisionNode's ROS2 parameters, sourced from
    config/hardware/vision/node.toml. ``rpi5_nodes.launch.py`` still overrides
    these at launch time via ROS2 parameters (e.g. to select the hailo
    backend and direct camera capture) -- this only changes what a node
    launched with no parameter overrides falls back to.
    """

    model_config = SettingsConfigDict(env_prefix="vision_node_", toml_file=CONFIG_DIR / "vision" / "node.toml")

    camera_topic: str = "/camera/image_raw"
    detections_topic: str = "/vision/detections"
    model_path: str = DEFAULT_YOLO_MODEL_PATH
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


class VisionNode(Node):
    """ROS2 node that runs YOLO detection on camera images."""

    def __init__(self) -> None:
        super().__init__("vision_detector")

        defaults = Config()
        self.declare_parameter("camera_topic", defaults.camera_topic)
        self.declare_parameter("detections_topic", defaults.detections_topic)
        self.declare_parameter("model_path", defaults.model_path)
        self.declare_parameter("backend", defaults.backend)
        self.declare_parameter("camera_source", defaults.camera_source)
        self.declare_parameter("capture_fps", defaults.capture_fps)
        self.declare_parameter("publish_annotated", value=defaults.publish_annotated)
        self.declare_parameter("annotated_topic", defaults.annotated_topic)
        self.declare_parameter("publish_raw", value=defaults.publish_raw)
        self.declare_parameter("debug_stream_fps", defaults.debug_stream_fps)

        camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        backend = self.get_parameter("backend").get_parameter_value().string_value
        camera_source = self.get_parameter("camera_source").get_parameter_value().string_value
        capture_fps = self.get_parameter("capture_fps").get_parameter_value().double_value
        self._publish_annotated = self.get_parameter("publish_annotated").get_parameter_value().bool_value
        self._annotated_topic = self.get_parameter("annotated_topic").get_parameter_value().string_value
        self._publish_raw = self.get_parameter("publish_raw").get_parameter_value().bool_value
        debug_stream_fps = self.get_parameter("debug_stream_fps").get_parameter_value().double_value
        self._annotated_min_interval = 1.0 / debug_stream_fps if debug_stream_fps > 0 else 0.0
        self._last_annotated_pub_time = 0.0

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

        self._publisher = self.create_publisher(String, detections_topic, 10)
        self._annotated_publisher = (
            self.create_publisher(Image, self._annotated_topic, 1) if self._publish_annotated else None
        )
        self._raw_publisher = self.create_publisher(Image, camera_topic, 1) if self._publish_raw else None
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._camera: CameraDriver | None = None
        self._subscription = None
        if camera_source == "direct":
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
        assert self._camera is not None
        try:
            frame = self._camera.capture_frame().frame
        except Exception as err:  # noqa: BLE001
            self.get_logger().error(f"Camera capture failed: {err}", throttle_duration_sec=5.0)
            return
        self._process(self._camera.to_rgb(frame))

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
            if self._annotated_publisher is not None:
                now = self.get_clock().now().nanoseconds / 1e9
                due = now - self._last_annotated_pub_time >= self._annotated_min_interval
                if self._annotated_min_interval <= 0 or due:
                    self._annotated_publisher.publish(self._to_image_msg(annotate(rgb, detections)))
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
