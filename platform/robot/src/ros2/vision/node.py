"""ROS2 Vision Node for YOLO detection.

Subscribes to camera images and publishes JSON detections using LocalYoloDetector.
"""

import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from src.vision import create_detector


class VisionNode(Node):
    """ROS2 node that runs YOLO detection on camera images."""

    def __init__(self) -> None:
        super().__init__("vision_detector")

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("backend", "yolo")  # 'yolo' or 'hailo'

        camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        backend = self.get_parameter("backend").get_parameter_value().string_value

        self.get_logger().info(f"Loading {backend.upper()} vision model from {model_path}...")

        from src.vision.detector import DetectorConfig, TrafficSignColor  # noqa: PLC0415

        config = DetectorConfig(
            model_path=model_path,
            class_to_color={
                0: TrafficSignColor.RED,
                1: TrafficSignColor.GREEN,
                2: TrafficSignColor.MAGENTA,
            },
        )
        detector = create_detector(backend, config)
        # Enter context manager for backends that hold hardware resources (Hailo).
        # For YOLO the __enter__ is a no-op; calling it unconditionally is safe.
        if hasattr(detector, "__enter__"):
            detector.__enter__()
        self.detector = detector

        self._publisher = self.create_publisher(String, detections_topic, 10)

        self._subscription = self.create_subscription(
            Image,
            camera_topic,
            self._image_callback,
            10,
        )
        self.get_logger().info(
            f"Vision Node ready. Subscribed to {camera_topic}, publishing to {detections_topic}",
        )

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

            # Perform detection
            detections = self.detector.detect(img)

            # Serialize and publish
            data = [d.to_dict() for d in detections]

            out_msg = String()
            out_msg.data = json.dumps(data)
            self._publisher.publish(out_msg)

        except (RuntimeError, ValueError, TypeError) as e:
            self.get_logger().error(f"Error processing image: {type(e).__name__}: {e}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Unexpected error processing image: {e}")


    def destroy_node(self) -> None:
        """Release the detector context, then tear down the node."""
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
