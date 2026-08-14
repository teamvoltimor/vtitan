"""Publish a known image to the camera topic and report what vision detects.

There is no ROS node publishing ``/camera/image_raw`` yet, so the live pipeline
cannot be exercised by pointing the camera at something. This injects an image
of known colour instead, which verifies the whole deployed chain end to end --
the running vision node, the Hailo driver, the NMS decoding and the class map --
against the real NPU.

Usage (from ``platform/robot``, with the stack running)::

    pixi run -e vision python scripts/hardware/diag_vision_topic.py IMAGE [IMAGE ...]
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from shared.config.ros_topics import RosTopicConfig
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

SETTLE_SEC = 2.0
REPLY_TIMEOUT_SEC = 5.0
_SUBSCRIPTION_QUEUE_DEPTH = 10
_SPIN_TIMEOUT_SEC = 0.3


class Injector(Node):
    """Publishes frames and collects the detections they produce."""

    def __init__(self) -> None:
        super().__init__("diag_vision_topic")
        topics = RosTopicConfig.load_default()
        self._publisher = self.create_publisher(Image, topics.sensors.camera_image_raw, qos_profile_sensor_data)
        self._replies: list[str] = []
        self.create_subscription(String, topics.sensors.vision_detections, self._on_detections, _SUBSCRIPTION_QUEUE_DEPTH)

    def _on_detections(self, msg: String) -> None:
        self._replies.append(msg.data)

    def send(self, image_path: Path) -> str | None:
        """Publish *image_path* and return the first detection payload for it."""
        bgr = cv2.imread(str(image_path))
        if bgr is None:
            return None
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        msg = Image()
        msg.header.frame_id = "camera"
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = rgb.tobytes()

        self._replies.clear()
        deadline = time.time() + REPLY_TIMEOUT_SEC
        while time.time() < deadline:
            self._publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=_SPIN_TIMEOUT_SEC)
            if self._replies:
                return self._replies[0]
        return None


def main() -> int:
    """Inject every image given on the command line and print the verdict."""
    images = [Path(a) for a in sys.argv[1:]]
    if not images:
        print("usage: diag_vision_topic.py IMAGE [IMAGE ...]")
        return 2

    rclpy.init()
    node = Injector()
    # Let discovery settle, otherwise the first frames go nowhere.
    end = time.time() + SETTLE_SEC
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    try:
        for path in images:
            reply = node.send(path)
            print(f"{path.name[:44]:46s} -> {reply if reply is not None else '(no response)'}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
