"""Record the annotated vision stream to an mp4 for review.

The robot is headless and the annotated topic only exists when debug video is
enabled, so this is the practical way to see what the detector saw. Requires
``VISION_DEBUG_VIDEO=1`` in ``src/.env`` (or ``publish_annotated:=true``
on the node) -- without it the topic has no publisher and this exits saying so
rather than waiting forever.

Usage (on the Pi, with the stack running)::

    pixi run -e vision python scripts/hardware/record_vision_video.py --seconds 20 --out /tmp/run.mp4

Then copy it off with ``scp rpi-5-local:/tmp/run.mp4 .``

To watch live instead of recording, ``rqt_image_view /vision/image_annotated``
or an RViz Image display on the same topic.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common.hardware_defaults import DISCOVERY_SEC


class Recorder(Node):
    """Writes frames from an Image topic into a video file."""

    def __init__(self, topic: str, out_path: Path, fps: float) -> None:
        super().__init__("vision_video_recorder")
        self._out_path = out_path
        self._fps = fps
        self._writer: cv2.VideoWriter | None = None
        self.frames = 0
        self.create_subscription(Image, topic, self._on_image, 10)

    def _on_image(self, msg: Image) -> None:
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warning(f"Unsupported encoding {msg.encoding}", throttle_duration_sec=5.0)
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        # VideoWriter wants BGR; the topic is rgb8 by convention here.
        bgr = frame[:, :, ::-1] if msg.encoding == "rgb8" else frame

        if self._writer is None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(str(self._out_path), fourcc, self._fps, (msg.width, msg.height))
            self.get_logger().info(f"Recording {msg.width}x{msg.height} to {self._out_path}")
        self._writer.write(np.ascontiguousarray(bgr))
        self.frames += 1

    def close(self) -> None:
        """Finalise the file so it is playable."""
        if self._writer is not None:
            self._writer.release()


def parse_args() -> argparse.Namespace:
    """Parse the recorder's command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/vision/image_annotated")
    parser.add_argument("--out", default="/tmp/vision.mp4")  # noqa: S108
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--fps", type=float, default=15.0, help="Playback rate written into the file")
    return parser.parse_args()


def main() -> int:
    """Record the topic for the requested duration."""
    args = parse_args()
    rclpy.init()
    node = Recorder(args.topic, Path(args.out), args.fps)

    deadline = time.monotonic() + DISCOVERY_SEC
    while time.monotonic() < deadline and node.count_publishers(args.topic) == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.count_publishers(args.topic) == 0:
        print(f"No publisher on {args.topic}. Set VISION_DEBUG_VIDEO=1 in .env and restart the service.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

    print(f"Wrote {node.frames} frames to {args.out}")
    return 0 if node.frames else 1


if __name__ == "__main__":
    raise SystemExit(main())
