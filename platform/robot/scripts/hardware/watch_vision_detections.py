"""Print live detections in one compact line per frame, for waving blocks at the camera.

``ros2 topic echo /vision/detections`` prints the full JSON of every frame at
15 Hz, which is unreadable while you are holding something up to the lens. This
prints one line per frame with the colours, confidences and where in the frame
each sits, and a tally at the end.

Usage (on the Pi, with the stack running)::

    pixi run -e vision python scripts/hardware/watch_vision_detections.py --seconds 60

Nothing here touches the NPU: it reads the topic the vision node already
publishes, so it is safe to run against a live robot.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

DISCOVERY_SEC = 3.0
# Frame width the detector works in, used to say left/centre/right.
_FRAME_WIDTH = 640.0


def _where(x: float, frame_width: float) -> str:
    """Describe roughly where in the frame a detection sits."""
    third = frame_width / 3.0
    if x < third:
        return "left"
    if x > 2 * third:
        return "right"
    return "centre"


class Watcher(Node):
    """Subscribes to the detections topic and prints what arrives."""

    def __init__(self, topic: str, quiet: bool) -> None:
        super().__init__("vision_detection_watcher")
        self._quiet = quiet
        self.frames = 0
        self.frames_with_detections = 0
        self.tally: Counter[str] = Counter()
        self.best: dict[str, float] = {}
        self.create_subscription(String, topic, self._on_detections, 10)

    def _on_detections(self, msg: String) -> None:
        self.frames += 1
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Undecodable payload", throttle_duration_sec=5.0)
            return

        if not detections:
            if not self._quiet:
                print(f"\r{'(nothing in view)':<70}", end="", flush=True)
            return

        self.frames_with_detections += 1
        parts = []
        for det in sorted(detections, key=lambda d: -d.get("confidence", 0.0)):
            name = det.get("class_name", "?")
            conf = det.get("confidence", 0.0)
            self.tally[name] += 1
            self.best[name] = max(self.best.get(name, 0.0), conf)
            parts.append(f"{name} {conf:.2f} @{_where(det.get('x', 0.0), _FRAME_WIDTH)}")
        print(f"\r{' | '.join(parts):<70}", flush=True)


def parse_args() -> argparse.Namespace:
    """Parse the watcher's command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/vision/detections")
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--quiet", action="store_true", help="Only print frames that have detections")
    return parser.parse_args()


def main() -> int:
    """Watch the topic for the requested duration and summarise."""
    args = parse_args()
    rclpy.init()
    node = Watcher(args.topic, quiet=args.quiet)

    deadline = time.monotonic() + DISCOVERY_SEC
    while time.monotonic() < deadline and node.count_publishers(args.topic) == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.count_publishers(args.topic) == 0:
        print(f"No publisher on {args.topic}. Is vtitan-pi5.service running?")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f"Watching {args.topic} for {args.seconds:g}s -- hold a block in front of the camera.\n")
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"\n\nframes: {node.frames}  with detections: {node.frames_with_detections}")
    if node.tally:
        for name, count in node.tally.most_common():
            print(f"  {name:<10} seen in {count:4d} detections, best confidence {node.best[name]:.2f}")
    else:
        print("  nothing detected -- check the camera is aimed at the block and in focus")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
