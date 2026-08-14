import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from shared.config.ros_topics import RosTopicConfig

_MIN_LIDAR_RANGE_M = 0.05
_SAMPLE_SIZE = 8
_FRONT_OFFSET = 20
_SPIN_TIMEOUT_SEC = 0.5
_PROBE_TIMEOUT_SEC = 8


class Probe(Node):
    def __init__(self):
        super().__init__("scan_probe")
        self.got = False
        self.sub = self.create_subscription(LaserScan, RosTopicConfig.load_default().sensors.scan, self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if self.got:
            return
        self.got = True
        n = len(msg.ranges)
        print(f"num_points={n}")

        def summarize(name, idxs):
            vals = [msg.ranges[i] for i in idxs if 0 <= i < n]
            finite = [v for v in vals if v > _MIN_LIDAR_RANGE_M and v != float("inf")]
            print(f"{name}: total={len(vals)} valid={len(finite)} sample={vals[:_SAMPLE_SIZE]} min_valid={min(finite) if finite else None}")

        front = range(n // 2 - _FRONT_OFFSET, n // 2 + _FRONT_OFFSET)
        left = range(n // 4, n // 3)
        right = range(2 * n // 3, 3 * n // 4)
        summarize("front", front)
        summarize("left", left)
        summarize("right", right)


def main():
    rclpy.init()
    node = Probe()
    import time

    start = time.time()
    while rclpy.ok() and not node.got and time.time() - start < _PROBE_TIMEOUT_SEC:
        rclpy.spin_once(node, timeout_sec=_SPIN_TIMEOUT_SEC)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
