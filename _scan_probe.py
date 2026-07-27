import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Probe(Node):
    def __init__(self):
        super().__init__("scan_probe")
        self.got = False
        self.sub = self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if self.got:
            return
        self.got = True
        n = len(msg.ranges)
        print(f"num_points={n}")

        def summarize(name, idxs):
            vals = [msg.ranges[i] for i in idxs if 0 <= i < n]
            finite = [v for v in vals if v > 0.05 and v != float("inf")]
            print(f"{name}: total={len(vals)} valid={len(finite)} sample={vals[:8]} min_valid={min(finite) if finite else None}")

        front = range(n // 2 - 20, n // 2 + 20)
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
    while rclpy.ok() and not node.got and time.time() - start < 8:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
