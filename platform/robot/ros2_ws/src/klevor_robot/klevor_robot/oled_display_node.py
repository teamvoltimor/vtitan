"""ROS2 node for OLED display with async page cycling and image mirroring.

Run on: Raspberry Pi 5

Usage:
    ros2 run klevor_robot oled_display_node

Topics:
    Subscribed:
        - /robot_state (std_msgs/String) - Current robot state
        - /system_status (diagnostic_msgs/DiagnosticArray) - System diagnostics
        - /race_metrics (std_msgs/String) - Race metrics (JSON)
        - /imu/data (sensor_msgs/Imu) - IMU data for gyro yaw
        - /scan (sensor_msgs/LaserScan) - LiDAR data for clearances
        - /hailo/fps (std_msgs/Float32) - Hailo inference FPS
    Published:
        - /ui/oled_mirror (sensor_msgs/Image) - Live mirror of OLED display
"""

import json
import math
import time
from typing import TYPE_CHECKING, override

import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from PIL import Image, ImageDraw, ImageFont
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, String

from src.hardware.display.ssd1306 import Driver as DisplayDriver
from src.state_machine import RobotState

if TYPE_CHECKING:
    from rclpy.publisher import Publisher
    from rclpy.subscription import Subscription
    from rclpy.timer import Timer


NODE_NAME = "oled_display_node"
"""ROS2 node name for OLED display controller."""

UI_REFRESH_RATE_HZ = 10.0
"""Rate for updating display data (fast updates)."""

PAGE_CYCLE_INTERVAL_SEC = 1.2
"""Interval for cycling between pages during RACING state."""


class OLEDDisplayNode(Node):
    """ROS2 node that manages the OLED display with state-based views.

    Responsibilities:
    - Display live hardware checklist during BOOT_CHECK
    - Display IP address and AI model during READY
    - Auto-cycle through Ackermann, Hailo, and LiDAR pages during RACING
    - Display final race results during FINISHED
    - Publish live mirror of display to /ui/oled_mirror for remote viewing
    """

    def __init__(self) -> None:
        """Initialize OLED display node."""
        super().__init__(NODE_NAME)

        self.get_logger().info("Initializing OLED Display Node")

        # Display driver
        try:
            self.display_driver = DisplayDriver()
            self.display_driver.connect()
            self.get_logger().info("Display driver connected")
        except Exception as e:
            self.get_logger().error(f"Failed to connect display driver: {e}")
            self.display_driver = None  # type: ignore

        # CV Bridge for image publishing
        self.bridge = CvBridge()

        # Publisher for display mirror
        self.oled_mirror_pub: Publisher[ImageMsg] = self.create_publisher(ImageMsg, "/ui/oled_mirror", 10)

        # Subscribers
        self.state_sub: Subscription[String] = self.create_subscription(
            String, "/robot_state", self._state_callback, 10
        )
        self.diagnostics_sub: Subscription[DiagnosticArray] = self.create_subscription(
            DiagnosticArray, "/system_status", self._diagnostics_callback, 10
        )
        self.metrics_sub: Subscription[String] = self.create_subscription(
            String, "/race_metrics", self._metrics_callback, 10
        )
        self.imu_sub: Subscription[Imu] = self.create_subscription(Imu, "/imu/data", self._imu_callback, 10)
        self.lidar_sub: Subscription[LaserScan] = self.create_subscription(LaserScan, "/scan", self._lidar_callback, 10)
        self.hailo_fps_sub: Subscription[Float32] = self.create_subscription(
            Float32, "/hailo/fps", self._hailo_fps_callback, 10
        )

        # State tracking
        self.current_state: str = RobotState.BOOT_CHECK.value
        self.system_status: dict[str, dict] = {}
        self.race_metrics: dict = {}

        # Sensor data
        self.gyro_yaw: float = 0.0
        self.lidar_front: float = 0.0
        self.lidar_left: float = 0.0
        self.lidar_right: float = 0.0
        self.hailo_fps: float = 0.0

        # Page cycling for RACING state
        self.current_page: int = 0
        self.last_page_cycle_time: float = time.time()
        self.racing_pages = ["ackermann", "hailo", "lidar"]

        # Timers
        self.ui_timer: Timer = self.create_timer(1.0 / UI_REFRESH_RATE_HZ, self._update_display)

        self.get_logger().info("OLED Display Node initialized")

    def _state_callback(self, msg: String) -> None:
        """Handle robot state updates."""
        self.current_state = msg.data

    def _diagnostics_callback(self, msg: DiagnosticArray) -> None:
        """Handle system diagnostics updates."""
        for status in msg.status:
            self.system_status[status.name] = {"level": status.level, "message": status.message, "values": {}}

            # Extract key-value pairs
            for kv in status.values:
                self.system_status[status.name]["values"][kv.key] = kv.value

    def _metrics_callback(self, msg: String) -> None:
        """Handle race metrics updates."""
        try:
            self.race_metrics = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _imu_callback(self, msg: Imu) -> None:
        """Handle IMU data for gyro yaw."""
        # Convert quaternion to yaw (simplified Euler extraction)
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.gyro_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _lidar_callback(self, msg: LaserScan) -> None:
        """Handle LiDAR data for spatial clearances."""
        ranges = msg.ranges
        num_points = len(ranges)

        if num_points == 0:
            return

        # Calculate clearances in different directions
        # Front: center ±15 degrees
        front_indices = list(range(num_points // 2 - 20, num_points // 2 + 20))
        front_ranges = [ranges[i] for i in front_indices if 0 <= i < num_points and ranges[i] > 0.01]
        self.lidar_front = min(front_ranges) * 100 if front_ranges else 0.0  # Convert to cm

        # Left: 60-120 degrees
        left_indices = list(range(num_points // 4, num_points // 3))
        left_ranges = [ranges[i] for i in left_indices if 0 <= i < num_points and ranges[i] > 0.01]
        self.lidar_left = min(left_ranges) * 100 if left_ranges else 0.0  # Convert to cm

        # Right: -60 to -120 degrees
        right_indices = list(range(2 * num_points // 3, 3 * num_points // 4))
        right_ranges = [ranges[i] for i in right_indices if 0 <= i < num_points and ranges[i] > 0.01]
        self.lidar_right = min(right_ranges) * 100 if right_ranges else 0.0  # Convert to cm

    def _hailo_fps_callback(self, msg: Float32) -> None:
        """Handle Hailo FPS updates."""
        self.hailo_fps = msg.data

    def _update_display(self) -> None:
        """Update display based on current state."""
        if self.display_driver is None:
            return

        # Check if we need to cycle pages (only in RACING state)
        if self.current_state == RobotState.RACING.value:
            current_time = time.time()
            if current_time - self.last_page_cycle_time >= PAGE_CYCLE_INTERVAL_SEC:
                self.current_page = (self.current_page + 1) % len(self.racing_pages)
                self.last_page_cycle_time = current_time

        # Create display image based on state
        if self.current_state == RobotState.BOOT_CHECK.value:
            image = self._render_boot_check()
        elif self.current_state == RobotState.READY.value:
            image = self._render_ready()
        elif self.current_state == RobotState.RACING.value:
            page_name = self.racing_pages[self.current_page]
            if page_name == "ackermann":
                image = self._render_ackermann()
            elif page_name == "hailo":
                image = self._render_hailo()
            else:  # lidar
                image = self._render_lidar()
        elif self.current_state == RobotState.FINISHED.value:
            image = self._render_finished()
        else:
            image = self.display_driver.get_blank_image()

        # Display on OLED
        self.display_driver.show_image(image)

        # Publish mirror for remote viewing
        self._publish_mirror_image(image)

    def _render_boot_check(self) -> Image.Image:
        """Render BOOT_CHECK view - hardware checklist."""
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        # Title
        draw.text((0, 0), "BOOT CHECK", fill=255)
        draw.line([(0, 10), (128, 10)], fill=255, width=1)

        # Component statuses
        y = 14
        components = ["IMU", "LiDAR", "Hailo", "Drive", "Network"]

        for component in components:
            if component in self.system_status:
                status = self.system_status[component]
                symbol = "✓" if status["level"] == 0 else "✗"
                text = f"{symbol} {component}"

                # For Network, show IP if available
                if component == "Network" and "ip_address" in status.get("values", {}):
                    ip = status["values"]["ip_address"]
                    text = f"{symbol} IP:{ip}"

                draw.text((0, y), text, fill=255)
            else:
                draw.text((0, y), f"? {component}", fill=255)

            y += 10

        return image

    def _render_ready(self) -> Image.Image:
        """Render READY view - IP, model name, ready status."""
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        # Title
        draw.text((0, 0), "READY TO START", fill=255)
        draw.line([(0, 10), (128, 10)], fill=255, width=1)

        # IP Address
        ip_address = "OFFLINE"
        if "Network" in self.system_status and "ip_address" in self.system_status["Network"].get("values", {}):
            ip_address = self.system_status["Network"]["values"]["ip_address"]

        draw.text((0, 14), f"IP: {ip_address}", fill=255)

        # AI Model
        draw.text((0, 24), "Model: yolov8n.hef", fill=255)

        # Steering status
        draw.text((0, 34), "Steering: READY", fill=255)

        # Instruction
        draw.text((0, 50), "Press to START", fill=255)

        return image

    def _render_ackermann(self) -> Image.Image:
        """Render Ackermann page - velocity, steering, gyro."""
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        # Title
        draw.text((0, 0), "ACKERMANN", fill=255)
        draw.line([(0, 10), (128, 10)], fill=255, width=1)

        # Velocity
        velocity = self.race_metrics.get("current_velocity", 0.0)
        draw.text((0, 14), f"Vel: {velocity:.2f} m/s", fill=255)

        # Steering
        steering = self.race_metrics.get("current_steering", 0.0)
        draw.text((0, 26), f"Steer: {steering:.1f} deg", fill=255)

        # Gyro Yaw
        draw.text((0, 38), f"Yaw: {self.gyro_yaw:.1f} deg", fill=255)

        # Laps
        laps = self.race_metrics.get("laps_completed", 0)
        draw.text((0, 50), f"Laps: {laps}/3", fill=255)

        return image

    def _render_hailo(self) -> Image.Image:
        """Render Hailo Vision page - NPU FPS, detections."""
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        # Title
        draw.text((0, 0), "HAILO VISION", fill=255)
        draw.line([(0, 10), (128, 10)], fill=255, width=1)

        # NPU FPS
        draw.text((0, 14), f"NPU: {self.hailo_fps:.1f} FPS", fill=255)

        # Target detection (placeholder - would need actual detection data)
        draw.text((0, 26), "Target: SEARCHING", fill=255)
        draw.text((0, 38), "Conf: --", fill=255)
        draw.text((0, 50), "Dist: -- m", fill=255)

        return image

    def _render_lidar(self) -> Image.Image:
        """Render LiDAR page - spatial clearances."""
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        # Title
        draw.text((0, 0), "LIDAR", fill=255)
        draw.line([(0, 10), (128, 10)], fill=255, width=1)

        # Clearances
        draw.text((0, 14), f"Front: {self.lidar_front:.0f} cm", fill=255)
        draw.text((0, 26), f"Left:  {self.lidar_left:.0f} cm", fill=255)
        draw.text((0, 38), f"Right: {self.lidar_right:.0f} cm", fill=255)

        # Path status
        path_status = "CLEAR"
        if self.lidar_front < 30:
            path_status = "BLOCKED"
        elif min(self.lidar_left, self.lidar_right) < 20:
            path_status = "NARROW"

        draw.text((0, 50), f"Path: {path_status}", fill=255)

        return image

    def _render_finished(self) -> Image.Image:
        """Render FINISHED view - final results."""
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        # Title
        draw.text((0, 0), "RACE FINISHED", fill=255)
        draw.line([(0, 10), (128, 10)], fill=255, width=1)

        # Laps completed
        laps = self.race_metrics.get("laps_completed", 0)
        draw.text((0, 18), f"Laps: {laps}/3", fill=255)

        # Total time
        race_time = self.race_metrics.get("total_race_time", 0.0)
        minutes = int(race_time // 60)
        seconds = race_time % 60
        draw.text((0, 30), f"Time: {minutes}:{seconds:05.2f}", fill=255)

        # Status
        status = "COMPLETE" if laps >= 3 else "E-STOP"
        draw.text((0, 50), f"Status: {status}", fill=255)

        return image

    def _publish_mirror_image(self, pil_image: Image.Image) -> None:
        """Publish display image to ROS2 topic for remote viewing.

        Args:
            pil_image: PIL Image to publish.
        """
        # Convert 1-bit image to 8-bit grayscale
        img_gray = pil_image.convert("L")

        # Convert to numpy array
        img_array = np.array(img_gray)

        # Convert to ROS2 Image message
        try:
            ros_img = self.bridge.cv2_to_imgmsg(img_array, encoding="mono8")
            ros_img.header.stamp = self.get_clock().now().to_msg()
            ros_img.header.frame_id = "oled_display"
            self.oled_mirror_pub.publish(ros_img)
        except Exception as e:
            self.get_logger().warning(f"Failed to publish mirror image: {e}")

    @override
    def destroy_node(self) -> None:
        """Shutdown the OLED display node."""
        self.get_logger().info("Shutting down OLED Display Node")

        # Clear display
        if self.display_driver is not None:
            self.display_driver.clear()
            self.display_driver.close()

        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Main entry point for OLED display node."""
    rclpy.init(args=args)
    node = OLEDDisplayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
