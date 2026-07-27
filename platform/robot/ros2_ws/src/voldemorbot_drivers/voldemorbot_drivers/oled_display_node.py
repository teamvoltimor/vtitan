"""ROS2 lifecycle node for OLED display with async page cycling and image mirroring.

Run on: Raspberry Pi 5

Hardware connects in on_configure() and page-update publishing starts in
on_activate(), matching the driver lifecycle pattern used by every hardware
node in this package (see button_node.py for the reference implementation).

Usage:
    ros2 run voldemorbot_drivers oled_display_node

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

from __future__ import annotations

import json
import math
import time
from contextlib import suppress
from typing import TYPE_CHECKING, override

import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from PIL import Image, ImageDraw
from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import (
    Image as ImageMsg,
    Imu,
    LaserScan,
)
from std_msgs.msg import Float32, String

from src.hardware.display.enums import DisplayBackend
from src.hardware.display.ssd1306 import (
    Driver as BlinkaDriver,
    RawI2CDriver,
)
from src.state_machine import RobotState

if TYPE_CHECKING:
    from rclpy.lifecycle.node import LifecycleState
    from rclpy.lifecycle.publisher import Publisher
    from rclpy.subscription import Subscription
    from rclpy.timer import Timer

    from src.hardware.display.base import Driver as DisplayDriver


NODE_NAME = "oled_display_node"
"""ROS2 node name for OLED display controller."""


class NodeConfig(BaseSettings):
    """Node-level timing and backend selection, configurable via .env.

    Matches every hardware driver's Config pattern. Timing fields were previously hardcoded
    as plain module constants, silently ignoring
    .env.example's documented UI_REFRESH_RATE_HZ / PAGE_CYCLE_INTERVAL_SEC entirely --
    changing those values had zero effect.
    """

    model_config = SettingsConfigDict(env_prefix="")

    ui_refresh_rate_hz: float = Field(default=10.0, validation_alias="UI_REFRESH_RATE_HZ")
    """Rate for updating display data (fast updates)."""

    page_cycle_interval_sec: float = Field(default=1.2, validation_alias="PAGE_CYCLE_INTERVAL_SEC")
    """Interval for cycling between pages during RACING state."""

    display_backend: DisplayBackend = Field(default=DisplayBackend.BLINKA, validation_alias="DISPLAY_BACKEND")
    """SSD1306 I2C backend -- blinka (Adafruit CircuitPython) or raw_i2c (direct /dev/i2c-N
    ioctl, no Blinka/smbus2 dependency). See src/hardware/display/ssd1306/driver_raw_i2c.py
    for why raw_i2c exists."""


_node_config = NodeConfig()
UI_REFRESH_RATE_HZ = _node_config.ui_refresh_rate_hz

_MARGIN_X = 0
"""Left margin for every line. The panel is 128px wide; text starts flush."""

_TITLE_Y = 0
_SEPARATOR_Y = 10
"""Baseline of the title and the rule drawn under it, on every page."""

_BODY_TOP_Y = 14
"""First body row, just below the separator."""

_ROW_H = 10
"""Vertical step between body rows, sized for the default PIL bitmap font."""

_FOOTER_Y = 50
"""Bottom row, used for the one instruction or headline value per page."""

_ON = 255
"""Monochrome "lit pixel" for a 1-bit SSD1306."""

_DEFAULT_TARGET_LAPS = 3
"""Shown only until /race_metrics arrives with the real figure."""

_NO_IP_VALUES = frozenset({"OFFLINE", "FETCHING...", "unknown", "-"})
"""Placeholders the state machine reports when there is no address to show.

Treated as "no IP" rather than printed: at competition there is no network, so
these are the normal case, not a fault worth a line on a 128x64 display.
"""

_QOS_LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
"""Matches state_machine_node's latched publishers.

The display is a late subscriber by nature -- it lives on the Pi Zero and is
restarted independently of the Pi 5 -- so it has to request the latched value
rather than wait for the next transition, which may be minutes away or may
already have happened.
"""
PAGE_CYCLE_INTERVAL_SEC = _node_config.page_cycle_interval_sec

_DISPLAY_DRIVER_BY_BACKEND = {
    DisplayBackend.BLINKA: BlinkaDriver,
    DisplayBackend.RAW_I2C: RawI2CDriver,
}

_MIN_VALID_LIDAR_RANGE_M = 0.01
"""LIDAR ranges at or below this are treated as invalid (no-return) readings."""

_PATH_BLOCKED_CLEARANCE_CM = 30
"""Front clearance below this (cm) is shown as a blocked path on the OLED."""

_PATH_NARROW_CLEARANCE_CM = 20
"""Side clearance below this (cm) is shown as a narrow path on the OLED."""


class OLEDDisplayNode(LifecycleNode):
    """ROS2 lifecycle node that manages the OLED display with state-based views.

    Responsibilities:
    - Display live hardware checklist during BOOT_CHECK
    - Display IP address and AI model during READY
    - Auto-cycle through Ackermann, Hailo, and LiDAR pages during RACING
    - Display final race results during FINISHED
    - Publish live mirror of display to /ui/oled_mirror for remote viewing
    """

    def __init__(self) -> None:
        """Construct the node (unconfigured -- no hardware I/O yet)."""
        super().__init__(NODE_NAME)
        self.get_logger().info("OLED Display Node constructed (unconfigured)")

        self.display_driver: DisplayDriver | None = None
        self.bridge: CvBridge | None = None
        self.oled_mirror_pub: Publisher | None = None
        self.state_sub: Subscription | None = None
        self.diagnostics_sub: Subscription | None = None
        self.metrics_sub: Subscription | None = None
        self.imu_sub: Subscription | None = None
        self.lidar_sub: Subscription | None = None
        self.hailo_fps_sub: Subscription | None = None
        self.ui_timer: Timer | None = None

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

    @override
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Connect the display driver and create the publisher/subscribers."""
        self.get_logger().info("Configuring OLED Display Node")

        try:
            display_driver_cls = _DISPLAY_DRIVER_BY_BACKEND[_node_config.display_backend]
            self.display_driver = display_driver_cls()
            self.display_driver.connect()
            self.get_logger().info(f"Display driver connected ({_node_config.display_backend.value} backend)")
        except (RuntimeError, OSError, ValueError, ImportError) as e:
            self.get_logger().error(f"Failed to connect display driver: {e}")
            self.display_driver = None

        self.bridge = CvBridge()

        self.oled_mirror_pub = self.create_lifecycle_publisher(ImageMsg, "/ui/oled_mirror", 10)

        # TRANSIENT_LOCAL to match state_machine_node, which publishes both of
        # these latched precisely so a late subscriber gets the current value.
        # Subscribing VOLATILE is *compatible*, so DDS reports no error and the
        # topic looks connected -- but the latched value is never delivered, so
        # the display only learns the state from the next transition.
        #
        # Observed on the robot: restarting the Pi 5 replaced the publisher, the
        # Zero's subscription did not re-match, and the OLED went on rendering
        # the RACING pages while /robot_state read "ready". The display was
        # silently reporting a state the robot had left, which is worse than
        # showing nothing -- the button and state machine were both working and
        # the display was the only thing saying otherwise.
        self.state_sub = self.create_subscription(String, "/robot_state", self._state_callback, _QOS_LATCHED)
        # /system_status stays VOLATILE, unlike /robot_state above. The state
        # machine publishes it latched, but telemetry_bridge_node publishes to
        # the same topic with default QoS -- and a TRANSIENT_LOCAL *subscriber*
        # cannot receive from a VOLATILE publisher at all. Requesting the
        # latched value here silently cut off the bridge's half:
        #   "New publisher discovered on topic '/system_status', offering
        #    incompatible QoS. No messages will be received from it."
        # Durability is asymmetric -- a publisher may offer more than a
        # subscriber asks for, never less.
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            "/system_status",
            self._diagnostics_callback,
            10,
        )
        self.metrics_sub = self.create_subscription(String, "/race_metrics", self._metrics_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/data",
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.lidar_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self._lidar_callback,
            qos_profile_sensor_data,
        )
        self.hailo_fps_sub = self.create_subscription(Float32, "/hailo/fps", self._hailo_fps_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    @override
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Start the display-update timer."""
        self.get_logger().info("Activating OLED Display Node")
        self.ui_timer = self.create_timer(1.0 / UI_REFRESH_RATE_HZ, self._update_display)
        return super().on_activate(state)

    @override
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop the display-update timer."""
        self.get_logger().info("Deactivating OLED Display Node")
        self._destroy_timer()
        return super().on_deactivate(state)

    @override
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Disconnect the display driver and tear down publisher/subscribers."""
        self.get_logger().info("Cleaning up OLED Display Node")
        self._disconnect_driver()
        self._destroy_pub_and_subs()
        return TransitionCallbackReturn.SUCCESS

    @override
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Tear down whatever exists, regardless of which state shutdown was triggered from."""
        self.get_logger().info("Shutting down OLED Display Node")
        self._destroy_timer()
        self._disconnect_driver()
        self._destroy_pub_and_subs()
        return TransitionCallbackReturn.SUCCESS

    def _destroy_timer(self) -> None:
        if self.ui_timer is not None:
            self.ui_timer.cancel()
            self.destroy_timer(self.ui_timer)
            self.ui_timer = None

    def _disconnect_driver(self) -> None:
        if self.display_driver is not None:
            try:
                self.display_driver.clear()
                self.display_driver.close()
            except Exception as e:  # noqa: BLE001 - cleanup must never fail node teardown
                self.get_logger().error(f"Error closing display driver: {e}")
            self.display_driver = None

    def _destroy_pub_and_subs(self) -> None:
        for sub_attr in ("state_sub", "diagnostics_sub", "metrics_sub", "imu_sub", "lidar_sub", "hailo_fps_sub"):
            sub = getattr(self, sub_attr)
            if sub is not None:
                self.destroy_subscription(sub)
                setattr(self, sub_attr, None)
        if self.oled_mirror_pub is not None:
            self.destroy_publisher(self.oled_mirror_pub)
            self.oled_mirror_pub = None

    @override
    def destroy_node(self) -> None:
        """Release hardware directly rather than trigger an on_shutdown transition.

        Handles a node destroyed without a clean lifecycle shutdown (e.g.
        process killed mid-active, or a test that never triggers shutdown).
        """
        self._destroy_timer()
        self._disconnect_driver()
        self._destroy_pub_and_subs()
        return super().destroy_node()

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
        with suppress(json.JSONDecodeError):
            self.race_metrics = json.loads(msg.data)

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
        front_ranges = [
            ranges[i] for i in front_indices if 0 <= i < num_points and ranges[i] > _MIN_VALID_LIDAR_RANGE_M
        ]
        self.lidar_front = min(front_ranges) * 100 if front_ranges else 0.0  # Convert to cm

        # Left: 60-120 degrees
        left_indices = list(range(num_points // 4, num_points // 3))
        left_ranges = [ranges[i] for i in left_indices if 0 <= i < num_points and ranges[i] > _MIN_VALID_LIDAR_RANGE_M]
        self.lidar_left = min(left_ranges) * 100 if left_ranges else 0.0  # Convert to cm

        # Right: -60 to -120 degrees
        right_indices = list(range(2 * num_points // 3, 3 * num_points // 4))
        right_ranges = [
            ranges[i] for i in right_indices if 0 <= i < num_points and ranges[i] > _MIN_VALID_LIDAR_RANGE_M
        ]
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
        assert self.display_driver is not None
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        # Title
        draw.text((_MARGIN_X, _TITLE_Y), "BOOT CHECK", fill=_ON)
        draw.line([(_MARGIN_X, _SEPARATOR_Y), (self.display_driver.get_width(), _SEPARATOR_Y)], fill=_ON, width=1)

        # Component statuses
        y = 14
        components = ["IMU", "LiDAR", "Hailo", "Drive", "Network", "ChallengeMode"]

        for component in components:
            if component in self.system_status:
                status = self.system_status[component]
                symbol = "✓" if status["level"] == 0 else "✗"
                text = f"{symbol} {component}"

                # For Network, show IP if available
                if component == "Network" and "ip_address" in status.get("values", {}):
                    ip = status["values"]["ip_address"]
                    text = f"{symbol} IP:{ip}"

                # ChallengeMode's message is the detected mode (or a fault) once known --
                # a distinct "CHECK JUMPER" page while unstable so it stands out.
                if component == "ChallengeMode":
                    if status["level"] == 0:
                        text = f"MODE: {status['message']}"
                    else:
                        return self._render_challenge_mode_fault()

                draw.text((_MARGIN_X, y), text, fill=_ON)
            else:
                draw.text((_MARGIN_X, y), f"? {component}", fill=_ON)

            y += 10

        return image

    def _render_challenge_mode_fault(self) -> Image.Image:
        """Render a distinct fault page while the challenge-mode jumper reading is unstable."""
        assert self.display_driver is not None
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        draw.text((_MARGIN_X, _TITLE_Y), "CHECK JUMPER", fill=_ON)
        draw.line([(_MARGIN_X, _SEPARATOR_Y), (self.display_driver.get_width(), _SEPARATOR_Y)], fill=_ON, width=1)
        draw.text((_MARGIN_X, _BODY_TOP_Y + _ROW_H), "Challenge-mode jumper", fill=_ON)
        draw.text((_MARGIN_X, _BODY_TOP_Y + 2 * _ROW_H), "reading is unstable.", fill=_ON)
        draw.text((_MARGIN_X, _BODY_TOP_Y + 3 * _ROW_H), "Reseat the GPIO23/GND", fill=_ON)
        draw.text((_MARGIN_X, _FOOTER_Y + _ROW_H // 2), "jumper cap.", fill=_ON)

        return image

    def _render_ready(self) -> Image.Image:
        """Render READY view - IP, model name, ready status."""
        assert self.display_driver is not None
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        # Title
        draw.text((_MARGIN_X, _TITLE_Y), "READY TO START", fill=_ON)
        draw.line([(_MARGIN_X, _SEPARATOR_Y), (self.display_driver.get_width(), _SEPARATOR_Y)], fill=_ON, width=1)

        # IP address, only when there is one. At competition the robot runs off
        # any network, so this resolves to OFFLINE and the line becomes a
        # permanent non-warning -- it draws the eye at the start line and
        # crowds out the challenge mode, which is the one thing actually worth
        # checking there. Shown on a bench where it is genuinely useful for
        # SSHing in, omitted where it is noise.
        ip_address = ""
        if "Network" in self.system_status:
            ip_address = self.system_status["Network"].get("values", {}).get("ip_address", "")
        has_ip = bool(ip_address) and ip_address not in _NO_IP_VALUES

        row = 14
        if has_ip:
            draw.text((_MARGIN_X, row), f"IP: {ip_address}", fill=_ON)
            row += 10

        # AI Model
        draw.text((_MARGIN_X, row), "Model: yolov8n.hef", fill=_ON)
        row += 10

        # Challenge mode (visual pre-race confirmation of the jumper reading)
        mode = "?"
        if "ChallengeMode" in self.system_status:
            mode = self.system_status["ChallengeMode"].get("message", "?")
        draw.text((_MARGIN_X, row), f"MODE: {mode}", fill=_ON)

        # Instruction
        draw.text((_MARGIN_X, _FOOTER_Y), "Press to START", fill=_ON)

        return image

    def _render_ackermann(self) -> Image.Image:
        """Render Ackermann page - velocity, steering, gyro."""
        assert self.display_driver is not None
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        # Title
        draw.text((_MARGIN_X, _TITLE_Y), "ACKERMANN", fill=_ON)
        draw.line([(_MARGIN_X, _SEPARATOR_Y), (self.display_driver.get_width(), _SEPARATOR_Y)], fill=_ON, width=1)

        # Velocity
        velocity = self.race_metrics.get("current_velocity", 0.0)
        draw.text((_MARGIN_X, _BODY_TOP_Y), f"Vel: {velocity:.2f} m/s", fill=_ON)

        # Steering
        steering = self.race_metrics.get("current_steering", 0.0)
        draw.text((_MARGIN_X, _BODY_TOP_Y + _ROW_H), f"Steer: {steering:.1f} deg", fill=_ON)

        # Gyro Yaw
        draw.text((_MARGIN_X, _BODY_TOP_Y + 2 * _ROW_H), f"Yaw: {self.gyro_yaw:.1f} deg", fill=_ON)

        # Laps
        laps = self.race_metrics.get("laps_completed", 0)
        draw.text((_MARGIN_X, _FOOTER_Y), f"Laps: {laps}/{self.race_metrics.get('target_laps', _DEFAULT_TARGET_LAPS)}", fill=_ON)

        return image

    def _render_hailo(self) -> Image.Image:
        """Render Hailo Vision page - NPU FPS, detections."""
        assert self.display_driver is not None
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        # Title
        draw.text((_MARGIN_X, _TITLE_Y), "HAILO VISION", fill=_ON)
        draw.line([(_MARGIN_X, _SEPARATOR_Y), (self.display_driver.get_width(), _SEPARATOR_Y)], fill=_ON, width=1)

        # NPU FPS
        draw.text((_MARGIN_X, _BODY_TOP_Y), f"NPU: {self.hailo_fps:.1f} FPS", fill=_ON)

        # Target detection (placeholder - would need actual detection data)
        draw.text((_MARGIN_X, _BODY_TOP_Y + _ROW_H), "Target: SEARCHING", fill=_ON)
        draw.text((_MARGIN_X, _BODY_TOP_Y + 2 * _ROW_H), "Conf: --", fill=_ON)
        draw.text((_MARGIN_X, _FOOTER_Y), "Dist: -- m", fill=_ON)

        return image

    def _render_lidar(self) -> Image.Image:
        """Render LiDAR page - spatial clearances."""
        assert self.display_driver is not None
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        # Title
        draw.text((_MARGIN_X, _TITLE_Y), "LIDAR", fill=_ON)
        draw.line([(_MARGIN_X, _SEPARATOR_Y), (self.display_driver.get_width(), _SEPARATOR_Y)], fill=_ON, width=1)

        # Clearances
        draw.text((_MARGIN_X, _BODY_TOP_Y), f"Front: {self.lidar_front:.0f} cm", fill=_ON)
        draw.text((_MARGIN_X, _BODY_TOP_Y + _ROW_H), f"Left:  {self.lidar_left:.0f} cm", fill=_ON)
        draw.text((_MARGIN_X, _BODY_TOP_Y + 2 * _ROW_H), f"Right: {self.lidar_right:.0f} cm", fill=_ON)

        # Path status
        path_status = "CLEAR"
        if self.lidar_front < _PATH_BLOCKED_CLEARANCE_CM:
            path_status = "BLOCKED"
        elif min(self.lidar_left, self.lidar_right) < _PATH_NARROW_CLEARANCE_CM:
            path_status = "NARROW"

        draw.text((_MARGIN_X, _FOOTER_Y), f"Path: {path_status}", fill=_ON)

        return image

    def _render_finished(self) -> Image.Image:
        """Render FINISHED view - final results."""
        assert self.display_driver is not None
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        # Title
        draw.text((_MARGIN_X, _TITLE_Y), "RACE FINISHED", fill=_ON)
        draw.line([(_MARGIN_X, _SEPARATOR_Y), (self.display_driver.get_width(), _SEPARATOR_Y)], fill=_ON, width=1)

        # Laps completed
        laps = self.race_metrics.get("laps_completed", 0)
        # Target comes from /race_metrics, not CompetitionSpecs.OPEN_CHALLENGE_LAPS:
        # the state machine picks the count from the challenge jumper, so
        # naming the Open Challenge constant here reports the wrong target for
        # an Obstacles run the moment the two figures differ.
        target = self.race_metrics.get("target_laps", _DEFAULT_TARGET_LAPS)
        draw.text((_MARGIN_X, _BODY_TOP_Y + _ROW_H // 2), f"Laps: {laps}/{target}", fill=_ON)

        # Total time
        race_time = self.race_metrics.get("total_race_time", 0.0)
        minutes = int(race_time // 60)
        seconds = race_time % 60
        draw.text((_MARGIN_X, _BODY_TOP_Y + 2 * _ROW_H), f"Time: {minutes}:{seconds:05.2f}", fill=_ON)

        # Status
        # Same reason as the lap count above: judged against the round's own
        # target, not the Open Challenge's, or an Obstacles run that finished
        # correctly would be reported as an E-STOP.
        status = "COMPLETE" if laps >= target else "E-STOP"
        draw.text((_MARGIN_X, _FOOTER_Y), f"Status: {status}", fill=_ON)

        return image

    def _publish_mirror_image(self, pil_image: Image.Image) -> None:
        """Publish display image to ROS2 topic for remote viewing.

        Args:
            pil_image: PIL Image to publish.
        """
        if self.oled_mirror_pub is None or self.bridge is None:
            return

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
        except (RuntimeError, ValueError) as e:
            self.get_logger().warning(f"Failed to publish mirror image: {e}")


def main(args: list[str] | None = None) -> None:
    """Run the OLED display node, auto-configuring and auto-activating on launch."""
    rclpy.init(args=args)
    node = OLEDDisplayNode()

    try:
        node.trigger_configure()
        node.trigger_activate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
