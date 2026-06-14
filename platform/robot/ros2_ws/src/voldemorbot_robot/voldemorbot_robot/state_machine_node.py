"""ROS2 node for WRO competition state machine control.

Run on: Raspberry Pi 5

Usage:
    ros2 run voldemorbot_robot state_machine_node

Topics:
    Subscribed:
        - /imu/data (sensor_msgs/Imu) - IMU data
        - /scan (sensor_msgs/LaserScan) - LiDAR data
        - /hailo/detections (vision_msgs/Detection2DArray) - Hailo AI detections
        - /hailo/fps (std_msgs/Float32) - Hailo inference FPS
    Published:
        - /robot_state (std_msgs/String) - Current robot state
        - /ackermann_cmd (ackermann_msgs/AckermannDriveStamped) - Drive commands
        - /system_status (diagnostic_msgs/DiagnosticArray) - System diagnostics
        - /race_metrics (std_msgs/String) - Race metrics (JSON)
"""

import asyncio
import json
import socket
import time
from concurrent.futures import ThreadPoolExecutor
from typing import TYPE_CHECKING, override

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, String

from src.hardware.button.gpio import Driver as ButtonDriver
from src.state_machine import (
    RaceMetrics,
    RobotState,
    SensorStatus,
    StateMachine,
    StateTransitionReason,
    SystemStatus,
)

# Latched QoS for state/diagnostics — late-joining nodes see the last value immediately.
_QOS_TRANSIENT = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

# Reliable + 200 ms deadline for motor commands — missed deadlines surface as warnings.
_QOS_ACKERMANN = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    deadline=Duration(nanoseconds=200_000_000),
)

if TYPE_CHECKING:
    from rclpy.publisher import Publisher
    from rclpy.subscription import Subscription
    from rclpy.timer import Timer


NODE_NAME = "state_machine_node"
"""ROS2 node name for state machine controller."""

PUBLISHER_RATE_HZ = 10.0
"""Rate for publishing state and diagnostics."""

HAILO_MODEL_PATH = "/home/pi/models/yolov8n.hef"
"""Path to Hailo .hef model file."""

TARGET_LAPS = 3
"""Number of laps required to complete race."""


class StateMachineNode(Node):
    """ROS2 node that manages the 4-stage state machine for WRO competition.

    Responsibilities:
    - Monitor button for state transitions and E-STOP
    - Verify hardware components during BOOT_CHECK
    - Fetch IP address asynchronously without blocking
    - Publish robot state and diagnostics
    - Control race start/stop via Ackermann commands
    """

    def __init__(self) -> None:
        """Initialize state machine node."""
        super().__init__(NODE_NAME)

        self.get_logger().info("Initializing WRO State Machine Node")

        # State machine
        self.state_machine = StateMachine()
        self.state_machine.register_transition_callback(self._on_state_transition)

        # Button driver for physical control
        try:
            self.button_driver = ButtonDriver()
            self.button_driver.connect()
            self.get_logger().info("Button driver connected")
        except (RuntimeError, OSError, ValueError, ImportError) as e:
            self.get_logger().error(f"Failed to connect button driver: {e}")
            self.button_driver = None

        # Publishers — robot_state and system_status are TRANSIENT_LOCAL so late
        # subscribers (RViz, dashboard) receive the last value without waiting.
        self.state_pub: Publisher[String] = self.create_publisher(String, "/robot_state", _QOS_TRANSIENT)
        self.ackermann_pub: Publisher[AckermannDriveStamped] = self.create_publisher(
            AckermannDriveStamped,
            "/ackermann_cmd",
            _QOS_ACKERMANN,
        )
        self.diagnostics_pub: Publisher[DiagnosticArray] = self.create_publisher(
            DiagnosticArray,
            "/system_status",
            _QOS_TRANSIENT,
        )
        self.metrics_pub: Publisher[String] = self.create_publisher(String, "/race_metrics", 10)

        # Subscribers — sensor topics use qos_profile_sensor_data (BEST_EFFORT +
        # VOLATILE, depth=10) to match the publisher QoS on sensor drivers.
        self.imu_sub: Subscription[Imu] = self.create_subscription(
            Imu,
            "/imu/data",
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.lidar_sub: Subscription[LaserScan] = self.create_subscription(
            LaserScan,
            "/scan",
            self._lidar_callback,
            qos_profile_sensor_data,
        )
        self.hailo_fps_sub: Subscription[Float32] = self.create_subscription(
            Float32,
            "/hailo/fps",
            self._hailo_fps_callback,
            qos_profile_sensor_data,
        )

        # Sensor status tracking
        self.imu_last_msg_time: float | None = None
        self.lidar_last_msg_time: float | None = None
        self.hailo_last_msg_time: float | None = None
        self.hailo_fps: float = 0.0

        # Network status
        self.ip_address: str = "FETCHING..."
        self.ip_fetch_complete: bool = False

        # Race metrics
        self.race_start_time: float | None = None
        self.laps_completed: int = 0
        self.current_velocity: float = 0.0
        self.current_steering: float = 0.0
        self.gyro_yaw: float = 0.0
        self.current_corridor: str = ""

        # Async executor for non-blocking operations
        self.executor = ThreadPoolExecutor(max_workers=2)

        # Timers
        self.state_timer: Timer = self.create_timer(1.0 / PUBLISHER_RATE_HZ, self._state_machine_loop)
        self.button_timer: Timer = self.create_timer(0.05, self._button_check_loop)  # 20Hz for responsive button

        # Start async IP fetch immediately
        self._fetch_ip_address_async()

        # Start boot check process
        self.get_logger().info("Starting BOOT_CHECK sequence")

    def _fetch_ip_address_async(self) -> None:
        """Fetch IP address in background thread without blocking."""

        def fetch_ip() -> str:
            """Fetch IP address from network interface."""
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.settimeout(2.0)
                s.connect(("8.8.8.8", 80))
                ip = s.getsockname()[0]
                s.close()
            except OSError:
                return "OFFLINE"
            else:
                return ip

        def on_complete(future: asyncio.Future) -> None:
            """Callback when IP fetch completes."""
            try:
                self.ip_address = future.result()
            except (RuntimeError, OSError) as e:
                self.ip_address = "OFFLINE"
                self.get_logger().warning("Failed to fetch IP: %s", e)
            else:
                self.get_logger().info("IP address resolved: %s", self.ip_address)
            self.ip_fetch_complete = True

        future = self.executor.submit(fetch_ip)
        future.add_done_callback(on_complete)

    def _imu_callback(self, _msg: Imu) -> None:
        """Handle IMU data."""
        self.imu_last_msg_time = time.time()
        self.gyro_yaw = 0.0

    def _lidar_callback(self, _msg: LaserScan) -> None:
        """Handle LiDAR scan data."""
        self.lidar_last_msg_time = time.time()

    def _hailo_fps_callback(self, msg: Float32) -> None:
        """Handle Hailo FPS updates."""
        self.hailo_last_msg_time = time.time()
        self.hailo_fps = msg.data

    def _button_check_loop(self) -> None:
        """Check button state at high frequency for responsive control."""
        if self.button_driver is None:
            return

        state = self.button_driver.get_state()

        # Handle button events based on current state
        current_state = self.state_machine.current_state

        if current_state == RobotState.READY and state.last_event and state.last_event.value == "short_press":
            self.get_logger().info("Button pressed - Starting race!")
            self.state_machine.transition_to(RobotState.RACING, StateTransitionReason.BUTTON_PRESSED)
            self.race_start_time = time.time()
            self.laps_completed = 0

        elif current_state == RobotState.RACING:
            # In RACING state, only long press triggers E-STOP
            if state.last_event and state.last_event.value == "long_press":
                self.get_logger().warning("EMERGENCY STOP activated!")
                self.state_machine.transition_to(RobotState.FINISHED, StateTransitionReason.EMERGENCY_STOP)
                self._publish_stop_command()

    def _state_machine_loop(self) -> None:
        """Main state machine loop - runs at 10Hz."""
        current_state = self.state_machine.current_state

        if current_state == RobotState.BOOT_CHECK:
            self._handle_boot_check()
        elif current_state == RobotState.READY:
            self._handle_ready()
        elif current_state == RobotState.RACING:
            self._handle_racing()
        elif current_state == RobotState.FINISHED:
            self._handle_finished()

        # Always publish current state and diagnostics
        self._publish_state()
        self._publish_diagnostics()

    def _handle_boot_check(self) -> None:
        """Handle BOOT_CHECK state - verify all hardware."""
        system_status = self._check_system_status()

        if system_status.all_ready:
            self.get_logger().info("All systems ready - transitioning to READY state")
            self.state_machine.transition_to(RobotState.READY, StateTransitionReason.BOOT_COMPLETE)
        else:
            # Log what's not ready
            if not system_status.imu_status.is_ready:
                self.get_logger().warning(f"IMU not ready: {system_status.imu_status.error_message}")
            if not system_status.lidar_status.is_ready:
                self.get_logger().warning(f"LiDAR not ready: {system_status.lidar_status.error_message}")
            if not system_status.hailo_status.is_ready:
                self.get_logger().warning(f"Hailo not ready: {system_status.hailo_status.error_message}")

    def _handle_ready(self) -> None:
        """Handle READY state - wait for button press."""
        # Just wait - button handling is done in button_check_loop

    def _handle_racing(self) -> None:
        """Handle RACING state - monitor for race completion."""
        # Check if laps completed
        if self.laps_completed >= TARGET_LAPS:
            self.get_logger().info(f"Race complete! {TARGET_LAPS} laps finished")
            self.state_machine.transition_to(RobotState.FINISHED, StateTransitionReason.LAPS_COMPLETED)
            self._publish_stop_command()

        # Publish race metrics
        self._publish_race_metrics()

    def _handle_finished(self) -> None:
        """Handle FINISHED state - display final results."""
        # Ensure robot is stopped
        self._publish_stop_command()

        # Publish final metrics
        self._publish_race_metrics()

    def _check_system_status(self) -> SystemStatus:
        """Check status of all hardware components."""
        current_time = time.time()
        timeout = 3.0  # 3 seconds timeout for sensor messages

        # Check IMU
        imu_ready = self.imu_last_msg_time is not None and (current_time - self.imu_last_msg_time) < timeout
        imu_status = SensorStatus(
            name="IMU",
            is_ready=imu_ready,
            error_message=None if imu_ready else "No IMU data received",
        )

        # Check LiDAR
        lidar_ready = self.lidar_last_msg_time is not None and (current_time - self.lidar_last_msg_time) < timeout
        lidar_status = SensorStatus(
            name="LiDAR",
            is_ready=lidar_ready,
            error_message=None if lidar_ready else "No LiDAR data received",
        )

        # Check Hailo (includes model loading verification via FPS > 0)
        hailo_ready = (
            self.hailo_last_msg_time is not None
            and (current_time - self.hailo_last_msg_time) < timeout
            and self.hailo_fps > 0.0
        )
        hailo_status = SensorStatus(
            name="Hailo",
            is_ready=hailo_ready,
            error_message=None if hailo_ready else "Hailo model not loaded or no inference",
        )

        # Check Drive (assume ready if we can publish - actual motor verification would need hardware driver)
        drive_status = SensorStatus(name="Drive", is_ready=True, error_message=None)

        # Network status (non-blocking)
        network_status = self.ip_address if self.ip_fetch_complete else "FETCHING..."

        all_ready = imu_ready and lidar_ready and hailo_ready and drive_status.is_ready and self.ip_fetch_complete

        return SystemStatus(
            imu_status=imu_status,
            lidar_status=lidar_status,
            hailo_status=hailo_status,
            drive_status=drive_status,
            network_status=network_status,
            all_ready=all_ready,
        )

    def _publish_state(self) -> None:
        """Publish current robot state."""
        msg = String()
        msg.data = self.state_machine.current_state.value
        self.state_pub.publish(msg)

    def _publish_diagnostics(self) -> None:
        """Publish system diagnostics."""
        system_status = self._check_system_status()

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Add status for each component
        for sensor in [
            system_status.imu_status,
            system_status.lidar_status,
            system_status.hailo_status,
            system_status.drive_status,
        ]:
            status = DiagnosticStatus()
            status.name = sensor.name
            status.level = DiagnosticStatus.OK if sensor.is_ready else DiagnosticStatus.ERROR
            status.message = sensor.error_message or "OK"
            msg.status.append(status)

        # Add network status
        network_status = DiagnosticStatus()
        network_status.name = "Network"
        network_status.level = DiagnosticStatus.OK
        network_status.message = system_status.network_status
        network_status.values.append(KeyValue(key="ip_address", value=system_status.network_status))
        msg.status.append(network_status)

        self.diagnostics_pub.publish(msg)

    def _publish_race_metrics(self) -> None:
        """Publish current race metrics."""
        elapsed_time = 0.0 if self.race_start_time is None else time.time() - self.race_start_time

        metrics = RaceMetrics(
            laps_completed=self.laps_completed,
            total_race_time=elapsed_time,
            current_velocity=self.current_velocity,
            current_steering=self.current_steering,
            gyro_yaw=self.gyro_yaw,
            current_corridor=self.current_corridor,
        )

        msg = String()
        msg.data = json.dumps(
            {
                "laps_completed": metrics.laps_completed,
                "total_race_time": round(metrics.total_race_time, 2),
                "current_velocity": round(metrics.current_velocity, 2),
                "current_steering": round(metrics.current_steering, 2),
                "gyro_yaw": round(metrics.gyro_yaw, 2),
                "current_corridor": metrics.current_corridor,
            },
        )
        self.metrics_pub.publish(msg)

    def _publish_stop_command(self) -> None:
        """Publish stop command (zero velocity and steering)."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.ackermann_pub.publish(msg)

        self.current_velocity = 0.0
        self.current_steering = 0.0

        self.get_logger().info("Published STOP command")

    def _on_state_transition(self, transition: object) -> None:
        """Callback for state transitions."""
        self.get_logger().info(
            f"State transition: {transition.from_state.value} -> {transition.to_state.value} "
            f"(reason: {transition.reason.value})",
        )

    @override
    def destroy_node(self) -> None:
        """Shutdown the state machine node."""
        self.get_logger().info("Shutting down State Machine Node")

        # Ensure robot is stopped
        self._publish_stop_command()

        # Close button driver
        if self.button_driver is not None:
            self.button_driver.close()

        # Shutdown executor
        self.executor.shutdown(wait=False)

        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Main entry point for state machine node."""
    rclpy.init(args=args)
    node = StateMachineNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
