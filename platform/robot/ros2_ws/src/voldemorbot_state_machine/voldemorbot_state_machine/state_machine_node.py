"""ROS2 node for WRO competition state machine control.

Run on: Raspberry Pi 5

Usage:
    ros2 run voldemorbot_state_machine state_machine_node

Topics:
    Subscribed:
        - /imu/data (sensor_msgs/Imu) - IMU data
        - /scan (sensor_msgs/LaserScan) - LiDAR data
        - /hailo/detections (vision_msgs/Detection2DArray) - Hailo AI detections
        - /hailo/fps (std_msgs/Float32) - Hailo inference FPS
    Subscribed:
        - /button/event (std_msgs/String) — button events from button_node (Pi Zero)
        - /challenge_mode/jumper_inserted (std_msgs/Bool) — challenge-mode jumper (Pi Zero)
    Published:
        - /robot_state (std_msgs/String) - Current robot state
        - /ackermann_cmd (ackermann_msgs/AckermannDriveStamped) - Drive commands
        - /system_status (diagnostic_msgs/DiagnosticArray) - System diagnostics
        - /race_metrics (std_msgs/String) - Race metrics (JSON)
"""

import json
import socket
import time
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
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
from shared.config.constants import CompetitionSpecs
from std_msgs.msg import Bool, Float32, String

from src.ros2.params import declare_and_get_float_param, declare_and_get_int_param
from src.state_machine import (
    RaceMetrics,
    RobotState,
    ScenarioType,
    SensorStatus,
    StateMachine,
    StateTransition,
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

_DEFAULT_PUBLISHER_RATE_HZ = 10.0
"""Default rate for publishing state and diagnostics; overridable via the
``publisher_rate_hz`` ROS2 parameter."""

_DEFAULT_TARGET_LAPS = CompetitionSpecs.OPEN_CHALLENGE_LAPS
"""Default laps required to complete race; overridable via the
``target_laps`` ROS2 parameter."""

_CHALLENGE_MODE_SAMPLES_REQUIRED = 3
"""Consecutive agreeing BOOT_CHECK-tick samples required before trusting the challenge-mode
jumper reading. At the default 10 Hz state-machine loop rate this spans ~300 ms -- the
200-300 ms debounce window from the jumper spec -- without a blocking sleep in the ROS2
spin loop (each tick reads the latest value received from the Pi Zero, not a driver-internal
sample loop)."""

_CHALLENGE_MODE_TIMEOUT_SEC = 15.0
"""How long to wait for the Pi Zero's jumper reading before defaulting to Open.

Generous on purpose: the Zero takes ~10 s from power-on to having its nodes up
(it shares power with this board), so a shorter window would routinely default
before the reading ever arrives. The topic is TRANSIENT_LOCAL, so a value
published before this node subscribed still arrives immediately.
"""

_JUMPER_TOPIC = "/challenge_mode/jumper_inserted"
"""Challenge-mode jumper state, published by the Pi Zero (which the wire is attached to)."""


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

        # simulator.launch.py doesn't launch the IMU/vision/LiDAR nodes, so those
        # sensor topics never publish under Gazebo -- BOOT_CHECK must not block
        # forever waiting for messages that will never arrive.
        self.declare_parameter("is_simulation", value=False)
        self.is_simulation: bool = self.get_parameter("is_simulation").get_parameter_value().bool_value
        if self.is_simulation:
            self.get_logger().info("Running in SIMULATION mode -- hardware readiness checks bypassed")

        self.publisher_rate_hz = declare_and_get_float_param(self, "publisher_rate_hz", _DEFAULT_PUBLISHER_RATE_HZ)
        self.target_laps = declare_and_get_int_param(self, "target_laps", _DEFAULT_TARGET_LAPS)
        # If the launch file explicitly overrode target_laps, honor that override instead of
        # letting jumper-based detection silently replace it once the mode is known below.
        self._target_laps_explicit = self.target_laps != _DEFAULT_TARGET_LAPS

        # State machine
        self.state_machine = StateMachine()
        self.state_machine.register_transition_callback(self._on_state_transition)

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
        self.button_sub: Subscription[String] = self.create_subscription(
            String,
            "/button/event",
            self._button_event_callback,
            10,
        )

        # Sensor status tracking
        self.imu_last_msg_time: float | None = None
        self.lidar_last_msg_time: float | None = None
        self.hailo_last_msg_time: float | None = None
        self.hailo_fps: float = 0.0

        # Skipped entirely under simulation: scenario_catalog.py already encodes
        # open-vs-obstacles per scenario.
        # The jumper lives on the Pi Zero's GPIO23, so its state arrives as a
        # topic rather than a local GPIO read (see _sample_challenge_mode).
        self._jumper_inserted: bool | None = None
        self._challenge_mode_wait_started = self.get_clock().now().nanoseconds / 1e9
        self.create_subscription(
            Bool,
            _JUMPER_TOPIC,
            self._on_jumper_state,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                # Must match the publisher: the mode is latched once at setup,
                # so a late-joining subscriber has to receive the last value
                # rather than wait for the next periodic publish.
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._challenge_mode_samples: deque[bool] = deque(maxlen=_CHALLENGE_MODE_SAMPLES_REQUIRED)
        self._challenge_mode_error: str | None = None
        self.challenge_mode: ScenarioType | None = None

    def _on_jumper_state(self, msg: Bool) -> None:
        """Latest challenge-mode jumper reading from the Pi Zero."""
        self._jumper_inserted = msg.data

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
        self._executor = ThreadPoolExecutor(max_workers=2)

        # Timers
        self.state_timer: Timer = self.create_timer(1.0 / self.publisher_rate_hz, self._state_machine_loop)

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
                ip: str = s.getsockname()[0]
                s.close()
            except OSError:
                return "OFFLINE"
            else:
                return ip

        def on_complete(future: Future[str]) -> None:
            """Callback when IP fetch completes."""
            try:
                self.ip_address = future.result()
            except (RuntimeError, OSError) as e:
                self.ip_address = "OFFLINE"
                self.get_logger().warning(f"Failed to fetch IP: {e}")
            else:
                self.get_logger().info(f"IP address resolved: {self.ip_address}")
            self.ip_fetch_complete = True

        future = self._executor.submit(fetch_ip)
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

    def _button_event_callback(self, msg: String) -> None:
        """Handle button events published by button_node on the Pi Zero."""
        current_state = self.state_machine.current_state
        event = msg.data

        if current_state == RobotState.READY and event == "short_press":
            self.get_logger().info("Button pressed - Starting race!")
            self.state_machine.transition_to(RobotState.RACING, StateTransitionReason.BUTTON_PRESSED)
            self.race_start_time = time.time()
            self.laps_completed = 0

        elif current_state == RobotState.RACING and event == "long_press":
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

    def _sample_challenge_mode(self) -> None:
        """Sample the jumper state published by the Pi Zero; stop once it stabilizes.

        The jumper is wired to the ZERO's GPIO23, so this consumes
        ``/challenge_mode/jumper_inserted`` rather than reading GPIO locally.
        Reading it here used to mean reading *Pi 5's* GPIO23, which nothing is
        attached to -- with the internal pull-up that always reads HIGH, so it
        silently latched Open Challenge on every boot and Obstacle could never
        be selected.

        Disagreeing samples (bounce/intermittent contact) clear progress and
        keep BOOT_CHECK waiting rather than guessing a mode. If the Zero never
        publishes at all, ``_challenge_mode_timed_out`` falls back to Open.
        """
        if self.is_simulation or self.challenge_mode is not None:
            return

        inserted = self._jumper_inserted
        if inserted is None:
            if self._challenge_mode_timed_out():
                self._latch_challenge_mode(inserted=False, reason="no reading from the Pi Zero")
                return
            self._challenge_mode_error = "waiting for /challenge_mode/jumper_inserted from the Pi Zero"
            self._challenge_mode_samples.clear()
            return

        self._challenge_mode_error = None
        self._challenge_mode_samples.append(inserted)
        if (
            len(self._challenge_mode_samples) < _CHALLENGE_MODE_SAMPLES_REQUIRED
            or len(set(self._challenge_mode_samples)) != 1
        ):
            return

        self._latch_challenge_mode(inserted=inserted)

    def _challenge_mode_timed_out(self) -> bool:
        """True once we've waited long enough for the Zero to publish the jumper state."""
        waited = self.get_clock().now().nanoseconds / 1e9 - self._challenge_mode_wait_started
        return waited >= _CHALLENGE_MODE_TIMEOUT_SEC

    def _latch_challenge_mode(self, *, inserted: bool, reason: str | None = None) -> None:
        """Fix the challenge mode for this run and derive the lap count."""
        self.challenge_mode = ScenarioType.OBSTACLES if inserted else ScenarioType.OPEN
        if not self._target_laps_explicit:
            self.target_laps = (
                CompetitionSpecs.OBSTACLE_CHALLENGE_LAPS
                if self.challenge_mode == ScenarioType.OBSTACLES
                else CompetitionSpecs.OPEN_CHALLENGE_LAPS
            )
        if reason is None:
            self._challenge_mode_error = None
            self.get_logger().info(
                f"Challenge mode detected: {self.challenge_mode.value} (target_laps={self.target_laps})",
            )
            return

        # Fell back rather than detected. Keep the error set so the OLED shows
        # the challenge-mode fault screen and the operator can see the jumper
        # was never read, instead of silently racing the wrong challenge.
        self._challenge_mode_error = f"defaulted to {self.challenge_mode.value}: {reason}"
        self.get_logger().error(
            f"Challenge-mode jumper unreadable ({reason}) after {_CHALLENGE_MODE_TIMEOUT_SEC}s - "
            f"defaulting to {self.challenge_mode.value} (target_laps={self.target_laps}). "
            "Check the jumper wiring on the Pi Zero's GPIO23.",
        )

    def _handle_boot_check(self) -> None:
        """Handle BOOT_CHECK state - verify all hardware."""
        self._sample_challenge_mode()
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
            # Not warned about in the Open Challenge: vision is not required there,
            # so warning would report a blocker that is not blocking anything.
            if not system_status.hailo_status.is_ready and self.challenge_mode != ScenarioType.OPEN:
                self.get_logger().warning(f"Hailo not ready: {system_status.hailo_status.error_message}")
            if not system_status.challenge_mode_status.is_ready:
                self.get_logger().warning(
                    f"Challenge mode not ready: {system_status.challenge_mode_status.error_message}",
                )

    def _handle_ready(self) -> None:
        """Handle READY state - wait for button press."""
        # Just wait - button handling is done in button_check_loop

    def _handle_racing(self) -> None:
        """Handle RACING state - monitor for race completion."""
        # Check if laps completed
        if self.laps_completed >= self.target_laps:
            self.get_logger().info(f"Race complete! {self.target_laps} laps finished")
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

        if self.is_simulation:
            # No IMU/vision/LiDAR nodes are launched under simulator.launch.py, so
            # these topics never publish -- treat sensors as ready unconditionally
            # instead of gating BOOT_CHECK on messages that will never arrive.
            imu_status = SensorStatus(name="IMU", is_ready=True, error_message=None)
            lidar_status = SensorStatus(name="LiDAR", is_ready=True, error_message=None)
            hailo_status = SensorStatus(name="Hailo", is_ready=True, error_message=None)
            challenge_mode_status = SensorStatus(name="ChallengeMode", is_ready=True, error_message=None)
        else:
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

            challenge_mode_ready = self.challenge_mode is not None
            challenge_mode_status = SensorStatus(
                name="ChallengeMode",
                is_ready=challenge_mode_ready,
                error_message=None
                if challenge_mode_ready
                else (self._challenge_mode_error or "Jumper reading not yet stable"),
            )

        # Check Drive (assume ready if we can publish - actual motor verification would need hardware driver)
        drive_status = SensorStatus(name="Drive", is_ready=True, error_message=None)

        # Network status (non-blocking)
        network_status = self.ip_address if self.ip_fetch_complete else "FETCHING..."

        # Vision is only needed to read traffic signs, which exist solely in the
        # Obstacle Challenge -- the Open Challenge navigates on LIDAR alone. Gating
        # it unconditionally meant an unloaded Hailo model blocked BOOT_CHECK for
        # both challenges, so the robot could not race at all without vision.
        # challenge_mode is latched earlier in BOOT_CHECK, so it is known here;
        # while it is still None the challenge_mode_status term below keeps us
        # waiting anyway, so treating vision as required until then is safe.
        hailo_required = self.challenge_mode != ScenarioType.OPEN

        all_ready = (
            imu_ready
            and lidar_ready
            and (hailo_ready or not hailo_required)
            and drive_status.is_ready
            and challenge_mode_status.is_ready
            and self.ip_fetch_complete
        )

        return SystemStatus(
            imu_status=imu_status,
            lidar_status=lidar_status,
            hailo_status=hailo_status,
            drive_status=drive_status,
            challenge_mode_status=challenge_mode_status,
            network_status=network_status,
            all_ready=all_ready,
            challenge_mode=self.challenge_mode,
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
            system_status.challenge_mode_status,
        ]:
            status = DiagnosticStatus()
            status.name = sensor.name
            status.level = DiagnosticStatus.OK if sensor.is_ready else DiagnosticStatus.ERROR
            if sensor.name == "ChallengeMode" and system_status.challenge_mode is not None:
                status.message = system_status.challenge_mode.value.upper()
            else:
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

    def _on_state_transition(self, transition: StateTransition) -> None:
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

        # Shutdown executor
        self._executor.shutdown(wait=False)

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
