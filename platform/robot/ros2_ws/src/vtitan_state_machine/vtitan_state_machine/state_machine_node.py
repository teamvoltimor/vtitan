"""ROS2 node for WRO competition state machine control.

Run on: Raspberry Pi 5

Usage:
    ros2 run vtitan_state_machine state_machine_node

Topics:
    Subscribed:
        - /imu/data (sensor_msgs/Imu) - IMU data
        - /scan (sensor_msgs/LaserScan) - LiDAR data
        - /hailo/detections (vision_msgs/Detection2DArray) - Hailo AI detections
        - /vision/detections (std_msgs/String, JSON) - vision pipeline liveness
    Subscribed:
        - /button/event (std_msgs/String) — button events from button_node (Pi Zero)
        - /challenge_mode/jumper_inserted (std_msgs/Bool) — challenge-mode jumper (Pi Zero)
        - /ackermann_cmd (ackermann_msgs/AckermannDriveStamped) — mirrors the real
          navigator's drive commands into /race_metrics' current_velocity/current_steering
    Published:
        - /robot_state (std_msgs/String) - Current robot state
        - /ackermann_cmd (ackermann_msgs/AckermannDriveStamped) - Drive commands
        - /system_status (diagnostic_msgs/DiagnosticArray) - System diagnostics
        - /race_metrics (std_msgs/String) - Race metrics (JSON)
"""


import math
import os
import socket
import subprocess
import time
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
from typing import TYPE_CHECKING, override

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pydantic import AliasChoices, Field
from pydantic_settings import SettingsConfigDict
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from shared.config.constants import CompetitionSpecs
from shared.config.ros_topics import RosTopicConfig
from std_msgs.msg import Bool, Int32, String

from src.hardware.settings_base import CONFIG_DIR, SAFE_SHUTDOWN_BOTH_SCRIPT, HardwareBaseSettings
from src.ros2.params import declare_and_get_bool_param, declare_and_get_float_param, declare_and_get_int_param
from src.ros2.qos import (
    QOS_ACKERMANN_CMD,
    QOS_LATCHED_STATE,
    QOS_LATCHED_STATE_RELIABLE,
    QOS_LIVE_READOUT,
    QOS_STREAM,
)
from src.ros2.resettable_node import ResettableNode
from src.ros2.wire_models import RaceMetricsWire
from src.state_machine import (
    RaceStatus,
    RobotState,
    ScenarioType,
    SensorStatus,
    StateMachine,
    StateTransition,
    StateTransitionReason,
    SystemStatus,
)

# State/diagnostics use the shared QOS_LATCHED_STATE (see src/ros2/qos.py):
# late-joining nodes see the last value immediately (TRANSIENT_LOCAL), and
# reliability is BEST_EFFORT, not RELIABLE, on purpose -- RELIABLE's flow
# control holds a writer's publish() call until the matched reader acks, and
# the Pi Zero's oled_display_node -- the only subscriber to either of these
# topics -- was measured stalling for 30+ seconds under its own CPU/memory
# contention (see telemetry_bridge_node.py's QOS_LIVE_READOUT for the full
# story). A RELIABLE /robot_state publisher would block this node's
# publish() for the same duration, which is exactly why the OLED was seen
# stuck on a stale BOOT_CHECK page well after the real state had moved on to
# READY. This is safe to drop reliability on: _publish_state runs every tick
# of _state_machine_loop (publisher_rate_hz, not just on transitions), so a
# single dropped sample is corrected within one tick, not lost until the
# next real transition.

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


class NodeConfig(HardwareBaseSettings):
    """Challenge-mode jumper debounce/timeout config.

    Configurable via config/hardware/state_machine/state_machine_node.toml.
    Matches every hardware driver's Config pattern. publisher_rate_hz/
    target_laps are NOT here -- those already go through
    declare_and_get_float_param/declare_and_get_int_param (ROS2 parameters).
    """

    model_config = SettingsConfigDict(
        env_prefix="", toml_file=CONFIG_DIR / "state_machine" / "state_machine_node.toml",
    )

    challenge_mode_samples_required: int = Field(
        default=3, validation_alias=AliasChoices("CHALLENGE_MODE_SAMPLES_REQUIRED", "challenge_mode_samples_required"),
    )
    """Consecutive agreeing BOOT_CHECK-tick samples required before trusting the challenge-mode
    jumper reading. At the default 10 Hz state-machine loop rate this spans ~300 ms -- the
    200-300 ms debounce window from the jumper spec -- without a blocking sleep in the ROS2
    spin loop (each tick reads the latest value received from the Pi Zero, not a driver-internal
    sample loop)."""

    challenge_mode_timeout_sec: float = Field(
        default=60.0, validation_alias=AliasChoices("CHALLENGE_MODE_TIMEOUT_SEC", "challenge_mode_timeout_sec"),
    )
    """How long to wait for the Pi Zero's jumper reading before defaulting to Open.

    Generous on purpose. The previous value (15 s) assumed the Zero takes ~10 s
    from power-on to having its nodes up; measured on a genuine simultaneous
    cold boot of both boards from battery (2026-08-08), the Zero's own local
    startup chain -- pixi task launch overhead, then ROS2 launch, then
    ackermann_motor_node and pi_zero_peripherals_node constructing their
    hardware drivers in sequence -- took 41 s end to end before the jumper
    GPIO connected at all, let alone published. 15 s made the fallback fire on
    every such cold boot, not just flaky ones (see also
    scripts/discovery-watchdog.sh, which repairs the separate, rarer case
    where the USB-gadget link itself comes up half-dead and never completes
    ROS2 discovery no matter how long this waits). The topic is
    TRANSIENT_LOCAL, so a value published before this node subscribed still
    arrives immediately -- this ceiling only matters for how long BOOT_CHECK
    is willing to wait when it hasn't yet.
    """


_node_config = NodeConfig()
_CHALLENGE_MODE_SAMPLES_REQUIRED = _node_config.challenge_mode_samples_required
_CHALLENGE_MODE_TIMEOUT_SEC = _node_config.challenge_mode_timeout_sec

class StateMachineNode(Node, ResettableNode):
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
        self.is_simulation: bool = declare_and_get_bool_param(self, "is_simulation", default=False)
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

        self._topics = RosTopicConfig.load_default()

        # Publishers — robot_state and system_status are TRANSIENT_LOCAL so late
        # subscribers (RViz, dashboard) receive the last value without waiting.
        self.state_pub: Publisher[String] = self.create_publisher(String, self._topics.state_machine.state, QOS_LATCHED_STATE)
        self.ackermann_pub: Publisher[AckermannDriveStamped] = self.create_publisher(
            AckermannDriveStamped,
            self._topics.commands.ackermann_cmd,
            QOS_ACKERMANN_CMD,
        )
        self.diagnostics_pub: Publisher[DiagnosticArray] = self.create_publisher(
            DiagnosticArray,
            self._topics.state_machine.system_status,
            QOS_LATCHED_STATE,
        )
        self.metrics_pub: Publisher[String] = self.create_publisher(String, self._topics.state_machine.race_metrics, QOS_STREAM)
        # track_navigator_node has no other way to learn which challenge the
        # jumper resolved to -- it never subscribed to anything from this node
        # before, so a real blind run silently defaulted to Open regardless of
        # the jumper. Latched so a late-joining/restarted navigator still gets
        # the current value immediately; republished by _latch_challenge_mode
        # on every BOOT_CHECK resolution, including the one after a
        # SYSTEM_RESET, which is what lets the robot switch challenges purely
        # from the button.
        self.challenge_mode_pub: Publisher[String] = self.create_publisher(
            String, self._topics.challenge_mode.active, QOS_LATCHED_STATE,
        )

        # /race_metrics' current_velocity/current_steering used to only ever be set
        # by this node's own _publish_stop_command (always to 0.0) -- nothing updated
        # them from the real driving commands Ros2HardwareGateway.publish_drive()
        # actually publishes to this same topic, so the metrics read zero for the
        # entire race. Subscribing here, rather than duplicating CoreNavigator's
        # command computation, means this always matches whatever the robot is
        # actually being told to do, from whichever publisher last sent it.
        self.ackermann_sub: Subscription[AckermannDriveStamped] = self.create_subscription(
            AckermannDriveStamped,
            self._topics.commands.ackermann_cmd,
            self._ackermann_callback,
            QOS_ACKERMANN_CMD,
        )

        # Subscribers — sensor topics use qos_profile_sensor_data (BEST_EFFORT +
        # VOLATILE, depth=10) to match the publisher QoS on sensor drivers.
        self.imu_sub: Subscription[Imu] = self.create_subscription(
            Imu,
            self._topics.sensors.imu,
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.lidar_sub: Subscription[LaserScan] = self.create_subscription(
            LaserScan,
            self._topics.sensors.scan,
            self._lidar_callback,
            qos_profile_sensor_data,
        )
        # Vision liveness is read off the DETECTIONS stream, not /hailo/fps.
        # Nothing has ever published /hailo/fps -- the topic exists in
        # ros_topics.toml and here, and nowhere else in the repo -- so the
        # readiness test below could never pass on hardware. It went unnoticed
        # because the BOOT_CHECK gate exempts the Open Challenge, which is the
        # only challenge that had ever been run on the robot; the first
        # Obstacles boot sat in BOOT_CHECK forever with vision healthy and
        # publishing at 15 Hz. Topic name comes from the config, like every
        # other subscription here.
        self.vision_detections_sub: Subscription[String] = self.create_subscription(
            String,
            self._topics.sensors.vision_detections,
            self._vision_detections_callback,
            qos_profile_sensor_data,
        )
        self.button_sub: Subscription[String] = self.create_subscription(
            String,
            self._topics.button.event,
            self._button_event_callback,
            QOS_STREAM,
        )
        # track_navigator_node's CoreNavigator/LapDetector is the only thing
        # that actually detects a lap crossing -- this node used to track its
        # own laps_completed with nothing anywhere incrementing it, so the
        # OLED's lap counter stayed 0 and _handle_racing's laps_completed >=
        # target_laps check could never fire, meaning a race never finished
        # on its own (only a manual E-STOP hold reached FINISHED). QoS must
        # match track_navigator_node's publisher (BEST_EFFORT) or this
        # receives nothing at all, same failure mode as /robot_state.
        self.laps_sub: Subscription[Int32] = self.create_subscription(
            Int32,
            self._topics.navigation.laps_completed,
            self._on_laps_completed,
            QOS_LIVE_READOUT,
        )
        # Active corridor, published every tick by track_navigator_node's
        # CoreNavigator (see its _control_loop). Used only to populate
        # /race_metrics' current_corridor for the OLED CORRIDOR line -- it never
        # drives state transitions here. Empty until the first sample arrives.
        self.corridor_sub: Subscription[String] = self.create_subscription(
            String,
            self._topics.navigation.current_corridor,
            self._on_corridor,
            QOS_LIVE_READOUT,
        )

        # Sensor status tracking
        self.imu_last_msg_time: float | None = None
        self.lidar_last_msg_time: float | None = None
        # Timestamp of the last vision detection message. Kept under the hailo_
        # name because it is the Hailo pipeline's liveness that BOOT_CHECK
        # gates on; what changed is the SIGNAL, from an FPS topic nobody
        # publishes to the detections the vision node really emits.
        self.hailo_last_msg_time: float | None = None

        # Skipped entirely under simulation: scenario_catalog.py already encodes
        # open-vs-obstacles per scenario.
        # The jumper lives on the Pi Zero's GPIO23, so its state arrives as a
        # topic rather than a local GPIO read (see _sample_challenge_mode).
        self._jumper_inserted: bool | None = None
        self._challenge_mode_wait_started = self.get_clock().now().nanoseconds / 1e9
        # QOS_LATCHED_STATE_RELIABLE must match the publisher (challenge_mode_node):
        # the mode is latched once at setup, so a late-joining subscriber has to
        # receive the last value rather than wait for the next periodic publish.
        self.create_subscription(
            Bool,
            self._topics.challenge_mode.jumper_inserted,
            self._on_jumper_state,
            QOS_LATCHED_STATE_RELIABLE,
        )
        self._challenge_mode_samples: deque[bool] = deque(maxlen=_CHALLENGE_MODE_SAMPLES_REQUIRED)
        self._challenge_mode_error: str | None = None
        self.challenge_mode: ScenarioType | None = None

        # Network status
        self.ip_address: str = "FETCHING..."
        self.ip_fetch_complete: bool = False

        # Race metrics
        self.race_start_time: float | None = None
        self.laps_completed: int = 0
        # Whether ``laps_completed`` describes the race now running. False from
        # the moment RACING is entered until track_navigator_node publishes the
        # zero produced by its own reset -- see _on_laps_completed.
        self._lap_count_is_current: bool = True
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

    def _on_jumper_state(self, msg: Bool) -> None:
        """Latest challenge-mode jumper reading from the Pi Zero.

        Only records the reading. Everything else that used to live here
        belonged to __init__ and had been swallowed into this callback, so the
        node's construction depended on a message arriving -- see the commit
        that moved it back.
        """
        self._jumper_inserted = msg.data

    def _on_laps_completed(self, msg: Int32) -> None:
        """Latest lap count from track_navigator_node's LapDetector.

        Replaces rather than increments: track_navigator_node's own reset() is
        the authoritative zero point, so mirroring its count exactly can never
        drift from it the way an independently-incremented counter could.

        That reset does not happen when this node's does, though. This node
        resets on FINISHED -> BOOT_CHECK; the navigator resets only on entering
        RACING, and until then keeps publishing the finished race's count every
        control tick. So a button-cycled re-run used to reach _handle_racing
        with the previous race's total still in hand and finish instantly --
        measured on hardware 2026-08-06, and only a reboot cleared it, because
        that is what reconstructed the navigator at zero.

        Ignoring non-zero counts until the navigator's post-reset zero arrives
        closes that window rather than racing it. The zero is unambiguous: the
        navigator publishes its stale total right up to its reset, so the first
        zero after RACING is entered can only be the reset's. Ordering per
        publisher makes the rest safe -- nothing from before it follows it.
        """
        if not self._lap_count_is_current:
            if msg.data != 0:
                return
            self._lap_count_is_current = True
        self.laps_completed = msg.data

    def _on_corridor(self, msg: String) -> None:
        """Latest active corridor from track_navigator_node's CoreNavigator.

        Stored verbatim (the Section name string, or "" before the first
        sample) and forwarded into /race_metrics' current_corridor -- it is
        display-only here and never gates a state transition.
        """
        self.current_corridor = msg.data

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

    def _imu_callback(self, msg: Imu) -> None:
        """Handle IMU data.

        ``gyro_yaw`` used to be hardcoded to 0.0 here regardless of the
        message -- /race_metrics reported a flat yaw for the entire race.
        Derived from the orientation quaternion (matching the convention used
        elsewhere for this same computation, e.g. real-hardware bag analysis),
        not the raw angular_velocity.z: that is a rate, not a heading, and
        integrating it here would drift independently of whatever heading
        reference the navigator itself is using.
        """
        self.imu_last_msg_time = time.time()
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.gyro_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _ackermann_callback(self, msg: AckermannDriveStamped) -> None:
        """Mirror the last commanded drive -- from whichever publisher sent it.

        ``Ros2HardwareGateway.publish_drive`` (the real navigator) and this
        node's own ``_publish_stop_command`` both publish to ``/ackermann_cmd``;
        subscribing to it rather than duplicating either's computation means
        ``current_velocity``/``current_steering`` always match what the robot
        was actually just told to do.
        """
        self.current_velocity = msg.drive.speed
        self.current_steering = math.degrees(msg.drive.steering_angle)

    def _lidar_callback(self, _msg: LaserScan) -> None:
        """Handle LiDAR scan data."""
        self.lidar_last_msg_time = time.time()

    def _vision_detections_callback(self, msg: String) -> None:
        """Note that the vision pipeline is alive.

        The message CONTENT is deliberately ignored: an empty detection list is
        a perfectly healthy frame -- most frames on an empty stretch of track
        carry no signs -- so arrival is the liveness signal, not payload.
        """
        del msg
        self.hailo_last_msg_time = time.time()

    def _button_event_callback(self, msg: String) -> None:
        """Handle button events published by button_node on the Pi Zero."""
        current_state = self.state_machine.current_state
        event = msg.data

        if current_state == RobotState.READY and event == "short_press":
            self.get_logger().info("Button pressed - Starting race!")
            self.state_machine.transition_to(RobotState.RACING, StateTransitionReason.BUTTON_PRESSED)
            self.race_start_time = time.time()
            self.laps_completed = 0
            # Zeroing the local copy is not enough: track_navigator_node
            # publishes its lap count every control tick regardless of state
            # (see its _control_loop), so the previous race's final count is
            # still arriving and would overwrite this within one tick.
            self._lap_count_is_current = False

        elif current_state == RobotState.RACING and event == "long_press":
            self.get_logger().warning("EMERGENCY STOP activated!")
            self.state_machine.transition_to(RobotState.FINISHED, StateTransitionReason.EMERGENCY_STOP)
            self._publish_stop_command()

        elif current_state == RobotState.FINISHED and event == "long_press":
            # The only way out of FINISHED. SYSTEM_RESET was the sole
            # transition defined from it and nothing in the codebase emitted
            # it, so finishing a race or hitting the E-STOP left the robot
            # unrecoverable from its own controls -- between rounds that meant
            # an SSH session or a power cycle.
            #
            # A hold rather than a tap, deliberately: the button is the only
            # control the operator has, and a knock against the chassis after a
            # round should not restart the boot sequence.
            self.get_logger().info("Reset requested - returning to BOOT_CHECK")
            self.state_machine.transition_to(RobotState.BOOT_CHECK, StateTransitionReason.SYSTEM_RESET)
            self.reset()

        elif event == "shutdown_press":
            # Accepted from any state on purpose. This escalates a hold rather
            # than competing with it: the robot already stopped when the same
            # press crossed the long-press threshold seven seconds earlier, so
            # by the time this arrives it is standing still whatever it was
            # doing. Refusing it while RACING would only mean refusing it right
            # after an E-STOP, which is exactly when an operator wants to pack
            # up and carry the robot away.
            self.get_logger().warning("Shutdown hold - powering both boards down cleanly")
            self._publish_stop_command()
            self._trigger_clean_shutdown()

    def _trigger_clean_shutdown(self) -> None:
        """Run safe-shutdown-both.sh detached from this process.

        systemd-run rather than a plain subprocess: the script shuts the Zero
        down first and this node last, and a child of this node would be killed
        the moment its own shutdown began -- leaving the Pi 5 running with no
        route left to the board it just powered off. A transient unit outlives
        the process that asked for it.

        Failure here is logged, never raised: the button handler must not take
        the state machine down with it, and an operator who gets no shutdown
        still has the same options they had before this existed.
        """
        script = SAFE_SHUTDOWN_BOTH_SCRIPT
        if not script.is_file():
            self.get_logger().error(f"Cannot shut down: {script} not found")
            return
        try:
            subprocess.run(  # noqa: S603 - fixed argv, no shell, no user input
                [  # noqa: S607
                    "sudo",
                    "systemd-run",
                    "--unit=vtitan-button-shutdown",
                    "--collect",
                    # As the invoking user, with their HOME. A transient unit
                    # defaults to root, and the script's first act is to SSH
                    # into the Zero -- as root that finds no key and no
                    # ~/.ssh/config, the preflight fails, and `set -e` aborts
                    # before anything is powered off. Observed exactly that:
                    # the unit exited 1 while this node had already reported
                    # success.
                    f"--uid={os.getuid()}",
                    f"--setenv=HOME={Path.home()}",
                    "bash",
                    str(script),
                ],
                check=True,
                capture_output=True,
                timeout=15,
            )
            # Deliberately "requested", not "started": all this call proves is
            # that systemd accepted the unit. Whether the shutdown actually
            # runs shows up in that unit's own journal, not here -- claiming
            # more than that is how the earlier failure went unnoticed.
            self.get_logger().info(
                "Clean shutdown requested - watch `journalctl -u vtitan-button-shutdown` if the boards stay up",
            )
        except (subprocess.SubprocessError, OSError) as exc:
            self.get_logger().error(f"Clean shutdown could not be requested: {exc}")

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

    @override
    def reset(self) -> None:
        """Clear per-round state so BOOT_CHECK starts genuinely fresh.

        The challenge mode is cleared too, not just the lap counters: between
        rounds the operator may move the jumper, and ``_sample_challenge_mode``
        returns early once it is latched. Leaving it set would silently carry
        the previous round's challenge -- and its lap count -- into the next
        one. The jumper topic is latched, so re-sampling costs nothing.
        """
        self.race_start_time = None
        self.laps_completed = 0
        # Re-armed rather than cleared: BOOT_CHECK is reached from FINISHED,
        # so the navigator is still publishing the finished race's count and
        # the gate has to survive until RACING re-arms it properly.
        self._lap_count_is_current = False
        self.current_velocity = 0.0
        self.current_steering = 0.0
        self.challenge_mode = None
        self._challenge_mode_samples.clear()
        self._challenge_mode_error = None

    def _sample_challenge_mode(self) -> None:
        """Sample the jumper state published by the Pi Zero; stop once it stabilizes.

        The jumper is wired to the ZERO's GPIO23, so this consumes
        ``/challenge_mode/jumper_inserted`` rather than reading GPIO locally.
        Reading it here used to mean reading *Pi 5's* GPIO23, which nothing is
        attached to -- with the internal pull-up that always reads HIGH, so it
        silently latched Open Challenge on every boot and Obstacle could never
        be selected.

        Disagreeing samples (bounce/intermittent contact) clear progress and
        keep BOOT_CHECK waiting rather than guessing a mode -- unless that
        bounce never settles, in which case ``_challenge_mode_timed_out``
        falls back to Open the same way it does when the Zero never
        publishes at all. A floating (open) pin is weakly pulled up and far
        more noise-susceptible than a solid short to GND, so persistent
        bounce shows up specifically when the jumper is absent -- without
        this fallback BOOT_CHECK could wait on 3 consecutive agreeing
        samples forever and the robot would never reach READY.
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
            if self._challenge_mode_timed_out():
                self._latch_challenge_mode(inserted=False, reason="jumper reading never stabilized (bounce/noise)")
            return

        self._latch_challenge_mode(inserted=inserted)

    def _challenge_mode_timed_out(self) -> bool:
        """True once we've waited long enough for the Zero to publish the jumper state."""
        waited = self.get_clock().now().nanoseconds / 1e9 - self._challenge_mode_wait_started
        return waited >= _CHALLENGE_MODE_TIMEOUT_SEC

    def _latch_challenge_mode(self, *, inserted: bool, reason: str | None = None) -> None:
        """Fix the challenge mode for this run and derive the lap count."""
        self.challenge_mode = ScenarioType.OBSTACLES if inserted else ScenarioType.OPEN
        # Latched (TRANSIENT_LOCAL): track_navigator_node may subscribe before
        # or after this fires and either way must see the current value. Fires
        # on every resolution, detected or defaulted, and again after each
        # SYSTEM_RESET re-sample -- that's what lets the navigator pick up a
        # challenge switch made purely from the button/jumper.
        self.challenge_mode_pub.publish(String(data=self.challenge_mode.value))
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
        # Check if laps completed. Not before the count is known to belong to
        # this race: this runs on our own tick, which beats the round trip out
        # to the navigator and back, so an ungated check reads the previous
        # race's total (see _on_laps_completed).
        if self._lap_count_is_current and self.laps_completed >= self.target_laps:
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

            # Check the vision pipeline: detections arriving recently. Same
            # shape as the IMU and LiDAR tests above, and for the same reason --
            # a stream that has stopped is what "not ready" means here.
            hailo_ready = (
                self.hailo_last_msg_time is not None and (current_time - self.hailo_last_msg_time) < timeout
            )
            hailo_status = SensorStatus(
                name="Hailo",
                is_ready=hailo_ready,
                error_message=None if hailo_ready else "No vision detections received",
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

        # .is_ready off each status object, not the raw imu_ready/lidar_ready/
        # hailo_ready locals -- those are only assigned in the non-simulation
        # branch above, so referencing them here raised UnboundLocalError on
        # every is_simulation=True boot, and always had (nothing previously
        # exercised that path -- see rpi5_nodes.launch.py's is_simulation
        # bench-test argument, added specifically to exercise it).
        all_ready = (
            imu_status.is_ready
            and lidar_status.is_ready
            and (hailo_status.is_ready or not hailo_required)
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

        metrics = RaceStatus(
            laps_completed=self.laps_completed,
            total_race_time=elapsed_time,
            current_velocity=self.current_velocity,
            current_steering=self.current_steering,
            gyro_yaw=self.gyro_yaw,
            current_corridor=self.current_corridor,
        )

        msg = String()
        msg.data = RaceMetricsWire(
            laps_completed=metrics.laps_completed,
            # Published so consumers do not have to assume it. The OLED
            # rendered "Laps: n/3" as a literal, which is right only for as
            # long as both challenges require three laps.
            target_laps=self.target_laps,
            total_race_time=round(metrics.total_race_time, 2),
            current_velocity=round(metrics.current_velocity, 2),
            current_steering=round(metrics.current_steering, 2),
            gyro_yaw=round(metrics.gyro_yaw, 2),
            current_corridor=metrics.current_corridor or "",
        ).model_dump_json()
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
