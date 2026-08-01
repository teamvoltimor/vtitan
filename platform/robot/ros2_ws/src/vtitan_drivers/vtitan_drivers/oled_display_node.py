"""ROS2 lifecycle node for OLED display with async page cycling and image mirroring.

Run on: Raspberry Pi Zero

Hardware connects in on_configure() and page-update publishing starts in
on_activate(), matching the driver lifecycle pattern used by every hardware
node in this package (see button_node.py for the reference implementation).

Usage:
    ros2 run vtitan_drivers oled_display_node

Topics:
    Subscribed:
        - /robot_state (std_msgs/String) - Current robot state
        - /system_status (diagnostic_msgs/DiagnosticArray) - System diagnostics
        - /race_metrics (std_msgs/String) - Race metrics (JSON)
        - /ui/telemetry_summary (std_msgs/String) - Aggregated lidar/yaw/detection summary
    Published:
        - /ui/oled_mirror (sensor_msgs/Image) - Live mirror of OLED display
"""

from __future__ import annotations

import json
import time
from contextlib import suppress
from typing import TYPE_CHECKING, override

import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from PIL import Image, ImageDraw
from pydantic import AliasChoices, Field
from pydantic_settings import SettingsConfigDict
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Float32, String

from src.hardware.display.enums import DisplayBackend
from src.hardware.display.ssd1306 import (
    Driver as BlinkaDriver,
    RawI2CDriver,
)
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings
from src.state_machine import RobotState, ScenarioType

if TYPE_CHECKING:
    from rclpy.lifecycle.node import LifecycleState
    from rclpy.lifecycle.publisher import Publisher
    from rclpy.subscription import Subscription
    from rclpy.timer import Timer

    from src.hardware.display.base import Driver as DisplayDriver


NODE_NAME = "oled_display_node"
"""ROS2 node name for OLED display controller."""


class NodeConfig(HardwareBaseSettings):
    """Node-level timing and backend selection, configurable via config/hardware/display/oled_node.toml.

    Matches every hardware driver's Config pattern. Timing fields were previously hardcoded
    as plain module constants, silently ignoring
    .env.example's documented UI_REFRESH_RATE_HZ entirely --
    changing that value had zero effect.
    """

    model_config = SettingsConfigDict(env_prefix="", toml_file=CONFIG_DIR / "display" / "oled_node.toml")

    ui_refresh_rate_hz: float = Field(
        default=10.0, validation_alias=AliasChoices("UI_REFRESH_RATE_HZ", "ui_refresh_rate_hz")
    )
    """Rate for updating display data (fast updates)."""

    display_backend: DisplayBackend = Field(
        default=DisplayBackend.BLINKA, validation_alias=AliasChoices("DISPLAY_BACKEND", "display_backend")
    )
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

_GRID_TOP_Y = _BODY_TOP_Y
"""Top of the RACING page's grid -- starts right where a single-column body would."""

_GRID_ROW_H = 12
"""Vertical step between RACING grid rows. 4 rows (V/St, F/L, R/Yaw, Obj) at
this height span 48px, ending well inside the 64px-tall panel."""
"""Bottom rule of the grid -- also the vertical divider's lower endpoint."""

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
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
"""Matches state_machine_node's latched publishers.

The display is a late subscriber by nature -- it lives on the Pi Zero and is
restarted independently of the Pi 5 -- so it has to request the latched value
rather than wait for the next transition, which may be minutes away or may
already have happened. TRANSIENT_LOCAL durability covers that.

Reliability is BEST_EFFORT, not RELIABLE, on purpose (as of 2026-07-28): a
RELIABLE publisher blocks its own publish() call until this reader acks, and
this board was measured stalling for 30+ seconds under its own CPU/memory
contention -- which was blocking state_machine_node's /robot_state publish()
on the Pi 5 right along with it, the direct cause of the OLED being seen
stuck on a stale BOOT_CHECK page well after the robot had actually reached
READY. A RELIABLE publisher is required to match a RELIABLE subscriber
exactly (durability may only be offered >=, reliability must match), so this
side has to drop to BEST_EFFORT too, not just tolerate it.
"""

_QOS_UI_SUMMARY = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
"""Matches telemetry_bridge_node's publisher: BEST_EFFORT so a slow frame on
this board (I2C write stalls under CPU/memory contention -- see
telemetry_bridge_node.py's _QOS_UI_SUMMARY for the full story) can never
make that publisher's own .publish() call block and stall its whole
pipeline. A RELIABLE subscriber cannot receive from a BEST_EFFORT
publisher at all, so this side has to match, not just be compatible.
"""

_DEG_PER_REV = 360.0
"""/motor/drive_speed reports the wheel's angular speed in degrees/s (from real
encoder feedback, see dc_encoder/driver.py's get_drive_speed()) -- rev/s is
just that divided by 360, with no wheel-radius conversion (and its
measurement uncertainty) involved at all."""

_DISPLAY_DRIVER_BY_BACKEND = {
    DisplayBackend.BLINKA: BlinkaDriver,
    DisplayBackend.RAW_I2C: RawI2CDriver,
}


class OLEDDisplayNode(LifecycleNode):
    """ROS2 lifecycle node that manages the OLED display with state-based views.

    Responsibilities:
    - Display live hardware checklist during BOOT_CHECK
    - Display IP address and AI model during READY
    - Display speed, steering, lidar clearances, yaw, and (Obstacles only) the
      most salient detection all at once during RACING
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
        self.ui_summary_sub: Subscription | None = None
        # drive_speed/steering_position come from ackermann_motor_node, which
        # runs on this same board (the Zero) -- subscribed directly rather
        # than round-tripped through state_machine_node's /race_metrics on the
        # Pi 5, which never actually wires them up (race_metrics.current_velocity
        # /current_steering are hardcoded 0.0 there; nothing populates them).
        self.drive_speed_sub: Subscription | None = None
        self.steering_position_sub: Subscription | None = None
        self.ui_timer: Timer | None = None
        # The display-refresh timer's own group, separate from the default
        # group every subscription above uses. Each tick blocks on a real
        # I2C write (raw_i2c backend: 32 sequential blocking os.write()
        # calls per frame) that can run close to the timer's own period --
        # sharing one MutuallyExclusiveCallbackGroup meant that write could
        # starve every subscription callback indefinitely, since the group
        # won't service a second callback until the first returns. Confirmed
        # on hardware: /ui/telemetry_summary never got processed at all,
        # RACING page stuck at 0.0 for its full lifetime, no errors anywhere.
        self._timer_callback_group = MutuallyExclusiveCallbackGroup()

        # State tracking
        self.current_state: str = RobotState.BOOT_CHECK.value
        self.system_status: dict[str, dict] = {}
        self.race_metrics: dict = {}

        # Sensor data
        self.gyro_yaw: float = 0.0
        self.lidar_front: float = 0.0
        self.lidar_left: float = 0.0
        self.lidar_right: float = 0.0
        self.drive_speed_dps: float = 0.0
        self.steering_position_deg: float = 0.0
        self.best_detection: tuple[str, float] | None = None
        """(class_id, confidence) of the detection scoring highest on
        confidence x bbox area, or None with no current detections."""

        # TEMP DIAGNOSTIC (2026-07-28): see _ui_summary_callback.
        self._last_ui_summary_receive_time: float | None = None

        self._last_frame_bytes: bytes | None = None
        """Last frame actually written to the panel, so an unchanged render
        (a static page, or telemetry that hasn't moved between ticks) skips
        the I2C write and mirror publish instead of paying their cost every
        tick regardless of whether anything visible changed."""

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
        # Now safe to request latched here too: telemetry_bridge_node's
        # /system_status publisher was TRANSIENT_LOCAL-only VOLATILE, which is
        # exactly the same stale-display bug /robot_state hit above -- fixed
        # by making that publisher TRANSIENT_LOCAL as well (see
        # telemetry_bridge_node.py's _QOS_SYSTEM_STATUS), so both publishers
        # on this topic now durability-match a latched subscriber.
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            "/system_status",
            self._diagnostics_callback,
            _QOS_LATCHED,
        )
        self.metrics_sub = self.create_subscription(String, "/race_metrics", self._metrics_callback, 10)

        # Hold feedback. BEST_EFFORT depth 1 to match button_node: this is a
        # live readout, so a late frame is worthless and a queue of them worse.
        self._button_hold: dict[str, object] = {}
        self.button_hold_sub = self.create_subscription(
            String,
            "/button/hold",
            self._button_hold_callback,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT),
        )
        self.ui_summary_sub = self.create_subscription(
            String,
            "/ui/telemetry_summary",
            self._ui_summary_callback,
            _QOS_UI_SUMMARY,
        )
        # ackermann_motor_node runs on this same board -- default (reliable,
        # volatile) QoS matches its create_lifecycle_publisher(..., 10) calls.
        self.drive_speed_sub = self.create_subscription(
            Float32,
            "/motor/drive_speed",
            self._drive_speed_callback,
            10,
        )
        self.steering_position_sub = self.create_subscription(
            Float32,
            "/motor/steering_position",
            self._steering_position_callback,
            10,
        )

        return TransitionCallbackReturn.SUCCESS

    @override
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Start the display-update timer."""
        self.get_logger().info("Activating OLED Display Node")
        self.ui_timer = self.create_timer(
            1.0 / UI_REFRESH_RATE_HZ,
            self._update_display,
            callback_group=self._timer_callback_group,
        )
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
        for sub_attr in (
            "state_sub",
            "diagnostics_sub",
            "metrics_sub",
            "ui_summary_sub",
            "drive_speed_sub",
            "steering_position_sub",
        ):
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

    def _ui_summary_callback(self, msg: String) -> None:
        """Handle aggregated lidar/yaw/detection telemetry from telemetry_bridge_node.

        Replaces this node's old direct /scan, /imu/data and /hailo/detections
        subscriptions -- those crossed the USB-gadget link to the Pi Zero at
        full sensor rate to feed a display that only samples them a couple
        times a second, and were suspected (confirmed live on hardware) of
        starving on the Zero's single weak core.
        """
        # TEMP DIAGNOSTIC (2026-07-28): pin down whether reported 1-2s OLED
        # freezes originate in telemetry_bridge_node's publish cadence (its
        # own matching instrumentation would show that), in transit over the
        # USB-gadget link, or in this node's own render path (see
        # _update_display's timing below). Remove once root-caused.
        now_monotonic = time.monotonic()
        if self._last_ui_summary_receive_time is not None:
            gap = now_monotonic - self._last_ui_summary_receive_time
            if gap > 0.5:
                self.get_logger().warning(f"[DIAG] ui_summary receive gap: {gap:.2f}s (expected ~0.1s)")
        self._last_ui_summary_receive_time = now_monotonic

        with suppress(json.JSONDecodeError):
            data = json.loads(msg.data)
            self.lidar_front = data.get("lidar_front_cm", self.lidar_front)
            self.lidar_left = data.get("lidar_left_cm", self.lidar_left)
            self.lidar_right = data.get("lidar_right_cm", self.lidar_right)
            self.gyro_yaw = data.get("gyro_yaw_deg", self.gyro_yaw)
            class_id = data.get("best_detection_class_id")
            self.best_detection = (
                (class_id, data["best_detection_confidence"]) if class_id is not None else None
            )

    def _drive_speed_callback(self, msg: Float32) -> None:
        """Wheel angular speed in degrees/s, from ackermann_motor_node's real encoder feedback."""
        self.drive_speed_dps = msg.data

    def _steering_position_callback(self, msg: Float32) -> None:
        """Current steering angle in degrees, from ackermann_motor_node's real feedback."""
        self.steering_position_deg = msg.data

    def _update_display(self) -> None:
        """Update display based on current state."""
        if self.display_driver is None:
            return

        # A hold in progress outranks every state page. The operator is
        # actively holding the one control they have and needs to know what it
        # is about to do -- ten seconds with no feedback is long enough to doubt
        # the press registered and let go a second early.
        held_sec = float(self._button_hold.get("held_sec", 0.0) or 0.0)
        if held_sec > 0.0:
            image = self._render_button_hold(held_sec)
        elif self.current_state == RobotState.BOOT_CHECK.value:
            image = self._render_boot_check()
        elif self.current_state == RobotState.READY.value:
            image = self._render_ready()
        elif self.current_state == RobotState.RACING.value:
            image = self._render_racing()
        elif self.current_state == RobotState.FINISHED.value:
            image = self._render_finished()
        else:
            image = self.display_driver.get_blank_image()

        frame_bytes = image.tobytes()
        if frame_bytes == self._last_frame_bytes:
            return
        self._last_frame_bytes = frame_bytes

        # Display on OLED
        # TEMP DIAGNOSTIC (2026-07-28): see _ui_summary_callback -- times the
        # actual I2C write (raw_i2c backend: 32 sequential blocking
        # os.write() calls per frame, previously measured close to this
        # timer's own ~100ms period). Remove once root-caused.
        write_start = time.monotonic()
        self.display_driver.show_image(image)
        write_duration = time.monotonic() - write_start
        if write_duration > 0.3:
            self.get_logger().warning(f"[DIAG] show_image took {write_duration:.2f}s (expected ~0.1s)")

        # Publish mirror for remote viewing
        self._publish_mirror_image(image)

    def _button_hold_callback(self, msg: String) -> None:
        """Track how long the button has been held, and what that will trigger."""
        try:
            self._button_hold = json.loads(msg.data)
        except (ValueError, TypeError):
            # A malformed frame must not blank the display mid-hold; keep the
            # last good one and let the next 50ms frame correct it.
            self.get_logger().warning("Ignoring malformed /button/hold payload", throttle_duration_sec=5.0)

    def _render_button_hold(self, held_sec: float) -> Image.Image:
        """Render the hold counter, with what the next threshold will do.

        The wording comes from button_node rather than from here, so the
        thresholds live in exactly one place -- the button driver's config --
        rather than drifting between a TOML on this board and a render function
        that nobody thinks to update alongside it.
        """
        assert self.display_driver is not None
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)

        draw.text((_MARGIN_X, _TITLE_Y), "HOLDING", fill=_ON)
        draw.line([(_MARGIN_X, _SEPARATOR_Y), (self.display_driver.get_width(), _SEPARATOR_Y)], fill=_ON, width=1)

        draw.text((_MARGIN_X, 20), f"{held_sec:.1f}s", fill=_ON)

        next_action = self._button_hold.get("next")
        next_at = self._button_hold.get("next_at_sec")
        if next_action and next_at:
            remaining = max(0.0, float(next_at) - held_sec)  # type: ignore[arg-type]
            draw.text((_MARGIN_X, 36), f"{next_action} in {remaining:.1f}s", fill=_ON)
            draw.text((_MARGIN_X, 50), "release to cancel", fill=_ON)
        else:
            # Past the last threshold: nothing further to warn about, and a
            # countdown to nothing would be a lie.
            draw.text((_MARGIN_X, 36), "POWERING OFF", fill=_ON)

        return image

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

    def _render_racing(self) -> Image.Image:
        """Render the single consolidated RACING page as a grid.

        A fixed 4-row x 2-col grid of column-aligned values instead of one
        "label:value label:value" text line per pair -- stacked text lines
        left every value competing for the same 128px of horizontal space,
        so labels and numbers ran together at this font size. Splitting
        into columns gives each value its own space. No ruled lines: column
        alignment alone reads as a grid without adding visual clutter on
        this small a panel.

        Left column: Front, Left, Right clearance, then lap count. Right
        column: speed, steering, yaw, then the current vision detection
        (Obstacles Challenge only) -- left is "what's around the robot",
        right is "what the robot is doing".

        IP/mode are READY-only by design: they're only useful before the
        round starts.
        """
        assert self.display_driver is not None
        image = self.display_driver.get_blank_image()
        draw = ImageDraw.Draw(image)
        width = self.display_driver.get_width()

        draw.text((_MARGIN_X, _TITLE_Y), "RACING", fill=_ON)
        draw.line([(_MARGIN_X, _SEPARATOR_Y), (width, _SEPARATOR_Y)], fill=_ON, width=1)

        rev_per_s = self.drive_speed_dps / _DEG_PER_REV
        laps = self.race_metrics.get("laps_completed", 0)
        target_laps = self.race_metrics.get("target_laps", _DEFAULT_TARGET_LAPS)
        cells = (
            (f"F:{self.lidar_front:.0f}cm", f"V:{rev_per_s:.1f}"),
            (f"L:{self.lidar_left:.0f}cm", f"St:{self.steering_position_deg:+.0f}"),
            (f"R:{self.lidar_right:.0f}cm", f"Yaw:{self.gyro_yaw:+.0f}"),
            (f"Laps:{laps}/{target_laps}", ""),
        )
        col_x = (_MARGIN_X + 2, width // 2 + 4)

        for row_index, (left, right) in enumerate(cells):
            row_y = _GRID_TOP_Y + row_index * _GRID_ROW_H
            draw.text((col_x[0], row_y), left, fill=_ON)
            if right:
                draw.text((col_x[1], row_y), right, fill=_ON)

        # Detection is Obstacles-only: the Open Challenge never runs vision, so
        # showing a stale/empty detection line there would be noise, not signal.
        # Shares the Laps row's right column rather than adding a 5th row --
        # the page's height stays fixed at 4 rows either way.
        if self._is_obstacles_challenge() and self.best_detection is not None:
            class_id, confidence = self.best_detection
            laps_row_y = _GRID_TOP_Y + (len(cells) - 1) * _GRID_ROW_H
            draw.text((col_x[1], laps_row_y), f"{class_id} {confidence:.2f}", fill=_ON)

        return image

    def _is_obstacles_challenge(self) -> bool:
        """True once BOOT_CHECK has latched the Obstacles Challenge.

        Reads the same ChallengeMode diagnostic message the BOOT_CHECK page
        already renders (state_machine_node sets it to the ScenarioType value,
        upper-cased, once the jumper reading is stable) rather than a second,
        possibly-diverging source of truth.
        """
        message = self.system_status.get("ChallengeMode", {}).get("message", "")
        return bool(message == ScenarioType.OBSTACLES.value.upper())

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
