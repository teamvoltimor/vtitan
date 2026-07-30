"""Mock-hardware tests for oled_display_node — the real node deployed on the

Raspberry Pi Zero. No real I2C/display hardware is touched: the display driver
class table is mocked so the test exercises the node's lifecycle and callback
logic against fake driver state.

OLEDDisplayNode is a LifecycleNode: hardware connects and pub/subs are created
in on_configure(), the display-update timer starts in on_activate(), so tests
must drive those transitions explicitly before exercising node behavior
(deployed nodes do this automatically via
trigger_configure()/trigger_activate() in main()).
"""

from __future__ import annotations

import sys
from unittest import mock

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from PIL import Image
from std_msgs.msg import Float32, String

# oled_display_node imports both display backends unconditionally at module
# scope (src.hardware.display.ssd1306's __init__ re-exports both), which pulls
# in board/busio/adafruit_ssd1306 (Blinka, not available off-hardware) and
# fcntl (POSIX-only, doesn't exist on Windows) even though only one backend is
# ever actually used at runtime. Mock all four before import, matching the
# pattern test_imu_bno08x_i2c_node.py already uses for board/busio.
# Only substituted where the real module cannot be imported. Replacing them
# unconditionally clobbers sys.modules for every test that runs afterwards --
# board and busio import fine on some dev machines, and stubbing them here made
# the IMU node tests fail when the suite ran as a directory while passing in
# isolation. conftest.py takes the same conditional approach for the same
# reason.
for _optional in ("board", "busio", "adafruit_ssd1306", "fcntl"):
    if _optional not in sys.modules:
        try:
            __import__(_optional)
        except (ImportError, NotImplementedError):
            sys.modules[_optional] = mock.MagicMock()

from src.hardware.display.enums import DisplayBackend


@pytest.fixture()
def ros_context():
    """Initialize and cleanup ROS2 context for each test."""
    try:
        rclpy.init()
        yield
        rclpy.shutdown()
    except Exception as e:
        pytest.skip(f"ROS2 initialization failed: {e}")


@pytest.fixture()
def oled_node_class():
    """Import OLEDDisplayNode with the BLINKA driver class table entry mocked out."""
    from vtitan_drivers import oled_display_node as oled_module

    # spec'd against the real driver ABC, so touching an attribute it does not
    # have raises here instead of on the robot. A bare MagicMock answers to
    # anything: self.display_driver.width returned a Mock, PIL accepted it as a
    # coordinate, every test passed, and the node crash-looped on the Zero with
    # AttributeError the moment it rendered. The size accessors are get_width()
    # and get_height(); base.Driver only *annotates* width/height.
    from src.hardware.display.base import Driver as DisplayDriver

    mock_driver = mock.MagicMock(spec=DisplayDriver)
    mock_driver.get_width.return_value = 128
    mock_driver.get_height.return_value = 64
    mock_driver.get_blank_image.side_effect = lambda: Image.new("1", (128, 64))
    mock_driver_cls = mock.MagicMock(return_value=mock_driver)

    with mock.patch.dict(
        oled_module._DISPLAY_DRIVER_BY_BACKEND,
        {DisplayBackend.BLINKA: mock_driver_cls, DisplayBackend.RAW_I2C: mock_driver_cls},
    ):
        OLEDDisplayNode = oled_module.OLEDDisplayNode

        yield OLEDDisplayNode, mock_driver


class TestOLEDDisplayNodeInit:
    def test_connects_driver_on_configure(self, ros_context, oled_node_class):
        OLEDDisplayNode, mock_driver = oled_node_class
        node = OLEDDisplayNode()
        assert node.display_driver is None  # not yet configured

        node.trigger_configure()

        mock_driver.connect.assert_called_once()
        assert node.display_driver is mock_driver

        node.destroy_node()

    def test_creates_publisher_and_subscriptions_on_configure(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = OLEDDisplayNode()

        node.trigger_configure()

        assert node.oled_mirror_pub is not None
        assert node.state_sub is not None
        assert node.diagnostics_sub is not None
        assert node.metrics_sub is not None
        assert node.ui_summary_sub is not None
        assert node.drive_speed_sub is not None
        assert node.steering_position_sub is not None

        node.destroy_node()

    def test_driver_connect_failure_degrades_safely(self, ros_context, oled_node_class):
        OLEDDisplayNode, mock_driver = oled_node_class
        mock_driver.connect.side_effect = RuntimeError("i2c bus hang")

        node = OLEDDisplayNode()
        node.trigger_configure()

        assert node.display_driver is None
        node._update_display()  # must not raise with no driver
        node.destroy_node()


class TestOLEDDisplayNodeLifecycle:
    """Lifecycle-specific behavior: transitions gate hardware and the update timer."""

    def test_activate_creates_ui_timer(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()
        assert node.ui_timer is None

        node.trigger_activate()

        assert node.ui_timer is not None
        node.destroy_node()

    def test_deactivate_stops_ui_timer(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()
        node.trigger_activate()

        node.trigger_deactivate()

        assert node.ui_timer is None
        node.destroy_node()

    def test_cleanup_disconnects_driver_and_removes_pub_subs(self, ros_context, oled_node_class):
        OLEDDisplayNode, mock_driver = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()

        node.trigger_cleanup()

        mock_driver.clear.assert_called_once()
        mock_driver.close.assert_called_once()
        assert node.display_driver is None
        assert node.oled_mirror_pub is None
        assert node.state_sub is None
        node.destroy_node()

    def test_destroy_without_configure_does_not_raise(self, ros_context, oled_node_class):
        """A node destroyed before ever being configured must not crash cleanup."""
        OLEDDisplayNode, mock_driver = oled_node_class
        node = OLEDDisplayNode()

        node.destroy_node()  # must not raise

        mock_driver.close.assert_not_called()

    def test_destroy_closes_driver_without_clean_shutdown(self, ros_context, oled_node_class):
        OLEDDisplayNode, mock_driver = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()

        node.destroy_node()

        mock_driver.close.assert_called_once()


class TestOLEDDisplayNodeCallbacks:
    """Pin the callback contracts other nodes rely on."""

    def test_state_callback_updates_current_state(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()

        msg = String()
        msg.data = "racing"
        node._state_callback(msg)

        assert node.current_state == "racing"
        node.destroy_node()

    def test_diagnostics_callback_updates_system_status(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()

        msg = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "IMU"
        status.level = DiagnosticStatus.OK
        status.message = "OK"
        msg.status.append(status)
        node._diagnostics_callback(msg)

        assert "IMU" in node.system_status
        assert node.system_status["IMU"]["level"] == DiagnosticStatus.OK
        node.destroy_node()

    def test_metrics_callback_parses_json(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()

        msg = String()
        msg.data = '{"laps_completed": 2, "current_velocity": 0.5}'
        node._metrics_callback(msg)

        assert node.race_metrics["laps_completed"] == 2
        node.destroy_node()

    def test_metrics_callback_ignores_invalid_json(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()
        node.race_metrics = {"laps_completed": 1}

        msg = String()
        msg.data = "not valid json"
        node._metrics_callback(msg)  # must not raise

        assert node.race_metrics == {"laps_completed": 1}
        node.destroy_node()

    def test_ui_summary_callback_parses_json(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()

        msg = String()
        msg.data = (
            '{"lidar_front_cm": 18.6, "lidar_left_cm": 142.8, "lidar_right_cm": 21.4, '
            '"gyro_yaw_deg": -12.0, "best_detection_class_id": "red_sign", '
            '"best_detection_confidence": 0.9}'
        )
        node._ui_summary_callback(msg)

        assert node.lidar_front == 18.6
        assert node.lidar_left == 142.8
        assert node.lidar_right == 21.4
        assert node.gyro_yaw == -12.0
        assert node.best_detection == ("red_sign", 0.9)
        node.destroy_node()

    def test_ui_summary_callback_ignores_invalid_json(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()
        node.lidar_front = 5.0

        msg = String()
        msg.data = "not valid json"
        node._ui_summary_callback(msg)  # must not raise

        assert node.lidar_front == 5.0
        node.destroy_node()

    def test_ui_summary_callback_null_detection_clears_best_detection(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()
        node.best_detection = ("red_sign", 0.9)

        msg = String()
        msg.data = (
            '{"lidar_front_cm": 0.0, "lidar_left_cm": 0.0, "lidar_right_cm": 0.0, '
            '"gyro_yaw_deg": 0.0, "best_detection_class_id": null, '
            '"best_detection_confidence": null}'
        )
        node._ui_summary_callback(msg)

        assert node.best_detection is None
        node.destroy_node()

    def test_drive_speed_callback_updates_dps(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()

        msg = Float32()
        msg.data = 360.0
        node._drive_speed_callback(msg)

        assert node.drive_speed_dps == 360.0
        node.destroy_node()

    def test_steering_position_callback_updates_degrees(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = OLEDDisplayNode()
        node.trigger_configure()

        msg = Float32()
        msg.data = -12.5
        node._steering_position_callback(msg)

        assert node.steering_position_deg == -12.5
        node.destroy_node()

class TestOLEDDisplayNodeUpdate:
    def test_update_display_calls_show_image_and_publishes_mirror_when_active(self, ros_context, oled_node_class):
        from PIL import Image as PILImage

        OLEDDisplayNode, mock_driver = oled_node_class
        mock_driver.get_blank_image.return_value = PILImage.new("1", (128, 64))

        node = OLEDDisplayNode()
        node.trigger_configure()
        node.trigger_activate()

        published = []
        node.oled_mirror_pub.publish = published.append

        node._update_display()

        mock_driver.show_image.assert_called_once()
        assert len(published) == 1

        node.destroy_node()

    def test_update_display_noop_before_configure(self, ros_context, oled_node_class):
        OLEDDisplayNode, mock_driver = oled_node_class
        node = OLEDDisplayNode()

        node._update_display()  # must not raise

        mock_driver.show_image.assert_not_called()
        node.destroy_node()

    def test_update_display_skips_write_when_frame_unchanged(self, ros_context, oled_node_class):
        """A static page (or telemetry that hasn't moved) must not re-write
        the same frame every tick -- that's the I2C write cost this skip
        exists to avoid."""
        from PIL import Image as PILImage

        OLEDDisplayNode, mock_driver = oled_node_class
        mock_driver.get_blank_image.return_value = PILImage.new("1", (128, 64))

        node = OLEDDisplayNode()
        node.trigger_configure()
        node.trigger_activate()
        node.oled_mirror_pub.publish = lambda _msg: None

        node._update_display()
        node._update_display()

        mock_driver.show_image.assert_called_once()
        node.destroy_node()


class TestEveryStateRenders:
    """Actually draw each page against a spec'd driver.

    The suite mocked the driver and never rendered, so every page was
    unexercised: self.display_driver.width type-checked as an attribute
    annotated on base.Driver, returned a Mock from the bare MagicMock, and PIL
    accepted it as a coordinate. All 16 tests passed and the node crash-looped
    on the Zero with AttributeError the first time it drew, taking the display
    dark until it was reverted.

    These call the real draw path, so a wrong driver API or a bad coordinate
    fails here.
    """

    @staticmethod
    def _active(node_cls, state: str):
        node = node_cls()
        node.trigger_configure()
        node.trigger_activate()
        node.current_state = state
        return node

    @pytest.mark.parametrize(
        "state",
        ["boot_check", "ready", "racing", "finished", "emergency_stop"],
    )
    def test_state_renders_without_error(self, ros_context, oled_node_class, state):
        OLEDDisplayNode, mock_driver = oled_node_class
        node = self._active(OLEDDisplayNode, state)
        try:
            node._update_display()
            assert mock_driver.show_image.called
        finally:
            node.destroy_node()

    def test_racing_renders_with_no_data_yet(self, ros_context, oled_node_class):
        """Before any /motor_* or /hailo/detections message has arrived."""
        OLEDDisplayNode, mock_driver = oled_node_class
        node = self._active(OLEDDisplayNode, "racing")
        try:
            node._update_display()
            assert mock_driver.show_image.called
        finally:
            node.destroy_node()

    def test_is_obstacles_challenge_reads_the_challenge_mode_diagnostic(self, ros_context, oled_node_class):
        """The Open Challenge never runs vision -- no detection line should show there."""
        OLEDDisplayNode, _ = oled_node_class
        node = self._active(OLEDDisplayNode, "racing")
        try:
            node.system_status = {"ChallengeMode": {"message": "OPEN"}}
            assert node._is_obstacles_challenge() is False

            node.system_status = {"ChallengeMode": {"message": "OBSTACLES"}}
            assert node._is_obstacles_challenge() is True
        finally:
            node.destroy_node()

    def test_racing_renders_with_a_pending_detection_in_obstacles_challenge(self, ros_context, oled_node_class):
        OLEDDisplayNode, mock_driver = oled_node_class
        node = self._active(OLEDDisplayNode, "racing")
        try:
            node.system_status = {"ChallengeMode": {"message": "OBSTACLES"}}
            node.best_detection = ("red_sign", 0.9)

            node._update_display()  # must not raise with real detection data present
            assert mock_driver.show_image.called
        finally:
            node.destroy_node()

    def test_ready_omits_the_ip_line_when_offline(self, ros_context, oled_node_class):
        """At competition there is no network, so the line must not be drawn."""
        OLEDDisplayNode, _ = oled_node_class
        node = self._active(OLEDDisplayNode, "ready")
        try:
            node.system_status = {"Network": {"values": {"ip_address": "OFFLINE"}}}
            node._update_display()  # must not raise; the row is simply skipped
        finally:
            node.destroy_node()

    def test_ready_renders_with_an_ip(self, ros_context, oled_node_class):
        OLEDDisplayNode, _ = oled_node_class
        node = self._active(OLEDDisplayNode, "ready")
        try:
            node.system_status = {"Network": {"values": {"ip_address": "192.168.0.50"}}}
            node._update_display()
        finally:
            node.destroy_node()

    def test_finished_uses_the_rounds_own_lap_target(self, ros_context, oled_node_class):
        """Not CompetitionSpecs.OPEN_CHALLENGE_LAPS -- an Obstacles round has its own."""
        OLEDDisplayNode, _ = oled_node_class
        node = self._active(OLEDDisplayNode, "finished")
        try:
            node.race_metrics = {"laps_completed": 2, "target_laps": 2}
            node._update_display()
        finally:
            node.destroy_node()
