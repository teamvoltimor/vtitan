"""Mock-hardware tests for button_node — the real node deployed on the

Raspberry Pi Zero 2W. No real GPIO is touched: the button driver is mocked so
the test exercises the node's poll-and-publish logic against fake button state.
"""

from __future__ import annotations

from unittest import mock

import pytest
import rclpy

from src.hardware.button.event import ButtonEvent
from src.hardware.button.state import ButtonState


@pytest.fixture()
def ros_context():
    """Initialize and cleanup ROS2 context for each test."""
    try:
        rclpy.init()
        yield
        rclpy.shutdown()
    except Exception as e:  # noqa: BLE001
        pytest.skip(f"ROS2 initialization failed: {e}")


@pytest.fixture()
def button_node_class():
    """Import ButtonNode with a mocked GPIO driver."""
    mock_driver = mock.MagicMock()

    with mock.patch("voldemorbot_robot.button_node.ButtonDriver", return_value=mock_driver):
        from voldemorbot_robot.button_node import ButtonNode

        yield ButtonNode, mock_driver


class TestButtonNodeInit:
    def test_connects_driver(self, ros_context, button_node_class):
        ButtonNode, mock_driver = button_node_class
        node = ButtonNode()

        mock_driver.connect.assert_called_once()
        assert node.driver is mock_driver

        node.destroy_node()

    def test_publishes_on_button_event_topic(self, ros_context, button_node_class):
        ButtonNode, _ = button_node_class
        node = ButtonNode()

        topics = dict(node.get_publisher_names_and_types_by_node(node.get_name(), ""))
        assert "/button/event" in topics
        assert topics["/button/event"] == ["std_msgs/msg/String"]

        node.destroy_node()

    def test_driver_connect_failure_degrades_safely(self, ros_context):
        mock_driver = mock.MagicMock()
        mock_driver.connect.side_effect = RuntimeError("gpio busy")

        with mock.patch("voldemorbot_robot.button_node.ButtonDriver", return_value=mock_driver):
            from voldemorbot_robot.button_node import ButtonNode

            node = ButtonNode()

        assert node.driver is None
        node._poll()  # must not raise with no driver
        node.destroy_node()


class TestButtonNodePolling:
    """Pin the wire contract button_node -> state_machine_node relies on."""

    def test_short_press_publishes_short_press_string(self, ros_context, button_node_class):
        ButtonNode, mock_driver = button_node_class
        node = ButtonNode()

        mock_driver.get_state.return_value = ButtonState(
            is_pressed=False, press_duration=0.0, last_event=ButtonEvent.SHORT_PRESS,
        )
        published = []
        node.pub.publish = published.append

        node._poll()

        assert len(published) == 1
        assert published[0].data == "short_press"

        node.destroy_node()

    def test_long_press_publishes_long_press_string(self, ros_context, button_node_class):
        ButtonNode, mock_driver = button_node_class
        node = ButtonNode()

        mock_driver.get_state.return_value = ButtonState(
            is_pressed=True, press_duration=2.5, last_event=ButtonEvent.LONG_PRESS,
        )
        published = []
        node.pub.publish = published.append

        node._poll()

        assert len(published) == 1
        assert published[0].data == "long_press"

        node.destroy_node()

    def test_no_event_publishes_nothing(self, ros_context, button_node_class):
        ButtonNode, mock_driver = button_node_class
        node = ButtonNode()

        mock_driver.get_state.return_value = ButtonState(is_pressed=False, press_duration=0.0, last_event=None)
        published = []
        node.pub.publish = published.append

        node._poll()

        assert published == []

        node.destroy_node()


class TestButtonNodeCleanup:
    def test_destroy_closes_driver(self, ros_context, button_node_class):
        ButtonNode, mock_driver = button_node_class
        node = ButtonNode()

        node.destroy_node()

        mock_driver.close.assert_called_once()
