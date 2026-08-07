"""Mock-hardware tests for button_node — the real node deployed on the

Raspberry Pi Zero 2W. No real GPIO is touched: the button driver is mocked so
the test exercises the node's poll-and-publish logic against fake button state.

button_node is a LifecycleNode: hardware connects in on_configure() and
polling starts in on_activate(), so tests must drive those transitions
explicitly before exercising node behavior (deployed nodes do this
automatically via trigger_configure()/trigger_activate() in main()).
"""

from __future__ import annotations

from unittest import mock

import pytest
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn

from src.hardware.button import base as button_base
from src.hardware.button.event import ButtonEvent
from src.hardware.button.state import ButtonState


def _mock_driver() -> mock.MagicMock:
    """A driver mock spec'd against the real interface.

    Spec'd, not bare: a bare MagicMock answers to any attribute, so a node that
    called a method the driver does not have would still pass here and only fail
    on the robot -- exactly how the OLED node's `.width`-vs-`get_width()` slip
    survived a green suite.
    """
    return mock.MagicMock(spec=button_base.Driver)


@pytest.fixture()
def button_node_class():
    """Import ButtonNode with a mocked GPIO driver."""
    mock_driver = _mock_driver()

    with mock.patch("vtitan_drivers.button_node.ButtonDriver", return_value=mock_driver):
        from vtitan_drivers.button_node import ButtonNode

        yield ButtonNode, mock_driver


class TestButtonNodeInit:
    def test_connects_driver_on_configure(self, ros_context, button_node_class):
        ButtonNode, mock_driver = button_node_class
        node = ButtonNode()
        assert node.driver is None  # not yet configured

        node.trigger_configure()

        mock_driver.connect.assert_called_once()
        assert node.driver is mock_driver

        node.destroy_node()

    def test_publishes_on_button_event_topic_after_activate(self, ros_context, button_node_class):
        """The lifecycle publisher isn't advertised on the graph until the node activates."""
        ButtonNode, _ = button_node_class
        node = ButtonNode()
        node.trigger_configure()
        node.trigger_activate()

        topics = dict(node.get_publisher_names_and_types_by_node(node.get_name(), ""))
        assert "/button/event" in topics
        assert topics["/button/event"] == ["std_msgs/msg/String"]

        node.destroy_node()

    def test_driver_connect_failure_degrades_safely(self, ros_context):
        mock_driver = _mock_driver()
        mock_driver.connect.side_effect = RuntimeError("gpio busy")

        with mock.patch("vtitan_drivers.button_node.ButtonDriver", return_value=mock_driver):
            from vtitan_drivers.button_node import ButtonNode

            node = ButtonNode()
            node.trigger_configure()

        assert node.driver is None
        node._poll()  # must not raise with no driver
        node.destroy_node()

    def test_dead_driver_keeps_reporting_the_fault_on_every_poll(self, ros_context):
        """A silent dead button is the failure mode that costs a round.

        Without this the node sits ACTIVE, publishes nothing, and the only trace
        is one startup line -- so the operator presses, nothing happens, and the
        log at that moment looks clean.
        """
        mock_driver = _mock_driver()
        mock_driver.connect.side_effect = RuntimeError("gpio busy")

        with mock.patch("vtitan_drivers.button_node.ButtonDriver", return_value=mock_driver):
            from vtitan_drivers.button_node import ButtonNode

            node = ButtonNode()
            node.trigger_configure()

        with mock.patch.object(node, "get_logger") as get_logger:
            node._poll()

        get_logger.return_value.error.assert_called_once()
        message = get_logger.return_value.error.call_args.args[0]
        assert "no button events" in message
        # The cause travels with it: a missing BUTTON_GPIO_PIN and a wiring
        # fault are indistinguishable without it, and both reach here.
        assert "gpio busy" in message

        node.destroy_node()


class TestButtonNodePolling:
    """Pin the wire contract button_node -> state_machine_node relies on."""

    def test_short_press_publishes_short_press_string(self, ros_context, button_node_class):
        ButtonNode, mock_driver = button_node_class
        node = ButtonNode()
        node.trigger_configure()
        node.trigger_activate()

        mock_driver.get_state.return_value = ButtonState(
            is_pressed=False,
            press_duration=0.0,
            last_event=ButtonEvent.SHORT_PRESS,
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
        node.trigger_configure()
        node.trigger_activate()

        mock_driver.get_state.return_value = ButtonState(
            is_pressed=True,
            press_duration=2.5,
            last_event=ButtonEvent.LONG_PRESS,
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
        node.trigger_configure()
        node.trigger_activate()

        mock_driver.get_state.return_value = ButtonState(is_pressed=False, press_duration=0.0, last_event=None)
        published = []
        node.pub.publish = published.append

        node._poll()

        assert published == []

        node.destroy_node()


class TestButtonNodeLifecycle:
    """Lifecycle-specific behavior: transitions gate hardware and polling."""

    def test_configure_returns_success(self, ros_context, button_node_class):
        ButtonNode, _ = button_node_class
        node = ButtonNode()

        result = node.trigger_configure()

        assert result == TransitionCallbackReturn.SUCCESS
        node.destroy_node()

    def test_activate_creates_poll_timer(self, ros_context, button_node_class):
        ButtonNode, _ = button_node_class
        node = ButtonNode()
        node.trigger_configure()
        assert node.timer is None

        node.trigger_activate()

        assert node.timer is not None
        node.destroy_node()

    def test_deactivate_stops_poll_timer(self, ros_context, button_node_class):
        ButtonNode, _ = button_node_class
        node = ButtonNode()
        node.trigger_configure()
        node.trigger_activate()

        node.trigger_deactivate()

        assert node.timer is None
        node.destroy_node()

    def test_cleanup_disconnects_driver_and_removes_publisher(self, ros_context, button_node_class):
        ButtonNode, mock_driver = button_node_class
        node = ButtonNode()
        node.trigger_configure()

        node.trigger_cleanup()

        mock_driver.close.assert_called_once()
        assert node.driver is None
        assert node.pub is None
        node.destroy_node()


class TestButtonNodeCleanup:
    def test_destroy_closes_driver(self, ros_context, button_node_class):
        ButtonNode, mock_driver = button_node_class
        node = ButtonNode()
        node.trigger_configure()

        node.destroy_node()

        mock_driver.close.assert_called_once()

    def test_destroy_without_configure_does_not_raise(self, ros_context, button_node_class):
        """A node destroyed before ever being configured must not crash cleanup."""
        ButtonNode, mock_driver = button_node_class
        node = ButtonNode()

        node.destroy_node()  # must not raise

        mock_driver.close.assert_not_called()
