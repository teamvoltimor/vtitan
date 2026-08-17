"""Shared assertions for LifecycleNode driver-connect/destroy mechanics.

These two behaviors are pinned identically across every hardware node
(button, OLED, ackermann motors, ...): a driver connects only once
``trigger_configure()`` runs, and a node destroyed before ever being
configured must not crash cleanup or touch a driver it never had. The
mechanics are identical; only the driver attribute name and the
teardown method being asserted differ per node.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from unittest import mock


def assert_connects_driver_on_configure(node_cls: type, driver: mock.MagicMock, driver_attr: str) -> None:
    """Configuring a fresh node connects its driver exactly once."""
    node = node_cls()
    assert getattr(node, driver_attr) is None  # not yet configured

    node.trigger_configure()

    driver.connect.assert_called_once()
    assert getattr(node, driver_attr) is driver

    node.destroy_node()


def assert_destroy_without_configure_does_not_raise(
    node_cls: type, driver: mock.MagicMock, close_attr: str = "close"
) -> None:
    """A node destroyed before ever being configured must not crash cleanup."""
    node = node_cls()  # never configured -- the driver is None

    node.destroy_node()  # must not raise

    getattr(driver, close_attr).assert_not_called()
