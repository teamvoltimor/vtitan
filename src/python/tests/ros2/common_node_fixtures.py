"""Shared assertions for LifecycleNode driver-connect/destroy mechanics, and
graph-query retry helpers for topic-contract tests.

The driver-connect/destroy behaviors are pinned identically across every
hardware node (button, OLED, ackermann motors, ...): a driver connects only
once ``trigger_configure()`` runs, and a node destroyed before ever being
configured must not crash cleanup or touch a driver it never had. The
mechanics are identical; only the driver attribute name and the
teardown method being asserted differ per node.

The graph-query helpers exist because a node's local ROS graph cache
(``get_publisher_names_and_types_by_node``, ``get_subscriptions_info_by_node``,
``get_subscriptions_info_by_topic``, ...) fills in off the middleware's
discovery thread rather than synchronously inside
``create_publisher()``/``create_subscription()`` -- querying it immediately
after construction is a race that shows up as a flaky empty result under the
CPU contention of a parallel test run. Only presence assertions need this;
an assertion that a topic was never wired up has no race to protect against,
since nothing is ever going to appear.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import rclpy

if TYPE_CHECKING:
    from collections.abc import Callable
    from unittest import mock

    from rclpy.node import Node

_DEFAULT_TIMEOUT_SEC = 2.0
_SPIN_INTERVAL_SEC = 0.05


def wait_for_graph_entry(
    node: Node,
    query: Callable[[], dict[str, list[str]]],
    key: str,
    timeout_sec: float = _DEFAULT_TIMEOUT_SEC,
) -> dict[str, list[str]]:
    """Poll a dict-returning graph query (bound to ``node``) until ``key`` appears.

    ``query`` is typically
    ``lambda: dict(node.get_publisher_names_and_types_by_node(node.get_name(), ""))``
    or the subscriber equivalent. Returns whatever ``query()`` last produced,
    whether or not ``key`` ever appeared -- the caller's own assertion is
    what should fail, not this helper.
    """
    deadline = time.monotonic() + timeout_sec
    result = query()
    while key not in result and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=_SPIN_INTERVAL_SEC)
        result = query()
    return result


def wait_for_subscriptions_info(node: Node, topic: str, timeout_sec: float = _DEFAULT_TIMEOUT_SEC) -> list:
    """Poll ``node.get_subscriptions_info_by_topic(topic)`` until it's non-empty."""
    deadline = time.monotonic() + timeout_sec
    result = node.get_subscriptions_info_by_topic(topic)
    while not result and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=_SPIN_INTERVAL_SEC)
        result = node.get_subscriptions_info_by_topic(topic)
    return result


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
