"""Shared assertions for the BNO08x IMU lifecycle nodes (I2C and UART-RVC).

Both nodes expose the same attribute surface (``driver``, ``publisher_``,
``publish_imu()``) for the mechanics that are genuinely identical between
them: configuring sets up the driver and publisher, the publish timer is
created on activate rather than configure, and publishing before configure
is a no-op. Everything downstream of that -- failure policy on a bad
connect, the sensor-reading dataclass, covariance semantics -- differs by
real hardware capability (RVC has no live gyro stream, I2C does) and is
intentionally NOT shared here; see the 2026-08-15 reuse audit.
"""

from __future__ import annotations


def assert_node_configures_correctly(node_cls: type, expected_name: str) -> None:
    node = node_cls()
    node.trigger_configure()
    assert node.get_name() == expected_name
    assert node.publisher_ is not None
    assert node.driver is not None
    node.destroy_node()


def assert_creates_timer_on_activate(node_cls: type) -> None:
    """The publish timer is created on activate, not on configure."""
    node = node_cls()
    node.trigger_configure()
    assert list(node.timers) == []

    node.trigger_activate()

    assert len(list(node.timers)) > 0
    node.destroy_node()


def assert_publish_imu_noop_before_configure(node_cls: type) -> None:
    node = node_cls()

    node.publish_imu()  # must not raise

    node.destroy_node()
