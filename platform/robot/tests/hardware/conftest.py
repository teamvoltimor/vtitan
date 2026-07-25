"""Shared config for hardware tests.

Every test under ``tests/hardware/`` talks to a physical peripheral (Build HAT
motors, I2C/UART IMU, camera, Hailo NPU). They can only pass on a real robot,
so they are auto-tagged with the ``hardware`` marker and excluded from the
default test run (``-m "not hardware"``). Run them on-device with the
``robot:test SCOPE=hardware`` task.
"""

from pathlib import Path

import pytest

_HARDWARE_DIR = Path(__file__).parent


def pytest_collection_modifyitems(items: list[pytest.Item]) -> None:
    """Tag tests collected under tests/hardware/ with the ``hardware`` marker.

    Note: this hook fires session-wide, so we must scope it to items whose file
    actually lives under this directory rather than marking everything.
    """
    for item in items:
        if _HARDWARE_DIR in Path(item.path).parents:
            item.add_marker(pytest.mark.hardware)
