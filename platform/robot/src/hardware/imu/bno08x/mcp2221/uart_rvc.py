"""Hardware driver for BNO08x IMU via MCP2221A UART RVC mode."""

import logging
import time
from dataclasses import dataclass
from threading import Event, Thread
from typing import cast, override

import serial
import serial.tools.list_ports
from adafruit_bno08x_rvc import BNO08x_RVC
from scipy.spatial.transform import Rotation as R

from src.env import EnvVar
from src.hardware.imu.base import (
    RVCData,
    RVCDriver as ABC_RVCDriver,
)
from src.logger import configure_json_logging

logger = configure_json_logging()


IMU_UART_RVC_PORT = EnvVar[str](key="IMU_UART_RVC_PORT", default="")
"""
Serial port for MCP2221 UART connection. If empty, the driver will attempt to auto-detect the port based on VID/PID.
"""

IMU_UART_RVC_BAUDRATE = EnvVar[int](key="IMU_UART_RVC_BAUDRATE", default=115200, cast=int)
"""
Baud rate for UART communication with the MCP2221. The BNO08x RVC library typically uses 115200 baud.
"""

IMU_UART_RVC_POLL_RATE = EnvVar[float](key="IMU_UART_RVC_POLL_RATE", default=100.0, cast=float)
"""
Polling rate in Hz for reading data from the IMU. Higher rates may increase CPU usage.
"""

IMU_UART_RVC_MCP2221_VID = EnvVar[int](
    key="IMU_UART_RVC_MCP2221_VID",
    default=0x04D8,
    cast=lambda x: int(x, 0) if x.startswith("0x") else int(x),
)
"""
USB Vendor ID (VID) for the MCP2221 USB bridge. Default is 0x04D8, which is the VID for Microchip Technology Inc.
"""

IMU_UART_RVC_MCP2221_PID = EnvVar[int](
    key="IMU_UART_RVC_MCP2221_PID",
    default=0x00DD,
    cast=lambda x: int(x, 0) if x.startswith("0x") else int(x),
)
"""
USB Product ID (PID) for the MCP2221 USB bridge. Default is 0x00DD, which is the PID for the MCP2221.
"""


@dataclass
class Config:
    """Configuration for BNO08x via MCP2221A UART RVC."""

    port: str = IMU_UART_RVC_PORT.value
    baudrate: int = IMU_UART_RVC_BAUDRATE.value
    poll_rate_hz: float = IMU_UART_RVC_POLL_RATE.value
    mcp2221_vid: int = IMU_UART_RVC_MCP2221_VID.value
    mcp2221_pid: int = IMU_UART_RVC_MCP2221_PID.value


class Driver(ABC_RVCDriver):
    """Driver for BNO08x IMU via MCP2221A UART RVC mode."""

    def __init__(self, config: Config | None = None):
        self.config: Config = config or Config()
        self._serial: serial.Serial | None = None
        self._rvc: BNO08x_RVC | None = None
        self._running: bool = False
        self._thread: Thread | None = None
        self._latest_data: RVCData | None = None
        self._lock: Event = Event()
        self.logger: logging.Logger = logging.getLogger(__name__)

    def find_mcp2221_port(self) -> str | None:
        """Auto-detect MCP2221 USB bridge port."""
        ports = serial.tools.list_ports.comports()

        # Look for a port with the matching VID and PID for the MCP2221
        for port in ports:
            if port.vid == self.config.mcp2221_vid and port.pid == self.config.mcp2221_pid:
                self.logger.info("Found MCP2221", extra={"details": {"port": port.device}})
                return port.device
        return None

    @override
    def connect(self) -> None:
        """Connect to IMU via MCP2221A UART."""
        port = self.config.port
        if not port:
            port = self.find_mcp2221_port()
            if not port:
                self.logger.warning("MCP2221 auto-detect failed, using /dev/ttyACM0")
                port = "/dev/ttyACM0"

        # The adafruit_bno08x_rvc library does not support direct USB communication,
        # but it can work with a serial port provided by the MCP2221 in UART mode.
        # We will use pyserial to open the serial port and pass it to the B
        self.logger.info(
            "Connecting to BNO08x via MCP2221",
            extra={"details": {"port": port, "baudrate": self.config.baudrate}},
        )
        self._serial = serial.Serial(port, baudrate=self.config.baudrate, timeout=1)

        self._rvc = BNO08x_RVC(self._serial)
        self.logger.info("Connected to BNO08x RVC")

    @override
    def start_polling(self) -> None:
        """Start background polling thread."""
        if self._running:
            return

        # The adafruit_bno08x_rvc library does not have a built-in polling mechanism, so we will implement our own
        # background thread that continuously reads data from the sensor at the specified poll rate.
        self._running = True
        self._thread = Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        self.logger.info("Polling started", extra={"details": {"rate_hz": self.config.poll_rate_hz}})

    @override
    def stop_polling(self) -> None:
        """Stop background polling thread."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

        self.logger.info("Polling stopped")

    def _poll_loop(self) -> None:
        """Background polling loop."""
        interval = 1.0 / self.config.poll_rate_hz
        while self._running:
            try:
                # The adafruit_bno08x_rvc library does not have a built-in method to check if new data is available,
                # so we will just read the latest heading data on each loop iteration. This may not be the most efficient approach,
                # but it should work for our purposes.
                if self._rvc is None:
                    break
                yaw_deg, pitch_deg, roll_deg, x_accel, y_accel, z_accel = self._rvc.heading

                # Convert Euler angles to quaternion using scipy.spatial.transform.Rotation
                r = R.from_euler("xyz", [roll_deg, pitch_deg, yaw_deg], degrees=True)

                # The adafruit_bno08x_rvc library does not provide quaternion data directly, but we can convert from Euler angles.
                qx, qy, qz, qw = cast("tuple[float, float, float, float]", cast("object", r.as_quat()))
                self._latest_data = RVCData(
                    yaw_deg=yaw_deg,
                    pitch_deg=pitch_deg,
                    roll_deg=roll_deg,
                    x_accel=x_accel,
                    y_accel=y_accel,
                    z_accel=z_accel,
                    quaternion=(qx, qy, qz, qw),
                )
            except (OSError, ValueError, AttributeError) as e:
                self.logger.warning("Polling error", extra={"error": str(e)})
            time.sleep(interval)

    @override
    def get_data(self) -> RVCData | None:
        """Get latest sensor data."""
        return self._latest_data

    def get_data_blocking(self) -> RVCData | None:
        """Get latest sensor data, blocking until available."""
        _ = self._lock.wait(timeout=2.0)
        return self._latest_data

    @override
    def close(self) -> None:
        """Close connection."""
        self.stop_polling()

        # The adafruit_bno08x_rvc library does not have a close method, but if it did, we would call it here.
        if self._serial and self._serial.is_open:
            self._serial.close()
        self.logger.info("Connection closed")
