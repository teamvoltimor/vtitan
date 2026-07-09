"""Hardware driver for BNO08x IMU via MCP2221A UART RVC mode."""

import os
from typing import override

import serial
import serial.tools.list_ports
from adafruit_bno08x_rvc import BNO08x_RVC
from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

os.environ.setdefault("BLINKA_MCP2221", "1")

from src.hardware.imu.bno08x.uart_rvc import (
    SERIAL_TIMEOUT,
    Config as UARTRVCConfig,
    Driver as UARTRVCDriver,
)
from src.hardware.imu.config import QuaternionConfig
from src.hardware.mcp2221.config import MCP2221Config


class Config(BaseSettings):
    """Configuration for BNO08x via MCP2221A UART RVC."""

    model_config = SettingsConfigDict(
        env_prefix="bno08x_uart_rvc_",
        # "__" so nested leaves with underscores parse, e.g.
        # BNO08X_UART_RVC_QUATERNION__NEGATE_YAW -> quaternion.negate_yaw.
        env_nested_delimiter="__",
    )

    # QuaternionConfig's negate_yaw/pitch/roll have no defaults -- this factory
    # only succeeds when the nested QUATERNION__* env vars are set; mypy
    # can't see that env resolution, hence the ignore.
    quaternion: QuaternionConfig = Field(default_factory=lambda: QuaternionConfig())  # type: ignore[call-arg]

    port: str
    """Serial port for UART connection. If empty, the driver will attempt to auto-detect the port based on VID/PID."""

    baudrate: int = 115200
    """Baud rate for UART communication. The BNO08x RVC library typically uses 115200 baud."""

    poll_rate_hz: float = 100.0
    """Polling rate in Hz for reading data from the IMU. Higher rates may increase CPU usage."""

    mcp2221: MCP2221Config = Field(default_factory=MCP2221Config)
    """MCP2221 USB bridge configuration."""


class Driver(UARTRVCDriver):
    """Driver for BNO08x IMU via MCP2221A UART RVC mode."""

    def __init__(self, config: Config | None = None):
        # port is required with no default -- resolved from the BNO08X_UART_RVC_PORT
        # env var when config isn't passed explicitly; mypy can't see that.
        config = config or Config()  # type: ignore[call-arg]
        super().__init__(
            config=UARTRVCConfig(
                quaternion=config.quaternion,
                port=config.port,
                baudrate=config.baudrate,
                poll_rate_hz=config.poll_rate_hz,
            ),
        )
        # Kept separate from self.config (typed UARTRVCConfig by the base class)
        # since this subclass's Config additionally carries the MCP2221 vid/pid
        # used only by find_mcp2221_port() below.
        self._mcp2221_config: Config = config

    def find_mcp2221_port(self) -> str | None:
        """Auto-detect MCP2221 USB bridge port."""
        self.logger.info("Auto-detecting MCP2221 USB bridge port...")
        ports = serial.tools.list_ports.comports()

        for port in ports:
            if port.vid == self._mcp2221_config.mcp2221.vid and port.pid == self._mcp2221_config.mcp2221.pid:
                self.logger.info("Found MCP2221", extra={"details": {"port": port.device}})
                return str(port.device)

        self.logger.warning(
            "MCP2221 not found during auto-detect",
            extra={
                "details": {
                    "vid": hex(self._mcp2221_config.mcp2221.vid),
                    "pid": hex(self._mcp2221_config.mcp2221.pid),
                },
            },
        )
        return None

    @override
    def connect(self) -> None:
        """Connect to IMU via MCP2221A UART."""
        self._conn_lock.acquire()
        self.logger.info("Connecting to BNO08x via MCP2221 UART RVC...")

        port: str | None = self.config.port
        if not port:
            port = self.find_mcp2221_port()
            if not port:
                self.logger.warning("MCP2221 auto-detect failed, using /dev/ttyACM0")
                port = "/dev/ttyACM0"

        self.logger.info(
            "Connecting to BNO08x via MCP2221",
            extra={"details": {"port": port, "baudrate": self.config.baudrate}},
        )
        self._serial = serial.Serial(port, baudrate=self.config.baudrate, timeout=SERIAL_TIMEOUT)

        self._rvc = BNO08x_RVC(self._serial)
        self.logger.info("Connected to BNO08x RVC")
        self._conn_lock.release()
