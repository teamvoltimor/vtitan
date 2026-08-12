"""Hardware driver for BNO08x IMU via UART RVC mode."""

import logging
import threading
import time
from threading import Thread
from typing import override

import serial
import serial.tools.list_ports
from adafruit_bno08x_rvc import BNO08x_RVC, RVCReadTimeoutError
from pydantic import Field
from pydantic_settings import SettingsConfigDict

from src.hardware.imu.base import (
    RVCDriver as ABC_RVCDriver,
)
from src.hardware.imu.bno08x.utils import calculate_quaternion_from_euler
from src.hardware.imu.config import QuaternionConfig
from src.hardware.imu.readings import RVCReading
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings
from src.logger import configure_json_logging
from src.logger.constants import DETAILS_KEY

configure_json_logging()


class Config(HardwareBaseSettings):
    """Configuration for BNO08x via UART RVC."""

    model_config = SettingsConfigDict(
        env_prefix="bno08x_uart_rvc_",
        # "__" so nested leaves with underscores parse, e.g.
        # BNO08X_UART_RVC_QUATERNION__NEGATE_YAW -> quaternion.negate_yaw.
        env_nested_delimiter="__",
        toml_file=CONFIG_DIR / "imu" / "bno08x_uart_rvc.toml",
    )

    quaternion: QuaternionConfig = Field(default_factory=QuaternionConfig)

    port: str
    """Serial port for UART connection. If empty, the driver will attempt to auto-detect the port based on VID/PID."""

    default_port: str = "/dev/ttyACM0"
    """Fallback port when port is not configured."""

    baudrate: int = 115200
    """Baud rate for UART communication. The BNO08x RVC library typically uses 115200 baud."""

    poll_rate_hz: float = 100.0
    """Polling rate in Hz for reading data from the IMU. Higher rates may increase CPU usage."""

    serial_timeout: float = 1.0
    """Timeout in seconds for serial communication.

    Distinct from NavigationTuning.sensors.STALE_TIMEOUT_SEC: that gates how
    old a cached reading may be before the navigator distrusts it, a
    control-loop concern. This is a raw pyserial read/thread-join timeout, a
    driver-internal implementation detail with no navigation meaning.
    """

    data_lock_timeout: float = 2.0
    """Timeout in seconds for waiting on new data to be available."""


class Driver(ABC_RVCDriver):
    """Driver for BNO08x IMU via UART RVC mode."""

    def __init__(self, config: Config | None = None):
        # port is required with no default -- resolved from the BNO08X_UART_RVC_PORT
        # env var when config isn't passed explicitly; mypy can't see that.
        self.config: Config = config or Config()  # type: ignore[call-arg]
        self._serial: serial.Serial | None = None
        self._rvc: BNO08x_RVC | None = None
        self._thread: Thread | None = None
        self._latest_data: RVCReading | None = None
        self._conn_lock: threading.Lock = threading.Lock()
        self._running: threading.Lock = threading.Lock()
        self._data_lock: threading.Event = threading.Event()
        self.logger: logging.Logger = logging.getLogger(__name__)

    @override
    def connect(self) -> None:
        """Connect to IMU via UART."""
        # Acquire lock to prevent concurrent connections
        self._conn_lock.acquire()
        self.logger.info("Connecting to BNO08x via UART RVC...")

        port = self.config.port
        if not port:
            port = self.config.default_port
            self.logger.info("No serial port specified, attempting with configured default port %s", port)

        # The adafruit_bno08x_rvc library does not support direct USB communication,
        # but it can work with a serial port provided in UART mode.
        # We will use pyserial to open the serial port and pass it to the B
        self.logger.info(
            "Connecting to BNO08x",
            extra={DETAILS_KEY: {"port": port, "baudrate": self.config.baudrate}},
        )
        self._serial = serial.Serial(port, baudrate=self.config.baudrate, timeout=self.config.serial_timeout)

        self._rvc = BNO08x_RVC(self._serial)
        self.logger.info("Connected to BNO08x RVC")
        self._conn_lock.release()

    @override
    def start_polling(self) -> None:
        """Start background polling thread."""
        if self._running.locked():
            return

        # The adafruit_bno08x_rvc library does not have a built-in polling mechanism, so we will implement our own
        # background thread that continuously reads data from the sensor at the specified poll rate.
        self._running.acquire()
        self._thread = Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        self.logger.info("Polling started", extra={DETAILS_KEY: {"rate_hz": self.config.poll_rate_hz}})

    @override
    def stop_polling(self) -> None:
        """Stop background polling thread."""
        if not self._running.locked():
            return

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=self.config.serial_timeout)

        self.logger.info("Polling stopped")
        self._running.release()

    def _poll_loop(self) -> None:
        """Background polling loop."""
        interval = 1.0 / self.config.poll_rate_hz

        while self._running.locked():
            try:
                # The adafruit_bno08x_rvc library does not have a built-in method to check if new data is available,
                # so we will just read the latest heading data on each loop iteration. This may not be the most efficient approach,
                # but it should work for our purposes.
                if self._rvc is None:
                    break
                yaw_deg, pitch_deg, roll_deg, x_accel, y_accel, z_accel = self._rvc.heading

                self._latest_data = RVCReading(
                    yaw_deg=yaw_deg,
                    pitch_deg=pitch_deg,
                    roll_deg=roll_deg,
                    x_accel=x_accel,
                    y_accel=y_accel,
                    z_accel=z_accel,
                    quaternion=calculate_quaternion_from_euler(
                        yaw_deg=yaw_deg,
                        pitch_deg=pitch_deg,
                        roll_deg=roll_deg,
                        euler_sequence=self.config.quaternion.euler_sequence,
                        negate_yaw=self.config.quaternion.negate_yaw,  # Negate yaw for ROS 2 CCW-positive Yaw
                        negate_pitch=self.config.quaternion.negate_pitch,  # Negate pitch if Pitch is inverted (Nose up = Model down)
                        negate_roll=self.config.quaternion.negate_roll,  # Negate roll if Roll is inverted (Bank right = Model left)
                    ),
                )
                self._data_lock.set()  # Signal that new data is available
            except (OSError, ValueError, AttributeError, RVCReadTimeoutError) as e:
                # A read timeout is a transient serial hiccup, not a dead
                # connection -- previously uncaught, it silently killed this
                # daemon thread on the first bad read and _latest_data froze
                # forever with nothing left to log the failure past a stray
                # traceback from Python's default thread excepthook.
                self.logger.warning("Polling error", extra={"error": str(e)})
            time.sleep(interval)

    @override
    def get_data(self) -> RVCReading | None:
        """Get latest sensor data."""
        return self._latest_data

    def get_data_blocking(self) -> RVCReading | None:
        """Get latest sensor data, blocking until available."""
        _ = self._data_lock.wait(timeout=self.config.data_lock_timeout)
        return self._latest_data

    @override
    def close(self) -> None:
        """Close connection."""
        self._conn_lock.acquire()
        self.logger.info("Closing connection to BNO08x RVC")
        self.stop_polling()

        # The adafruit_bno08x_rvc library does not have a close method, but if it did, we would call it here.
        if self._serial and self._serial.is_open:
            self._serial.close()
        self.logger.info("Connection closed")
        self._conn_lock.release()
