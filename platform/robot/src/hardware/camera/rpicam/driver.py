"""Camera capture through the ``rpicam-vid`` CLI.

The Picamera2 driver next door is the nicer API, but ``picamera2`` is not
installed in any environment on this robot -- not in the pixi envs, not in
system python, not via apt -- and it cannot simply be added: its ``libcamera``
bindings are built against the system libcamera for the system interpreter,
while the ROS nodes run a pixi-managed Python of a different minor version.
OpenCV cannot stand in either: the Pi 5's ``/dev/video*`` entries are libcamera
media nodes, and ``VideoCapture`` fails on them with "Not a video capture
device".

What is installed and working is the ``rpicam-*`` CLI. This streams MJPEG from
``rpicam-vid`` on stdout and decodes frame by frame, which needs no Python
bindings at all. MJPEG is chosen over raw YUV because its frames are
self-delimiting, so a short read can never desynchronise the stream.
"""

from __future__ import annotations

import logging
import subprocess
import time
from typing import TYPE_CHECKING, Self

import cv2
import numpy as np
from pydantic import AliasChoices, Field
from pydantic_settings import SettingsConfigDict

from shared.domain.models import CameraSize, ImageRotation
from src.hardware.camera.base import Driver as CameraDriver, Frame
from src.hardware.settings_base import CONFIG_DIR, HardwareBaseSettings
from src.logger import configure_json_logging
from src.logger.constants import DETAILS_KEY

if TYPE_CHECKING:
    from types import TracebackType

configure_json_logging()
log = logging.getLogger(__name__)

# JPEG start/end of image markers.
_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"
_READ_CHUNK = 65536


class Config(HardwareBaseSettings):
    """Capture settings, sharing the CAMERA_* variables with the Picamera2 driver."""

    model_config = SettingsConfigDict(env_prefix="", toml_file=CONFIG_DIR / "camera" / "rpicam.toml")

    width: int = Field(default=1536, validation_alias=AliasChoices("CAMERA_WIDTH", "camera_width"))
    height: int = Field(default=864, validation_alias=AliasChoices("CAMERA_HEIGHT", "camera_height"))
    fps: int = Field(default=30, validation_alias=AliasChoices("CAMERA_FPS", "camera_fps"))

    inverted: bool = Field(default=False, validation_alias=AliasChoices("CAMERA_INVERTED", "camera_inverted"))
    """
    True when the camera is mounted upside-down. Applies a 180 degree rotation, which matters beyond looking right: an unrotated frame mirrors which side of the image a sign falls on, so a sign to be passed on the left is reported to the right of centre.
    """

    hflip: bool = Field(default=False, validation_alias=AliasChoices("CAMERA_HFLIP", "camera_hflip"))
    vflip: bool = Field(default=False, validation_alias=AliasChoices("CAMERA_VFLIP", "camera_vflip"))

    timeout_sec: float = Field(
        default=5.0, validation_alias=AliasChoices("CAMERA_READ_TIMEOUT_SEC", "camera_read_timeout_sec")
    )
    """
    How long to wait for a complete frame before reporting the stream dead.
    """


class Driver(CameraDriver):
    """Streams frames from ``rpicam-vid``.

    Mirrors the shape of the Picamera2 driver (``connect`` / ``capture_frame``
    / ``to_rgb`` / ``close``) so callers can hold either without caring which.
    """

    def __init__(self, config: Config | None = None) -> None:
        self.config = config or Config()
        self._process: subprocess.Popen[bytes] | None = None
        self._buffer = b""

    def _command(self) -> list[str]:
        """Build the rpicam-vid invocation."""
        # 180 degrees is both mirrors at once; rpicam-vid takes --hflip/--vflip
        # as flags, and --rotation only accepts 0 or 180 on this pipeline.
        hflip = self.config.hflip != self.config.inverted
        vflip = self.config.vflip != self.config.inverted
        cmd = [
            "rpicam-vid",
            "-t",
            "0",  # run until killed
            "-n",  # no preview window; there is no display
            "--codec",
            "mjpeg",
            "-o",
            "-",
            "--width",
            str(self.config.width),
            "--height",
            str(self.config.height),
            "--framerate",
            str(self.config.fps),
        ]
        if hflip:
            cmd.append("--hflip")
        if vflip:
            cmd.append("--vflip")
        return cmd

    def connect(self) -> None:
        """Start the capture process."""
        cmd = self._command()
        log.info("Starting rpicam-vid", extra={DETAILS_KEY: {"cmd": " ".join(cmd)}})
        self._process = subprocess.Popen(  # noqa: S603
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        self._buffer = b""

    def capture_frame(self) -> Frame:
        """Return the next decoded frame.

        Returns:
            The frame in OpenCV's BGR order; pass it through :meth:`to_rgb`
            before handing it to the detector.

        Raises:
            RuntimeError: If the stream ends or no frame arrives in time.
        """
        if self._process is None or self._process.stdout is None:
            msg = "Camera not connected; call connect() first."
            raise RuntimeError(msg)

        deadline = time.monotonic() + self.config.timeout_sec
        while time.monotonic() < deadline:
            frame = self._take_frame()
            if frame is not None:
                height, width = frame.shape[:2]
                return Frame(frame=frame, timestamp=time.time(), width=width, height=height)

            chunk = self._process.stdout.read(_READ_CHUNK)
            if not chunk:
                msg = "rpicam-vid stream ended unexpectedly."
                raise RuntimeError(msg)
            self._buffer += chunk

        msg = f"No frame within {self.config.timeout_sec}s."
        raise RuntimeError(msg)

    def _take_frame(self) -> np.ndarray | None:
        """Pull one complete JPEG out of the buffer, if there is one."""
        start = self._buffer.find(_SOI)
        if start < 0:
            # No frame boundary in sight: drop the garbage rather than let the
            # buffer grow without bound.
            self._buffer = b""
            return None
        end = self._buffer.find(_EOI, start + 2)
        if end < 0:
            self._buffer = self._buffer[start:]
            return None

        jpeg = self._buffer[start : end + 2]
        self._buffer = self._buffer[end + 2 :]
        return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)

    def get_resolution(self) -> CameraSize:
        """Return the configured capture resolution and orientation metadata."""
        return CameraSize(
            width_px=self.config.width,
            height_px=self.config.height,
            rotation_deg=ImageRotation.CW_180 if self.config.inverted else ImageRotation.NONE,
            hflip=self.config.hflip,
            vflip=self.config.vflip,
        )

    def close(self) -> None:
        """Stop the capture process. Safe to call more than once."""
        if self._process is None:
            return
        self._process.terminate()
        try:
            self._process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._process.kill()
        self._process = None
        self._buffer = b""
        log.info("Stopped rpicam-vid")

    def __enter__(self) -> Self:
        self.connect()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()
