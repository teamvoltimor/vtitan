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
from enum import StrEnum
from typing import TYPE_CHECKING, Self

import cv2
import numpy as np
from pydantic import AliasChoices, Field, model_validator
from pydantic_settings import SettingsConfigDict

from src.hardware.camera.base import (
    Config as CameraConfig,
    Driver as CameraDriver,
    Frame,
)
from src.hardware.settings_base import CONFIG_DIR
from src.logger import configure_json_logging
from src.logger.constants import DETAILS_KEY

if TYPE_CHECKING:
    from types import TracebackType

    from shared.domain.models import CameraSize

configure_json_logging()
log = logging.getLogger(__name__)

# JPEG start/end of image markers.
_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"
_READ_CHUNK = 65536


class ExposureMode(StrEnum):
    """Auto-exposure bias, matching the Picamera2 driver's `AeExposureMode` naming.

    rpicam-vid's ``--exposure`` flag has no literal "short" value; SHORT maps
    to its closest equivalent, "sport".
    """

    NORMAL = "normal"
    SHORT = "short"
    LONG = "long"

    @property
    def rpicam_value(self) -> str:
        """The ``--exposure`` value rpicam-vid expects for this mode."""
        return "sport" if self is ExposureMode.SHORT else self.value


class MeteringMode(StrEnum):
    """Which part of the frame auto-exposure measures.

    Inert once auto-exposure is off (`exposure_time_us` set), which is the
    real answer to a metering problem: a track dominated by white mat drags
    average metering toward under-exposure, but correcting that with SPOT or
    positive `exposure_value` lengthens the shutter, which is what smears
    signs while cornering.

    SPOT is a poor fit here despite the temptation -- it measures the frame
    centre, and signs are not reliably centred, so it often meters bare mat.
    """

    CENTRE = "centre"
    SPOT = "spot"
    AVERAGE = "average"
    CUSTOM = "custom"


class AfMode(StrEnum):
    """Autofocus mode, matching the Picamera2 driver's `AfMode` naming.

    rpicam-vid's own default is CONTINUOUS, which re-hunts every time the
    scene changes -- i.e. continuously while the robot is driving, settling
    only once it stops. MANUAL with `lens_position` set keeps the voice coil
    still and is what this robot wants.
    """

    MANUAL = "manual"
    AUTO = "auto"
    CONTINUOUS = "continuous"


class AfSpeed(StrEnum):
    """Autofocus search speed. Only applies when `AfMode` is AUTO or CONTINUOUS."""

    NORMAL = "normal"
    FAST = "fast"


class AwbMode(StrEnum):
    """Auto white balance mode, matching the Picamera2 driver's `AwbMode` naming.

    Sign colour classification (red vs green) is threshold-based, so a fixed
    mode stops AWB drift from shifting hue readings as the framing changes.
    Every member has a same-named ``--awb`` value in rpicam-vid.
    """

    AUTO = "auto"
    TUNGSTEN = "tungsten"
    FLUORESCENT = "fluorescent"
    INDOOR = "indoor"
    DAYLIGHT = "daylight"
    CLOUDY = "cloudy"


class NoiseReductionMode(StrEnum):
    """Noise reduction mode, matching the Picamera2 driver's naming plus rpicam-vid's own AUTO.

    rpicam-vid spells these as colour-denoise variants, so each member maps to
    a ``cdn_*`` value rather than to its own name. MINIMAL has no exact
    counterpart; it maps to "cdn_off", which keeps spatial denoise but drops
    the colour pass.
    """

    AUTO = "auto"
    OFF = "off"
    FAST = "fast"
    HIGH_QUALITY = "high_quality"
    MINIMAL = "minimal"

    @property
    def rpicam_value(self) -> str:
        """The ``--denoise`` value rpicam-vid expects for this mode."""
        return _DENOISE_VALUES[self]


_DENOISE_VALUES = {
    NoiseReductionMode.AUTO: "auto",
    NoiseReductionMode.OFF: "off",
    NoiseReductionMode.FAST: "cdn_fast",
    NoiseReductionMode.HIGH_QUALITY: "cdn_hq",
    NoiseReductionMode.MINIMAL: "cdn_off",
}


class Config(CameraConfig):
    """Capture settings, sharing the CAMERA_* variables with the Picamera2 driver."""

    model_config = SettingsConfigDict(env_prefix="", toml_file=CONFIG_DIR / "camera" / "rpicam.toml")

    timeout_sec: float = Field(
        default=5.0, validation_alias=AliasChoices("CAMERA_READ_TIMEOUT_SEC", "camera_read_timeout_sec")
    )
    """
    How long to wait for a complete frame before reporting the stream dead.
    """

    af_mode: AfMode = Field(
        default=AfMode.CONTINUOUS, validation_alias=AliasChoices("CAMERA_AF_MODE", "camera_af_mode")
    )
    """
    Passed as ``--autofocus-mode``. Defaults to CONTINUOUS to match rpicam-vid's own default; set MANUAL with `lens_position` to stop the lens hunting while the robot drives.
    """

    lens_position: float | None = Field(
        default=None, validation_alias=AliasChoices("CAMERA_LENS_POSITION", "camera_lens_position")
    )
    """
    Dioptres (1/distance_m), passed as ``--lens-position``. Only sent when `af_mode` is MANUAL, because rpicam-vid ignores it in the AF modes.
    """

    af_speed: AfSpeed = Field(
        default=AfSpeed.NORMAL, validation_alias=AliasChoices("CAMERA_AF_SPEED", "camera_af_speed")
    )
    """Passed as ``--autofocus-speed``. Only applies when `af_mode` is AUTO or CONTINUOUS."""

    exposure_mode: ExposureMode = Field(
        default=ExposureMode.NORMAL, validation_alias=AliasChoices("CAMERA_EXPOSURE_MODE", "camera_exposure_mode")
    )
    """SHORT biases auto-exposure toward shorter exposure times (less motion blur, more noise)."""

    exposure_time_us: int | None = Field(
        default=None, validation_alias=AliasChoices("CAMERA_EXPOSURE_TIME_US", "camera_exposure_time_us")
    )
    """
    Manual shutter time in microseconds, passed as ``--shutter``. Set together with `analogue_gain` to disable auto-exposure entirely; leave unset to keep AE (biased by `exposure_mode`) enabled.
    """

    analogue_gain: float = Field(
        default=1.0, validation_alias=AliasChoices("CAMERA_ANALOGUE_GAIN", "camera_analogue_gain")
    )
    """Sensor gain, passed as ``--gain``. Only fixed when `exposure_time_us` is also set; otherwise AE is free to adjust it."""

    awb_mode: AwbMode = Field(default=AwbMode.AUTO, validation_alias=AliasChoices("CAMERA_AWB_MODE", "camera_awb_mode"))
    """
    Passed as ``--awb``. Defaults to AUTO to match rpicam-vid's own default; pin it to the venue's lighting so the downstream red/green sign thresholds see a stable hue.
    """

    noise_reduction_mode: NoiseReductionMode = Field(
        default=NoiseReductionMode.AUTO,
        validation_alias=AliasChoices("CAMERA_NOISE_REDUCTION_MODE", "camera_noise_reduction_mode"),
    )
    """Passed as ``--denoise``. HIGH_QUALITY adds latency the control loop can't afford."""

    sharpness: float = Field(default=1.0, validation_alias=AliasChoices("CAMERA_SHARPNESS", "camera_sharpness"))
    """Passed as ``--sharpness``. 1.0 is rpicam-vid's neutral; above it sharpens, 0 disables."""

    awb_gains: tuple[float, float] | None = Field(
        default=None, validation_alias=AliasChoices("CAMERA_AWB_GAINS", "camera_awb_gains")
    )
    """
    Fixed red and blue gains, passed as ``--awbgains R,B``. Pins colour exactly instead of picking a named preset that may not match how the detector's training images were captured; read the gains auto AWB settles on for the venue, then set them here. Setting this disables auto AWB, so `awb_mode` no longer applies.
    """

    metering_mode: MeteringMode = Field(
        default=MeteringMode.CENTRE, validation_alias=AliasChoices("CAMERA_METERING_MODE", "camera_metering_mode")
    )
    """Passed as ``--metering``. Ignored once `exposure_time_us` turns auto-exposure off."""

    exposure_value: float = Field(
        default=0.0, validation_alias=AliasChoices("CAMERA_EXPOSURE_VALUE", "camera_exposure_value")
    )
    """
    Exposure compensation in stops, passed as ``--ev``. Positive lifts a scene the meter is under-exposing (a frame dominated by white mat), at the cost of a longer shutter. Ignored once `exposure_time_us` turns auto-exposure off.
    """

    flicker_period_us: int | None = Field(
        default=None, validation_alias=AliasChoices("CAMERA_FLICKER_PERIOD_US", "camera_flicker_period_us")
    )
    """
    Mains flicker period in microseconds, passed as ``--flicker-period``. Artificial venue lighting pulses at twice the mains frequency, so 50 Hz mains needs 10000 (10 ms) and 60 Hz needs 8333; leave unset outdoors. Short shutters are what make the banding visible, so this matters more as `exposure_time_us` comes down.
    """

    @model_validator(mode="after")
    def _manual_focus_needs_a_lens_position(self) -> Self:
        """Reject MANUAL focus with no lens position.

        rpicam-vid would accept it and simply leave the lens wherever the
        previous run parked it, giving a focus that silently varies run to
        run -- the exact failure this mode exists to remove.
        """
        if self.af_mode is AfMode.MANUAL and self.lens_position is None:
            msg = "camera_af_mode = 'manual' requires camera_lens_position (dioptres, 1/distance_m)."
            raise ValueError(msg)
        return self

    @model_validator(mode="after")
    def _fixed_gains_and_a_preset_are_contradictory(self) -> Self:
        """Reject fixed AWB gains combined with a non-auto preset.

        rpicam-vid silently drops --awb when --awbgains is present, so the
        config would read as though a preset were in force while the gains
        actually decided the colour.
        """
        if self.awb_gains is not None and self.awb_mode is not AwbMode.AUTO:
            msg = (
                f"camera_awb_gains overrides camera_awb_mode, so setting both is contradictory: "
                f"drop one (awb_mode is currently '{self.awb_mode.value}')."
            )
            raise ValueError(msg)
        return self


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
        # rpicam-vid takes --hflip/--vflip as flags, and --rotation only
        # accepts 0 or 180 on this pipeline.
        hflip, vflip = self.config.resolved_flips()
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
            "--exposure",
            self.config.exposure_mode.rpicam_value,
        ]
        if hflip:
            cmd.append("--hflip")
        if vflip:
            cmd.append("--vflip")
        # Fixed gains and a named preset are alternatives, not layers:
        # rpicam-vid ignores --awb once --awbgains is given.
        if self.config.awb_gains is not None:
            red, blue = self.config.awb_gains
            cmd += ["--awbgains", f"{red},{blue}"]
        else:
            cmd += ["--awb", self.config.awb_mode.value]
        if self.config.exposure_time_us is None:
            # Metering and exposure compensation only steer auto-exposure, so
            # emitting them alongside a fixed --shutter would just be noise.
            cmd += ["--metering", self.config.metering_mode.value, "--ev", str(self.config.exposure_value)]
        if self.config.flicker_period_us is not None:
            cmd += ["--flicker-period", f"{self.config.flicker_period_us}us"]
        cmd += [
            "--denoise",
            self.config.noise_reduction_mode.rpicam_value,
            "--sharpness",
            str(self.config.sharpness),
            "--autofocus-mode",
            self.config.af_mode.value,
        ]
        if self.config.af_mode is AfMode.MANUAL:
            if self.config.lens_position is not None:
                cmd += ["--lens-position", str(self.config.lens_position)]
        else:
            cmd += ["--autofocus-speed", self.config.af_speed.value]
        if self.config.exposure_time_us is not None:
            cmd += ["--shutter", str(self.config.exposure_time_us), "--gain", str(self.config.analogue_gain)]
        return cmd

    def connect(self) -> None:
        """Start the capture process."""
        cmd = self._command()
        log.info("Starting rpicam-vid", extra={DETAILS_KEY: {"cmd": " ".join(cmd)}})
        self._process = subprocess.Popen(
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
        return self.config.get_resolution()

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
