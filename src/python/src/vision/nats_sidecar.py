"""Standalone, ROS2-free camera-detections publisher for the Go navigator.

src/ros2/vision/node.py's actual vision logic (camera capture, the YOLO/
Hailo detector) has no rclpy dependency at all -- only its thin ROS2 Node
wrapper does. This reuses that same core directly, publishing raw
detections over NATS instead of a ROS2 topic, so the Go stack (which talks
NATS+protobuf exclusively and has no ROS2 dependency anywhere else) can
consume them without pulling in rclpy/DDS for a component that does not
otherwise need either.

Geometry (bbox -> world-frame sign position) is NOT done here: that stays
on the Go side (internal/nav/signrouter's DetectionToObservation, already a
1:1 port of sign_discovery.py's pinhole math), using Go's own already-local
pose estimate. This script's only job is "detect and publish raw boxes",
mirroring cmd/lidar-node's "read hardware, publish raw" shape.

Usage:
    pixi run -e vision python -m src.vision.nats_sidecar --backend hailo
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import TYPE_CHECKING

import nats
from shared.domain.models import SignColor

from src.vision import VisionBackend, create_detector

if TYPE_CHECKING:
    from shared.domain.models import Detection

    from src.hardware.camera.base import Driver as CameraDriver
    from src.vision.detector import DetectorBase

logger = logging.getLogger(__name__)

# buf's Python plugin generates gen/vtitan/vision/v1/detections_pb2.py rooted
# so that "vtitan.vision.v1.detections_pb2" is a top-level import -- so
# src/gen itself, not just src, has to be on sys.path. Matches
# command_channel.py's own _GEN_ROOT convention for the same reason.
_GEN_ROOT = Path(__file__).resolve().parents[2] / "gen"
if str(_GEN_ROOT) not in sys.path:
    sys.path.insert(0, str(_GEN_ROOT))

from vtitan.vision.v1 import detections_pb2  # noqa: E402

# Matches internal/schema/pb/vtitan/vision/v1/subjects.go's DetectionsSubject.
DETECTIONS_SUBJECT = "vtitan.vision.v1.detections"

# Matches internal/transport/nats.DefaultDevURL/EnvURLVar.
_DEFAULT_NATS_URL = "nats://127.0.0.1:4222"
_NATS_URL_ENV_VAR = "VTITAN_NATS_URL"

_SIGN_COLOR_TO_PROTO = {
    SignColor.RED: detections_pb2.SIGN_COLOR_RED,
    SignColor.GREEN: detections_pb2.SIGN_COLOR_GREEN,
    SignColor.MAGENTA: detections_pb2.SIGN_COLOR_MAGENTA,
    SignColor.UNKNOWN: detections_pb2.SIGN_COLOR_UNSPECIFIED,
}


def _open_camera() -> CameraDriver:
    """Open the CSI camera in-process, matching VisionNode._start_direct_capture.

    Prefers Picamera2, falls back to the rpicam CLI: picamera2 is absent from
    every pixi env on the robot and cannot be installed into the conda-managed
    Python build its bindings would have to match.
    """
    try:
        from src.hardware.camera.rpi.camera_module_3.driver import (
            Config as PicamConfig,
            Driver as PicamDriver,
        )

        camera: CameraDriver = PicamDriver(PicamConfig())
        backend = "picamera2"
    except ImportError:
        from src.hardware.camera.rpicam.driver import (
            Config as RpicamConfig,
            Driver as RpicamDriver,
        )

        camera = RpicamDriver(RpicamConfig())
        backend = "rpicam-cli"
    camera.connect()
    size = camera.get_resolution()
    logger.info("camera opened via %s at %dx%d", backend, size.width_px, size.height_px)
    return camera


def _to_proto(det: Detection) -> detections_pb2.Detection:
    """Convert a Detection dataclass to its wire message, 1:1.

    See detections.proto's own comment for why every derivable field
    (centroid, width/height, area) is carried rather than recomputed
    downstream.
    """
    proto = detections_pb2.Detection()
    proto.class_name = _SIGN_COLOR_TO_PROTO.get(det.class_name, detections_pb2.SIGN_COLOR_UNSPECIFIED)
    proto.confidence = det.confidence
    proto.bbox.x_min, proto.bbox.y_min, proto.bbox.x_max, proto.bbox.y_max = det.bbox
    proto.x = det.x
    proto.y = det.y
    proto.width = det.width
    proto.height = det.height
    proto.area = det.area
    return proto


def _grab_detections(camera: CameraDriver, detector: DetectorBase) -> list[Detection]:
    """Capture one frame and run detection over it; runs off the event loop.

    Both calls are synchronous and CPU/hardware-bound (frame grab plus YOLO or
    Hailo NPU inference), so calling them directly inside ``run`` would stall
    the loop -- and with it NATS keepalive -- for the whole inference.
    """
    frame = camera.capture_frame().frame
    return detector.detect(camera.to_rgb(frame))


async def run(nats_url: str, backend: str, fps: float) -> None:
    """Connect to NATS, open the camera/detector, and publish Detections at ~fps until cancelled."""
    nc = await nats.connect(nats_url)
    logger.info("connected to NATS at %s", nats_url)

    camera = _open_camera()
    detector: DetectorBase = create_detector(VisionBackend(backend))
    # For YOLO __enter__ is a no-op; calling it unconditionally is safe --
    # matches VisionNode's own hasattr guard, since only HailoDetector needs
    # the device handle opened/closed around the session.
    if hasattr(detector, "__enter__"):
        detector.__enter__()

    interval_s = 1.0 / max(fps, 1.0)
    # Single dedicated worker, not the shared default executor: the detector is
    # opened on this thread (above) and every frame runs on the same one, so a
    # backend with thread affinity (HailoRT) never sees a different thread.
    loop = asyncio.get_running_loop()
    executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vision-detect")
    try:
        while True:
            start = time.monotonic()
            detections = await loop.run_in_executor(executor, _grab_detections, camera, detector)

            msg = detections_pb2.Detections()
            msg.stamp.GetCurrentTime()
            msg.frame_id = "camera_link"
            msg.detections.extend(_to_proto(det) for det in detections)
            await nc.publish(DETECTIONS_SUBJECT, msg.SerializeToString())

            elapsed = time.monotonic() - start
            await asyncio.sleep(max(0.0, interval_s - elapsed))
    finally:
        executor.shutdown(wait=True)
        if hasattr(detector, "__exit__"):
            detector.__exit__(None, None, None)
        camera.close()
        await nc.drain()


def main() -> None:
    """Parse CLI args and run the publish loop until interrupted."""
    logging.basicConfig(level=logging.INFO)
    parser = argparse.ArgumentParser(
        description="Publish camera detections on vtitan.vision.v1.detections for a Go navigator to consume."
    )
    parser.add_argument(
        "--nats-url", default=None, help=f"nats-server URL (default: ${_NATS_URL_ENV_VAR} or {_DEFAULT_NATS_URL})"
    )
    parser.add_argument("--backend", choices=["yolo", "hailo"], default="hailo")
    parser.add_argument("--fps", type=float, default=15.0)
    args = parser.parse_args()

    nats_url = args.nats_url or os.environ.get(_NATS_URL_ENV_VAR) or _DEFAULT_NATS_URL
    asyncio.run(run(nats_url, args.backend, args.fps))


if __name__ == "__main__":
    main()
