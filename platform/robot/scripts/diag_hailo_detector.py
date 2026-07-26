"""Verify the compiled detector against the real NPU, on real images.

Two things about the Hailo path cannot be settled off-device, and both fail
silently rather than loudly:

* **The NMS tensor layout.** ``HAILO NMS BY CLASS`` carries the class
  positionally, and HailoRT's exact packing differs from the SDK emulator's.
  Reading it wrong yields a coordinate where a confidence belongs.
* **The channel order.** The model wants RGB; OpenCV and some Picamera2 formats
  give BGR. Swapping them leaves green detections intact while collapsing red,
  which reads as a bad model rather than bad plumbing.

This runs the real HEF on the real device over images of known colour and
reports both, so each is answered by evidence instead of assumption.

Usage (from ``platform/robot``, with PYTHONPATH=.)::

    pixi run -e vision python scripts/diag_hailo_detector.py IMAGE [IMAGE ...]

Images are matched to an expected colour by filename: a name containing
``red``/``green``/``magenta`` sets the expectation for that image.
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.hardware.hailo.base import Config as HailoConfig
from src.hardware.hailo.hailo_8.driver import Driver
from src.hardware.hailo.inferences import NmsFormatError, iter_nms_by_class
from src.vision.detector import DEFAULT_CLASS_TO_COLOR

COLOURS = ("red", "green", "magenta")


def expected_colour(path: Path) -> str | None:
    """Infer the expected colour from *path*'s filename, if it names one."""
    name = path.name.lower()
    for colour in COLOURS:
        if colour in name:
            return colour
    return None


def describe(raw: object) -> str:
    """Summarise the driver's raw output without assuming a layout."""
    if isinstance(raw, (list, tuple)):
        inner = ", ".join(f"{np.asarray(item).shape}" for item in raw[:4])
        return f"{type(raw).__name__}[{len(raw)}] of {inner}"
    array = np.asarray(raw)
    return f"ndarray shape={array.shape} dtype={array.dtype}"


def best_per_colour(raw: object, min_conf: float) -> dict[str, float]:
    """Return the highest confidence seen for each colour."""
    best: dict[str, float] = {}
    for class_id, confidence, _ in iter_nms_by_class(raw):
        if confidence < min_conf:
            continue
        colour = DEFAULT_CLASS_TO_COLOR.get(class_id)
        if colour is None:
            continue
        name = str(colour)
        best[name] = max(best.get(name, 0.0), confidence)
    return best


def main() -> int:
    """Run the probe over every image given on the command line."""
    images = [Path(a) for a in sys.argv[1:]]
    if not images:
        print("usage: diag_hailo_detector.py IMAGE [IMAGE ...]")
        return 2

    config = HailoConfig()
    print(f"model      : {config.model_path}")
    print(f"class map  : { ({k: str(v) for k, v in DEFAULT_CLASS_TO_COLOR.items()}) }")
    print(f"driver map : {config.class_map}")

    driver = Driver(config)
    driver.connect()
    driver.load_model(config.model_path)
    try:
        return _probe(driver, config, images)
    finally:
        driver.close()


def _probe(driver: Driver, config: HailoConfig, images: list[Path]) -> int:
    """Run every image through the device and report."""
    height, width = driver.get_input_shape()[0], driver.get_input_shape()[1]
    print(f"input shape: {driver.get_input_shape()}")

    layout_reported = False
    agree = disagree = 0

    for path in images:
        bgr = cv2.imread(str(path))
        if bgr is None:
            print(f"!! unreadable: {path}")
            continue
        resized_bgr = cv2.resize(bgr, (width, height))
        resized_rgb = cv2.cvtColor(resized_bgr, cv2.COLOR_BGR2RGB)

        results = {}
        for order, frame in (("RGB", resized_rgb), ("BGR", resized_bgr)):
            raw = driver.infer(frame)
            if not layout_reported:
                print(f"raw output : {describe(raw)}")
                layout_reported = True
            try:
                results[order] = best_per_colour(raw, config.min_confidence)
            except NmsFormatError as err:
                print(f"!! {err}")
                return 1

        want = expected_colour(path)
        rgb_top = max(results["RGB"], key=results["RGB"].get, default="-")
        bgr_top = max(results["BGR"], key=results["BGR"].get, default="-")
        mark = "" if want is None else ("  OK" if rgb_top == want else "  <-- RGB mismatch")
        if want is not None:
            agree += rgb_top == want
            disagree += rgb_top != want
        print(f"{path.name[:44]:46s} expect={want or '?':8s} RGB->{rgb_top:8s} BGR->{bgr_top:8s}{mark}")

    print(f"\nRGB agrees with filename on {agree}/{agree + disagree} images")
    if disagree > agree:
        print("Feed BGR instead: the RGB reading disagrees with the labels more often than not.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
