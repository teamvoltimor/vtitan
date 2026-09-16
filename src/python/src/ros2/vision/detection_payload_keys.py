"""Shared keys for detection JSON payload sent from vision node to hardware gateway."""

from shared.domain.models import Detection, SignColor

CLASS_NAME_KEY = "class_name"
CONFIDENCE_KEY = "confidence"
BBOX_KEY = "bbox"
X_KEY = "x"
Y_KEY = "y"
WIDTH_KEY = "width"
HEIGHT_KEY = "height"
AREA_KEY = "area"
CAPTURED_AT_KEY = "captured_at"
"""Clock reading when the FRAME WAS GRABBED, not when the box was published.

The consumer converts a bearing into a world position using the pose the
camera saw from; pairing a detection with the pose at RECEIPT instead is late
by the vision latency and leaves a bearing residual. See
``adr:0058-sign-discovery-range-and-barrier-belief``.

Absent from payloads written by an older vision_node, so every reader must
treat it as optional and fall back to ``VISION_LATENCY_S``."""


def parse_detection(d: dict) -> Detection | None:
    """Parse one JSON dict from vision_node's /vision/detections payload.

    Returns ``None`` for a class_name that isn't a valid SignColor -- e.g. a
    payload from a future vision_node build with a class this build doesn't
    know about -- rather than raising or silently defaulting to some other
    color (the latter is exactly the bug detection_to_observation had before
    class_name was typed).

    Shared by :class:`~src.ros2.navigation.ros2_hardware_gateway.ROS2HardwareGateway`
    and ``telemetry_bridge_node``, which used to hand-roll two independent,
    identical copies of this parse.
    """
    try:
        color = SignColor(str(d.get(CLASS_NAME_KEY)))
    except ValueError:
        return None
    return Detection(
        color=color,
        confidence=d.get(CONFIDENCE_KEY, 0.0),
        bbox=tuple(d.get(BBOX_KEY, (0.0, 0.0, 0.0, 0.0))),
        x=d.get(X_KEY, 0.0),
        y=d.get(Y_KEY, 0.0),
        width=d.get(WIDTH_KEY, 0.0),
        height=d.get(HEIGHT_KEY, 0.0),
        area=d.get(AREA_KEY, 0.0),
    )
