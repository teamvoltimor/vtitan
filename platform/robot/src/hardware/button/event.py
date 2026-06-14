from enum import StrEnum


class ButtonEvent(StrEnum):
    """Button event types."""

    PRESSED = "pressed"
    RELEASED = "released"
    SHORT_PRESS = "short_press"
    LONG_PRESS = "long_press"
