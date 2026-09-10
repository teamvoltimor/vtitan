from enum import StrEnum


class ButtonEvent(StrEnum):
    """Button event types."""

    PRESSED = "pressed"
    RELEASED = "released"
    SHORT_PRESS = "short_press"
    LONG_PRESS = "long_press"

    SHUTDOWN_PRESS = "shutdown_press"
    """A very long hold, meaning "power the robot down cleanly".

    Exists because the only safe shutdown today needs a laptop, a network and
    SSH -- none of which are guaranteed at a competition table, where cutting
    power to a running Zero risks the ext4 corruption safe-shutdown-zero.sh
    documents. This is the same intent as a physical power gesture.

    Never emitted while racing: see the state machine's handler. Between the
    long-press threshold and this one, a hold always means stop, never
    power-off.
    """
