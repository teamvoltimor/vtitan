from pydantic import BaseModel


class Config(BaseModel):
    """Configuration for button driver."""

    pull_up: bool
    """Whether to use internal pull-up resistor."""

    debounce_ms: int
    """Debounce delay in milliseconds."""

    long_press_threshold_sec: float
    """Duration threshold for long press detection in seconds."""

    shutdown_press_threshold_sec: float
    """Duration threshold for the clean-shutdown hold, in seconds.

    Must sit well above long_press_threshold_sec. The gap is the margin an
    operator has to hold the button in an emergency without accidentally
    asking for a power-off instead of a stop -- and nobody counts seconds
    during one.
    """
