from pydantic import BaseModel


class Config(BaseModel):
    """Configuration for button driver."""

    pull_up: bool
    """Whether to use internal pull-up resistor."""

    debounce_ms: int
    """Debounce delay in milliseconds."""

    long_press_threshold_sec: float
    """Duration threshold for long press detection in seconds."""
