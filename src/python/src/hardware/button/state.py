from dataclasses import dataclass

from src.hardware.button.event import ButtonEvent


@dataclass(frozen=True, slots=True)
class ButtonState:
    """Current button state data."""

    is_pressed: bool
    """Whether the button is currently pressed."""

    press_duration: float
    """Duration of current press in seconds."""

    last_event: ButtonEvent | None
    """Last detected button event."""
