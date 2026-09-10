"""Episode splitting shared across diag scripts.

diag_bag_escape_aftermath and diag_bag_escape_reverse maintained drifting
copies of the same state machine: walk nav_debug ticks, mark a reversing
episode where ``maneuver_speed_mps`` goes negative, and reduce each one to
its duration and the yaw turned over it (the atan2-wrapped wrap-around
expression was verbatim in both). diag_bag_proximity separately copied a
gap-based grouping of flagged ticks. Only the per-episode EXTRAS differ
between the scripts, so the split itself lives here and the callers keep
the part that is genuinely theirs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence

__all__ = ["ReversingSpan", "reversing_spans", "split_by_gap"]


@dataclass(frozen=True, slots=True)
class ReversingSpan[T]:
    """One contiguous reversing-maneuver episode.

    ``t_end``/``end_x``/``end_y`` describe the first tick AFTER the episode
    (the release sample -- where the clearance the escape left behind is
    read); ``t_start``/``t_last`` bound the reversing ticks themselves.
    """

    t_start: float
    t_last: float
    t_end: float
    end_x: float
    end_y: float
    turned_deg: float
    end_row: T
    """The terminating tick's snapshot, for per-script extras (clearance...)."""


def turned_deg(yaw0: float, yaw1: float) -> float:
    """Magnitude of the yaw change between two headings, wrapped to [-180, 180]."""
    return math.degrees(abs(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))))


def reversing_spans[T](rows: Sequence[tuple[float, T]]) -> list[ReversingSpan[T]]:
    """Split sorted ``(t, snapshot)`` nav_debug rows into reversing episodes.

    A tick reverses when ``maneuver_speed_mps`` is present and negative; the
    episode also absorbs consecutive reversing ticks and ends on the first
    non-reversing one, whose snapshot becomes ``end_row``.
    """
    spans: list[ReversingSpan[T]] = []
    start: tuple[float, float, float, float] | None = None  # t, yaw, x, y
    last: tuple[float, float, float, float] | None = None
    for rel, d in rows:
        speed = d.maneuver_speed_mps
        reversing = speed is not None and speed < 0.0
        yaw = d.pose_yaw if d.pose_yaw is not None else 0.0
        x = d.pose_x if d.pose_x is not None else 0.0
        y = d.pose_y if d.pose_y is not None else 0.0
        if reversing and start is None:
            start = (rel, yaw, x, y)
        if reversing:
            last = (rel, yaw, x, y)
        elif start is not None and last is not None:
            spans.append(
                ReversingSpan(
                    t_start=start[0],
                    t_last=last[0],
                    t_end=rel,
                    end_x=x,
                    end_y=y,
                    turned_deg=turned_deg(start[1], last[1]),
                    end_row=d,
                )
            )
            start, last = None, None
    return spans


def split_by_gap[T](items: Sequence[tuple[float, T]], gap_s: float = 0.5) -> list[list[tuple[float, T]]]:
    """Group consecutive ``(t, x)`` items whose timestamps are less than ``gap_s`` apart."""
    groups: list[list[tuple[float, T]]] = []
    for item in items:
        if groups and item[0] - groups[-1][-1][0] <= gap_s:
            groups[-1].append(item)
        else:
            groups.append([item])
    return groups
