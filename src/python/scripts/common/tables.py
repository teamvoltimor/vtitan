"""Table formatting shared by every diagnostic, bag-replay or not.

Extracted from ``bag_io``, which is where these two started. That put the only
table formatter in the repo behind ``rosbag2_py``, ``rclpy`` and
``sensor_msgs``: six simulation scripts and one hardware probe were importing a
bag reader to print a grid, and a new sim script had no cheap way to match the
house style, so it hand-rolled f-string columns instead -- which is the very
thing ``print_table`` exists to stop.

Nothing here reads a bag, so nothing here needs ROS.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from tabulate import tabulate

if TYPE_CHECKING:
    from collections.abc import Sequence


def fmt_optional(value: float | None, spec: str = ".3f") -> str:
    """Format an optional float for a table cell.

    Anything non-numeric prints as "None", right-justified to the width a
    real number would occupy under `spec` -- so a column of these lines up
    with a column of `format(x, spec)` even where nothing else pads it.
    """
    if isinstance(value, (int, float)):
        return format(value, spec)
    return "None".rjust(len(format(0.0, spec)))


def print_table(rows: Sequence[Sequence[object]], headers: Sequence[str], *, floatfmt: str | Sequence[str] = ".3f") -> None:
    """Print `rows` as a GitHub-flavored markdown table.

    Picked over hand-aligned f-string columns (the pattern every diag_bag_*.py
    script used before this) for two audiences at once: `|`-delimited column
    boundaries are unambiguous to a human skimming a terminal AND to an LLM
    reading the transcript, where neither has to infer where one column ends
    and the next begins from a run of whitespace of uncertain width -- exactly
    the class of bug fmt_optional's width-derivation was hand-patching one
    call site at a time. `None` cells print as the literal string "None"
    (tabulate's own missing-value convention would otherwise print an empty
    cell, which reads as a formatting glitch rather than an absent reading).
    """
    print(tabulate(rows, headers=headers, tablefmt="github", floatfmt=floatfmt, missingval="None"))
