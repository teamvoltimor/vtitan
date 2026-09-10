"""Value banding shared across diag scripts.

Three diag scripts hand-rolled the same "bin a float against a ladder of
edges" idea, with the two usual drift shapes: the half-open ``[lo, hi)``
copy-pair (contact_bearing vs creep_stall, differing only in label format
and out-of-range fallback) and a cumulative-threshold copy (review). The
bin decision and the label live separately so callers keep their own
display format without one drifting the bin boundaries again.
"""

from __future__ import annotations

from itertools import pairwise
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

__all__ = ["band_label", "half_open_bin", "threshold_band"]


def half_open_bin(v: float, edges: Sequence[float]) -> tuple[float, float] | None:
    """The ``(lo, hi)`` half-open ``[lo, hi)`` band containing ``v``, or None outside."""
    for lo, hi in pairwise(edges):
        if lo <= v < hi:
            return (lo, hi)
    return None


def band_label(
    v: float,
    edges: Sequence[float],
    label: Callable[[float, float], str],
) -> str | None:
    """The band label for ``v``, formatted by ``label(lo, hi)``, or None outside."""
    span = half_open_bin(v, edges)
    return None if span is None else label(*span)


def threshold_band(v: float, edges: Sequence[float]) -> str:
    """Cumulative band: the first threshold ``v`` falls under, or the top edge's ``>=``."""
    for e in edges:
        if v < e:
            return f"<{e}"
    return f">={edges[-1]}"
