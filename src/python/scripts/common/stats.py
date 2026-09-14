"""Percentile/median and nearest-by-timestamp lookups shared across diag scripts.

Several diag scripts independently hand-rolled sorted-list percentile math
and binary-search-for-nearest-timestamp pairing, with small correctness
drift between copies (nearest-rank vs. linear-interpolated percentile, one
nearest-timestamp lookup missing a tolerance guard entirely). These use
numpy/bisect instead of reimplementing the search/interpolation.
"""

from __future__ import annotations

import bisect
import math
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Iterable, Sequence


def _as_array(values: Iterable[float]) -> np.ndarray:
    """Coerce any iterable to a float array.

    These replaced the ``statistics`` functions, which accepted ANY iterable,
    so call sites legitimately pass views like ``dict.values()``. numpy rejects
    those, and the failure surfaced as a ``TypeError`` raised deep inside
    ``np.median`` rather than at the call. Materialising here also makes the
    two-percentile helpers safe against generators, which they were not.
    """
    return np.asarray(list(values), dtype=float)


def percentile(values: Iterable[float], q: float) -> float:
    """Linear-interpolated percentile ``q`` (0-1) of ``values``."""
    arr = _as_array(values)
    if arr.size == 0:
        return math.nan
    return float(np.percentile(arr, q * 100))


def fmt_p50_p90(values: Iterable[float], unit: str = "m") -> str:
    """`p50 / p90` for a sample, or a dash placeholder when it is empty."""
    arr = _as_array(values)
    if arr.size == 0:
        return "  --  "
    return f"{percentile(arr, 0.5):.2f} / {percentile(arr, 0.9):.2f} {unit}"


def median(values: Iterable[float]) -> float:
    """Median of ``values``."""
    arr = _as_array(values)
    if arr.size == 0:
        return math.nan
    return float(np.median(arr))


def fmean(values: Iterable[float]) -> float:
    """Arithmetic mean of ``values`` (``statistics.fmean``'s replacement)."""
    arr = _as_array(values)
    if arr.size == 0:
        return math.nan
    return float(np.mean(arr))


def mean(values: Iterable[float]) -> float:
    """Alias of :func:`fmean`, matching ``statistics.mean``'s spelling."""
    return fmean(values)


def pstdev(values: Iterable[float]) -> float:
    """Population standard deviation of ``values`` (``statistics.pstdev``)."""
    arr = _as_array(values)
    if arr.size == 0:
        return math.nan
    return float(np.std(arr))


def nearest_by_time[T](
    series: Sequence[tuple[float, T]],
    times: Sequence[float],
    t: float,
    tolerance: float | None = None,
) -> T | None:
    """The record in ``series`` (sorted by ``times``) closest to ``t``.

    ``times`` is taken separately rather than derived from ``series`` on
    every call so a caller pairing many timestamps against the same series
    (e.g. one per scan in a bag) can extract it once outside the loop.
    Returns None if ``series`` is empty, or if ``tolerance`` is given and the
    closest match falls outside it.
    """
    if not series:
        return None
    idx = bisect.bisect_left(times, t)
    candidates = [i for i in (idx - 1, idx) if 0 <= i < len(series)]
    best = min(candidates, key=lambda i: abs(times[i] - t))
    if tolerance is not None and abs(times[best] - t) > tolerance:
        return None
    return series[best][1]
