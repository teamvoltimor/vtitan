"""Shared tuning helpers.

Covers the ``tuning or NavigationTuning.load_default()`` pattern repeated
across every navigation/simulation component that accepts an optional
``NavigationTuning`` override, and the ``*Context`` base class built on it.
"""

from __future__ import annotations

from typing import ClassVar, Protocol, cast

from shared.config.navigation_tuning import NavigationTuning


def get_tuning(tuning: NavigationTuning | None) -> NavigationTuning:
    """Return ``tuning`` unchanged, or the checked-in default if ``None``."""
    return tuning or NavigationTuning.load_default()


class _ConstantsFromTuning(Protocol):
    @classmethod
    def from_tuning(cls, tuning: NavigationTuning) -> _ConstantsFromTuning: ...


class TuningContext[C: _ConstantsFromTuning]:
    """Base for the repeated ``self.tuning`` / ``self.constants`` wiring.

    Every ``*Context`` class (``SignRouterContext``, ``ParkingContext``,
    ``KinematicsContext``, ``SimulatorContext``, ``WallHeadingContext``,
    ``EstimatorContext``) resolved tuning and built its constants dataclass in
    an identical two-line ``__init__``. Subclass and set ``_constants_cls`` to
    the frozen dataclass (with a ``from_tuning`` classmethod) to inherit it::

        class FooContext(TuningContext[_FooConstants]):
            _constants_cls = _FooConstants
    """

    _constants_cls: ClassVar[type[C]]

    def __init__(self, tuning: NavigationTuning | None = None) -> None:
        self.tuning = get_tuning(tuning)
        self.constants: C = cast("C", self._constants_cls.from_tuning(self.tuning))
