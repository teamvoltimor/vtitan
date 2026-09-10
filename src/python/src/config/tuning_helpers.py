"""Shared tuning helpers.

Covers the ``tuning or NavigationTuning.load_default()`` pattern repeated
across every navigation/simulation component that accepts an optional
``NavigationTuning`` override, and the ``*Context`` base class built on it.
"""

from __future__ import annotations

from dataclasses import replace
from typing import TYPE_CHECKING, ClassVar, Protocol, Self

from shared.config.navigation_tuning import NavigationTuning

if TYPE_CHECKING:
    from collections.abc import Mapping


def get_tuning(tuning: NavigationTuning | None) -> NavigationTuning:
    """Return ``tuning`` unchanged, or the checked-in default if ``None``."""
    return tuning or NavigationTuning.load_default()


def tuning_with_overrides(
    changes: Mapping[str, object],
    *,
    group: str = "corridor_follower",
    base: NavigationTuning | None = None,
) -> NavigationTuning:
    """Shipped tuning with only the named fields of one group moved.

    ``NavigationTuning`` is a frozen dataclass of pydantic groups, so the group
    is revalidated and the aggregate replaced rather than mutated. Values may
    arrive as strings (they usually come from a CLI); pydantic coerces them and
    rejects what it cannot, which is why they are not pre-parsed here -- a
    hand-rolled parse would have to duplicate each field's declared type and
    would drift from it.

    Deliberately NOT routed through ``NavigationTuning``'s mapping loader,
    which resets every group the mapping omits to code defaults: a partial
    override there silently discards the entire shipped profile.

    Raises:
        ValueError: If ``group`` is not a tuning group, or a field is not one
            of its members. Silently ignoring an unknown name would report a
            sweep of a constant that was never applied.
    """
    resolved = get_tuning(base)
    if not changes:
        return resolved
    if not hasattr(resolved, group):
        message = f"no such tuning group: {group}"
        raise ValueError(message)
    current = getattr(resolved, group)
    for field in changes:
        if not hasattr(current, field):
            message = f"no such {group} field: {field}"
            raise ValueError(message)
    updated = current.model_validate({**current.model_dump(), **changes})
    return replace(resolved, **{group: updated})


class _ConstantsFromTuning(Protocol):
    @classmethod
    def from_tuning(cls, tuning: NavigationTuning) -> Self: ...


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
        self.constants: C = self._constants_cls.from_tuning(self.tuning)
