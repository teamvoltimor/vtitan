"""Shared helpers used by every tuning-group module in this package."""

from __future__ import annotations

from shared.config.defaults_model import DefaultsModel


class TuningModel(DefaultsModel):
    """A navigation tuning group's shipped fallbacks.

    A package-local name for :class:`shared.config.defaults_model.DefaultsModel`
    so every tuning group in this package keeps a single, navigation-scoped
    base while the generic ``_DEFAULTS``/``_apply_defaults`` machinery lives
    in the shared config package.
    """
