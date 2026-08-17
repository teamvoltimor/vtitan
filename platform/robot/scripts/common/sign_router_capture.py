"""Shared save/patch/restore skeleton for probing ``SignRouter.deform_waypoint``.

Three diag scripts each monkeypatch ``deform_waypoint`` to capture its input
and output for a probe, then restore the original -- same mechanism every
time, different capture payload. This factors the mechanism only; each
script still supplies its own wrapper, since what it records is genuinely
different per probe.
"""

from __future__ import annotations

from contextlib import contextmanager
from typing import TYPE_CHECKING

import src.navigation.planning.sign_router as sign_router_module

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator


@contextmanager
def patched_deform_waypoint(make_wrapper: Callable[[Callable], Callable]) -> Iterator[None]:
    """Monkeypatch ``SignRouter.deform_waypoint`` for the duration of the block.

    ``make_wrapper(original_deform_waypoint)`` must return the replacement
    function -- it receives the real implementation so it can call through to
    it and capture the result, the way every existing probe already does.
    Restores the original on exit, even if the block raises.

    Example:
        def make_capturing(original):
            def capturing(router, waypoint, robot_pos, robot_yaw, *a, **kw):
                result = original(router, waypoint, robot_pos, robot_yaw, *a, **kw)
                last["deformed"] = result
                return result
            return capturing

        with patched_deform_waypoint(make_capturing):
            sim.run(...)
    """
    original = sign_router_module.SignRouter.deform_waypoint
    sign_router_module.SignRouter.deform_waypoint = make_wrapper(original)  # type: ignore[method-assign]
    try:
        yield
    finally:
        sign_router_module.SignRouter.deform_waypoint = original  # type: ignore[method-assign]
