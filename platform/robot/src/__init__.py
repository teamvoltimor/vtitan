"""src namespace package — extends path to platform/simulation/src for shared config."""

from __future__ import annotations

from pathlib import Path

_SIM_SRC = Path(__file__).resolve().parent.parent.parent / "simulation" / "src"
if _SIM_SRC.is_dir() and str(_SIM_SRC) not in __path__:
    __path__.append(str(_SIM_SRC))
