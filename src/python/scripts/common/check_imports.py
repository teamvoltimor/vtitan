"""Import every diagnostic script, to prove the post-move import paths resolve.

Loading a module runs its top-level -- the sys.path insert and every import --
without running main(), which stays behind its ``__name__ == "__main__"`` guard.
That is exactly the surface a folder move breaks.

Usage:
    pixi run -e dev python scripts/common/check_imports.py
"""

from __future__ import annotations

import importlib.util
import sys
import traceback
from pathlib import Path

SELF = Path(__file__).resolve()
SCRIPTS = SELF.parents[1]
sys.path.insert(0, str(SCRIPTS.parent))

failures: list[tuple[str, str]] = []
checked = 0
for folder in ("bag", "sim", "hardware", "common"):
    for path in sorted((SCRIPTS / folder).glob("*.py")):
        if path.name.startswith("__"):
            continue
        # This checker does its work at module level, so importing it would rerun
        # the whole sweep -- once per level, until the recursion limit stops it.
        if path.resolve() == SELF:
            continue
        checked += 1
        name = f"_check_{folder}_{path.stem}"
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        # @dataclass resolves string annotations through sys.modules[cls.__module__],
        # so a module that is executed but never registered blows up there rather
        # than anywhere the move could have broken.
        sys.modules[name] = module
        try:
            spec.loader.exec_module(module)
        except Exception:  # noqa: BLE001 - reporting every failure, not handling one
            failures.append((f"{folder}/{path.name}", traceback.format_exc().strip().splitlines()[-1]))

print(f"{checked} scripts imported, {len(failures)} failed")
for name, err in failures:
    print(f"  FAIL {name}: {err}")
