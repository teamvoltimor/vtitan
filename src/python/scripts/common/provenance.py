"""Hardware profile and tree state, stamped on every sweep.

A corpus number is not a result unless what produced it is written down beside
it. This was learned the expensive way: a 122/68 routing-vs-tracking split
chose an investigation's target for a day, and re-running the control at the
very commit that reported it reproduced 91/95 instead -- near-even, a different
conclusion entirely. The difference was environmental. The worktree it ran in
had already been auto-removed, which left no way to recover which profile or
working-tree edits produced it.

``VTITAN_HARDWARE_PROFILE`` in particular is not optional context: the tuning
resolves speed tiers and the steering limit from it, so two runs under
different profiles are not comparable even at an identical revision.
"""

from __future__ import annotations

import os
import subprocess


def environment() -> str:
    """One-line ``profile=... rev=...`` stamp for the current run."""
    profile = os.environ.get("VTITAN_HARDWARE_PROFILE", "UNSET")
    try:
        revision = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, check=True
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "status", "--porcelain"], capture_output=True, text=True, check=True
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        revision, dirty = "unknown", ""
    return f"profile={profile}  rev={revision}{'+dirty' if dirty else ''}"
