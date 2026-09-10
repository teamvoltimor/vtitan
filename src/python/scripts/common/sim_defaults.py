"""Shared step budgets and paths for diag scripts driving the closed-loop sim."""

from __future__ import annotations

from pathlib import Path

CORPUS_DIR = Path(__file__).resolve().parents[2] / ".corpus" / "obstacles" / "scenarios"
"""Pinned-seed sweep corpus, built by ``task gen:corpus CHALLENGE=obstacles``.

Gitignored and regenerated rather than committed -- same generator, same seed,
identical output. Pass ``--corpus`` to use it instead of the committed 16.
Attributions must come from here: the 16 gave the right aggregate but two wrong
diagnoses (see docs/sign-avoidance-investigation.md).
"""

OBSTACLES_MAX_STEPS = 6000
"""The Obstacles diag harness's own run budget: OBSTACLES_MAX_STEPS * CONTROL_DT = 300s.

Not the same as CompetitionSpecs.ROUND_TIME_LIMIT_S (180s) -- see
diag_sign_sweep.py's SweepResult.sim_time_s docstring for why the gap
matters: a run can finish under this budget, be counted a pass here, and
still be stopped by the judges.
"""
