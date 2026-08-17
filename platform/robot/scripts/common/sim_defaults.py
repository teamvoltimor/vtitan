"""Shared step budgets for diag scripts driving the closed-loop sim."""

from __future__ import annotations

OBSTACLES_MAX_STEPS = 6000
"""The Obstacles diag harness's own run budget: OBSTACLES_MAX_STEPS * CONTROL_DT = 300s.

Not the same as CompetitionSpecs.ROUND_TIME_LIMIT_S (180s) -- see
diag_sign_sweep.py's SweepResult.sim_time_s docstring for why the gap
matters: a run can finish under this budget, be counted a pass here, and
still be stopped by the judges.
"""
