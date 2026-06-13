"""Synthetic planners — no ROS, no GPU.

These fabricate plausible trajectories with planner-specific characteristics (speed,
success rate, smoothness, path length). They exist so the *entire* pipeline
(generate -> run -> metrics -> report) can run and be unit-tested offline, and so the
analysis/plotting code can be exercised on realistic data before any real planner is
wired up. They are also a worked example of the adapter interface.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from ..scenario import Obstacle, Query
from .base import PlannerAdapter, PlanResult, register


@dataclass
class _Profile:
    success_prob: float
    time_mean: float
    time_std: float
    n_waypoints: int
    detour: float  # how far the path bulges off the straight line (rad)
    noise: float  # per-waypoint jitter (rad) -> affects smoothness
    requires_warmup: bool = False


# Caricatures of real planners, useful for demo/plots. Not physically calibrated.
PROFILES = {
    "synthetic:rrtconnect": _Profile(0.97, 0.08, 0.04, 24, 0.9, 0.05),
    "synthetic:rrtstar": _Profile(0.90, 0.9, 0.3, 30, 0.5, 0.02),
    "synthetic:bitstar": _Profile(0.93, 1.4, 0.4, 36, 0.35, 0.01),
    "synthetic:prm": _Profile(0.85, 0.5, 0.2, 28, 0.7, 0.04),
    "synthetic:curobo": _Profile(0.99, 0.05, 0.02, 40, 0.25, 0.005, requires_warmup=True),
}


class SyntheticPlanner(PlannerAdapter):
    def __init__(self, name: str, profile: _Profile):
        self.name = name
        self._p = profile
        self.requires_warmup = profile.requires_warmup
        self._n_joints = 6

    def setup(self, robot_name: str, obstacles: List[Obstacle]) -> None:  # noqa: D401
        # Synthetic planner ignores the world; metrics still score the fake path against
        # the real world afterwards, which is exactly the point of the offline demo.
        self._obstacles = obstacles

    def plan(self, query: Query, timeout: float, seed: int, run: int = 0) -> PlanResult:
        rng = np.random.default_rng((seed, query.id, run, hash(self.name) & 0xFFFF))
        p = self._p
        start = np.asarray(query.start, dtype=float)
        goal = np.asarray(query.goal_joint if query.goal_joint is not None else query.start, dtype=float)

        plan_time = float(max(1e-3, rng.normal(p.time_mean, p.time_std)))
        success = bool(rng.random() < p.success_prob) and plan_time <= timeout

        result = PlanResult(
            planner=self.name, scenario="",  # runner fills in the scenario name
            query_id=query.id, run=run, seed=seed,
            success=success, planning_time_s=plan_time, solve_time_s=plan_time,
            meta={"synthetic": True, "profile": self.name},
        )
        if not success:
            result.error = None if plan_time <= timeout else "timeout"
            return result

        n = p.n_waypoints
        ts = np.linspace(0.0, 1.0, n)
        traj = start[None, :] + np.outer(ts, goal - start)
        # smooth bulge off the straight line
        bump_dir = rng.normal(size=self._n_joints)
        bump_dir /= np.linalg.norm(bump_dir) + 1e-9
        traj += np.outer(np.sin(np.pi * ts) * p.detour, bump_dir)
        # interior jitter -> drives the smoothness metric
        jitter = rng.normal(scale=p.noise, size=(n, self._n_joints))
        jitter[0] = 0.0
        jitter[-1] = 0.0
        traj += jitter

        result.trajectory = traj.tolist()
        result.time_from_start = (ts * (0.1 * n)).tolist()
        return result


# Register one factory per profile.
for _name, _profile in PROFILES.items():
    register(_name, (lambda n, pr: (lambda: SyntheticPlanner(n, pr)))(_name, _profile))
