"""TEMPLATE adapter / worked example for adding a planner (task 5).

This is a *real*, dependency-free baseline: it connects start and goal with a straight
line in joint space and reports success only if that line is collision-free. It is handy
as a lower bound in plots, and it is the canonical example to copy when integrating a new
planner.

To add YOUR planner:
  1. Copy this file to ``adapters/<your>_adapter.py``.
  2. Implement ``setup`` (build your planner from the robot + obstacles) and ``plan``
     (return a :class:`PlanResult` with a joint-space ``trajectory``; set
     ``time_from_start`` if you have timing). Defer heavy imports into ``setup``.
  3. ``register("<name>", lambda: YourAdapter())`` at module bottom.
  4. Import it in ``adapters/__init__.py``.
That's the entire contract -- the runner, metrics and analysis pick it up automatically.
"""

from __future__ import annotations

from typing import List

import numpy as np

from ..collision import path_clearance
from ..kinematics import get_robot
from ..scenario import Obstacle, Query
from .base import PlannerAdapter, PlanResult, register


class StraightLinePlanner(PlannerAdapter):
    name = "straightline"

    def __init__(self, n_waypoints: int = 30):
        self.n_waypoints = n_waypoints
        self._model = None
        self._obstacles: List[Obstacle] = []

    def setup(self, robot_name: str, obstacles: List[Obstacle]) -> None:
        self._model = get_robot(robot_name)
        self._obstacles = obstacles

    def plan(self, query: Query, timeout: float, seed: int, run: int = 0) -> PlanResult:
        start = np.asarray(query.start, dtype=float)
        goal = np.asarray(query.goal_joint, dtype=float)
        ts = np.linspace(0.0, 1.0, self.n_waypoints)
        traj = start[None, :] + np.outer(ts, goal - start)
        _, valid = path_clearance(self._model, traj, self._obstacles)
        return PlanResult(
            planner=self.name, scenario="", query_id=query.id, run=run, seed=seed,
            success=bool(valid), planning_time_s=1e-4, solve_time_s=1e-4,
            trajectory=traj.tolist(),
            time_from_start=(ts * (0.1 * self.n_waypoints)).tolist(),
            error=None if valid else "straight-line path in collision",
        )


register("straightline", lambda: StraightLinePlanner())
