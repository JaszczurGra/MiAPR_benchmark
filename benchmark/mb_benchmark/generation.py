"""Seeded, reproducible generation of planning queries for a scenario.

Random start/goal joint configurations are sampled within (a tamed range of) the joint
limits and rejected if they collide with the scenario world. The goal end-effector pose
is derived from the goal joint config via FK, so the same query can be posed to a
joint-space planner (OMPL) or a pose-space planner (cuRobo). Identical seed -> identical
queries -> fair, reproducible comparison across all planners and both pipelines.
"""

from __future__ import annotations

import warnings
from typing import List, Optional

import numpy as np

from .collision import in_collision
from .geometry import matrix_to_quat
from .kinematics import RobotModel, get_robot
from .scenario import Obstacle, Query, Scenario

# Default sampling range (radians) -- tamer than the physical +/-2*pi so configs look
# reasonable. Override per-scenario via generation: {sample_low, sample_high}.
_DEFAULT_LOW = np.array([-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi])
_DEFAULT_HIGH = np.array([np.pi, np.pi, np.pi, np.pi, np.pi, np.pi])


def _sample_free_config(
    rng: np.random.Generator,
    model: RobotModel,
    obstacles: List[Obstacle],
    low: np.ndarray,
    high: np.ndarray,
    max_tries: int = 300,
) -> Optional[np.ndarray]:
    for _ in range(max_tries):
        q = rng.uniform(low, high)
        if not in_collision(model, q, obstacles):
            return q
    return None


def _ee_pose_dict(model: RobotModel, q: np.ndarray) -> dict:
    t = model.fk_ee(q)
    return {
        "xyz": [float(v) for v in t[:3, 3]],
        "quat": [float(v) for v in matrix_to_quat(t[:3, :3])],
    }


def generate_queries(
    scenario: Scenario,
    num_queries: Optional[int] = None,
    seed: Optional[int] = None,
    goal_type: str = "joint",
) -> List[Query]:
    """Fill ``scenario.queries`` with seeded collision-free start/goal pairs.

    ``goal_type`` is recorded for the adapters' benefit; both ``goal_joint`` and
    ``goal_pose`` are always stored so either planner style works.
    """
    gen = scenario.generation or {}
    num_queries = num_queries if num_queries is not None else int(gen.get("num_queries", 20))
    seed = seed if seed is not None else int(gen.get("seed", 0))
    low = np.asarray(gen.get("sample_low", _DEFAULT_LOW), dtype=float)
    high = np.asarray(gen.get("sample_high", _DEFAULT_HIGH), dtype=float)

    model = get_robot(scenario.robot)
    rng = np.random.default_rng(seed)
    queries: List[Query] = []
    qid = 0
    attempts = 0
    # Bounded work: if free configs are very rare (e.g. an over-constrained scene), warn
    # and return what we found instead of looping forever.
    max_attempts = num_queries * 50 + 100
    while len(queries) < num_queries and attempts < max_attempts:
        attempts += 1
        start = _sample_free_config(rng, model, scenario.obstacles, low, high)
        goal = _sample_free_config(rng, model, scenario.obstacles, low, high)
        if start is None or goal is None:
            continue
        if np.linalg.norm(goal - start) < 0.5:  # avoid trivial queries
            continue
        queries.append(
            Query(id=qid, start=start, goal_joint=goal, goal_pose=_ee_pose_dict(model, goal))
        )
        qid += 1

    if len(queries) < num_queries:
        warnings.warn(
            f"scenario {scenario.name!r}: only generated {len(queries)}/{num_queries} "
            f"collision-free queries (scene may be over-constrained; widen sample range "
            f"or relax obstacles).",
            stacklevel=2,
        )
    scenario.queries = queries
    scenario.generation = {**gen, "num_queries": num_queries, "seed": seed, "goal_type": goal_type}
    return queries
