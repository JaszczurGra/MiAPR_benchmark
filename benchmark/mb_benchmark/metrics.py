"""The uniform metric layer.

Given a :class:`PlanResult` (just a trajectory + timing) plus the scenario world and a
:class:`RobotModel`, compute every comparison metric with the *same* code regardless of
which planner produced the trajectory. This is the heart of a fair cross-framework
comparison. Formulas are documented in ``../../METHODOLOGY.md``.

Metrics
-------
success               : planner reported a valid solution within the timeout
planning_time_s       : wall-clock around the plan call (steady-state; warmup excluded)
solve_time_s          : time to first valid solution (<= planning_time_s)
num_waypoints         : number of trajectory waypoints
joint_path_length     : sum of L2 joint-space step norms (rad)
cartesian_path_length : sum of L2 EE position step norms via FK (m)
smoothness_geom       : OMPL PathGeometric::smoothness (turning-angle, lower=smoother)
smoothness_jerk       : normalized integrated squared jerk (only if timing present)
clearance             : min sphere-clearance to obstacles along the densified path (m)
path_valid            : True if no config along the densified path penetrates an obstacle
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

import numpy as np

from .collision import path_clearance
from .kinematics import RobotModel
from .scenario import Obstacle


def joint_path_length(traj: np.ndarray) -> float:
    if len(traj) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(traj, axis=0), axis=1)))


def cartesian_path_length(traj: np.ndarray, model: RobotModel) -> float:
    if len(traj) < 2:
        return 0.0
    pts = np.array([model.fk_ee(q)[:3, 3] for q in traj])
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def smoothness_geom(traj: np.ndarray) -> float:
    """OMPL ``PathGeometric::smoothness`` in the joint-space metric.

    For each interior vertex, the turning angle ``(pi - interior_angle)`` is normalized
    by the local segment lengths; squared contributions are summed. 0 == perfectly
    straight; larger == more jagged. Distance metric is Euclidean in joint space."""
    n = len(traj)
    if n < 3:
        return 0.0
    s = 0.0
    a = float(np.linalg.norm(traj[1] - traj[0]))
    for i in range(2, n):
        b = float(np.linalg.norm(traj[i] - traj[i - 1]))
        c = float(np.linalg.norm(traj[i] - traj[i - 2]))
        if a > 1e-12 and b > 1e-12:
            acos_value = (a * a + b * b - c * c) / (2.0 * a * b)
            if -1.0 < acos_value < 1.0:
                angle = math.pi - math.acos(acos_value)
                k = 2.0 * angle / (a + b)
                s += k * k
        a = b
    return float(s)


def smoothness_jerk(traj: np.ndarray, time_from_start: Optional[List[float]]) -> float:
    """Normalized integrated squared jerk, summed over joints. Requires per-waypoint
    timing; returns NaN otherwise. Resamples to a uniform grid before differentiating."""
    if time_from_start is None or len(traj) < 4:
        return float("nan")
    t = np.asarray(time_from_start, dtype=float)
    duration = t[-1] - t[0]
    if duration <= 0:
        return float("nan")
    grid = np.linspace(t[0], t[-1], max(len(t), 50))
    dt = grid[1] - grid[0]
    total = 0.0
    for j in range(traj.shape[1]):
        pos = np.interp(grid, t, traj[:, j])
        jerk = np.gradient(np.gradient(np.gradient(pos, dt), dt), dt)
        total += float(np.sum(jerk * jerk) * dt)
    # dimensionless-ish normalization by duration^5 / length keeps values comparable
    return float(total * (duration ** 5) / max(len(grid), 1))


def compute_metrics(
    result,
    obstacles: List[Obstacle],
    model: RobotModel,
    max_joint_step: float = 0.1,
) -> Dict:
    """Compute the full metric row for one :class:`PlanResult`."""
    row: Dict = {
        "planner": result.planner,
        "scenario": result.scenario,
        "query_id": result.query_id,
        "run": result.run,
        "seed": result.seed,
        "success": bool(result.success),
        "planning_time_s": float(result.planning_time_s),
        "solve_time_s": float(result.solve_time_s),
        "error": result.error,
    }
    traj = result.trajectory_array
    row["num_waypoints"] = int(len(traj))

    if not result.success or len(traj) < 2:
        for key in ("joint_path_length", "cartesian_path_length", "smoothness_geom",
                    "smoothness_jerk", "clearance"):
            row[key] = float("nan")
        row["path_valid"] = False
        return row

    row["joint_path_length"] = joint_path_length(traj)
    row["cartesian_path_length"] = cartesian_path_length(traj, model)
    row["smoothness_geom"] = smoothness_geom(traj)
    row["smoothness_jerk"] = smoothness_jerk(traj, result.time_from_start)
    clearance, valid = path_clearance(model, traj, obstacles, max_joint_step)
    row["clearance"] = clearance
    row["path_valid"] = valid
    return row
