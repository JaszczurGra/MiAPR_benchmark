"""Collision / clearance queries against a scenario world.

All distances use the coarse sphere model from :mod:`kinematics` and the primitive
SDFs from :mod:`geometry`. This is the *uniform* collision representation used by the
metric layer for every planner, so clearance numbers are comparable across frameworks.
(Mesh obstacles fall back to their bounding sphere here -- documented approximation.)

The SDFs and these helpers are vectorised over points so scoring whole trajectories
stays fast in pure Python/numpy.
"""

from __future__ import annotations

from typing import List

import numpy as np

from .geometry import sdf_box, sdf_cylinder, sdf_sphere, world_to_local
from .kinematics import RobotModel
from .scenario import Obstacle


def obstacle_distance(obstacle: Obstacle, points: np.ndarray):
    """Signed distance from world ``points`` (``(3,)`` or ``(M, 3)``) to an obstacle
    surface (>=0 outside). Returns a scalar or ``(M,)`` array to match the input."""
    local = world_to_local(points, obstacle.rotation(), obstacle.xyz)
    p = obstacle.params
    if obstacle.type == "box":
        return sdf_box(local, 0.5 * np.asarray(p["size"], dtype=float))
    if obstacle.type == "sphere":
        return sdf_sphere(local, float(p["radius"]))
    if obstacle.type == "cylinder":
        return sdf_cylinder(local, float(p["radius"]), 0.5 * float(p["length"]))
    if obstacle.type == "mesh":
        return sdf_sphere(local, float(p.get("bounding_radius", 0.0)))
    raise ValueError(f"unsupported obstacle type {obstacle.type!r}")


def _min_clearance(centers: np.ndarray, radii: np.ndarray, obstacles: List[Obstacle]) -> float:
    """Minimum over all (sphere, obstacle) pairs of (surface distance - sphere radius)."""
    worst = np.inf
    for obs in obstacles:
        d = np.asarray(obstacle_distance(obs, centers)) - radii
        worst = min(worst, float(np.min(d)))
    return worst


def config_clearance(model: RobotModel, q: np.ndarray, obstacles: List[Obstacle]) -> float:
    """Minimum clearance at a single config. ``+inf`` if no obstacles; <0 = penetration."""
    if not obstacles:
        return float("inf")
    centers, radii = model.collision_spheres(q)
    return _min_clearance(centers, radii, obstacles)


def in_collision(model: RobotModel, q: np.ndarray, obstacles: List[Obstacle]) -> bool:
    return config_clearance(model, q, obstacles) < 0.0


def densify(trajectory: np.ndarray, max_joint_step: float = 0.1) -> np.ndarray:
    """Linearly interpolate a joint path so consecutive samples differ by at most
    ``max_joint_step`` (rad, inf-norm). Used before clearance/validity checks."""
    traj = np.asarray(trajectory, dtype=float)
    if len(traj) < 2:
        return traj
    out = [traj[0]]
    for a, b in zip(traj[:-1], traj[1:]):
        steps = max(1, int(np.ceil(np.max(np.abs(b - a)) / max_joint_step)))
        for k in range(1, steps + 1):
            out.append(a + (b - a) * (k / steps))
    return np.asarray(out)


def path_clearance(
    model: RobotModel,
    trajectory: np.ndarray,
    obstacles: List[Obstacle],
    max_joint_step: float = 0.1,
):
    """Return (min_clearance_along_path, all_configs_valid). Vectorised: all densified
    configs' collision spheres are scored against each obstacle in one numpy call."""
    if not obstacles or len(trajectory) == 0:
        return float("inf"), True
    dense = densify(trajectory, max_joint_step)
    all_centers = []
    all_radii = []
    for q in dense:
        c, r = model.collision_spheres(q)
        all_centers.append(c)
        all_radii.append(r)
    centers = np.concatenate(all_centers, axis=0)
    radii = np.concatenate(all_radii, axis=0)
    worst = _min_clearance(centers, radii, obstacles)
    return worst, bool(worst >= 0.0)
