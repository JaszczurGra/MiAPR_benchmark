"""Scenario model — the single source of truth shared by every planner and by both
benchmark pipelines.

A scenario describes (a) the robot, (b) a static collision world made of primitive
obstacles, and (c) a list of planning queries (start config + goal). Queries may be
authored by hand or filled in by ``generation.py`` (seeded, reproducible). Goals are
stored as a joint configuration *and* the corresponding end-effector pose (FK), so a
joint-space planner (OMPL) and a pose-space planner (cuRobo) hit an identical target.

YAML schema
-----------
```yaml
name: shelf
robot: ur5e
world:
  obstacles:
    - {type: box,      name: table, size: [1.0, 1.0, 0.05], pose: {xyz: [0.5, 0.0, 0.0], rpy: [0, 0, 0]}}
    - {type: cylinder, name: pole,  radius: 0.05, length: 0.8, pose: {xyz: [0.4, 0.2, 0.4]}}
    - {type: sphere,   name: ball,  radius: 0.1, pose: {xyz: [0.3, -0.3, 0.5]}}
queries:
  - {id: 0, start: [...6...], goal_joint: [...6...], goal_pose: {xyz: [...], quat: [x, y, z, w]}}
generation: {seed: 42, num_queries: 25, goal_type: joint}
```
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml

from .geometry import matrix_to_quat, quat_to_matrix, rpy_to_matrix

OBSTACLE_TYPES = {"box", "sphere", "cylinder", "mesh"}


@dataclass
class Obstacle:
    type: str
    name: str
    xyz: np.ndarray  # (3,) world position
    quat: np.ndarray  # (4,) [x, y, z, w] world orientation
    params: Dict  # type-specific: size / radius / length / file+scale

    def rotation(self) -> np.ndarray:
        return quat_to_matrix(self.quat)

    @staticmethod
    def from_dict(d: Dict) -> "Obstacle":
        otype = d["type"]
        if otype not in OBSTACLE_TYPES:
            raise ValueError(f"unknown obstacle type {otype!r} (allowed: {sorted(OBSTACLE_TYPES)})")
        pose = d.get("pose", {})
        xyz = np.asarray(pose.get("xyz", [0.0, 0.0, 0.0]), dtype=float)
        if "quat" in pose:
            quat = np.asarray(pose["quat"], dtype=float)
        else:
            quat = matrix_to_quat(rpy_to_matrix(pose.get("rpy", [0.0, 0.0, 0.0])))
        params = {k: v for k, v in d.items() if k not in ("type", "name", "pose")}
        return Obstacle(type=otype, name=d.get("name", otype), xyz=xyz, quat=quat, params=params)

    def to_dict(self) -> Dict:
        out = {"type": self.type, "name": self.name,
               "pose": {"xyz": [float(v) for v in self.xyz],
                        "quat": [float(v) for v in self.quat]}}
        out.update({k: (list(v) if isinstance(v, (list, tuple, np.ndarray)) else v)
                    for k, v in self.params.items()})
        return out


@dataclass
class Query:
    id: int
    start: np.ndarray  # (n,)
    goal_joint: Optional[np.ndarray] = None  # (n,)
    goal_pose: Optional[Dict] = None  # {"xyz": [...], "quat": [...]}

    @staticmethod
    def from_dict(d: Dict) -> "Query":
        gp = d.get("goal_pose")
        if gp is not None:
            gp = {"xyz": [float(v) for v in gp["xyz"]], "quat": [float(v) for v in gp["quat"]]}
        gj = d.get("goal_joint")
        return Query(
            id=int(d["id"]),
            start=np.asarray(d["start"], dtype=float),
            goal_joint=None if gj is None else np.asarray(gj, dtype=float),
            goal_pose=gp,
        )

    def to_dict(self) -> Dict:
        out: Dict = {"id": self.id, "start": [float(v) for v in self.start]}
        if self.goal_joint is not None:
            out["goal_joint"] = [float(v) for v in self.goal_joint]
        if self.goal_pose is not None:
            out["goal_pose"] = self.goal_pose
        return out


@dataclass
class Scenario:
    name: str
    robot: str = "ur5e"
    obstacles: List[Obstacle] = field(default_factory=list)
    queries: List[Query] = field(default_factory=list)
    generation: Dict = field(default_factory=dict)

    def to_dict(self) -> Dict:
        return {
            "name": self.name,
            "robot": self.robot,
            "world": {"obstacles": [o.to_dict() for o in self.obstacles]},
            "queries": [q.to_dict() for q in self.queries],
            "generation": self.generation,
        }


def load_scenario(path) -> Scenario:
    path = Path(path)
    with open(path, "r") as fh:
        d = yaml.safe_load(fh)
    if not isinstance(d, dict) or "name" not in d:
        raise ValueError(f"{path}: not a valid scenario file (missing 'name')")
    obstacles = [Obstacle.from_dict(o) for o in d.get("world", {}).get("obstacles", [])]
    queries = [Query.from_dict(q) for q in d.get("queries", [])]
    return Scenario(
        name=d["name"],
        robot=d.get("robot", "ur5e"),
        obstacles=obstacles,
        queries=queries,
        generation=d.get("generation", {}),
    )


def save_scenario(scenario: Scenario, path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as fh:
        yaml.safe_dump(scenario.to_dict(), fh, sort_keys=False)


def load_library(directory) -> Dict[str, Scenario]:
    """Load every ``*.yaml`` scenario in a directory, keyed by scenario name."""
    directory = Path(directory)
    out: Dict[str, Scenario] = {}
    for p in sorted(directory.glob("*.yaml")):
        sc = load_scenario(p)
        out[sc.name] = sc
    return out
