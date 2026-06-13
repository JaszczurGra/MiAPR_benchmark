"""Adapter interface + result type + planner registry.

Adding a new planner to the comparison (task 5) is exactly: subclass
:class:`PlannerAdapter`, implement ``setup`` / ``plan`` / ``teardown``, and
``register(name, factory)``. See ``template_adapter.py``.
"""

from __future__ import annotations

import json
from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

from ..scenario import Obstacle, Query


@dataclass
class PlanResult:
    """Raw, framework-agnostic planner output. Adapters produce ONLY this; all metrics
    are computed downstream so every planner is measured with identical code."""

    planner: str
    scenario: str
    query_id: int
    run: int = 0
    seed: int = 0
    success: bool = False
    planning_time_s: float = float("nan")  # wall-clock around the plan call
    solve_time_s: float = float("nan")  # time to first valid solution (<= planning_time)
    trajectory: List[List[float]] = field(default_factory=list)  # joint waypoints
    time_from_start: Optional[List[float]] = None  # seconds per waypoint, if available
    meta: Dict = field(default_factory=dict)  # free-form planner metadata
    error: Optional[str] = None

    def to_dict(self) -> Dict:
        return asdict(self)

    @staticmethod
    def from_dict(d: Dict) -> "PlanResult":
        return PlanResult(**d)

    def save(self, path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as fh:
            json.dump(self.to_dict(), fh, indent=2)

    @staticmethod
    def load(path) -> "PlanResult":
        with open(path, "r") as fh:
            return PlanResult.from_dict(json.load(fh))

    @property
    def trajectory_array(self) -> np.ndarray:
        return np.asarray(self.trajectory, dtype=float)


class PlannerAdapter(ABC):
    """One planner the harness can drive. Stateless across scenarios is fine; the
    runner calls ``setup`` once per (scenario) then ``plan`` once per (query, run)."""

    #: human-readable planner name used in result tables / file paths
    name: str = "abstract"
    #: if True the runner runs one throwaway plan first and discards it (e.g. cuRobo /
    #: CUDA-graph warmup, JIT). Critical for fair timing.
    requires_warmup: bool = False

    @abstractmethod
    def setup(self, robot_name: str, obstacles: List[Obstacle]) -> None:
        """Prepare the planner for a scenario world (build planning scene, load model)."""

    @abstractmethod
    def plan(self, query: Query, timeout: float, seed: int, run: int = 0) -> PlanResult:
        """Plan one query and return a :class:`PlanResult` (success or failure)."""

    def teardown(self) -> None:  # optional override
        """Release resources (GPU memory, ROS nodes)."""


# --------------------------------------------------------------------------- #
# Registry
# --------------------------------------------------------------------------- #
PLANNER_REGISTRY: Dict[str, Callable[[], PlannerAdapter]] = {}


def register(name: str, factory: Callable[[], PlannerAdapter]) -> None:
    """Register a zero-arg ``factory`` that builds a configured adapter for ``name``."""
    PLANNER_REGISTRY[name] = factory


def get_adapter(name: str) -> PlannerAdapter:
    if name not in PLANNER_REGISTRY:
        raise KeyError(
            f"planner {name!r} is not registered. Available: {available_planners()}"
        )
    return PLANNER_REGISTRY[name]()


def available_planners() -> List[str]:
    return sorted(PLANNER_REGISTRY)
