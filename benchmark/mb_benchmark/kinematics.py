"""Robot kinematics used by the *uniform* metric layer.

We deliberately use a self-contained analytic model (standard Denavit-Hartenberg
forward kinematics + a coarse collision-sphere approximation of the arm) so that the
metric/analysis half of the harness has **zero heavy dependencies** and runs offline,
without ROS, a URDF parser, pinocchio, or a GPU.

For production-grade collision fidelity you can swap in pinocchio + hpp-fcl against the
real meshes; the metric formulas in ``metrics.py`` are agnostic to the model used, as
long as it implements the :class:`RobotModel` interface.

UR5e DH parameters are the official Universal Robots values.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Tuple

import numpy as np


def _dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


class RobotModel(ABC):
    """Minimal interface the metric layer needs from a robot."""

    name: str
    joint_names: List[str]

    @property
    def n_joints(self) -> int:
        return len(self.joint_names)

    @abstractmethod
    def joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return (lower, upper) joint-limit arrays."""

    @abstractmethod
    def fk_frames(self, q: np.ndarray) -> List[np.ndarray]:
        """Return the 4x4 frame of every link, base first, end-effector last."""

    def fk_ee(self, q: np.ndarray) -> np.ndarray:
        """4x4 end-effector pose."""
        return self.fk_frames(q)[-1]

    @abstractmethod
    def collision_spheres(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Return (centers Nx3, radii N) approximating the arm body at config ``q``."""


class URModel(RobotModel):
    """Universal Robots e-series via standard DH parameters."""

    # (a, d, alpha) per joint.  theta is the joint variable.
    _PARAMS = {
        "ur5e": dict(
            a=[0.0, -0.425, -0.3922, 0.0, 0.0, 0.0],
            d=[0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996],
            alpha=[np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0],
        ),
        # UR5 (CB-series) kinematics differ slightly; provided for the documented
        # "strict UR5" extension.  Collision spheres below are a coarse approximation
        # either way -- see module docstring.
        "ur5": dict(
            a=[0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0],
            d=[0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823],
            alpha=[np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0],
        ),
    }

    def __init__(self, name: str = "ur5e"):
        if name not in self._PARAMS:
            raise ValueError(f"unknown UR model {name!r}; choose from {sorted(self._PARAMS)}")
        self.name = name
        self._p = self._PARAMS[name]
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    def joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        # UR joints physically allow +/-2*pi; harness default samples a tamer range
        # (see generation.py).  Here we report the physical limits.
        lo = np.full(6, -2 * np.pi)
        hi = np.full(6, 2 * np.pi)
        return lo, hi

    def fk_frames(self, q: np.ndarray) -> List[np.ndarray]:
        q = np.asarray(q, dtype=float)
        frames = [np.eye(4)]
        t = np.eye(4)
        for i in range(6):
            t = t @ _dh_transform(q[i], self._p["d"][i], self._p["a"][i], self._p["alpha"][i])
            frames.append(t.copy())
        return frames

    def collision_spheres(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Coarse body approximation: spheres at each joint origin plus a couple
        interpolated along every link.  Conservative radii.  Good enough for a
        *relative* clearance metric across planners; not a certified collision model.

        The static base column (base -> shoulder) is intentionally EXCLUDED: the robot
        is mounted on the floor, so that segment would otherwise always read as
        colliding with a ground/table slab. Only the moving arm (shoulder -> EE) is
        represented, which is also all that matters for a path clearance metric."""
        frames = self.fk_frames(q)
        origins = [f[:3, 3] for f in frames[1:]]  # skip base; start at shoulder
        link_radius = 0.06  # ~ arm tube radius, padded
        centers: List[np.ndarray] = []
        radii: List[float] = []
        for a, b in zip(origins[:-1], origins[1:]):
            for s in (0.0, 0.5, 1.0):
                centers.append(a * (1 - s) + b * s)
                radii.append(link_radius)
        return np.asarray(centers), np.asarray(radii)


_ROBOTS = {"ur5e": lambda: URModel("ur5e"), "ur5": lambda: URModel("ur5")}


def get_robot(name: str) -> RobotModel:
    if name not in _ROBOTS:
        raise ValueError(f"unknown robot {name!r}; available: {sorted(_ROBOTS)}")
    return _ROBOTS[name]()
