"""cuRobo planner adapter — the non-MoveIt algorithm (task 4).

cuRobo is NVIDIA's CUDA-accelerated motion generation library. Its core ``MotionGen`` is
standalone (no Isaac Sim, no ROS), which is ideal for an apples-to-apples adapter: it
loads the same robot + the same world (translated from our scenario) and plans to the
same target, then we score its trajectory with the same metric code as every other
planner.

ENVIRONMENT: requires an NVIDIA GPU + CUDA + PyTorch + cuRobo. Runs in the ``curobo``
container only. Heavy imports are deferred to ``setup()`` so this module imports with no
torch/cuRobo present.

NOTE FOR LATER AGENTS (cannot be tested without a GPU):
* cuRobo quaternions are ``[w, x, y, z]``; our scenario stores ``[x, y, z, w]`` -> convert.
* cuRobo world poses are ``[x, y, z, qw, qx, qy, qz]``.
* The host GPU is an 8 GB RTX 3060 Ti. If you hit OOM, reduce ``num_trajopt_seeds`` /
  ``num_graph_seeds`` and the collision-checker resolution in ``_MOTION_GEN_KWARGS``.
* ``requires_warmup=True`` -> the runner discards the first timed plan; we also call
  ``motion_gen.warmup()`` in ``setup()``. Both matter: the first plan triggers CUDA-graph
  capture / JIT and is not representative.
* Default robot config is ``ur5e.yml`` (ships with cuRobo). For strict UR5, generate a
  ur5 config + collision spheres (see docs/04) and pass ``robot_config="ur5.yml"``.
"""

from __future__ import annotations

import time
from typing import List

import numpy as np

from ..scenario import Obstacle, Query
from .base import PlannerAdapter, PlanResult, register

# Tunables for an 8 GB GPU. Increase seeds for quality if you have more VRAM.
_MOTION_GEN_KWARGS = dict(
    interpolation_dt=0.02,
    num_trajopt_seeds=4,
    num_graph_seeds=4,
)


class CuRoboAdapter(PlannerAdapter):
    requires_warmup = True

    def __init__(self, robot_config: str = "ur5e.yml", goal_mode: str = "pose"):
        self.robot_config = robot_config
        self.goal_mode = goal_mode  # "pose" (natural for cuRobo) or "joint"
        self.name = "curobo"
        self._mg = None
        self._tensor_args = None

    # ------------------------------------------------------------------ #
    def setup(self, robot_name: str, obstacles: List[Obstacle]) -> None:  # pragma: no cover
        try:
            from curobo.types.base import TensorDeviceType
            from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
        except Exception as exc:
            raise RuntimeError(
                "cuRobo/torch not importable. Run this adapter in the `curobo` container "
                "with a GPU. Original error: %r" % exc
            ) from exc

        self._tensor_args = TensorDeviceType()
        world_config = self._build_world(obstacles)
        cfg = MotionGenConfig.load_from_robot_config(
            self.robot_config, world_config, self._tensor_args, **_MOTION_GEN_KWARGS
        )
        self._mg = MotionGen(cfg)
        self._mg.warmup()  # discard first-call CUDA-graph/JIT cost

    def _build_world(self, obstacles: List[Obstacle]):  # pragma: no cover
        from curobo.geom.types import Cuboid, Cylinder, Sphere, WorldConfig

        def pose7(obs):  # [x, y, z, qw, qx, qy, qz]
            x, y, z = (float(v) for v in obs.xyz)
            qx, qy, qz, qw = (float(v) for v in obs.quat)
            return [x, y, z, qw, qx, qy, qz]

        cuboids, spheres, cylinders = [], [], []
        for obs in obstacles:
            if obs.type == "box":
                cuboids.append(Cuboid(name=obs.name, pose=pose7(obs),
                                      dims=[float(v) for v in obs.params["size"]]))
            elif obs.type == "sphere":
                spheres.append(Sphere(name=obs.name, position=[float(v) for v in obs.xyz],
                                      radius=float(obs.params["radius"])))
            elif obs.type == "cylinder":
                cylinders.append(Cylinder(name=obs.name, pose=pose7(obs),
                                          radius=float(obs.params["radius"]),
                                          height=float(obs.params["length"])))
            # mesh: add WorldConfig.from_dict mesh entry in a follow-up.

        # cuRobo's primitive collision checker raises "Primitive Collision has no obstacles"
        # for a fully empty world, so the `empty` scenario would otherwise fail at setup. Add
        # one tiny cuboid placed far outside the UR5e's ~0.85 m reach: it keeps the collision
        # world non-degenerate while being unreachable, so it never affects any plan.
        if not (cuboids or spheres or cylinders):
            cuboids.append(Cuboid(name="_empty_world_placeholder",
                                  pose=[100.0, 100.0, 100.0, 1.0, 0.0, 0.0, 0.0],
                                  dims=[0.01, 0.01, 0.01]))
        return WorldConfig(cuboid=cuboids, sphere=spheres, cylinder=cylinders)

    def plan(self, query: Query, timeout: float, seed: int, run: int = 0) -> PlanResult:  # pragma: no cover
        import torch
        from curobo.types.math import Pose
        from curobo.types.robot import JointState
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

        result = PlanResult(planner=self.name, scenario="", query_id=query.id, run=run, seed=seed)
        ta = self._tensor_args

        start = JointState.from_position(
            ta.to_device(np.asarray(query.start, dtype=np.float32)).view(1, -1)
        )
        plan_cfg = MotionGenPlanConfig(max_attempts=1, timeout=float(timeout))

        t0 = time.perf_counter()
        if self.goal_mode == "pose" and query.goal_pose is not None:
            xyz = query.goal_pose["xyz"]
            qx, qy, qz, qw = query.goal_pose["quat"]
            goal = Pose(
                position=ta.to_device(np.asarray(xyz, dtype=np.float32)).view(1, 3),
                quaternion=ta.to_device(np.asarray([qw, qx, qy, qz], dtype=np.float32)).view(1, 4),
            )
            mg_result = self._mg.plan_single(start, goal, plan_cfg)
        else:
            goal_js = JointState.from_position(
                ta.to_device(np.asarray(query.goal_joint, dtype=np.float32)).view(1, -1)
            )
            mg_result = self._mg.plan_single_js(start, goal_js, plan_cfg)
        elapsed = time.perf_counter() - t0
        result.planning_time_s = elapsed
        result.solve_time_s = float(getattr(mg_result, "solve_time", elapsed))

        if bool(mg_result.success.item()):
            interp = mg_result.get_interpolated_plan()
            positions = interp.position.cpu().numpy()
            dt = float(getattr(mg_result, "interpolation_dt", _MOTION_GEN_KWARGS["interpolation_dt"]))
            result.success = True
            result.trajectory = positions.tolist()
            result.time_from_start = [i * dt for i in range(len(positions))]
        else:
            result.success = False
            result.error = str(getattr(mg_result, "status", "cuRobo failure"))
        return result

    def teardown(self) -> None:  # pragma: no cover
        self._mg = None


register("curobo", lambda: CuRoboAdapter())
register("curobo:joint", lambda: CuRoboAdapter(goal_mode="joint"))
