"""MoveIt 2 planner adapter (via the ``moveit_py`` Python bindings).

Runs ANY MoveIt planning pipeline / planner (OMPL, Pilz, CHOMP, STOMP) so the harness
can re-measure MoveIt planners with the same metric code used for cuRobo.

ENVIRONMENT: requires ROS 2 + MoveIt 2 + ``moveit_py`` + a UR MoveIt config. It can
only run inside the ``ros`` container; the UR MoveIt parameters are assembled in
``setup()`` via ``MoveItConfigsBuilder`` (no launch wrapper needed). The heavy imports
are deferred to ``setup()`` so this module imports fine with no ROS installed -- which is
why ``moveit:*`` planners always appear in ``list-planners``.

NOTE FOR LATER AGENTS (cannot be tested without ROS): the exact ``moveit_py`` symbol
names have drifted across distros. The trajectory is extracted via
``get_robot_trajectory_msg()`` (stable across versions). Verify the planning-component
group name (``ur_manipulator``) and planner ids against the installed ur_moveit_config.

Fallback if ``moveit_py`` is unusable on the chosen distro: a MoveGroup action client
(documented in docs/03), behind the same :class:`PlannerAdapter` interface.
"""

from __future__ import annotations

import time
from typing import List, Optional

import numpy as np

from ..scenario import Obstacle, Query
from .base import PlannerAdapter, PlanResult, register

# OMPL planner ids exposed for the comparison. These must match the config names in the
# UR ur_moveit_config ompl_planning.yaml; adjust if your config uses the *kConfigDefault
# suffix form.
OMPL_PLANNERS = [
    "RRTConnect", "RRT", "RRTstar", "PRM", "PRMstar",
    "BiTRRT", "BITstar", "EST", "KPIECE", "LBKPIECE", "SBL",
]

# ONE MoveItPy for the whole process, shared by every MoveItAdapter (all planners + the
# query validator). MoveItPy must not be re-created or garbage-collected mid-process: its
# C++ teardown segfaults, which would crash `mb-benchmark run` between planners. The
# singleton lives until process exit, where cli._hard_exit avoids the crash by exiting
# before Python tears the C++ side down.
_SHARED_MOVEIT = None
_SHARED_ROBOT_MODEL = None


class MoveItAdapter(PlannerAdapter):
    def __init__(
        self,
        planner_id: str = "RRTConnect",
        pipeline: str = "ompl",
        group: str = "ur_manipulator",
        node_name: str = "mb_moveit_py",
    ):
        self.planner_id = planner_id
        self.pipeline = pipeline
        self.group = group
        self.node_name = node_name
        self.name = f"moveit:{planner_id}"
        self._moveit = None
        self._component = None
        self._robot_model = None

    # ------------------------------------------------------------------ #
    def _require_moveit(self):
        try:
            from moveit.planning import MoveItPy  # noqa: F401
        except Exception as exc:  # pragma: no cover - needs ROS
            raise RuntimeError(
                "moveit_py is not importable. Run this adapter inside the `ros` "
                "container with the UR MoveIt config loaded. Original error: %r" % exc
            ) from exc

    def setup(self, robot_name: str, obstacles: List[Obstacle]) -> None:  # pragma: no cover
        self._require_moveit()

        # Build/reuse ONE process-wide MoveItPy (see module note): re-creating it per
        # planner/scenario is slow and its GC mid-run segfaults. Each scenario just swaps
        # the collision world (see _apply_world).
        global _SHARED_MOVEIT, _SHARED_ROBOT_MODEL
        if _SHARED_MOVEIT is None:
            _SHARED_MOVEIT = self._build_moveit_py(robot_name)
            _SHARED_ROBOT_MODEL = _SHARED_MOVEIT.get_robot_model()
        self._moveit = _SHARED_MOVEIT
        self._robot_model = _SHARED_ROBOT_MODEL
        self._component = self._moveit.get_planning_component(self.group)
        self._apply_world(obstacles)

    def _build_moveit_py(self, robot_name: str):  # pragma: no cover
        # Build a SELF-CONTAINED UR MoveIt config so `mb-benchmark run` works as a plain
        # process (no launch/move_group needed). The URDF is generated from ur_description's
        # xacro (nothing publishes /robot_description here); the SRDF/kinematics/joint_limits/
        # planner configs come from ur_moveit_config. URDF (tf_prefix:="") and SRDF share
        # joint names shoulder_pan_joint..wrist_3_joint, so the planning group
        # "ur_manipulator" resolves. (Same MoveItConfigsBuilder recipe a benchmark launch
        # file would use.) We
        # fail loudly rather than silently degrade to a no-model MoveItPy, which would only
        # error later in a confusing way.
        import os
        from pathlib import Path

        from ament_index_python.packages import get_package_share_directory
        from moveit_configs_utils import MoveItConfigsBuilder

        config_dict = (
            MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
            .robot_description(
                file_path=os.path.join(
                    get_package_share_directory("ur_description"), "urdf", "ur.urdf.xacro"
                ),
                mappings={"ur_type": robot_name, "name": "ur", "tf_prefix": ""},
            )
            # SRDF robot name must equal the URDF robot name ("ur" above), else MoveIt
            # rejects it: "Semantic description is not specified for the same robot".
            .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur"})
            .robot_description_kinematics(Path("config") / "kinematics.yaml")
            .joint_limits(Path("config") / "joint_limits.yaml")
            .planning_pipelines(
                pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
                default_planning_pipeline="ompl",
            )
            .to_moveit_configs()
            .to_dict()
        )

        # MoveItConfigsBuilder emits a top-level `planning_pipelines: [<names>]` list (the
        # form move_group reads), but MoveItCpp (which moveit_py uses) reads the names from
        # `planning_pipelines.pipeline_names`. Without this rewrite MoveItPy finds zero
        # pipelines and aborts with "Failed to load any planning pipelines". The per-pipeline
        # configs stay at the top level (e.g. config_dict["ompl"]), which is where
        # createPlanningPipelineMap looks for them.
        pipeline_names = config_dict.get("planning_pipelines")
        if isinstance(pipeline_names, list):
            config_dict["planning_pipelines"] = {"pipeline_names": pipeline_names}

        # ur_moveit_config's ompl_planning.yaml defines only plugins/adapters -- it has NO
        # per-group section, so MoveIt can't find a planning configuration for
        # "ur_manipulator" and silently falls back to broken defaults (no projection
        # evaluator; coarse motion validation). That makes OMPL fail even in free space and
        # produce paths that fail ValidateSolution. Inject the standard UR group config so
        # the group maps to the available planners, gets a projection evaluator, and uses a
        # fine collision-validation segment.
        ompl = config_dict.setdefault("ompl", {})
        planner_cfg_names = list(ompl.get("planner_configs", {}).keys())
        ompl[self.group] = {
            "planner_configs": planner_cfg_names,
            "projection_evaluator": "joints(shoulder_pan_joint,shoulder_lift_joint)",
            "longest_valid_segment_fraction": 0.005,
        }

        from moveit.planning import MoveItPy
        return MoveItPy(node_name=self.node_name, config_dict=config_dict)

    def _apply_world(self, obstacles: List[Obstacle]) -> None:  # pragma: no cover
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive

        psm = self._moveit.get_planning_scene_monitor()
        with psm.read_write() as scene:
            # MoveItPy is reused across scenarios, so wipe the previous world first. A
            # CollisionObject with operation REMOVE and an empty id removes ALL world objects.
            clear = CollisionObject()
            clear.operation = CollisionObject.REMOVE
            scene.apply_collision_object(clear)
            for obs in obstacles:
                co = CollisionObject()
                co.id = obs.name
                co.header.frame_id = "base_link"
                prim = SolidPrimitive()
                if obs.type == "box":
                    prim.type = SolidPrimitive.BOX
                    prim.dimensions = [float(v) for v in obs.params["size"]]
                elif obs.type == "sphere":
                    prim.type = SolidPrimitive.SPHERE
                    prim.dimensions = [float(obs.params["radius"])]
                elif obs.type == "cylinder":
                    prim.type = SolidPrimitive.CYLINDER
                    prim.dimensions = [float(obs.params["length"]), float(obs.params["radius"])]
                else:
                    continue  # mesh: load via pyassimp in a follow-up; skipped here
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = [float(v) for v in obs.xyz]
                (pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w) = [float(v) for v in obs.quat]
                co.primitives.append(prim)
                co.primitive_poses.append(pose)
                co.operation = CollisionObject.ADD
                scene.apply_collision_object(co)
            scene.current_state.update()

    def plan(self, query: Query, timeout: float, seed: int, run: int = 0) -> PlanResult:  # pragma: no cover
        from moveit.core.robot_state import RobotState
        from moveit.planning import PlanRequestParameters

        result = PlanResult(planner=self.name, scenario="", query_id=query.id, run=run, seed=seed)

        start_state = RobotState(self._robot_model)
        start_state.set_joint_group_positions(self.group, np.asarray(query.start, dtype=float))
        start_state.update()
        self._component.set_start_state(robot_state=start_state)

        goal_state = RobotState(self._robot_model)
        goal_state.set_joint_group_positions(self.group, np.asarray(query.goal_joint, dtype=float))
        goal_state.update()
        self._component.set_goal_state(robot_state=goal_state)

        params = PlanRequestParameters(self._moveit, self.pipeline)
        # Must set planning_pipeline explicitly: PlanRequestParameters defaults it from
        # `<pipeline>.plan_request_params.planning_pipeline`, which our config doesn't set,
        # leaving it empty -> "No planning pipeline available for name ''".
        params.planning_pipeline = self.pipeline
        params.planner_id = self.planner_id
        params.planning_time = float(timeout)
        params.planning_attempts = 1

        t0 = time.perf_counter()
        plan_solution = self._component.plan(single_plan_parameters=params)
        elapsed = time.perf_counter() - t0
        result.planning_time_s = elapsed
        result.solve_time_s = elapsed

        if plan_solution and getattr(plan_solution, "trajectory", None) is not None:
            waypoints, times = _extract_trajectory(plan_solution.trajectory)
            result.success = len(waypoints) >= 2
            result.trajectory = waypoints
            result.time_from_start = times
        else:
            result.success = False
            result.error = "no solution"
        return result

    def is_state_valid(self, q) -> bool:  # pragma: no cover
        """True if joint config ``q`` is collision-free (self + world) in the CURRENT
        planning scene. Used to validate generated queries against MoveIt's mesh collision
        model, which is stricter than the harness sphere model -- so the shared query set is
        valid for every planner (cuRobo/straightline are more permissive). Call after
        setup() so the scenario's world is loaded."""
        from moveit.core.robot_state import RobotState

        state = RobotState(self._robot_model)
        state.set_joint_group_positions(self.group, np.asarray(q, dtype=float))
        state.update()
        psm = self._moveit.get_planning_scene_monitor()
        with psm.read_only() as scene:
            # is_state_valid = collision-free (self + world) AND within joint bounds AND
            # satisfies constraints -- i.e. exactly the validity the OMPL planner enforces.
            return scene.is_state_valid(
                robot_state=state, joint_model_group_name=self.group
            )

    def is_solvable(self, start, goal, timeout: float = 5.0) -> bool:  # pragma: no cover
        """True if MoveIt can actually find a path from ``start`` to ``goal`` in the CURRENT
        scene within ``timeout``. Used to keep only WELL-POSED queries during generation:
        random (start, goal) pairs in the UR self-collision-constrained C-space are often in
        disconnected regions (no path exists), which no planner can solve and which make the
        benchmark meaningless. Filtering by solvability guarantees a path exists; the more
        capable planners (cuRobo) solve these too, so the comparison stays fair. Call after
        setup() so the scenario's world is loaded."""
        from moveit.core.robot_state import RobotState
        from moveit.planning import PlanRequestParameters

        ss = RobotState(self._robot_model)
        ss.set_joint_group_positions(self.group, np.asarray(start, dtype=float))
        ss.update()
        self._component.set_start_state(robot_state=ss)
        gs = RobotState(self._robot_model)
        gs.set_joint_group_positions(self.group, np.asarray(goal, dtype=float))
        gs.update()
        self._component.set_goal_state(robot_state=gs)

        params = PlanRequestParameters(self._moveit, self.pipeline)
        params.planning_pipeline = self.pipeline
        params.planner_id = self.planner_id
        params.planning_time = float(timeout)
        params.planning_attempts = 1
        sol = self._component.plan(single_plan_parameters=params)
        return bool(sol) and getattr(sol, "trajectory", None) is not None

    def teardown(self) -> None:  # pragma: no cover
        # Intentionally keep self._moveit alive: the runner calls teardown() after every
        # scenario, but MoveItPy is built once and reused (see setup). Destroying/recreating
        # it per scenario is slow and segfaults in moveit_py's C++ teardown. It is released
        # when the adapter object is garbage-collected at process end.
        self._component = None


def _extract_trajectory(robot_trajectory):  # pragma: no cover - needs ROS
    """Return (waypoints, time_from_start) from a moveit_py RobotTrajectory using the
    stable msg accessor."""
    msg = robot_trajectory.get_robot_trajectory_msg()
    jt = msg.joint_trajectory
    waypoints: List[List[float]] = []
    times: Optional[List[float]] = []
    for pt in jt.points:
        waypoints.append([float(v) for v in pt.positions])
        times.append(pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9)
    return waypoints, times


# Register one factory per OMPL planner id.
for _pid in OMPL_PLANNERS:
    register(f"moveit:{_pid}", (lambda pid: (lambda: MoveItAdapter(planner_id=pid)))(_pid))
# A couple of non-OMPL pipelines for contrast with cuRobo's optimization approach.
register("moveit:pilz_ptp", lambda: MoveItAdapter(planner_id="PTP", pipeline="pilz_industrial_motion_planner"))
register("moveit:stomp", lambda: MoveItAdapter(planner_id="STOMP", pipeline="stomp"))
register("moveit:chomp", lambda: MoveItAdapter(planner_id="CHOMP", pipeline="chomp"))
