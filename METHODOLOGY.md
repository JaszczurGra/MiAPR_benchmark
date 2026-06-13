# Methodology

How the comparison is made fair and reproducible, the exact metric definitions, and the
design decisions (with their rationale) for a university submission.

## Two pipelines, reported separately

| | Pipeline A (`moveit_ros_benchmarks`) | Pipeline B (custom harness) |
|---|---|---|
| Planners | MoveIt only (OMPL, Pilz, CHOMP, STOMP) | **MoveIt + cuRobo + any** |
| Metric code | MoveIt/OMPL internal | our `metrics.py` (one path for all) |
| Output | OMPL logs → Planner Arena | tidy `metrics.csv` + plots |
| Role | task-2 literal; standard, credible MoveIt-internal view | the fair cross-framework comparison (tasks 3/4/5) |

**They are never merged into one table** — different code measures them. Pipeline B
re-measures the MoveIt planners itself, so MoveIt-vs-cuRobo numbers are always
apples-to-apples.

## Fair cross-framework comparison (Pipeline B)

1. **Identical problems.** One scenario library (`scenarios/library/*.yaml`) defines the
   world; one seeded generator (`generation.py`) produces collision-free start/goal
   queries. Same seed → identical queries for every planner and both pipelines.
2. **Natural goals, same target.** Each query stores a `goal_joint` *and* its FK
   `goal_pose`. A joint-space planner (OMPL) uses the joint goal; a pose-space planner
   (cuRobo) uses the pose — both reach the same configuration.
3. **Adapters emit only trajectories.** No planner scores itself. All metrics are computed
   afterwards by `metrics.py` from the trajectory + the (shared) world + a common robot
   model. This removes per-framework measurement bias.
4. **Warmup excluded.** Planners with `requires_warmup` (cuRobo: CUDA-graph/JIT) get one
   throwaway plan that is run but not recorded (`runner.py`).
5. **Repetitions.** Each (scenario, query) is planned `runs` times; randomness in the
   planners shows up as spread in the plots.

## Metric definitions (`metrics.py`)

Let a trajectory be joint waypoints `q_0 … q_N`. Distances are Euclidean in joint space
unless noted. EE position `p(q)` comes from analytic UR FK.

- **success** — planner reported a valid solution within `timeout`.
- **planning_time_s** — wall-clock around the `plan()` call (steady-state; warmup excluded).
- **solve_time_s** — time to first valid solution (≤ planning_time_s).
- **joint_path_length** — `Σ ‖q_{i+1} − q_i‖₂` (rad).
- **cartesian_path_length** — `Σ ‖p(q_{i+1}) − p(q_i)‖₂` (m).
- **smoothness_geom** — OMPL `PathGeometric::smoothness`: for each interior vertex with
  adjacent segment lengths `a, b` and chord `c`, turning angle `θ = π − acos((a²+b²−c²)/2ab)`,
  contribution `(2θ/(a+b))²`, summed. **0 = perfectly straight; larger = more jagged.**
  (Same definition MoveIt/OMPL uses, so it’s comparable to Pipeline A’s smoothness.)
- **smoothness_jerk** — normalized integrated squared jerk (only if per-waypoint timing is
  present; resampled to a uniform grid, 3× finite-differenced). Optional/secondary.
- **clearance** — minimum, over a densified path (≤ 0.1 rad steps), of the distance from
  the arm’s collision spheres to the nearest obstacle surface (m). Higher = safer;
  `+inf` when a scenario has no obstacles; `< 0` = penetration.
- **num_waypoints**, **path_valid** (no densified config penetrates an obstacle).

### Collision model used by the metric layer
A self-contained analytic model: UR DH forward kinematics + a coarse sphere
approximation of the moving arm (the static base column is excluded so it never reads as
colliding with the floor it’s mounted on). It’s deliberately dependency-free (numpy only)
so metrics run offline. It is a **relative** clearance model, not a certified collision
checker. For certified geometry, swap in pinocchio + hpp-fcl against the real meshes —
`metrics.py` is agnostic to the model behind the `RobotModel` interface.

### A note on what “planning time” and “path length” mean across frameworks
OMPL planners output a *geometric* path (then time-parameterized); cuRobo outputs an
*optimized, time-parameterized* trajectory. We report planning time to first valid
solution and compute all geometric metrics by one code path after a common densification.
This is the standard, defensible comparison; the difference in planner *philosophy*
(sampling vs optimization) is part of what the benchmark reveals, not a bias to hide.

## Design decisions (defaults + rationale)

| Decision | Default | Why |
|---|---|---|
| Delivery | Docker (multi-service) | Arch has no ROS 2 binaries; reproducible; isolates GPU |
| ROS distro | Humble | Most mature MoveIt 2 + UR + benchmarks; well-trodden cuRobo path |
| Robot | **ur5e everywhere** | cuRobo ships a validated `ur5e.yml`; a hand-made, untestable ur5 sphere config would be a *silent* correctness bug. ur5e is a UR5-family arm. (Strict UR5 is a documented extension.) |
| Simulation | Mock hardware | Planning needs only a planning scene, not physics; deterministic & fast |
| MoveIt interface | `moveit_py` | In-process, any pipeline/planner, returns full trajectory; MoveGroup fallback documented |
| cuRobo | standalone `MotionGen` | No Isaac/ROS needed → clean, isolated adapter |
| Metric backend | analytic UR FK + sphere collision (numpy) | Offline-testable; uniform across planners |

See `HANDOFF.md` for which of these still need validation on a real ROS/GPU box.
