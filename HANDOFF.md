# HANDOFF — status & remaining work

This project was built **without ROS or a GPU available**, by design: the architecture
isolates all ROS/GPU code behind a thin adapter interface so the rest is verifiable
offline. This file tells the next agent/person exactly what is done, what is written but
unvalidated, and what to do on a real ROS/GPU box.

---

## ✅ DONE and TESTED (offline, no ROS/GPU) — don't redo

| Component | Files | Verified by |
|---|---|---|
| Scenario model + YAML loader | `benchmark/mb_benchmark/scenario.py`, `scenarios/library/*.yaml` | `tests/test_scenario.py` |
| UR5e analytic FK + collision spheres | `kinematics.py` | `tests/test_kinematics.py` |
| Geometry + SDFs (vectorised) | `geometry.py`, `collision.py` | `tests/test_geometry.py`, `test_collision_generation.py` |
| Seeded query generation (collision-free start/goal) | `generation.py` | `test_collision_generation.py` |
| Uniform metrics (time, joint/cartesian length, smoothness, clearance) | `metrics.py` | `tests/test_metrics.py` |
| Adapter interface + registry (task-5 seam) | `adapters/base.py` | `tests/test_adapters_pipeline.py` |
| Synthetic planners + straightline baseline | `adapters/synthetic_adapter.py`, `template_adapter.py` | pipeline test |
| Runner (warmup-discard) + analysis/report | `runner.py`, `analysis.py`, `cli.py` | full offline pipeline test + `mb-benchmark demo` |

**30 tests pass; `mb-benchmark demo` runs end-to-end in ~11 s** and produces a report
with plots. Re-confirm anytime: `make offline`.

---

## ⚠️ WRITTEN but UNVALIDATED — needs a real ROS 2 / GPU box

These compile against the documented APIs but could not be executed here.

### 1. ROS environment / images
- [ ] Install **nvidia-container-toolkit** on the host (Arch AUR) — `docs/00`, `RUN.md` B0. **Step 0; nothing GPU works without it.**
- [ ] `make build` — build `docker/Dockerfile.ros` and `docker/Dockerfile.curobo`.
  - [ ] **cuRobo image is the riskiest artifact.** Verify `CUROBO_REF`, the torch/CUDA
        pins, and `TORCH_CUDA_ARCH_LIST=8.6+PTX` against the current cuRobo release. If the
        build breaks, prefer cuRobo's official Dockerfile as the base.
  - [ ] Confirm the `ros-humble-*` apt package names in `Dockerfile.ros` resolve (esp.
        `ros-humble-moveit-py`, `ros-humble-ur*`, `ros-humble-warehouse-ros-mongo`).

### 2. TASK 1 — UR5e simulation (`ros2_ws/src/mb_bringup`)
- [ ] `make sim` brings up UR5e (mock hw) + MoveIt + RViz.
- [ ] Verify the mock-hardware controller fallback (`joint_trajectory_controller`) and the
      `ur_moveit.launch.py` argument names for your UR driver version.

### 3. Pipeline B — harness adapters (the heart: tasks 3/4)
- [ ] **MoveIt adapter** (`adapters/moveit_adapter.py`): verify `moveit_py` symbol names
      on Humble (`MoveItPy`, `PlanRequestParameters`, `RobotState`), the planning group
      name (`ur_manipulator`), and the OMPL planner-id strings vs `ur_moveit_config`'s
      `ompl_planning.yaml`. **Fallback** if `moveit_py` is unusable: a MoveGroup action
      client (see `docs/03`). The `config_dict` build via `MoveItConfigsBuilder` lets
      `mb-benchmark run` work without a launch wrapper — confirm it.
- [ ] **cuRobo adapter** (`adapters/curobo_adapter.py`): verify `MotionGen` API
      (`plan_single`, `get_interpolated_plan`, `success.item()`), the **wxyz** quaternion
      convention, and that the **first plan is discarded** (warmup) — `runner.py` already
      does this via `requires_warmup`. Tune `_MOTION_GEN_KWARGS` for the **8 GB** GPU if OOM.
- [ ] Run `make benchmark`; sanity-check `results/report/report.md` (cuRobo should be
      fast/smooth; MoveIt planners spread across time/quality).

### 4. TASK 2 — Pipeline A (`ros2_ws/src/mb_moveit_benchmarks`)
- [ ] Verify `config/benchmark.yaml` keys against your `moveit_ros_benchmarks`
      `BenchmarkOptions.hpp` (schema drifts across versions).
- [ ] **Finish warehouse population** — `scripts/populate_warehouse.py` exports `.scene`
      files and can apply scenes live, but the **persist-to-MongoDB** step is a TODO.
      Easiest reliable path: import the `.scene` files via RViz (Stored Scenes → Import
      From Text → Save) + add Stored States. Then `make benchmark-moveit`.
- [ ] `process_results.sh` → `benchmark.db` → Planner Arena.

> Pipeline A is intentionally **non-blocking**: Pipeline B independently re-measures the
> MoveIt planners, so task-2's intent is covered even if the warehouse step lags.

### 5. Robot fidelity (decision to confirm)
- [ ] Default is **ur5e everywhere** (cuRobo ships a validated `ur5e.yml`). If the
      assignment strictly needs **UR5 (CB-series)**, follow `config/robot/README.md` to
      generate a `ur5.yml` + **fresh collision spheres** (do NOT reuse ur5e spheres), set
      `ur_type:=ur5`, and pass `robot_config="ur5.yml"` to `CuRoboAdapter`.

---

## Suggested order for the next session
1. `docs/00` host setup → `make build`.
2. `make sim` (proves task 1 + the ROS image).
3. `make benchmark` with **MoveIt planners only** first (drop cuRobo) to validate the
   harness on ROS, then add cuRobo once its image builds.
4. cuRobo: get one `plan_single` working in `make shell-curobo` before the full sweep.
5. Pipeline A last (most fiddly; optional for the core result).

## Gotchas already designed-around (don't reintroduce)
- Base collision sphere vs ground slab → `collision_spheres` skips the static base column.
- cuRobo warmup → `runner.py` discards the first plan for `requires_warmup` planners.
- Metrics computed downstream (not per-adapter) → identical scoring for every planner.
- Buffered stdout hides progress under pipes → use `python -u` when debugging hangs.
