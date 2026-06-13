# MiAPR — ROS 2 Manipulator Motion-Planning Benchmark

An environment for **benchmarking motion-planning algorithms for a UR5(e) manipulator in
ROS 2**, comparing MoveIt's planners against a non-MoveIt planner (**cuRobo**) on
identical scenarios, with reproducible metrics — and an interface to drop in more planners.

Covers the five assignment goals (`task.txt`):

| # | Goal | Where |
|---|------|-------|
| 1 | Run a UR5(e) simulation in ROS 2 | `ros2_ws/src/mb_bringup` (mock-hw + MoveIt + RViz) |
| 2 | Benchmark planners via `moveit_ros_benchmarks` | `ros2_ws/src/mb_moveit_benchmarks` (Pipeline A → Planner Arena) |
| 3 | Comparative env: obstacles, random start/goal, repetitions, metrics | `benchmark/` harness + `scenarios/` (Pipeline B) |
| 4 | Integrate a non-MoveIt planner (cuRobo) | `benchmark/mb_benchmark/adapters/curobo_adapter.py` |
| 5 | Allow adding more planners | `PlannerAdapter` interface + `template_adapter.py` + `docs/05` |

## Two pipelines (kept numerically separate — see `METHODOLOGY.md`)

- **Pipeline A — `moveit_ros_benchmarks`**: the *standard* MoveIt benchmark + Planner
  Arena visualization. Satisfies task 2 literally, but only covers MoveIt planners.
- **Pipeline B — custom harness (`benchmark/`)**: drives **MoveIt and cuRobo** (and any
  future planner) through one adapter interface, on identical scenarios, scoring every
  trajectory with **one** metric code path. This is the only fair MoveIt-vs-cuRobo path
  and the home of tasks 3/4/5.

## Why Docker

The host is **Arch Linux** (no official ROS 2 binaries) with an **NVIDIA RTX 3060 Ti**.
Everything ROS/GPU runs in containers; the host never needs ROS installed. The
pure-Python harness core (scenarios, metrics, analysis) also runs **without** Docker.

## Quickstart — offline, runs today (no ROS, no GPU)

```bash
make offline        # venv + install + run the test suite (30 tests)
make demo           # generate -> run synthetic planners -> metrics -> report
#  -> open results/demo/report/report.md  (comparison table + plots)
```

## Full stack (after installing Docker + nvidia-container-toolkit — see `docs/00`)

```bash
make build              # build ros + curobo images
make sim                # TASK 1: UR5e sim + MoveIt + RViz
make benchmark-moveit   # TASK 2: moveit_ros_benchmarks -> Planner Arena db
make benchmark          # TASKS 3+4: harness over MoveIt + cuRobo -> results/report
```

## Where to look

- **[`RUN.md`](RUN.md)** — exact step-by-step, offline now and full-stack later.
- **[`HANDOFF.md`](HANDOFF.md)** — status + the checklist of what still needs a real
  ROS/GPU box (for whoever continues this).
- **[`METHODOLOGY.md`](METHODOLOGY.md)** — metric definitions, fairness, design decisions.
- **[`docs/`](docs/)** — per-component guides (setup, sim, pipelines, cuRobo, adding planners).
- **[`benchmark/README.md`](benchmark/README.md)** — the harness package.

## Status

- ✅ Offline core (scenarios, generation, metrics, analysis, synthetic planners): **done & tested** (30 passing tests, demo runs in ~11 s).
- ✅ MoveIt / cuRobo / Pipeline-A code: **written**, behind the adapter interface; needs a ROS/GPU box to validate (see `HANDOFF.md`).
