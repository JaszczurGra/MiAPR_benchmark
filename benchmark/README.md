# `mb_benchmark` — cross-framework motion-planning benchmark harness

Pure-Python framework that runs **any** motion planner through a common adapter
interface, on **identical** scenarios, and measures every planner with **one**
metric code path so results are comparable across frameworks (MoveIt vs cuRobo vs …).

## Why a separate harness (and not just `moveit_ros_benchmarks`)?

`moveit_ros_benchmarks` can only benchmark planners that live *inside* MoveIt. To
compare MoveIt planners against a non-MoveIt planner like **cuRobo** fairly, we need a
layer that drives both and measures both identically. That layer is this package.
See `../METHODOLOGY.md`.

## The key idea: thin adapters, central metrics

```
Adapter.plan(start, goal)  ->  PlanResult{ success, planning_time, trajectory }
                                          |
                       (saved to results/raw/*.json, per planner, per run)
                                          |
            MetricsCalculator(trajectory, robot FK, world)  ->  metrics.csv
                                          |
                                  analysis -> plots + report
```

Adapters **only produce trajectories**. All geometric metrics (path length,
smoothness, clearance) are computed afterwards by the same code for every planner.
This is what makes a MoveIt-vs-cuRobo comparison fair, and it means the whole
measurement + analysis half is testable **offline, with no ROS and no GPU**.

## Install (offline)

```bash
cd benchmark
pip install -e .
pytest                       # the offline test suite — green with no ROS/GPU
```

## Run the offline demo (no ROS, no GPU)

```bash
mb-benchmark demo --out ../results/demo
# generates seeded queries -> runs the SYNTHETIC planners -> metrics -> report
# look in ../results/demo/report/ for the comparison table + plots
```

## CLI

| Command | What it does | Needs ROS/GPU? |
|---|---|---|
| `mb-benchmark generate` | Sample seeded collision-free start/goal queries for scenarios | No |
| `mb-benchmark run`      | Run planners (registered adapters) on scenarios → `results/raw/` | depends on planner |
| `mb-benchmark metrics`  | Compute `metrics.csv` from `results/raw/` (one uniform code path) | No |
| `mb-benchmark report`   | Summary table + plots from `metrics.csv` | No |
| `mb-benchmark demo`     | `generate → run(synthetic) → metrics → report`, all offline | No |
| `mb-benchmark list-planners` | Show registered adapters available in this environment | No |

## Adding a planner (task 5)

Subclass `PlannerAdapter`, register it, done. See
`mb_benchmark/adapters/template_adapter.py` and `../docs/05_adding_planners.md`.
