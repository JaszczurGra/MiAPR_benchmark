# CLAUDE.md — orientation for agents working in this repo

## What this is
A ROS 2 benchmark environment comparing motion planners for a UR5e arm: MoveIt planners
vs **cuRobo** (a non-MoveIt, GPU planner), on shared scenarios with uniform metrics, plus
a plugin interface to add more planners. Implements the 5 goals in `task.txt`.

## The one constraint that shaped everything
It was built **with no ROS and no GPU available**. So: the pure-Python core is fully
testable offline, and *all* ROS/GPU code is isolated behind a thin adapter interface in
exactly two files (`adapters/moveit_adapter.py`, `adapters/curobo_adapter.py`), whose
heavy imports are deferred into `setup()`. **Keep it that way** — don't add ROS/torch
imports at module top level or into the offline core.

## Architecture in one picture
```
scenario YAML ─► Adapter.plan() ─► PlanResult{trajectory} ─► results/raw/*.json
                  (per-framework)                              │
                                         metrics.py (ONE path) ─► metrics.csv ─► analysis ─► report
```
Adapters produce only trajectories; metrics are computed once, downstream, identically
for every planner. This is the core fairness invariant — preserve it.

## Map
- `benchmark/mb_benchmark/` — the harness (Pipeline B). Offline core: `scenario.py`,
  `generation.py`, `kinematics.py`, `geometry.py`, `collision.py`, `metrics.py`,
  `runner.py`, `analysis.py`, `cli.py`. Adapters in `adapters/`.
- `scenarios/library/*.yaml` — obstacle worlds + generation settings (single source of truth).
- `ros2_ws/src/mb_bringup` — task 1 (UR5e sim launch).
- `ros2_ws/src/mb_moveit_benchmarks` — task 2 (Pipeline A: moveit_ros_benchmarks).
- `docker/`, `Makefile`, `scripts/` — build & run orchestration.
- `HANDOFF.md` — **read this first**: what's done vs what needs a real ROS/GPU box.
- `METHODOLOGY.md` — metric formulas + design decisions.

## How to verify your changes (offline, do this before/after editing the core)
```bash
make offline      # venv + install + 30 tests   (or: . .venv/bin/activate && pytest benchmark/tests -q)
make demo         # full pipeline on synthetic planners -> results/demo/report
```
Run these without ROS/GPU. If you touch metrics/collision/generation, add/extend a test.

## Conventions
- Defaults are **ur5e**, **Humble**, **mock hardware** (see `METHODOLOGY.md` for why).
- Quaternions are `[x,y,z,w]` in our code; **cuRobo uses `[w,x,y,z]`** (convert at the boundary).
- Add a planner = subclass `PlannerAdapter`, `register(...)`, import in `adapters/__init__.py`
  (worked example: `template_adapter.py`; guide: `docs/05`).
- Keep stdout `python -u` when debugging container/pipe hangs (buffering hides progress).
