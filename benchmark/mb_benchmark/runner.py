"""Benchmark runner: iterate (scenario x planner x query x repetition), drive each
planner through its adapter, and persist raw :class:`PlanResult`s.

Crucially, this is the single place that handles **warmup exclusion**: planners with
``requires_warmup`` get one throwaway plan that is run but not recorded, so the first
CUDA-graph/JIT call never pollutes the timing. Metrics are NOT computed here -- adapters
emit only trajectories; scoring happens later in one uniform pass (``cli metrics``).
"""

from __future__ import annotations

import traceback
from pathlib import Path
from typing import Callable, Dict, List, Optional

from .adapters.base import PlanResult, get_adapter
from .scenario import Scenario


def _safe(name: str) -> str:
    return name.replace(":", "_").replace("/", "_")


def run_benchmark(
    scenarios: Dict[str, Scenario],
    planner_names: List[str],
    runs: int = 5,
    timeout: float = 10.0,
    seed: int = 0,
    out_dir: Optional[str] = None,
    warmup: bool = True,
    log: Callable[[str], None] = print,
) -> List[PlanResult]:
    """Run every planner on every scenario/query and return all raw results.

    Results are also written to ``<out_dir>/raw/<scenario>/<planner>/qID_rRUN.json``
    when ``out_dir`` is given.
    """
    raw_root = Path(out_dir) / "raw" if out_dir else None
    results: List[PlanResult] = []

    for planner_name in planner_names:
        log(f"[planner] {planner_name}")
        try:
            adapter = get_adapter(planner_name)
        except Exception as exc:
            log(f"  !! cannot construct adapter: {exc}")
            continue

        for scenario in scenarios.values():
            if not scenario.queries:
                log(f"  [scenario {scenario.name}] no queries -- run `generate` first; skipping")
                continue
            try:
                adapter.setup(scenario.robot, scenario.obstacles)
            except Exception as exc:
                log(f"  [scenario {scenario.name}] setup failed: {exc}")
                continue

            # Warmup: one un-recorded plan to absorb JIT / CUDA-graph capture.
            if warmup and getattr(adapter, "requires_warmup", False):
                try:
                    adapter.plan(scenario.queries[0], timeout, seed, run=-1)
                    log(f"  [scenario {scenario.name}] warmup done (discarded)")
                except Exception as exc:
                    log(f"  [scenario {scenario.name}] warmup error (continuing): {exc}")

            n_ok = 0
            for query in scenario.queries:
                for run in range(runs):
                    try:
                        res = adapter.plan(query, timeout, seed, run=run)
                    except Exception as exc:  # never let one query kill the sweep
                        res = PlanResult(
                            planner=planner_name, scenario=scenario.name,
                            query_id=query.id, run=run, seed=seed,
                            success=False, error=f"{exc}\n{traceback.format_exc()}",
                        )
                    res.planner = planner_name
                    res.scenario = scenario.name
                    if res.success:
                        n_ok += 1
                    results.append(res)
                    if raw_root is not None:
                        path = raw_root / scenario.name / _safe(planner_name) / f"q{query.id}_r{run}.json"
                        res.save(path)

            total = len(scenario.queries) * runs
            log(f"  [scenario {scenario.name}] {n_ok}/{total} solved")
            try:
                adapter.teardown()
            except Exception:
                pass

    if raw_root is not None:
        log(f"[done] raw results under {raw_root}")
    return results
