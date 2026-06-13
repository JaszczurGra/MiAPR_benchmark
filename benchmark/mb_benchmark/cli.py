"""Command-line entry point: ``mb-benchmark <command>``.

Commands
--------
generate       seed + write collision-free queries for each scenario
run            drive planners over scenarios -> results/raw/*.json
metrics        score raw results -> metrics.csv (uniform code path)
report         metrics.csv (or raw) -> summary table + plots + report.md
demo           generate -> run(synthetic) -> metrics -> report, fully offline
list-planners  show adapters registered in this environment
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, List

import pandas as pd

from . import adapters  # noqa: F401  (populates the registry)
from .adapters.base import available_planners
from .analysis import build_report, metrics_from_raw
from .generation import generate_queries
from .runner import run_benchmark
from .scenario import Scenario, load_library, save_scenario

DEFAULT_LIBRARY = "scenarios/library"
DEFAULT_GENERATED = "scenarios/generated"


def _expand_planners(spec: str) -> List[str]:
    avail = available_planners()
    if spec in ("all", "*"):
        return avail
    if spec == "all-synthetic":
        return [p for p in avail if p.startswith("synthetic:")]
    if spec == "demo":
        return [p for p in avail if p.startswith("synthetic:")] + ["straightline"]
    return [p.strip() for p in spec.split(",") if p.strip()]


def _load_scenarios(directory: str, autogen: bool, seed: int, num: int) -> Dict[str, Scenario]:
    scenarios = load_library(directory)
    if not scenarios:
        raise SystemExit(f"no scenarios found in {directory}")
    if autogen:
        for sc in scenarios.values():
            if not sc.queries:
                generate_queries(sc, num_queries=num, seed=seed)
    return scenarios


# --------------------------------------------------------------------------- #
def cmd_generate(args) -> None:
    scenarios = load_library(args.scenarios)
    out = Path(args.out)
    for sc in scenarios.values():
        generate_queries(sc, num_queries=args.num, seed=args.seed, goal_type=args.goal_type)
        save_scenario(sc, out / f"{sc.name}.yaml")
        print(f"[generate] {sc.name}: {len(sc.queries)} queries -> {out / (sc.name + '.yaml')}")


def cmd_run(args) -> None:
    scenarios = _load_scenarios(args.scenarios, autogen=not args.no_autogen, seed=args.seed, num=args.num)
    planners = _expand_planners(args.planners)
    print(f"[run] planners={planners}")
    run_benchmark(
        scenarios, planners, runs=args.runs, timeout=args.timeout,
        seed=args.seed, out_dir=args.out, warmup=not args.no_warmup,
    )


def cmd_metrics(args) -> None:
    scenarios = _load_scenarios(args.scenarios, autogen=not args.no_autogen, seed=args.seed, num=args.num)
    df = metrics_from_raw(Path(args.raw), scenarios)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out, index=False)
    print(f"[metrics] {len(df)} rows -> {out}")


def cmd_report(args) -> None:
    if args.metrics:
        df = pd.read_csv(args.metrics)
    else:
        scenarios = _load_scenarios(args.scenarios, autogen=not args.no_autogen, seed=args.seed, num=args.num)
        df = metrics_from_raw(Path(args.raw), scenarios)
    report = build_report(df, args.out)
    print(f"[report] -> {report}")


def cmd_demo(args) -> None:
    out = Path(args.out)
    scenarios = load_library(args.scenarios)
    if not scenarios:
        raise SystemExit(f"no scenarios found in {args.scenarios}")
    for sc in scenarios.values():
        generate_queries(sc, num_queries=args.num, seed=args.seed)
        save_scenario(sc, out / "scenarios" / f"{sc.name}.yaml")
    planners = _expand_planners("demo")
    print(f"[demo] scenarios={list(scenarios)} planners={planners}")
    run_benchmark(scenarios, planners, runs=args.runs, timeout=args.timeout,
                  seed=args.seed, out_dir=str(out))
    df = metrics_from_raw(out / "raw", scenarios)
    report = build_report(df, out / "report")
    print(f"[demo] done. Open {report}")


def cmd_list(args) -> None:
    print("Registered planners (constructible; ROS/GPU ones still need their runtime):")
    for p in available_planners():
        print(f"  {p}")


# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="mb-benchmark", description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)

    def common(sp, scenarios_default=DEFAULT_LIBRARY):
        sp.add_argument("--scenarios", default=scenarios_default)
        sp.add_argument("--seed", type=int, default=42)
        sp.add_argument("--num", type=int, default=20, help="queries per scenario (autogen)")
        sp.add_argument("--no-autogen", action="store_true")

    g = sub.add_parser("generate", help="write seeded queries")
    g.add_argument("--scenarios", default=DEFAULT_LIBRARY)
    g.add_argument("--out", default=DEFAULT_GENERATED)
    g.add_argument("--num", type=int, default=20)
    g.add_argument("--seed", type=int, default=42)
    g.add_argument("--goal-type", default="joint", choices=["joint", "pose"])
    g.set_defaults(func=cmd_generate)

    r = sub.add_parser("run", help="run planners -> results/raw")
    common(r, DEFAULT_LIBRARY)
    r.add_argument("--planners", default="demo", help="comma list, or all / all-synthetic / demo")
    r.add_argument("--runs", type=int, default=5)
    r.add_argument("--timeout", type=float, default=10.0)
    r.add_argument("--out", default="results")
    r.add_argument("--no-warmup", action="store_true")
    r.set_defaults(func=cmd_run)

    m = sub.add_parser("metrics", help="score raw results -> metrics.csv")
    common(m, DEFAULT_LIBRARY)
    m.add_argument("--raw", default="results/raw")
    m.add_argument("--out", default="results/metrics.csv")
    m.set_defaults(func=cmd_metrics)

    rep = sub.add_parser("report", help="summary table + plots")
    common(rep, DEFAULT_LIBRARY)
    rep.add_argument("--raw", default="results/raw")
    rep.add_argument("--metrics", default=None, help="use an existing metrics.csv instead of --raw")
    rep.add_argument("--out", default="results/report")
    rep.set_defaults(func=cmd_report)

    d = sub.add_parser("demo", help="offline end-to-end (no ROS/GPU)")
    d.add_argument("--scenarios", default=DEFAULT_LIBRARY)
    d.add_argument("--out", default="results/demo")
    d.add_argument("--num", type=int, default=15)
    d.add_argument("--seed", type=int, default=42)
    d.add_argument("--runs", type=int, default=5)
    d.add_argument("--timeout", type=float, default=10.0)
    d.set_defaults(func=cmd_demo)

    lst = sub.add_parser("list-planners", help="show registered adapters")
    lst.set_defaults(func=cmd_list)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    args.func(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
