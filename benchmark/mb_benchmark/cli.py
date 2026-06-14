"""Command-line entry point: ``mb-benchmark <command>``.

Commands
--------
generate       seed + write collision-free queries for each scenario
run            drive planners over scenarios -> results/raw/*.json
metrics        score raw results -> metrics.csv (uniform code path)
report         metrics.csv (or raw) -> summary table + plots + report.md
list-planners  show adapters registered in this environment
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Dict, List

import pandas as pd

from . import adapters  # noqa: F401  (populates the registry)
from .adapters.base import available_planners
from .analysis import build_report, metrics_from_raw
from .config import HarnessConfig, load_harness_config
from .generation import generate_queries
from .runner import run_benchmark
from .scenario import Scenario, load_library, save_scenario

DEFAULT_LIBRARY = "scenarios/library"
DEFAULT_GENERATED = "scenarios/generated"


def _expand_planners(spec: str, cfg: HarnessConfig) -> List[str]:
    """Turn a --planners spec into concrete planner names. Tokens are comma-separated;
    each is a planner name, ``all``/``*`` (every registered adapter), or ``@group`` to
    pull a list from config/planners.yaml (``harness.<group>``). Order is preserved and
    duplicates removed, so e.g. ``@baselines,curobo`` works."""
    avail = available_planners()
    out: List[str] = []
    for tok in (t.strip() for t in spec.split(",")):
        if not tok:
            continue
        if tok in ("all", "*"):
            out.extend(avail)
        elif tok.startswith("@"):
            out.extend(cfg.group(tok[1:]))
        else:
            out.append(tok)
    seen: set = set()
    return [p for p in out if not (p in seen or seen.add(p))]


def _resolve_num(args) -> int:
    """Queries-per-scenario ("maps"): explicit --num wins, else config defaults.num."""
    if args.num is not None:
        return args.num
    return load_harness_config(getattr(args, "config", None)).num


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
def _hard_exit() -> None:
    """Flush and os._exit(0). moveit_py's MoveItCpp segfaults during normal interpreter
    shutdown (a C++ global-teardown bug); all results are already on disk by the time we
    call this, so a hard exit keeps the process's exit code clean (0) -- otherwise the
    crash would surface as 139 and abort `set -e` scripts like run_harness.sh. Only used on
    the moveit_py code paths, so offline commands/tests exit normally."""
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


def cmd_generate(args) -> None:
    scenarios = load_library(args.scenarios)
    out = Path(args.out)
    num = _resolve_num(args)

    # Optional: validate start/goal states with MoveIt's mesh + self-collision model so the
    # shared query set is valid under the strictest planner (must run in the `ros` container).
    validator = None
    if getattr(args, "validate_moveit", False):
        from .adapters.moveit_adapter import MoveItAdapter

        validator = MoveItAdapter(node_name="mb_moveit_validate")

    for sc in scenarios.values():
        is_valid = is_solvable = None
        if validator is not None:
            validator.setup(sc.robot, sc.obstacles)  # builds MoveItPy once, loads this world
            is_valid = validator.is_state_valid
            # keep only pairs MoveIt can actually connect (a path exists)
            is_solvable = lambda s, g: validator.is_solvable(s, g, timeout=args.solve_timeout)
        generate_queries(
            sc, num_queries=num, seed=args.seed, goal_type=args.goal_type,
            is_valid=is_valid, is_solvable=is_solvable,
        )
        save_scenario(sc, out / f"{sc.name}.yaml")
        print(f"[generate] {sc.name}: {len(sc.queries)} queries -> {out / (sc.name + '.yaml')}")

    if validator is not None:
        _hard_exit()


def cmd_run(args) -> None:
    cfg = load_harness_config(args.config)
    num = args.num if args.num is not None else cfg.num
    scenarios = _load_scenarios(args.scenarios, autogen=not args.no_autogen, seed=args.seed, num=num)
    try:
        planners = _expand_planners(args.planners, cfg)
    except KeyError as e:
        raise SystemExit(str(e).strip('"'))
    # config/planners.yaml supplies runs/timeout unless overridden on the command line.
    runs = args.runs if args.runs is not None else cfg.runs
    timeout = args.timeout if args.timeout is not None else cfg.timeout
    print(f"[run] planners={planners} runs={runs} timeout={timeout}  "
          f"(config: {cfg.path or 'built-in defaults'})")
    run_benchmark(
        scenarios, planners, runs=runs, timeout=timeout,
        seed=args.seed, out_dir=args.out, warmup=not args.no_warmup,
    )
    # moveit_py segfaults on interpreter shutdown; results are already saved, so hard-exit.
    if any(p.startswith("moveit:") for p in planners):
        _hard_exit()


def cmd_metrics(args) -> None:
    scenarios = _load_scenarios(args.scenarios, autogen=not args.no_autogen, seed=args.seed, num=_resolve_num(args))
    df = metrics_from_raw(Path(args.raw), scenarios)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out, index=False)
    print(f"[metrics] {len(df)} rows -> {out}")


def cmd_report(args) -> None:
    if args.metrics:
        df = pd.read_csv(args.metrics)
    else:
        scenarios = _load_scenarios(args.scenarios, autogen=not args.no_autogen, seed=args.seed, num=_resolve_num(args))
        df = metrics_from_raw(Path(args.raw), scenarios)
    build_report(df, args.out)


def cmd_list(args) -> None:
    print("Registered planners (constructible; ROS/GPU ones still need their runtime):")
    for p in available_planners():
        print(f"  {p}")
    cfg = load_harness_config(getattr(args, "config", None))
    if cfg.groups:
        print(f"\nPlanner groups from {cfg.path} (use as --planners @<group>):")
        for g, members in cfg.groups.items():
            print(f"  @{g}: {', '.join(members)}")
        print(f"\nRun defaults from config: runs={cfg.runs} timeout={cfg.timeout} num={cfg.num}")


# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="mb-benchmark", description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)

    def common(sp, scenarios_default=DEFAULT_LIBRARY):
        sp.add_argument("--scenarios", default=scenarios_default)
        sp.add_argument("--seed", type=int, default=42)
        sp.add_argument("--num", type=int, default=None,
                        help="queries per scenario / 'maps' (autogen); "
                             "default: config/planners.yaml defaults.num")
        sp.add_argument("--no-autogen", action="store_true")

    g = sub.add_parser("generate", help="write seeded queries")
    g.add_argument("--scenarios", default=DEFAULT_LIBRARY)
    g.add_argument("--out", default=DEFAULT_GENERATED)
    g.add_argument("--num", type=int, default=None,
                   help="queries per scenario / 'maps'; default: config/planners.yaml defaults.num")
    g.add_argument("--seed", type=int, default=42)
    g.add_argument("--goal-type", default="joint", choices=["joint", "pose"])
    g.add_argument("--validate-moveit", action="store_true",
                   help="validate start/goal with MoveIt (mesh+self collision) AND keep only "
                        "pairs MoveIt can actually connect; run in the `ros` container so the "
                        "shared query set is valid and well-posed for every planner")
    g.add_argument("--solve-timeout", type=float, default=5.0,
                   help="per-query planning budget for the --validate-moveit solvability check")
    g.set_defaults(func=cmd_generate)

    r = sub.add_parser("run", help="run planners -> results/raw")
    common(r, DEFAULT_LIBRARY)
    r.add_argument("--planners", default="straightline",
                   help="comma list of names, 'all', or @group "
                        "(groups from config/planners.yaml: @moveit/@curobo/@baselines)")
    r.add_argument("--runs", type=int, default=None,
                   help="repeats per query; default: config/planners.yaml defaults.runs")
    r.add_argument("--timeout", type=float, default=None,
                   help="per-plan budget (s); default: config/planners.yaml defaults.timeout")
    r.add_argument("--config", default=None,
                   help="path to planners.yaml (default: auto-locate config/planners.yaml)")
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

    lst = sub.add_parser("list-planners", help="show registered adapters + config groups")
    lst.add_argument("--config", default=None, help="path to planners.yaml (default: auto-locate)")
    lst.set_defaults(func=cmd_list)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    args.func(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
