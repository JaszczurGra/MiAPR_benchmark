"""Turn raw planner results into the comparison table + plots.

Two stages:
  1. ``metrics_from_raw`` — load every raw :class:`PlanResult` and score it with the
     uniform :func:`metrics.compute_metrics` (same code for all planners). -> tidy DF.
  2. ``build_report`` — aggregate per (planner, scenario) and render plots.

Pure Python; runs offline. Plotting uses the headless Agg backend.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

from .adapters.base import PlanResult  # noqa: E402
from .kinematics import get_robot  # noqa: E402
from .metrics import compute_metrics  # noqa: E402
from .scenario import Scenario  # noqa: E402

# Metrics shown in the report. (metric, nicer label, lower_is_better, success_only, log_scale)
_METRICS = [
    ("planning_time_s", "Czas planowania [s]", True, True, True),
    ("joint_path_length", "Długość ścieżki w przestrzeni stawów [rad]", True, True, False),
    ("cartesian_path_length", "Długość ścieżki kartezjańskiej [m]", True, True, False),
    ("smoothness_geom", "Gładkość (niżej = gładsza)", True, True, True),
    ("clearance", "Minimalny prześwit [m]", False, True, False),
]


def _to_markdown(df: pd.DataFrame) -> str:
    """Markdown table without requiring the optional ``tabulate`` package."""
    df = df.round(4)
    try:
        return df.to_markdown(index=False)
    except Exception:
        cols = list(df.columns)
        lines = ["| " + " | ".join(map(str, cols)) + " |",
                 "| " + " | ".join("---" for _ in cols) + " |"]
        for _, row in df.iterrows():
            lines.append("| " + " | ".join(str(row[c]) for c in cols) + " |")
        return "\n".join(lines)


def metrics_from_raw(raw_dir, scenarios: Dict[str, Scenario]) -> pd.DataFrame:
    """Score every raw result and return a tidy DataFrame (one row per plan attempt)."""
    raw_dir = Path(raw_dir)
    models = {name: get_robot(sc.robot) for name, sc in scenarios.items()}
    rows: List[Dict] = []
    for path in sorted(raw_dir.rglob("*.json")):
        res = PlanResult.load(path)
        sc = scenarios.get(res.scenario)
        if sc is None:
            continue
        rows.append(compute_metrics(res, sc.obstacles, models[res.scenario]))
    if not rows:
        raise RuntimeError(f"no scored results found under {raw_dir}")
    return pd.DataFrame(rows)


def summarize(df: pd.DataFrame) -> pd.DataFrame:
    """Aggregate per (planner, scenario): success rate + median/IQR of key metrics."""
    succ = df.groupby(["scenario", "planner"])["success"].mean().rename("success_rate")
    ok = df[df["success"]]
    agg = ok.groupby(["scenario", "planner"]).agg(
        planning_time_median=("planning_time_s", "median"),
        planning_time_mean=("planning_time_s", "mean"),
        joint_len_median=("joint_path_length", "median"),
        cartesian_len_median=("cartesian_path_length", "median"),
        smoothness_median=("smoothness_geom", "median"),
        clearance_median=("clearance", "median"),
        n_solved=("success", "size"),
    )
    return succ.to_frame().join(agg).reset_index()


def _boxplot(df: pd.DataFrame, metric: str, label: str, success_only: bool, out_path: Path,
             log_scale: bool = False) -> None:
    data = df[df["success"]] if success_only else df
    data = data[pd.notna(data[metric])]
    planners = sorted(data["planner"].unique())
    series = [data[data["planner"] == p][metric].values for p in planners]
    if not any(len(s) for s in series):
        return
    fig, ax = plt.subplots(figsize=(max(6, 1.1 * len(planners)), 4.5))
    ax.boxplot(series, showfliers=False)
    ax.set_xticks(range(1, len(planners) + 1))
    ax.set_xticklabels(planners)
    # ax.set_ylabel(label)
    ax.set_title(label)# + "  (wszystkie scenariusze)")
    if log_scale:
        ax.set_yscale("log")
    ax.tick_params(axis="x", rotation=45)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def _success_bar(df: pd.DataFrame, out_path: Path) -> None:
    rate = df.groupby("planner")["success"].mean().sort_index()
    fig, ax = plt.subplots(figsize=(max(6, 1.1 * len(rate)), 4.5))
    ax.bar(rate.index, rate.values)
    ax.set_ylim(0, 1)
    ax.set_ylabel("Współczynnik sukcesu")
    ax.set_title("Współczynnik sukcesu")
    ax.tick_params(axis="x", rotation=45)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def build_report(df: pd.DataFrame, out_dir, scenarios: Optional[Dict[str, Scenario]] = None) -> Path:
    """Write metrics.csv, summary.csv, plots and a Markdown report. Returns report path."""
    out_dir = Path(out_dir)
    plots_dir = out_dir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    df.to_csv(out_dir / "metrics.csv", index=False)
    summary = summarize(df)
    summary.to_csv(out_dir / "summary.csv", index=False)

    _success_bar(df, plots_dir / "success_rate.png")
    plot_files = ["plots/success_rate.png"]
    for metric, label, _lower, success_only, log_scale in _METRICS:
        if metric not in df.columns:
            continue
        fname = f"plots/{metric}.png"
        _boxplot(df, metric, label, success_only, plots_dir / f"{metric}.png", log_scale=log_scale)
        if (plots_dir / f"{metric}.png").exists():
            plot_files.append(fname)


    
    report = out_dir / "summary.txt"
    with open(report, "w") as fh:
        fh.write(f"Planners: {', '.join(sorted(df['planner'].unique()))}\n\n")
        fh.write(f"Scenarios: {', '.join(sorted(df['scenario'].unique()))}\n\n")
        fh.write(f"Total plan attempts: {len(df)}\n\n")
    print('Saved report to ', report)