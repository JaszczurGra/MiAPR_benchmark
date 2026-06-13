"""End-to-end offline pipeline + registry + adapter contract tests."""

import numpy as np

from mb_benchmark.adapters import available_planners, get_adapter
from mb_benchmark.analysis import build_report, metrics_from_raw, summarize
from mb_benchmark.generation import generate_queries
from mb_benchmark.runner import run_benchmark
from mb_benchmark.scenario import Obstacle, Scenario


def test_registry_has_all_families():
    avail = set(available_planners())
    assert "synthetic:rrtconnect" in avail
    assert "synthetic:curobo" in avail
    assert "straightline" in avail
    assert "curobo" in avail  # constructible even with no GPU
    assert any(p.startswith("moveit:") for p in avail)


def test_straightline_adapter_contract():
    a = get_adapter("straightline")
    a.setup("ur5e", [])
    from mb_benchmark.scenario import Query

    q = Query(id=0, start=np.zeros(6), goal_joint=np.full(6, 0.4))
    res = a.plan(q, timeout=5.0, seed=0)
    assert res.success
    assert len(res.trajectory) >= 2


def test_curobo_setup_errors_without_gpu():
    """The cuRobo adapter must be constructible offline and fail with a clear message
    only when setup() is actually called (heavy import deferred)."""
    a = get_adapter("curobo")
    import pytest

    with pytest.raises(RuntimeError):
        a.setup("ur5e", [])


def _toy_scenario():
    sc = Scenario(
        name="toy", robot="ur5e",
        obstacles=[Obstacle.from_dict({"type": "box", "name": "b", "size": [0.1, 0.4, 0.4],
                                       "pose": {"xyz": [0.45, 0.0, 0.35]}})],
        generation={"seed": 7},
    )
    generate_queries(sc, num_queries=4, seed=7)
    return {"toy": sc}


def test_full_offline_pipeline(tmp_path):
    scenarios = _toy_scenario()
    planners = ["synthetic:rrtconnect", "synthetic:curobo", "straightline"]
    run_benchmark(scenarios, planners, runs=3, timeout=10.0, seed=7,
                  out_dir=str(tmp_path), log=lambda *_: None)
    df = metrics_from_raw(tmp_path / "raw", scenarios)
    assert len(df) == 4 * 3 * len(planners)
    assert set(df["planner"].unique()) == set(planners)

    summary = summarize(df)
    assert "success_rate" in summary.columns

    report = build_report(df, tmp_path / "report")
    assert report.exists()
    assert (tmp_path / "report" / "metrics.csv").exists()
    assert (tmp_path / "report" / "summary.csv").exists()
    assert (tmp_path / "report" / "plots" / "success_rate.png").exists()


def test_warmup_run_is_discarded(tmp_path):
    """curobo synthetic has requires_warmup=True; the warmup plan (run=-1) must NOT be
    persisted, so only the requested runs land in raw results."""
    scenarios = _toy_scenario()
    run_benchmark(scenarios, ["synthetic:curobo"], runs=2, timeout=10.0, seed=7,
                  out_dir=str(tmp_path), log=lambda *_: None)
    raw_files = list((tmp_path / "raw").rglob("*.json"))
    assert all("r-1" not in f.name for f in raw_files)
    assert len(raw_files) == 4 * 2  # 4 queries x 2 runs, no warmup file
