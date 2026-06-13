import numpy as np

from mb_benchmark.adapters.base import PlanResult
from mb_benchmark.kinematics import get_robot
from mb_benchmark.metrics import (
    cartesian_path_length,
    compute_metrics,
    joint_path_length,
    smoothness_geom,
)


def test_joint_path_length_straight():
    a, b = np.zeros(6), np.ones(6)
    traj = np.array([a, b])
    assert np.isclose(joint_path_length(traj), np.linalg.norm(b - a))


def test_smoothness_zero_for_collinear():
    # points along a straight line in joint space -> zero turning -> smoothness 0
    traj = np.array([np.full(6, t) for t in np.linspace(0, 1, 10)])
    assert smoothness_geom(traj) < 1e-9


def test_smoothness_positive_for_zigzag():
    traj = np.array([[0, 0, 0, 0, 0, 0],
                     [1, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0],
                     [1, 1, 0, 0, 0, 0]], dtype=float)
    assert smoothness_geom(traj) > 0.0


def test_cartesian_length_nonnegative():
    m = get_robot("ur5e")
    traj = np.array([np.zeros(6), np.full(6, 0.3), np.full(6, 0.6)])
    assert cartesian_path_length(traj, m) >= 0


def test_compute_metrics_failure_row():
    m = get_robot("ur5e")
    res = PlanResult(planner="p", scenario="s", query_id=0, success=False)
    row = compute_metrics(res, [], m)
    assert row["success"] is False
    assert row["path_valid"] is False
    assert np.isnan(row["joint_path_length"])


def test_compute_metrics_success_row():
    m = get_robot("ur5e")
    traj = np.array([np.zeros(6), np.full(6, 0.2), np.full(6, 0.4)])
    res = PlanResult(planner="p", scenario="s", query_id=0, success=True,
                     planning_time_s=0.1, trajectory=traj.tolist(),
                     time_from_start=[0.0, 0.5, 1.0])
    row = compute_metrics(res, [], m)
    assert row["success"] is True
    assert row["num_waypoints"] == 3
    assert row["clearance"] == float("inf")  # no obstacles
    assert row["joint_path_length"] > 0
