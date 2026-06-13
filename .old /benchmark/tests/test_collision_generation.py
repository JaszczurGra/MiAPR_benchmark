import numpy as np

from mb_benchmark.collision import config_clearance, densify, in_collision, path_clearance
from mb_benchmark.generation import generate_queries
from mb_benchmark.kinematics import get_robot
from mb_benchmark.scenario import Obstacle, Scenario


def _sphere_at(center, radius):
    return Obstacle.from_dict(
        {"type": "sphere", "name": "o", "radius": radius, "pose": {"xyz": list(center)}}
    )


def test_clearance_far_obstacle_positive():
    m = get_robot("ur5e")
    q = np.zeros(6)
    far = _sphere_at([5.0, 5.0, 5.0], 0.1)
    assert config_clearance(m, q, [far]) > 0
    assert not in_collision(m, q, [far])


def test_collision_obstacle_on_arm():
    m = get_robot("ur5e")
    q = np.zeros(6)
    centers, _ = m.collision_spheres(q)
    obs = _sphere_at(centers[len(centers) // 2], 0.3)
    assert in_collision(m, q, [obs])


def test_no_obstacles_is_inf():
    m = get_robot("ur5e")
    assert config_clearance(m, np.zeros(6), []) == float("inf")


def test_densify_respects_step():
    traj = np.array([[0.0] * 6, [1.0] * 6])
    dense = densify(traj, max_joint_step=0.1)
    diffs = np.max(np.abs(np.diff(dense, axis=0)), axis=1)
    assert np.all(diffs <= 0.1 + 1e-9)
    assert np.allclose(dense[0], traj[0]) and np.allclose(dense[-1], traj[-1])


def test_generation_is_deterministic_and_free():
    sc1 = Scenario(name="s", robot="ur5e", obstacles=[_sphere_at([0.4, 0.0, 0.4], 0.1)])
    sc2 = Scenario(name="s", robot="ur5e", obstacles=[_sphere_at([0.4, 0.0, 0.4], 0.1)])
    q1 = generate_queries(sc1, num_queries=8, seed=123)
    q2 = generate_queries(sc2, num_queries=8, seed=123)
    assert len(q1) == 8
    m = get_robot("ur5e")
    for a, b in zip(q1, q2):
        assert np.allclose(a.start, b.start)
        assert np.allclose(a.goal_joint, b.goal_joint)
        # generated configs must be collision-free
        assert not in_collision(m, a.start, sc1.obstacles)
        assert not in_collision(m, a.goal_joint, sc1.obstacles)
        # goal_pose is FK of goal_joint
        assert np.allclose(a.goal_pose["xyz"], m.fk_ee(a.goal_joint)[:3, 3], atol=1e-6)
