import numpy as np

from mb_benchmark.kinematics import URModel, get_robot


def test_fk_shapes_and_orthonormal():
    m = get_robot("ur5e")
    q = np.zeros(6)
    frames = m.fk_frames(q)
    assert len(frames) == 7  # base + 6 joints
    ee = m.fk_ee(q)
    assert ee.shape == (4, 4)
    r = ee[:3, :3]
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-8)


def test_fk_within_reach():
    m = get_robot("ur5e")
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.uniform(-np.pi, np.pi, size=6)
        p = m.fk_ee(q)[:3, 3]
        assert np.linalg.norm(p) < 1.2  # UR5e reach ~0.85 m + offsets


def test_joint1_rotates_about_base_z():
    m = get_robot("ur5e")
    p0 = m.fk_ee(np.array([0.0, -0.5, 0.5, 0, 0, 0]))[:3, 3]
    p1 = m.fk_ee(np.array([np.pi / 2, -0.5, 0.5, 0, 0, 0]))[:3, 3]
    assert np.isclose(p0[2], p1[2], atol=1e-6)  # z unchanged by base rotation
    assert not np.allclose(p0[:2], p1[:2])


def test_collision_spheres_nonempty():
    m = get_robot("ur5e")
    centers, radii = m.collision_spheres(np.zeros(6))
    assert len(centers) == len(radii) > 0
    assert np.all(radii > 0)


def test_unknown_robot():
    import pytest

    with pytest.raises(ValueError):
        get_robot("panda")
