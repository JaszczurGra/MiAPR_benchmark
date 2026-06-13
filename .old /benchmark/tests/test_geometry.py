import numpy as np

from mb_benchmark.geometry import (
    matrix_to_quat,
    quat_to_matrix,
    rpy_to_matrix,
    sdf_box,
    sdf_cylinder,
    sdf_sphere,
    world_to_local,
)


def test_quat_matrix_roundtrip():
    for rpy in ([0, 0, 0], [0.3, -0.2, 1.1], [np.pi / 2, 0, 0]):
        r = rpy_to_matrix(rpy)
        q = matrix_to_quat(r)
        r2 = quat_to_matrix(q)
        assert np.allclose(r, r2, atol=1e-8)


def test_rotation_is_orthonormal():
    r = rpy_to_matrix([0.5, 1.2, -0.7])
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(r), 1.0)


def test_sdf_sphere():
    assert np.isclose(sdf_sphere(np.array([2.0, 0, 0]), 1.0), 1.0)
    assert np.isclose(sdf_sphere(np.array([0.5, 0, 0]), 1.0), -0.5)


def test_sdf_box():
    half = np.array([1.0, 1.0, 1.0])
    assert np.isclose(sdf_box(np.array([2.0, 0, 0]), half), 1.0)
    assert sdf_box(np.array([0.0, 0, 0]), half) < 0  # inside


def test_sdf_cylinder():
    # radius 1, half-length 1 along z
    assert np.isclose(sdf_cylinder(np.array([2.0, 0, 0]), 1.0, 1.0), 1.0)
    assert sdf_cylinder(np.array([0.0, 0, 0]), 1.0, 1.0) < 0


def test_world_to_local():
    r = rpy_to_matrix([0, 0, np.pi / 2])
    t = np.array([1.0, 0.0, 0.0])
    p = np.array([1.0, 1.0, 0.0])
    local = world_to_local(p, r, t)
    assert np.allclose(local, [1.0, 0.0, 0.0], atol=1e-9)
