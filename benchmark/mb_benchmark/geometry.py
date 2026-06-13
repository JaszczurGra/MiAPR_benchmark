"""Small, dependency-light geometry helpers: rotations, homogeneous transforms and
signed-distance functions for primitive shapes.

Conventions
-----------
* Rotations are 3x3 numpy arrays. Quaternions are ``[x, y, z, w]``.
* ``rpy`` follows the URDF/ROS convention: fixed-axis roll-pitch-yaw, i.e.
  ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``.
* Signed-distance functions (``sdf_*``) take a point already expressed in the
  shape's *local* frame and return distance to the surface: negative inside,
  positive outside, zero on the surface.
"""

from __future__ import annotations

import numpy as np


# --------------------------------------------------------------------------- #
# Rotations & transforms
# --------------------------------------------------------------------------- #
def rpy_to_matrix(rpy) -> np.ndarray:
    """URDF fixed-axis roll/pitch/yaw -> 3x3 rotation matrix."""
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def quat_to_matrix(quat) -> np.ndarray:
    """Quaternion [x, y, z, w] -> 3x3 rotation matrix."""
    x, y, z, w = (float(v) for v in quat)
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(r: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> quaternion [x, y, z, w]."""
    r = np.asarray(r, dtype=float)
    tr = np.trace(r)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def make_transform(rotation: np.ndarray, translation) -> np.ndarray:
    """Compose a 4x4 homogeneous transform from R (3x3) and t (3,)."""
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = np.asarray(translation, dtype=float)
    return t


def world_to_local(point, rotation: np.ndarray, translation):
    """Express world ``point`` (shape ``(3,)`` or ``(M, 3)``) in a frame (R, t)."""
    p = np.asarray(point, dtype=float)
    t = np.asarray(translation, dtype=float)
    if p.ndim == 1:
        return rotation.T @ (p - t)
    return (p - t) @ rotation  # row-wise R.T @ (p - t)


# --------------------------------------------------------------------------- #
# Signed-distance functions. Each accepts a single point ``(3,)`` -> float, or a
# batch ``(M, 3)`` -> ``(M,)`` array. Point(s) must already be in the shape's local
# frame. Negative inside, positive outside.
# --------------------------------------------------------------------------- #
def sdf_sphere(p_local: np.ndarray, radius: float):
    p = np.asarray(p_local, dtype=float)
    return np.linalg.norm(p, axis=-1) - radius


def sdf_box(p_local: np.ndarray, half_extents: np.ndarray):
    """Signed distance to an axis-aligned box (in local frame), given half sizes."""
    q = np.abs(np.asarray(p_local, dtype=float)) - np.asarray(half_extents, dtype=float)
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
    inside = np.minimum(np.max(q, axis=-1), 0.0)
    return outside + inside


def sdf_cylinder(p_local: np.ndarray, radius: float, half_length: float):
    """Signed distance to a finite cylinder aligned with local +Z, centred at origin
    (Inigo-Quilez capped-cylinder SDF)."""
    p = np.asarray(p_local, dtype=float)
    radial = np.hypot(p[..., 0], p[..., 1]) - radius
    axial = np.abs(p[..., 2]) - half_length
    d = np.stack([radial, axial], axis=-1)
    outside = np.linalg.norm(np.maximum(d, 0.0), axis=-1)
    inside = np.minimum(np.max(d, axis=-1), 0.0)
    return outside + inside
