# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass, asdict
from math import cos, sin, radians, degrees, atan2, asin, sqrt
from typing import Iterable, List, Sequence, Tuple


@dataclass
class Pose6D:
    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_any(cls, value):
        if isinstance(value, cls):
            return value
        if isinstance(value, dict):
            return cls(**{k: float(value.get(k, 0.0) or 0.0) for k in cls.__dataclass_fields__})
        if isinstance(value, (list, tuple)):
            vals = list(value) + [0.0] * 6
            return cls(*[float(x) for x in vals[:6]])
        return cls()


def pose_matrix(pose: Pose6D) -> List[List[float]]:
    """WORLD_T_LOCAL, Euler ZYX: yaw(Z) * pitch(Y) * roll(X)."""
    r, p, y = map(radians, (pose.roll_deg, pose.pitch_deg, pose.yaw_deg))
    cr, sr, cp, sp, cy, sy = cos(r), sin(r), cos(p), sin(p), cos(y), sin(y)
    R = [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ]
    return [
        [R[0][0], R[0][1], R[0][2], pose.x_mm],
        [R[1][0], R[1][1], R[1][2], pose.y_mm],
        [R[2][0], R[2][1], R[2][2], pose.z_mm],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matmul4(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def invert_rigid4(m):
    """Inverse of a homogeneous rigid transform without external dependencies."""
    r_t = [[float(m[j][i]) for j in range(3)] for i in range(3)]
    t = [float(m[i][3]) for i in range(3)]
    inv_t = [-sum(r_t[i][j] * t[j] for j in range(3)) for i in range(3)]
    return [
        [r_t[0][0], r_t[0][1], r_t[0][2], inv_t[0]],
        [r_t[1][0], r_t[1][1], r_t[1][2], inv_t[1]],
        [r_t[2][0], r_t[2][1], r_t[2][2], inv_t[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matrix_pose(m) -> Pose6D:
    # inverse of the ZYX convention above
    sp = max(-1.0, min(1.0, -m[2][0]))
    pitch = asin(sp)
    cp = cos(pitch)
    if abs(cp) > 1e-8:
        roll = atan2(m[2][1], m[2][2])
        yaw = atan2(m[1][0], m[0][0])
    else:
        roll = 0.0
        yaw = atan2(-m[0][1], m[1][1])
    return Pose6D(
        x_mm=m[0][3], y_mm=m[1][3], z_mm=m[2][3],
        roll_deg=degrees(roll), pitch_deg=degrees(pitch), yaw_deg=degrees(yaw),
    )


def compose_pose(parent: Pose6D, mount: Pose6D) -> Pose6D:
    return matrix_pose(matmul4(pose_matrix(parent), pose_matrix(mount)))


def relative_pose(parent: Pose6D, child: Pose6D) -> Pose6D:
    """Return parent_T_child for two WORLD poses."""
    return matrix_pose(matmul4(invert_rigid4(pose_matrix(parent)), pose_matrix(child)))


def parent_pose_for_child(child_world: Pose6D, child_local: Pose6D) -> Pose6D:
    """Solve WORLD_T_parent when WORLD_T_child and parent_T_child are known."""
    return matrix_pose(matmul4(pose_matrix(child_world), invert_rigid4(pose_matrix(child_local))))


def transform_point(pose: Pose6D, point_xyz: Sequence[float]) -> List[float]:
    m = pose_matrix(pose)
    x, y, z = [float(v) for v in point_xyz[:3]]
    return [
        m[0][0]*x + m[0][1]*y + m[0][2]*z + m[0][3],
        m[1][0]*x + m[1][1]*y + m[1][2]*z + m[1][3],
        m[2][0]*x + m[2][1]*y + m[2][2]*z + m[2][3],
    ]


def rotate_vector(pose: Pose6D, vec: Sequence[float]) -> List[float]:
    m = pose_matrix(pose)
    x, y, z = [float(v) for v in vec[:3]]
    return [
        m[0][0]*x + m[0][1]*y + m[0][2]*z,
        m[1][0]*x + m[1][1]*y + m[1][2]*z,
        m[2][0]*x + m[2][1]*y + m[2][2]*z,
    ]


def vsub(a, b): return [float(a[i]) - float(b[i]) for i in range(3)]
def vadd(a, b): return [float(a[i]) + float(b[i]) for i in range(3)]
def vscale(a, s): return [float(a[i]) * float(s) for i in range(3)]
def dot(a, b): return sum(float(a[i]) * float(b[i]) for i in range(3))
def cross(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
def norm(a): return sqrt(dot(a, a))


def plane_from_points(p1, p2, p3):
    n = cross(vsub(p2, p1), vsub(p3, p1))
    ln = norm(n)
    if ln < 1e-8:
        raise ValueError("三个点无法确定有效平面")
    n = [v / ln for v in n]
    d = -dot(n, p1)
    return n, d


def ray_plane_intersection(origin, direction, plane_normal, plane_d):
    den = dot(plane_normal, direction)
    if abs(den) < 1e-9:
        return None
    t = -(dot(plane_normal, origin) + plane_d) / den
    if t <= 0:
        return None
    return vadd(origin, vscale(direction, t))
