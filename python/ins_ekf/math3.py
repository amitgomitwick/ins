"""Minimal 3D math helpers for the strapdown INS: quaternion + skew-symmetric
matrix utilities on top of plain numpy arrays. Vectors are ``np.ndarray``
shape ``(3,)``; rotation matrices are ``np.ndarray`` shape ``(3, 3)``.

This mirrors ``include/ins/Math3.h`` in the C++ implementation (same
conventions, same method names where they translate) so the two can be
read side by side.
"""
from __future__ import annotations

import numpy as np

DEG_TO_RAD = np.pi / 180.0
RAD_TO_DEG = 180.0 / np.pi


def skew_symmetric(a: np.ndarray) -> np.ndarray:
    """Cross-product matrix such that skew(a) @ b == cross(a, b)."""
    return np.array([
        [0.0, -a[2], a[1]],
        [a[2], 0.0, -a[0]],
        [-a[1], a[0], 0.0],
    ])


class Quaternion:
    """Hamilton quaternion, scalar-first, representing a body->NED rotation
    (i.e. v_ned = q.rotate(v_body))."""

    __slots__ = ("w", "x", "y", "z")

    def __init__(self, w: float = 1.0, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.w, self.x, self.y, self.z = w, x, y, z

    def normalized(self) -> "Quaternion":
        n = np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if n < 1e-12:
            return Quaternion()
        return Quaternion(self.w / n, self.x / n, self.y / n, self.z / n)

    def __mul__(self, o: "Quaternion") -> "Quaternion":
        return Quaternion(
            self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        )

    @staticmethod
    def from_rotation_vector(rv: np.ndarray) -> "Quaternion":
        """Rotation vector (axis * angle, rad) -> unit quaternion. Exact for
        any magnitude; used both for gyro delta-angle integration and for
        injecting EKF attitude-error corrections."""
        angle = float(np.linalg.norm(rv))
        if angle < 1e-8:
            return Quaternion(1.0, 0.5 * rv[0], 0.5 * rv[1], 0.5 * rv[2])
        half = 0.5 * angle
        s = np.sin(half) / angle
        return Quaternion(np.cos(half), rv[0] * s, rv[1] * s, rv[2] * s)

    def to_matrix(self) -> np.ndarray:
        w, x, y, z = self.w, self.x, self.y, self.z
        ww, xx, yy, zz = w * w, x * x, y * y, z * z
        return np.array([
            [ww + xx - yy - zz, 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), ww - xx + yy - zz, 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), ww - xx - yy + zz],
        ])

    def rotate(self, v: np.ndarray) -> np.ndarray:
        return self.to_matrix() @ v

    def to_euler_rad(self) -> tuple[float, float, float]:
        """roll, pitch, yaw (ZYX / aerospace convention), body->NED."""
        R = self.to_matrix()
        pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
        if abs(R[2, 0]) < 0.999999:
            roll = np.arctan2(R[2, 1], R[2, 2])
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            yaw = 0.0
        return float(roll), float(pitch), float(yaw)

    @staticmethod
    def from_euler_rad(roll: float, pitch: float, yaw: float) -> "Quaternion":
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        q = Quaternion(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
        return q.normalized()
