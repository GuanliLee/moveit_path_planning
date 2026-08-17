from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Iterable, Tuple

from geometry_msgs.msg import Pose, Transform, TransformStamped


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def _as_float_tuple(values: Iterable[float], size: int) -> Tuple[float, ...]:
    result = tuple(float(v) for v in values)
    if len(result) != size:
        raise ValueError(f"expected {size} values, got {len(result)}")
    return result


def normalize_quaternion(q: Iterable[float]) -> Quaternion:
    x, y, z, w = _as_float_tuple(q, 4)
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("zero-length quaternion")
    return (x / norm, y / norm, z / norm, w / norm)


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize_quaternion((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def quaternion_inverse(q: Quaternion) -> Quaternion:
    x, y, z, w = normalize_quaternion(q)
    return (-x, -y, -z, w)


def rotate_vector(q: Quaternion, v: Vector3) -> Vector3:
    x, y, z = v
    qx, qy, qz, qw = normalize_quaternion(q)

    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)

    rx = x + qw * tx + (qy * tz - qz * ty)
    ry = y + qw * ty + (qz * tx - qx * tz)
    rz = z + qw * tz + (qx * ty - qy * tx)
    return (rx, ry, rz)


@dataclass(frozen=True)
class RigidTransform:
    translation: Vector3
    rotation: Quaternion

    @staticmethod
    def identity() -> "RigidTransform":
        return RigidTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    @staticmethod
    def from_xyz_xyzw(xyz: Iterable[float], xyzw: Iterable[float]) -> "RigidTransform":
        return RigidTransform(
            _as_float_tuple(xyz, 3), normalize_quaternion(xyzw)
        )

    @staticmethod
    def from_pose(pose: Pose) -> "RigidTransform":
        return RigidTransform.from_xyz_xyzw(
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    @staticmethod
    def from_transform(transform: Transform) -> "RigidTransform":
        return RigidTransform.from_xyz_xyzw(
            (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )

    @staticmethod
    def from_transform_stamped(transform: TransformStamped) -> "RigidTransform":
        return RigidTransform.from_transform(transform.transform)

    def inverse(self) -> "RigidTransform":
        inv_rotation = quaternion_inverse(self.rotation)
        inv_translation = rotate_vector(
            inv_rotation,
            (
                -self.translation[0],
                -self.translation[1],
                -self.translation[2],
            ),
        )
        return RigidTransform(inv_translation, inv_rotation)

    def __mul__(self, other: "RigidTransform") -> "RigidTransform":
        rotated_translation = rotate_vector(self.rotation, other.translation)
        translation = (
            self.translation[0] + rotated_translation[0],
            self.translation[1] + rotated_translation[1],
            self.translation[2] + rotated_translation[2],
        )
        rotation = quaternion_multiply(self.rotation, other.rotation)
        return RigidTransform(translation, rotation)

    def to_pose(self) -> Pose:
        pose = Pose()
        pose.position.x = self.translation[0]
        pose.position.y = self.translation[1]
        pose.position.z = self.translation[2]
        pose.orientation.x = self.rotation[0]
        pose.orientation.y = self.rotation[1]
        pose.orientation.z = self.rotation[2]
        pose.orientation.w = self.rotation[3]
        return pose
