"""Pure transform helpers for eye-in-hand camera calibration chains."""

from __future__ import annotations

import math

import numpy as np


def _as_transform(value: np.ndarray, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f"{name} must be a 4x4 matrix")
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must contain only finite values")
    return matrix


def world_from_link6_offset(
    world_from_link6: np.ndarray,
    link6_local_z_offset_m: float,
) -> np.ndarray:
    """Return the pose of a point offset along link6's local +Z axis."""

    offset_m = float(link6_local_z_offset_m)
    if not math.isfinite(offset_m):
        raise ValueError("link6_local_z_offset_m must be finite")

    link6_from_offset_point = np.eye(4, dtype=np.float64)
    link6_from_offset_point[2, 3] = offset_m
    return _as_transform(world_from_link6, "world_from_link6") @ link6_from_offset_point


def compose_world_from_handeye_camera(
    world_from_link6: np.ndarray,
    handeye_parent_from_camera: np.ndarray,
    link6_to_handeye_parent_m: float,
) -> np.ndarray:
    """Compose world<-camera for hand-eye data calibrated at gripper_end_pos.

    The calibration JSON stores ``T_gripper_end_camera``.  The gripper-end
    parent used during calibration is a different point from both link6 and
    the planning TCP, so its link6-local offset must be applied explicitly.
    """

    world_from_handeye_parent = world_from_link6_offset(
        world_from_link6,
        link6_to_handeye_parent_m,
    )
    return world_from_handeye_parent @ _as_transform(
        handeye_parent_from_camera,
        "handeye_parent_from_camera",
    )


def compose_capture_frame_extrinsics(
    world_from_link6: np.ndarray,
    arm_base_from_world: np.ndarray,
    handeye_parent_from_camera: np.ndarray,
    link6_to_handeye_parent_m: float,
    link6_to_planning_tcp_m: float,
) -> dict[str, np.ndarray]:
    """Compose one capture snapshot in both world and arm-base frames.

    The remote observation protocol names its dynamic transform
    ``T_<arm>_base_from_camera`` and defines the observed active pose at the
    hand-eye parent / gripper-end control point.  A returned grasp goal is a
    separate physical planning-TCP pose and is not rebased through this helper.
    ``world_from_camera`` converts that camera-frame grasp back to world without
    introducing the fixed world-to-arm-base translation.
    """

    world_from_handeye_parent = world_from_link6_offset(
        world_from_link6,
        link6_to_handeye_parent_m,
    )
    parent_from_camera = _as_transform(
        handeye_parent_from_camera,
        "handeye_parent_from_camera",
    )
    world_from_camera = world_from_handeye_parent @ parent_from_camera
    world_from_planning_tcp = world_from_link6_offset(
        world_from_link6,
        link6_to_planning_tcp_m,
    )
    base_from_world = _as_transform(arm_base_from_world, "arm_base_from_world")
    base_from_handeye_parent = base_from_world @ world_from_handeye_parent
    base_from_planning_tcp = base_from_world @ world_from_planning_tcp
    base_from_camera = base_from_world @ world_from_camera
    planning_tcp_from_camera = (
        np.linalg.inv(world_from_planning_tcp) @ world_from_camera
    )

    return {
        "world_from_handeye_parent": world_from_handeye_parent,
        "world_from_planning_tcp": world_from_planning_tcp,
        "world_from_camera": world_from_camera,
        "base_from_handeye_parent": base_from_handeye_parent,
        "base_from_planning_tcp": base_from_planning_tcp,
        "base_from_camera": base_from_camera,
        "planning_tcp_from_camera": planning_tcp_from_camera,
    }


def transform_pose_delta(
    first: np.ndarray,
    second: np.ndarray,
) -> tuple[float, float]:
    """Return translation (m) and rotation (deg) between two poses."""

    first_matrix = _as_transform(first, "first")
    second_matrix = _as_transform(second, "second")
    translation_m = float(
        np.linalg.norm(second_matrix[:3, 3] - first_matrix[:3, 3])
    )
    relative_rotation = first_matrix[:3, :3].T @ second_matrix[:3, :3]
    cosine = float(np.clip((np.trace(relative_rotation) - 1.0) * 0.5, -1.0, 1.0))
    rotation_deg = math.degrees(math.acos(cosine))
    return translation_m, rotation_deg
