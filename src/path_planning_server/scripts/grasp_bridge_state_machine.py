#!/usr/bin/env python3

"""Upper task state machine for 121 vision inference and the local grasp bridge."""

from __future__ import annotations

import asyncio
import base64
import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

import aiohttp
import cv2
import msgpack
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from graspnet_bridge_interfaces.srv import ExecuteNamedGraspTask, ExecuteTask
from path_planning_interfaces.srv import PlanToJoints
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from capture_sync import TimedMessageSample, message_stamp_seconds, select_synced_pair
from handeye_transform import (
    compose_capture_frame_extrinsics,
    compose_world_from_handeye_camera,
    transform_pose_delta,
    world_from_link6_offset,
)


Quaternion = Tuple[float, float, float, float]


_CARRYING_BRIDGE_STEPS = frozenset(
    {"RETREAT_AFTER_GRASP", "PLACE_WAYPOINT", "PLACE_XY", "PLACE_ORIENT"}
)

_PLACE_TRANSPORT_STEP_ORDER = ("PLACE_LIFT", "PLACE_ORIENT", "PLACE_XY")

_PLACE_ORIENTATION_TARGET_STEPS = frozenset(
    {"PLACE_ORIENT", "PLACE_WAYPOINT", "PLACE"}
)

_GRASP_CANDIDATE_STEPS = frozenset(
    {"PRE_GRASP", "GRASP_APPROACH", "GRASP", "RETREAT_AFTER_GRASP"}
)

_GRASP_CANDIDATE_SELECTION_STEPS = frozenset(
    {"PRE_GRASP", "GRASP_APPROACH", "GRASP"}
)

_MAX_IK_FALLBACK_ROTATION_DEG = 15.0


def _bridge_task_type_for_step(task_type: str) -> str:
    """Map state-machine steps to bridge motion semantics."""
    if task_type in ("LIFT_AFTER_GRASP", "PLACE_LIFT"):
        return "LIFT"
    if task_type in _CARRYING_BRIDGE_STEPS:
        return "CARRY"
    if task_type in ("OPEN", "PRE_GRASP", "GRASP_APPROACH", "RETREAT_AFTER_PLACE"):
        return "MOVE"
    return task_type


def _return_home_step(
    initial_pose: PoseStamped,
) -> Tuple[str, PoseStamped, bool, float]:
    """Build the final non-gripper motion back to the task's initial pose."""
    return ("RETURN_HOME", initial_pose, False, 0.0)


def _place_completion_steps(
    final_place_pose: PoseStamped,
    open_width: float,
    *,
    retreat_after_place: bool,
    return_home_pose: Optional[PoseStamped],
) -> list[Tuple[str, Optional[PoseStamped], bool, float]]:
    """Order release, post-place retreat, then the optional photo-position return."""
    steps: list[Tuple[str, Optional[PoseStamped], bool, float]] = [
        ("PLACE", final_place_pose, True, float(open_width))
    ]
    if retreat_after_place:
        steps.append(("RETREAT_AFTER_PLACE", None, False, 0.0))
    if return_home_pose is not None:
        steps.append(_return_home_step(return_home_pose))
    return steps


def _capture_joint_target_for_profile(
    task_profile: str,
    arm_name: str,
    capture_targets: Dict[str, Optional[Tuple[float, ...]]],
    pick_capture_targets: Dict[str, Optional[Tuple[float, ...]]],
) -> Optional[Tuple[float, ...]]:
    targets = pick_capture_targets if task_profile == "pick" else capture_targets
    target = targets.get(arm_name)
    return tuple(target) if target is not None else None


def _elevated_place_waypoint(
    final_place_pose: PoseStamped,
    offset_m: float,
) -> PoseStamped:
    """Copy the final PLACE pose and raise only its world-Z waypoint."""
    waypoint = PoseStamped()
    waypoint.header = final_place_pose.header
    waypoint.pose.position.x = final_place_pose.pose.position.x
    waypoint.pose.position.y = final_place_pose.pose.position.y
    waypoint.pose.position.z = final_place_pose.pose.position.z + float(offset_m)
    waypoint.pose.orientation.x = final_place_pose.pose.orientation.x
    waypoint.pose.orientation.y = final_place_pose.pose.orientation.y
    waypoint.pose.orientation.z = final_place_pose.pose.orientation.z
    waypoint.pose.orientation.w = final_place_pose.pose.orientation.w
    return waypoint


@dataclass(frozen=True)
class RemoteGrasp:
    arm_name: str
    tcp_pose: PoseStamped
    gripper_width_m: float


@dataclass(frozen=True)
class GraspPoseCandidate:
    name: str
    local_axis: str
    rotation_degrees: float
    pose: PoseStamped


@dataclass(frozen=True)
class SensorBundle:
    rgb: Image
    depth: Image
    info: CameraInfo
    rgb_sequence: int
    depth_sequence: int
    rgb_arrival_monotonic_s: float
    depth_arrival_monotonic_s: float
    rgb_stamp_s: float
    depth_stamp_s: float
    capture_stamp_s: float
    stamp_skew_s: float
    arrival_skew_s: float
    arrival_age_s: float
    matched_pair_count: int
    barrier_monotonic_s: float
    rgb_receipt_ros_s: float
    depth_receipt_ros_s: float
    tf_before_stamp_s: float = 0.0
    tf_after_stamp_s: float = 0.0
    tf_before_age_s: float = 0.0
    tf_after_age_s: float = 0.0
    tf_translation_delta_m: float = 0.0
    tf_rotation_delta_deg: float = 0.0


def _normalize_quaternion(values: Iterable[float]) -> Quaternion:
    q = tuple(float(value) for value in values)
    if len(q) != 4:
        raise ValueError("quaternion must contain four values")
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("quaternion must be finite and non-zero")
    return tuple(value / norm for value in q)  # type: ignore[return-value]


def _pose_from_xyzw(frame_id: str, values: Iterable[float]) -> PoseStamped:
    pose_values = tuple(float(value) for value in values)
    if len(pose_values) != 7:
        raise ValueError("grasp_tcp_pose_xyzw must contain 7 values")
    qx, qy, qz, qw = _normalize_quaternion(pose_values[3:7])
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = pose_values[0]
    pose.pose.position.y = pose_values[1]
    pose.pose.position.z = pose_values[2]
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def _pack_msgpack(payload: Any) -> bytes:
    return msgpack.packb(payload, default=_msgpack_default, use_bin_type=True)


def _msgpack_default(obj: Any) -> Any:
    if isinstance(obj, np.ndarray):
        contiguous = np.ascontiguousarray(obj)
        return {
            b"nd": True,
            b"type": str(contiguous.dtype),
            b"shape": contiguous.shape,
            b"data": contiguous.tobytes(),
        }
    if isinstance(obj, np.generic):
        return {
            b"__npgeneric__": True,
            b"dtype": obj.dtype.str,
            b"data": obj.item(),
        }
    raise TypeError(f"Object of type {type(obj).__name__} is not msgpack serializable")


def _unpack_msgpack(payload: bytes) -> Any:
    return msgpack.unpackb(
        payload,
        raw=False,
        object_hook=_msgpack_object_hook,
        strict_map_key=False,
    )


def _msgpack_object_hook(obj: Dict[Any, Any]) -> Any:
    try:
        if b"__ndarray__" in obj:
            return np.ndarray(
                buffer=obj[b"data"],
                dtype=np.dtype(obj[b"dtype"]),
                shape=tuple(obj[b"shape"]),
            )
        if b"__npgeneric__" in obj:
            return np.dtype(obj[b"dtype"]).type(obj[b"data"])
        if b"nd" in obj:
            if obj[b"nd"] is True:
                return np.ndarray(
                    buffer=obj[b"data"],
                    dtype=np.dtype(obj[b"type"]),
                    shape=tuple(obj[b"shape"]),
                )
            return np.frombuffer(obj[b"data"], dtype=np.dtype(obj[b"type"]))[0]
    except Exception:
        return obj
    return obj


def _image_to_array(message: Image, image_size: int) -> np.ndarray:
    if message.encoding not in ("rgb8", "bgr8"):
        raise RuntimeError(f"unsupported RGB image encoding: {message.encoding}")
    array = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
        int(message.height),
        int(message.width),
        3,
    )
    if message.encoding == "bgr8":
        array = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
    if image_size > 0:
        array = cv2.resize(array, (image_size, image_size), interpolation=cv2.INTER_AREA)
    return array.astype(np.uint8)


def _depth_to_array_m(
    message: Image,
    image_size: int,
    uint16_scale: float,
    max_m: float,
) -> np.ndarray:
    if message.encoding == "16UC1":
        depth = np.frombuffer(bytes(message.data), dtype=np.uint16).reshape(
            int(message.height),
            int(message.width),
        ).astype(np.float32) * float(uint16_scale)
    elif message.encoding == "32FC1":
        depth = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(
            int(message.height),
            int(message.width),
        ).astype(np.float32)
    else:
        raise RuntimeError(f"unsupported depth image encoding: {message.encoding}")
    depth = np.where(np.isfinite(depth), depth, 0.0)
    if max_m > 0.0:
        depth = np.clip(depth, 0.0, max_m)
    if image_size > 0:
        depth = cv2.resize(depth, (image_size, image_size), interpolation=cv2.INTER_NEAREST)
    return depth.astype(np.float32)


def _quaternion_to_matrix(quaternion: Quaternion) -> np.ndarray:
    x, y, z, w = _normalize_quaternion(quaternion)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _matrix_to_quaternion(matrix: np.ndarray) -> Quaternion:
    rotation = np.asarray(matrix, dtype=np.float64)[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / scale
        qx = 0.25 * scale
        qy = (rotation[0, 1] + rotation[1, 0]) / scale
        qz = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / scale
        qx = (rotation[0, 1] + rotation[1, 0]) / scale
        qy = 0.25 * scale
        qz = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / scale
        qx = (rotation[0, 2] + rotation[2, 0]) / scale
        qy = (rotation[1, 2] + rotation[2, 1]) / scale
        qz = 0.25 * scale
    return _normalize_quaternion((qx, qy, qz, qw))


def _matrix_from_xyz_xyzw(xyz: Iterable[float], xyzw: Iterable[float]) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = _quaternion_to_matrix(_normalize_quaternion(xyzw))
    matrix[:3, 3] = np.asarray(tuple(float(v) for v in xyz), dtype=np.float64)
    return matrix


def _matrix_from_pose(pose: PoseStamped) -> np.ndarray:
    return _matrix_from_xyz_xyzw(
        (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ),
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ),
    )


def _pose_from_matrix(frame_id: str, matrix: np.ndarray) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(matrix[0, 3])
    pose.pose.position.y = float(matrix[1, 3])
    pose.pose.position.z = float(matrix[2, 3])
    qx, qy, qz, qw = _matrix_to_quaternion(matrix)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def _validated_ik_fallback_degrees(
    values: Iterable[float],
    parameter_name: str,
) -> Tuple[float, ...]:
    """Validate and deduplicate bounded local-axis IK fallback angles."""
    result = []
    seen = set()
    for raw_value in values:
        value = float(raw_value)
        if not math.isfinite(value) or abs(value) < 1e-9:
            raise ValueError(f"{parameter_name} must contain finite non-zero degrees")
        if abs(value) > _MAX_IK_FALLBACK_ROTATION_DEG:
            raise ValueError(
                f"{parameter_name} values must not exceed "
                f"{_MAX_IK_FALLBACK_ROTATION_DEG:.1f} degrees"
            )
        key = round(value, 9)
        if key not in seen:
            seen.add(key)
            result.append(value)
    return tuple(result)


def _local_axis_rotation_matrix(local_axis: str, degrees: float) -> np.ndarray:
    """Return an active rotation about a TCP-local axis."""
    radians = math.radians(float(degrees))
    cosine = math.cos(radians)
    sine = math.sin(radians)
    axis = local_axis.strip().lower()
    if axis == "y":
        return np.asarray(
            [
                [cosine, 0.0, sine],
                [0.0, 1.0, 0.0],
                [-sine, 0.0, cosine],
            ],
            dtype=np.float64,
        )
    if axis == "z":
        return np.asarray(
            [
                [cosine, -sine, 0.0],
                [sine, cosine, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
    raise ValueError(f"unsupported TCP-local rotation axis: {local_axis!r}")


def _pose_with_local_orientation_offset(
    pose: PoseStamped,
    local_axis: str,
    degrees: float,
) -> PoseStamped:
    """Rotate the full TCP orientation locally without changing its origin."""
    transform = _matrix_from_pose(pose)
    rotated = transform.copy()
    rotated[:3, :3] = transform[:3, :3] @ _local_axis_rotation_matrix(
        local_axis,
        degrees,
    )
    output = _pose_from_matrix(pose.header.frame_id, rotated)
    output.header.stamp = pose.header.stamp
    return output


def _place_pose_with_grasp_orientation_compensation(
    place_pose: PoseStamped,
    original_grasp_pose: PoseStamped,
    selected_grasp_pose: PoseStamped,
) -> PoseStamped:
    """Apply the selected-vs-original grasp rotation delta to a place pose."""
    original_frame = (original_grasp_pose.header.frame_id or "").strip()
    selected_frame = (selected_grasp_pose.header.frame_id or "").strip()
    if original_frame != selected_frame:
        raise ValueError(
            "original and selected grasp poses must use the same frame; "
            f"got {original_frame!r} and {selected_frame!r}"
        )

    original_grasp = _matrix_from_pose(original_grasp_pose)
    selected_grasp = _matrix_from_pose(selected_grasp_pose)
    grasp_rotation_delta = (
        original_grasp[:3, :3].T @ selected_grasp[:3, :3]
    )

    nominal_place = _matrix_from_pose(place_pose)
    compensated_place = nominal_place.copy()
    compensated_place[:3, :3] = (
        nominal_place[:3, :3] @ grasp_rotation_delta
    )
    output = _pose_from_matrix(place_pose.header.frame_id, compensated_place)
    output.header.stamp = place_pose.header.stamp
    return output


def _build_ik_orientation_candidates(
    grasp_pose: PoseStamped,
    local_y_degrees: Iterable[float],
    local_z_degrees: Iterable[float],
) -> list[GraspPoseCandidate]:
    """Build deterministic small-angle candidates after the original pose."""
    candidates = [
        GraspPoseCandidate(
            name="original",
            local_axis="none",
            rotation_degrees=0.0,
            pose=grasp_pose,
        )
    ]
    for local_axis, values in (("y", local_y_degrees), ("z", local_z_degrees)):
        for degrees in values:
            value = float(degrees)
            candidates.append(
                GraspPoseCandidate(
                    name=f"local_{local_axis}_{value:+g}deg",
                    local_axis=local_axis,
                    rotation_degrees=value,
                    pose=_pose_with_local_orientation_offset(
                        grasp_pose,
                        local_axis,
                        value,
                    ),
                )
            )
    return candidates


def _pose_local_axis_world(pose: PoseStamped, local_axis: str) -> np.ndarray:
    """Return a normalized TCP-local axis expressed in the world frame."""
    axis_indexes = {"x": 0, "y": 1, "z": 2}
    axis_name = local_axis.strip().lower()
    if axis_name not in axis_indexes:
        raise ValueError(f"unsupported TCP-local axis: {local_axis!r}")
    axis = np.asarray(
        _matrix_from_pose(pose)[:3, axis_indexes[axis_name]],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9 or not math.isfinite(norm):
        raise ValueError(f"TCP-local {axis_name.upper()} axis is zero or non-finite")
    return axis / norm


def _candidate_indexes_for_step(
    task_type: str,
    candidate_count: int,
    selected_grasp_candidate_index: Optional[int],
) -> Tuple[int, ...]:
    """Lock all coupled grasp steps to the first successful candidate."""
    if candidate_count < 1:
        raise ValueError("candidate_count must be positive")
    if task_type in _GRASP_CANDIDATE_STEPS and selected_grasp_candidate_index is not None:
        if not 0 <= selected_grasp_candidate_index < candidate_count:
            raise ValueError("selected grasp candidate index is out of range")
        return (selected_grasp_candidate_index,)
    return tuple(range(candidate_count))


def _build_target_marker_array(
    arm_name: str,
    target_name: str,
    grasp_pose: PoseStamped,
    pre_grasp_pose: Optional[PoseStamped],
    tcp_to_link6_m: float,
    status: str,
    status_detail: str,
    stamp: Any,
) -> MarkerArray:
    """Build RViz markers for the exact planning TCP and derived link6 goal."""

    frame_id = grasp_pose.header.frame_id or "world"
    namespace = f"{arm_name}_live_grasp_target"
    target_matrix = _matrix_from_pose(grasp_pose)
    tcp_xyz = target_matrix[:3, 3]
    link6_from_tcp = np.eye(4, dtype=np.float64)
    link6_from_tcp[2, 3] = -float(tcp_to_link6_m)
    link6_xyz = (target_matrix @ link6_from_tcp)[:3, 3]

    status_key = status.strip().lower()
    target_color = {
        "pending": (0.85, 0.10, 1.00, 0.95),
        "success": (0.10, 0.90, 0.20, 0.95),
        "failed": (1.00, 0.10, 0.10, 0.95),
    }.get(status_key, (0.85, 0.10, 1.00, 0.95))
    status_label = {
        "pending": "PLANNING",
        "success": "SUCCESS",
        "failed": "FAILED",
    }.get(status_key, status_key.upper() or "TARGET")

    def marker(marker_id: int, marker_type: int) -> Marker:
        output = Marker()
        output.header.frame_id = frame_id
        output.header.stamp = stamp
        output.ns = namespace
        output.id = marker_id
        output.type = marker_type
        output.action = Marker.ADD
        output.pose.orientation.w = 1.0
        return output

    def point(values: Iterable[float]) -> Point:
        values_tuple = tuple(float(value) for value in values)
        output = Point()
        output.x, output.y, output.z = values_tuple
        return output

    def set_color(output: Marker, rgba: Tuple[float, float, float, float]) -> None:
        output.color.r, output.color.g, output.color.b, output.color.a = rgba

    markers = MarkerArray()
    clear = Marker()
    clear.header.frame_id = frame_id
    clear.header.stamp = stamp
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)

    tcp_marker = marker(0, Marker.SPHERE)
    tcp_marker.pose.position = point(tcp_xyz)
    tcp_marker.pose.orientation = grasp_pose.pose.orientation
    tcp_marker.scale.x = tcp_marker.scale.y = tcp_marker.scale.z = 0.022
    set_color(tcp_marker, target_color)
    markers.markers.append(tcp_marker)

    link6_marker = marker(1, Marker.SPHERE)
    link6_marker.pose.position = point(link6_xyz)
    link6_marker.scale.x = link6_marker.scale.y = link6_marker.scale.z = 0.014
    set_color(link6_marker, (1.00, 0.55, 0.05, 0.95))
    markers.markers.append(link6_marker)

    tcp_link6_line = marker(2, Marker.LINE_STRIP)
    tcp_link6_line.scale.x = 0.003
    set_color(tcp_link6_line, (0.90, 0.90, 0.90, 0.90))
    tcp_link6_line.points = [point(link6_xyz), point(tcp_xyz)]
    markers.markers.append(tcp_link6_line)

    axis_colors = (
        (1.00, 0.08, 0.08, 1.00),
        (0.08, 1.00, 0.15, 1.00),
        (0.10, 0.35, 1.00, 1.00),
    )
    axis_length_m = 0.08
    for axis_index, rgba in enumerate(axis_colors):
        axis = marker(3 + axis_index, Marker.ARROW)
        axis.scale.x = 0.005
        axis.scale.y = 0.010
        axis.scale.z = 0.016
        set_color(axis, rgba)
        axis_end = tcp_xyz + target_matrix[:3, axis_index] * axis_length_m
        axis.points = [point(tcp_xyz), point(axis_end)]
        markers.markers.append(axis)

    target_text = marker(6, Marker.TEXT_VIEW_FACING)
    target_text.pose.position = point((tcp_xyz[0], tcp_xyz[1], tcp_xyz[2] + 0.04))
    target_text.scale.z = 0.020
    set_color(target_text, target_color)
    detail = status_detail.strip()
    target_text.text = f"{target_name} | {status_label}"
    if detail:
        target_text.text += f" | {detail[:80]}"
    markers.markers.append(target_text)

    link6_text = marker(7, Marker.TEXT_VIEW_FACING)
    link6_text.pose.position = point(
        (
            (tcp_xyz[0] + link6_xyz[0]) * 0.5,
            (tcp_xyz[1] + link6_xyz[1]) * 0.5,
            (tcp_xyz[2] + link6_xyz[2]) * 0.5 + 0.025,
        )
    )
    link6_text.scale.z = 0.016
    set_color(link6_text, (1.00, 0.75, 0.20, 1.00))
    link6_text.text = f"link6 -> TCP {float(tcp_to_link6_m) * 1000.0:.1f} mm"
    markers.markers.append(link6_text)

    if pre_grasp_pose is not None:
        pre_xyz = _matrix_from_pose(pre_grasp_pose)[:3, 3]
        pre_marker = marker(8, Marker.SPHERE)
        pre_marker.pose.position = point(pre_xyz)
        pre_marker.scale.x = pre_marker.scale.y = pre_marker.scale.z = 0.018
        set_color(pre_marker, (1.00, 0.85, 0.05, 0.95))
        markers.markers.append(pre_marker)

        approach_line = marker(9, Marker.LINE_STRIP)
        approach_line.scale.x = 0.004
        set_color(approach_line, (1.00, 0.75, 0.05, 0.90))
        approach_line.points = [point(pre_xyz), point(tcp_xyz)]
        markers.markers.append(approach_line)

        pre_text = marker(10, Marker.TEXT_VIEW_FACING)
        pre_text.pose.position = point((pre_xyz[0], pre_xyz[1], pre_xyz[2] + 0.03))
        pre_text.scale.z = 0.017
        set_color(pre_text, (1.00, 0.85, 0.05, 1.00))
        pre_text.text = "PRE_GRASP"
        markers.markers.append(pre_text)

    return markers


def _quaternion_angular_distance(a: Quaternion, b: Quaternion) -> float:
    aq = _normalize_quaternion(a)
    bq = _normalize_quaternion(b)
    dot = abs(sum(aq[index] * bq[index] for index in range(4)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def _matrix_from_transform(transform: Any) -> np.ndarray:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return _matrix_from_xyz_xyzw(
        (translation.x, translation.y, translation.z),
        (rotation.x, rotation.y, rotation.z, rotation.w),
    )


def _load_handeye_matrix(path_value: str) -> np.ndarray:
    path = Path(path_value).expanduser()
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    return _matrix_from_xyz_xyzw(data["translation_m"], data["orientation_xyzw"])


def _mapping_value(mapping: Dict[str, Any], key: str) -> Any:
    if key in mapping:
        return mapping[key]
    for nested_key in ("data", "result", "response", "visualization"):
        nested = mapping.get(nested_key)
        if isinstance(nested, dict) and key in nested:
            return nested[key]
    return None


def _first_candidate(response: Dict[str, Any]) -> Dict[str, Any]:
    for key in (
        "grasp_goal",
        "grasp_candidates",
        "grasp_goals",
        "grasp_goal_candidates",
        "candidates",
        "grasps",
    ):
        value = _mapping_value(response, key)
        if value is None:
            continue
        if isinstance(value, dict):
            return value
        if isinstance(value, (list, tuple)) and value:
            if isinstance(value[0], dict):
                return value[0]
    return response


def _pose_log_text(label: str, pose: PoseStamped) -> str:
    p = pose.pose.position
    q = pose.pose.orientation
    return (
        f"{label}: frame={pose.header.frame_id}, "
        f"p=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}), "
        f"q=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})"
    )


def _is_planning_failure(detail: str) -> bool:
    return "planning failed" in detail.strip().lower()


def _is_no_ik_failure(detail: str) -> bool:
    return "no ik solution" in detail.strip().lower()


class GraspBridgeStateMachine(Node):
    def __init__(self) -> None:
        super().__init__("grasp_bridge_state_machine")

        self._default_arm_name = str(self.declare_parameter("default_arm_name", "right").value)
        self._default_scene_id = int(self.declare_parameter("default_scene_id", 1).value)
        self._world_frame = str(self.declare_parameter("world_frame", "world").value)
        self._left_camera_frame = str(
            self.declare_parameter("left_camera_frame", "cam_left_color_optical_frame").value
        )
        self._right_camera_frame = str(
            self.declare_parameter("right_camera_frame", "cam_right_color_optical_frame").value
        )
        self._bridge_service_name = str(
            self.declare_parameter("bridge_service_name", "/grasp_bridge/execute_task").value
        )
        self._joint_plan_service_name = str(
            self.declare_parameter("joint_plan_service_name", "/plan_to_joints").value
        )
        self._target_marker_topics = {
            "left": str(
                self.declare_parameter(
                    "left_target_marker_topic",
                    "/rviz/left/planning_tcp_target_markers",
                ).value
            ),
            "right": str(
                self.declare_parameter(
                    "right_target_marker_topic",
                    "/rviz/right/planning_tcp_target_markers",
                ).value
            ),
        }
        self._inference_request_url = str(
            self.declare_parameter("inference_request_url", "ws://192.168.4.121:1234").value
        )
        self._inference_timeout_s = float(self.declare_parameter("inference_timeout_s", 20.0).value)
        self._inference_candidate_count = int(
            self.declare_parameter("inference_candidate_count", 1).value
        )
        self._inference_image_size = int(self.declare_parameter("inference_image_size", 224).value)
        self._depth_uint16_scale = float(self.declare_parameter("depth_uint16_scale", 0.001).value)
        self._depth_max_m = float(self.declare_parameter("depth_max_m", 10.0).value)
        self._default_gripper_width_m = float(
            self.declare_parameter("default_gripper_width_m", 0.08).value
        )
        self._closed_gripper_width_m = float(
            self.declare_parameter("closed_gripper_width_m", 0.0).value
        )
        self._open_before_grasp = bool(self.declare_parameter("open_before_grasp", True).value)
        self._open_before_inference = bool(
            self.declare_parameter("open_before_inference", False).value
        )
        self._open_before_inference_settle_s = float(
            self.declare_parameter("open_before_inference_settle_s", 0.2).value
        )
        self._pick_open_before_inference = bool(
            self.declare_parameter("pick_open_before_inference", True).value
        )
        self._pick_open_settle_s = float(
            self.declare_parameter("pick_open_settle_s", 0.2).value
        )
        self._pre_grasp_before_grasp = bool(
            self.declare_parameter("pre_grasp_before_grasp", False).value
        )
        self._pre_grasp_offset_m = float(
            self.declare_parameter("pre_grasp_offset_m", 0.06).value
        )
        self._pre_grasp_min_z_m = float(
            self.declare_parameter("pre_grasp_min_z_m", 0.0).value
        )
        self._retreat_after_grasp = bool(
            self.declare_parameter("retreat_after_grasp", True).value
        )
        self._retreat_after_grasp_offset_m = float(
            self.declare_parameter("retreat_after_grasp_offset_m", 0.06).value
        )
        self._lift_after_grasp = bool(self.declare_parameter("lift_after_grasp", True).value)
        self._lift_offset_m = float(self.declare_parameter("lift_offset_m", 0.03).value)
        self._place_lift_before_place = bool(
            self.declare_parameter("place_lift_before_place", False).value
        )
        self._place_lift_height_m = float(
            self.declare_parameter("place_lift_height_m", 0.0).value
        )
        self._place_descend_before_place = bool(
            self.declare_parameter("place_descend_before_place", True).value
        )
        self._place_descend_offset_m = float(
            self.declare_parameter("place_descend_offset_m", 0.05).value
        )
        self._retreat_after_place = bool(
            self.declare_parameter("retreat_after_place", True).value
        )
        self._retreat_after_place_offset_m = max(
            0.0,
            float(self.declare_parameter("retreat_after_place_offset_m", 0.05).value),
        )
        self._grasp_z_offset_m = float(self.declare_parameter("grasp_z_offset_m", 0.0).value)
        self._grasp_orientation_mode = str(
            self.declare_parameter("grasp_orientation_mode", "remote").value
        ).strip().lower()
        self._pick_grasp_orientation_mode = str(
            self.declare_parameter("pick_grasp_orientation_mode", "").value
        ).strip().lower()
        self._enable_roll_180_grasp_branch = bool(
            self.declare_parameter("enable_roll_180_grasp_branch", True).value
        )
        self._grasp_ik_fallback_enabled = bool(
            self.declare_parameter("grasp_ik_fallback_enabled", False).value
        )
        self._compensate_place_orientation_from_grasp_delta = bool(
            self.declare_parameter(
                "compensate_place_orientation_from_grasp_delta",
                False,
            ).value
        )
        self._grasp_ik_fallback_profiles = frozenset(
            str(value).strip().lower()
            for value in self.declare_parameter(
                "grasp_ik_fallback_profiles",
                ["grasp"],
            ).value
            if str(value).strip()
        )
        unsupported_fallback_profiles = self._grasp_ik_fallback_profiles.difference(
            {"grasp", "pick"}
        )
        if unsupported_fallback_profiles:
            raise ValueError(
                "grasp_ik_fallback_profiles only supports grasp and pick; got "
                f"{sorted(unsupported_fallback_profiles)}"
            )
        self._grasp_ik_fallback_local_y_degrees = _validated_ik_fallback_degrees(
            self.declare_parameter(
                "grasp_ik_fallback_local_y_degrees",
                [3.0, -3.0, 5.0, -5.0, 8.0, -8.0],
            ).value,
            "grasp_ik_fallback_local_y_degrees",
        )
        self._grasp_ik_fallback_local_z_degrees = _validated_ik_fallback_degrees(
            self.declare_parameter(
                "grasp_ik_fallback_local_z_degrees",
                [3.0, -3.0, 5.0, -5.0],
            ).value,
            "grasp_ik_fallback_local_z_degrees",
        )
        if (
            self._grasp_ik_fallback_enabled
            and not self._grasp_ik_fallback_profiles
        ):
            raise ValueError(
                "grasp_ik_fallback_profiles must not be empty when fallback is enabled"
            )
        if (
            self._grasp_ik_fallback_enabled
            and not self._grasp_ik_fallback_local_y_degrees
            and not self._grasp_ik_fallback_local_z_degrees
        ):
            raise ValueError(
                "at least one grasp IK fallback angle must be configured when enabled"
            )
        self._vertical_down_tcp_orientation = _normalize_quaternion(
            self.declare_parameter(
                "vertical_down_tcp_orientation_xyzw",
                [1.0, 0.0, 0.0, 0.0],
            ).value
        )
        self._fixed_tcp_orientation_by_arm = {
            "left": _normalize_quaternion(
                self.declare_parameter(
                    "left_fixed_tcp_orientation_xyzw",
                    self._vertical_down_tcp_orientation,
                ).value
            ),
            "right": _normalize_quaternion(
                self.declare_parameter(
                    "right_fixed_tcp_orientation_xyzw",
                    self._vertical_down_tcp_orientation,
                ).value
            ),
        }
        self._tcp_to_link6_m = float(self.declare_parameter("tcp_to_link6_m", 0.1358).value)
        self._link6_to_handeye_parent_m = float(
            self.declare_parameter("link6_to_handeye_parent_m", 0.0593).value
        )
        self._left_handeye_matrix = _load_handeye_matrix(
            str(
                self.declare_parameter(
                    "left_handeye_file",
                    "/home/ligl/path_planning_service0807/calibration/extrinsics/cam_left_handeye.json",
                ).value
            )
        )
        self._right_handeye_matrix = _load_handeye_matrix(
            str(
                self.declare_parameter(
                    "right_handeye_file",
                    "/home/ligl/path_planning_service0807/calibration/extrinsics/cam_right_handeye.json",
                ).value
            )
        )
        self._tf_timeout_s = float(self.declare_parameter("tf_timeout_s", 1.0).value)
        self._service_timeout_s = float(self.declare_parameter("service_timeout_s", 90.0).value)
        self._planning_retry_attempts = max(
            1,
            int(self.declare_parameter("planning_retry_attempts", 1).value),
        )
        self._cancel_plan_service_name = str(
            self.declare_parameter("cancel_plan_service_name", "/cancel_plan_execution").value
        )
        self._cancel_service_timeout_s = max(
            0.1, float(self.declare_parameter("cancel_service_timeout_s", 1.0).value)
        )

        self._left_rgb_topic = str(
            self.declare_parameter("left_rgb_topic", "/cam_left/color/image_raw").value
        )
        self._left_depth_topic = str(
            self.declare_parameter("left_depth_topic", "/cam_left/depth/image_rect_raw").value
        )
        self._left_camera_info_topic = str(
            self.declare_parameter("left_camera_info_topic", "/cam_left/color/camera_info").value
        )
        self._right_rgb_topic = str(
            self.declare_parameter("right_rgb_topic", "/cam_right/color/image_raw").value
        )
        self._right_depth_topic = str(
            self.declare_parameter("right_depth_topic", "/cam_right/depth/image_rect_raw").value
        )
        self._right_camera_info_topic = str(
            self.declare_parameter("right_camera_info_topic", "/cam_right/color/camera_info").value
        )
        self._sensor_sync_timeout_s = max(
            0.1,
            float(self.declare_parameter("sensor_sync_timeout_s", 3.0).value),
        )
        self._sensor_max_stamp_skew_s = max(
            0.0,
            float(self.declare_parameter("sensor_max_stamp_skew_s", 0.005).value),
        )
        self._sensor_max_arrival_skew_s = max(
            0.0,
            float(self.declare_parameter("sensor_max_arrival_skew_s", 0.02).value),
        )
        self._sensor_max_arrival_age_s = max(
            0.01,
            float(self.declare_parameter("sensor_max_arrival_age_s", 0.15).value),
        )
        self._sensor_queue_size = max(
            5,
            int(self.declare_parameter("sensor_queue_size", 30).value),
        )
        self._sensor_discard_pairs_after_barrier = max(
            0,
            int(
                self.declare_parameter(
                    "sensor_discard_pairs_after_barrier",
                    2,
                ).value
            ),
        )
        if self._sensor_discard_pairs_after_barrier >= self._sensor_queue_size:
            raise ValueError(
                "sensor_discard_pairs_after_barrier must be smaller than "
                f"sensor_queue_size ({self._sensor_queue_size})"
            )
        self._capture_stationary_translation_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "capture_stationary_translation_m",
                    0.002,
                ).value
            ),
        )
        self._capture_stationary_rotation_deg = max(
            0.0,
            float(
                self.declare_parameter(
                    "capture_stationary_rotation_deg",
                    1.0,
                ).value
            ),
        )
        self._capture_tf_max_age_s = max(
            0.01,
            float(self.declare_parameter("capture_tf_max_age_s", 0.20).value),
        )
        self._capture_tf_advance_timeout_s = max(
            0.01,
            float(
                self.declare_parameter("capture_tf_advance_timeout_s", 0.50).value
            ),
        )
        self._sensor_require_post_open_frames = bool(
            self.declare_parameter("sensor_require_post_open_frames", True).value
        )

        self._configured_place_pose = self._make_configured_place_pose()
        self._configured_pick_place_pose = self._make_configured_pick_place_pose()
        self._move_to_capture_pose_before_inference = bool(
            self.declare_parameter("move_to_capture_pose_before_inference", False).value
        )
        self._return_to_initial_pose_after_place = bool(
            self.declare_parameter("return_to_initial_pose_after_place", True).value
        )
        self._capture_settle_s = float(self.declare_parameter("capture_settle_s", 0.5).value)
        self._pre_close_snapshot_enabled = bool(
            self.declare_parameter("pre_close_snapshot_enabled", False).value
        )
        self._pre_close_snapshot_settle_s = max(
            0.0,
            float(self.declare_parameter("pre_close_snapshot_settle_s", 0.3).value),
        )
        self._pre_close_snapshot_dir = Path(
            str(
                self.declare_parameter(
                    "pre_close_snapshot_dir",
                    "/home/ligl/path_planning_service0807/debug/pre_close",
                ).value
            )
        ).expanduser()
        self._configured_capture_pose = {
            "left": self._make_configured_capture_pose("left"),
            "right": self._make_configured_capture_pose("right"),
        }
        self._configured_pick_capture_pose = {
            "left": self._make_configured_capture_pose(
                "left",
                param_prefix="pick_left_capture_tcp",
                default_pose=self._configured_capture_pose["left"],
            ),
            "right": self._make_configured_capture_pose(
                "right",
                param_prefix="pick_right_capture_tcp",
                default_pose=self._configured_capture_pose["right"],
            ),
        }
        self._configured_capture_joints = {
            arm_name: self._make_configured_capture_joints(
                f"{arm_name}_capture_joint"
            )
            for arm_name in ("left", "right")
        }
        self._configured_pick_capture_joints = {
            arm_name: self._make_configured_capture_joints(
                f"pick_{arm_name}_capture_joint"
            )
            for arm_name in ("left", "right")
        }
        self._callback_group = ReentrantCallbackGroup()
        self._busy_lock = threading.Lock()
        self._cancel_requested = threading.Event()
        self._latest_lock = threading.Lock()
        self._sensor_condition = threading.Condition(self._latest_lock)
        self._latest_rgb: Dict[str, TimedMessageSample] = {}
        self._latest_depth: Dict[str, TimedMessageSample] = {}
        self._latest_camera_info: Dict[str, TimedMessageSample] = {}
        self._rgb_queues = {
            "left": deque(maxlen=self._sensor_queue_size),
            "right": deque(maxlen=self._sensor_queue_size),
        }
        self._depth_queues = {
            "left": deque(maxlen=self._sensor_queue_size),
            "right": deque(maxlen=self._sensor_queue_size),
        }
        self._sensor_sequence: Dict[str, Dict[str, int]] = {
            "left": {"rgb": 0, "depth": 0, "info": 0},
            "right": {"rgb": 0, "depth": 0, "info": 0},
        }
        self._latest_joint_state: Optional[JointState] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._bridge_client = self.create_client(
            ExecuteTask,
            self._bridge_service_name,
            callback_group=self._callback_group,
        )
        self._joint_plan_client = self.create_client(
            PlanToJoints,
            self._joint_plan_service_name,
            callback_group=self._callback_group,
        )
        self._cancel_plan_client = self.create_client(
            Trigger,
            self._cancel_plan_service_name,
            callback_group=self._callback_group,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._target_marker_publishers = {
            arm_name: self.create_publisher(MarkerArray, topic, marker_qos)
            for arm_name, topic in self._target_marker_topics.items()
            if topic
        }

        self._create_camera_subscriptions()
        self.create_subscription(
            JointState,
            "/joint_states",
            self._store_joint_state,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(
            ExecuteNamedGraspTask,
            "/execute_named_grasp_task",
            self._handle_execute_named_grasp_task,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/cancel_named_grasp_task",
            self._handle_cancel_named_task,
            callback_group=self._callback_group,
        )
        self.create_service(
            ExecuteNamedGraspTask,
            "/execute_named_pick_task",
            self._handle_execute_named_pick_task,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "Ready: /execute_named_grasp_task and /execute_named_pick_task -> 121 inference -> "
            f"{self._bridge_service_name}; "
            f"joint_plan_service={self._joint_plan_service_name}; "
            f"link6_to_handeye_parent_m={self._link6_to_handeye_parent_m:.4f}, "
            f"tcp_to_link6_m={self._tcp_to_link6_m:.4f}, "
            f"return_home_after_place={self._return_to_initial_pose_after_place}, "
            f"pre_grasp_offset_m={self._pre_grasp_offset_m:.4f}, "
            f"retreat_after_grasp=({self._retreat_after_grasp}, "
            f"offset_m={self._retreat_after_grasp_offset_m:.4f}), "
            f"retreat_after_place=({self._retreat_after_place}, "
            f"world_z_offset_m={self._retreat_after_place_offset_m:.4f}), "
            f"place_orientation_grasp_delta_compensation="
            f"{self._compensate_place_orientation_from_grasp_delta}, "
            f"ik_orientation_fallback=(enabled={self._grasp_ik_fallback_enabled}, "
            f"profiles={sorted(self._grasp_ik_fallback_profiles)}, "
            f"local_y_deg={list(self._grasp_ik_fallback_local_y_degrees)}, "
            f"local_z_deg={list(self._grasp_ik_fallback_local_z_degrees)}), "
            f"sensor_sync=(queue={self._sensor_queue_size}, "
            f"discard={self._sensor_discard_pairs_after_barrier}, "
            f"header_skew_ms={self._sensor_max_stamp_skew_s * 1000.0:.1f}, "
            f"arrival_skew_ms={self._sensor_max_arrival_skew_s * 1000.0:.1f}, "
            f"max_age_ms={self._sensor_max_arrival_age_s * 1000.0:.1f}), "
            f"capture_stationary=(translation_mm="
            f"{self._capture_stationary_translation_m * 1000.0:.1f}, "
            f"rotation_deg={self._capture_stationary_rotation_deg:.1f}, "
            f"tf_max_age_ms={self._capture_tf_max_age_s * 1000.0:.1f}), "
            f"target_markers={self._target_marker_topics}"
        )

    def _clear_target_markers(self, arm_name: str) -> None:
        publisher = self._target_marker_publishers.get(arm_name)
        if publisher is None:
            return
        clear = Marker()
        clear.header.frame_id = self._world_frame
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        message = MarkerArray()
        message.markers.append(clear)
        publisher.publish(message)

    def _publish_target_markers(
        self,
        arm_name: str,
        target_name: str,
        grasp_pose: PoseStamped,
        pre_grasp_pose: Optional[PoseStamped],
        status: str,
        status_detail: str = "",
    ) -> None:
        publisher = self._target_marker_publishers.get(arm_name)
        if publisher is None:
            return
        message = _build_target_marker_array(
            arm_name=arm_name,
            target_name=target_name,
            grasp_pose=grasp_pose,
            pre_grasp_pose=pre_grasp_pose,
            tcp_to_link6_m=self._tcp_to_link6_m,
            status=status,
            status_detail=status_detail,
            stamp=self.get_clock().now().to_msg(),
        )
        publisher.publish(message)
        self.get_logger().info(
            "Published live RViz grasp target: "
            f"arm={arm_name} target={target_name!r} status={status} "
            + _pose_log_text("tcp_world", grasp_pose)
        )

    def _create_camera_subscriptions(self) -> None:
        for arm_name, rgb_topic, depth_topic, info_topic in (
            ("left", self._left_rgb_topic, self._left_depth_topic, self._left_camera_info_topic),
            ("right", self._right_rgb_topic, self._right_depth_topic, self._right_camera_info_topic),
        ):
            if rgb_topic:
                self.create_subscription(
                    Image,
                    rgb_topic,
                    lambda msg, arm_name=arm_name: self._store_rgb(arm_name, msg),
                    qos_profile_sensor_data,
                    callback_group=self._callback_group,
                )
            if depth_topic:
                self.create_subscription(
                    Image,
                    depth_topic,
                    lambda msg, arm_name=arm_name: self._store_depth(arm_name, msg),
                    qos_profile_sensor_data,
                    callback_group=self._callback_group,
                )
            if info_topic:
                self.create_subscription(
                    CameraInfo,
                    info_topic,
                    lambda msg, arm_name=arm_name: self._store_camera_info(arm_name, msg),
                    qos_profile_sensor_data,
                    callback_group=self._callback_group,
                )

    def _store_rgb(self, arm_name: str, msg: Image) -> None:
        self._store_camera_sample(self._latest_rgb, arm_name, "rgb", msg)

    def _store_depth(self, arm_name: str, msg: Image) -> None:
        self._store_camera_sample(self._latest_depth, arm_name, "depth", msg)

    def _store_camera_info(self, arm_name: str, msg: CameraInfo) -> None:
        self._store_camera_sample(self._latest_camera_info, arm_name, "info", msg)

    def _store_camera_sample(
        self,
        storage: Dict[str, TimedMessageSample],
        arm_name: str,
        stream_name: str,
        msg: Any,
    ) -> None:
        arrival_monotonic_s = time.monotonic()
        receipt_ros_s = float(self.get_clock().now().nanoseconds) * 1.0e-9
        with self._sensor_condition:
            sequence = self._sensor_sequence[arm_name][stream_name] + 1
            self._sensor_sequence[arm_name][stream_name] = sequence
            sample = TimedMessageSample(
                message=msg,
                sequence=sequence,
                arrival_monotonic_s=arrival_monotonic_s,
                receipt_ros_s=receipt_ros_s,
            )
            storage[arm_name] = sample
            if stream_name == "rgb":
                self._rgb_queues[arm_name].append(sample)
            elif stream_name == "depth":
                self._depth_queues[arm_name].append(sample)
            self._sensor_condition.notify_all()

    def _store_joint_state(self, msg: JointState) -> None:
        with self._latest_lock:
            self._latest_joint_state = msg

    def _make_configured_place_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(self.declare_parameter("place_frame", self._world_frame).value)
        pose.pose.position.x = float(self.declare_parameter("place_position.x", 0.40).value)
        pose.pose.position.y = float(self.declare_parameter("place_position.y", -0.13).value)
        pose.pose.position.z = float(self.declare_parameter("place_position.z", 0.49).value)
        pose.pose.orientation.x = float(self.declare_parameter("place_orientation.x", 0.0).value)
        pose.pose.orientation.y = float(self.declare_parameter("place_orientation.y", 0.707).value)
        pose.pose.orientation.z = float(self.declare_parameter("place_orientation.z", 0.0).value)
        pose.pose.orientation.w = float(self.declare_parameter("place_orientation.w", 0.707).value)
        return pose

    def _make_configured_pick_place_pose(self) -> PoseStamped:
        pose = PoseStamped()
        default_place = self._configured_place_pose
        pose.header.frame_id = str(
            self.declare_parameter(
                "pick_place_frame",
                default_place.header.frame_id or self._world_frame,
            ).value
        )
        pose.pose.position.x = float(
            self.declare_parameter("pick_place_position.x", default_place.pose.position.x).value
        )
        pose.pose.position.y = float(
            self.declare_parameter("pick_place_position.y", default_place.pose.position.y).value
        )
        pose.pose.position.z = float(
            self.declare_parameter("pick_place_position.z", default_place.pose.position.z).value
        )
        pose.pose.orientation.x = float(
            self.declare_parameter("pick_place_orientation.x", default_place.pose.orientation.x).value
        )
        pose.pose.orientation.y = float(
            self.declare_parameter("pick_place_orientation.y", default_place.pose.orientation.y).value
        )
        pose.pose.orientation.z = float(
            self.declare_parameter("pick_place_orientation.z", default_place.pose.orientation.z).value
        )
        pose.pose.orientation.w = float(
            self.declare_parameter("pick_place_orientation.w", default_place.pose.orientation.w).value
        )
        return pose

    def _make_configured_capture_pose(
        self,
        arm_name: str,
        param_prefix: str | None = None,
        default_pose: PoseStamped | None = None,
    ) -> PoseStamped:
        prefix = param_prefix or f"{arm_name}_capture_tcp"
        default_frame = (
            default_pose.header.frame_id
            if default_pose is not None and default_pose.header.frame_id
            else self._world_frame
        )
        pose = PoseStamped()
        pose.header.frame_id = str(self.declare_parameter(f"{prefix}_frame", default_frame).value)
        pose.pose.position.x = float(
            self.declare_parameter(
                f"{prefix}_position.x",
                default_pose.pose.position.x if default_pose is not None else 0.0,
            ).value
        )
        pose.pose.position.y = float(
            self.declare_parameter(
                f"{prefix}_position.y",
                default_pose.pose.position.y if default_pose is not None else 0.0,
            ).value
        )
        pose.pose.position.z = float(
            self.declare_parameter(
                f"{prefix}_position.z",
                default_pose.pose.position.z if default_pose is not None else 0.0,
            ).value
        )
        pose.pose.orientation.x = float(
            self.declare_parameter(
                f"{prefix}_orientation.x",
                default_pose.pose.orientation.x if default_pose is not None else 0.0,
            ).value
        )
        pose.pose.orientation.y = float(
            self.declare_parameter(
                f"{prefix}_orientation.y",
                default_pose.pose.orientation.y if default_pose is not None else 0.0,
            ).value
        )
        pose.pose.orientation.z = float(
            self.declare_parameter(
                f"{prefix}_orientation.z",
                default_pose.pose.orientation.z if default_pose is not None else 0.0,
            ).value
        )
        pose.pose.orientation.w = float(
            self.declare_parameter(
                f"{prefix}_orientation.w",
                default_pose.pose.orientation.w if default_pose is not None else 1.0,
            ).value
        )
        return pose

    def _make_configured_capture_joints(
        self,
        param_prefix: str,
    ) -> Optional[Tuple[float, ...]]:
        enabled = bool(
            self.declare_parameter(f"{param_prefix}_enabled", False).value
        )
        values = tuple(
            float(value)
            for value in self.declare_parameter(
                f"{param_prefix}_positions",
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ).value
        )
        if not enabled:
            return None
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError(
                f"{param_prefix}_positions must contain six finite radians"
            )
        return values

    def _handle_cancel_named_task(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        self._cancel_requested.set()
        with self._sensor_condition:
            self._sensor_condition.notify_all()

        planner_detail = "planner cancel service unavailable"
        if self._cancel_plan_client.wait_for_service(timeout_sec=0.2):
            done = threading.Event()
            future = self._cancel_plan_client.call_async(Trigger.Request())
            future.add_done_callback(lambda _: done.set())
            if done.wait(timeout=self._cancel_service_timeout_s):
                try:
                    result = future.result()
                    planner_detail = str(result.message) if result else "planner returned no response"
                except Exception as exc:  # noqa: BLE001
                    planner_detail = f"planner cancel failed: {exc}"
            else:
                planner_detail = "planner cancel request timed out"

        response.success = True
        response.message = f"stop requested; {planner_detail}"
        self.get_logger().warn(response.message)
        return response

    def _is_cancel_requested(self) -> bool:
        return self._cancel_requested.is_set()

    def _sleep_interruptibly(self, seconds: float) -> bool:
        return not self._cancel_requested.wait(timeout=max(0.0, seconds))

    def _handle_execute_named_grasp_task(
        self,
        request: ExecuteNamedGraspTask.Request,
        response: ExecuteNamedGraspTask.Response,
    ) -> ExecuteNamedGraspTask.Response:
        if not self._busy_lock.acquire(blocking=False):
            response.success = False
            response.message = "state machine is busy"
            return response
        try:
            success, message = self._execute_named(request)
            response.success = success
            response.message = message
            return response
        finally:
            self._cancel_requested.clear()
            self._busy_lock.release()

    def _handle_execute_named_pick_task(
        self,
        request: ExecuteNamedGraspTask.Request,
        response: ExecuteNamedGraspTask.Response,
    ) -> ExecuteNamedGraspTask.Response:
        if not self._busy_lock.acquire(blocking=False):
            response.success = False
            response.message = "state machine is busy"
            return response
        try:
            success, message = self._execute_named(request, task_profile="pick")
            response.success = success
            response.message = message
            return response
        finally:
            self._cancel_requested.clear()
            self._busy_lock.release()

    def _execute_named(
        self,
        request: ExecuteNamedGraspTask.Request,
        task_profile: str = "grasp",
    ) -> Tuple[bool, str]:
        task_started = time.monotonic()
        if self._is_cancel_requested():
            return False, "execution canceled"
        arm_name = (request.arm_name or self._default_arm_name).strip().lower()
        if arm_name not in ("left", "right"):
            return False, "arm_name must be left or right"
        target_name = request.target_name.strip()
        if not target_name:
            return False, "target_name is required"
        camera_frame = request.camera_frame.strip() or self._camera_frame_for_arm(arm_name)
        scene_id = int(request.scene_id) if int(request.scene_id) > 0 else self._default_scene_id
        open_width = float(request.gripper_width_m) if request.gripper_width_m > 0.0 else self._default_gripper_width_m
        if request.place_tcp_pose.header.frame_id:
            place_pose = request.place_tcp_pose
        elif task_profile == "pick":
            place_pose = self._configured_pick_place_pose
        else:
            place_pose = self._configured_place_pose

        self.get_logger().info(
            "[TIMING] state_machine phase=task_start "
            f"profile={task_profile} target={target_name!r} arm={arm_name} "
            f"scene_id={scene_id} camera_frame={camera_frame}"
        )
        self._clear_target_markers(arm_name)

        opened_before_inference = False
        if task_profile == "pick":
            initial_pose = self._configured_pick_capture_pose[arm_name]
            capture_label = "pick_capture_tcp_target"
        else:
            initial_pose = self._configured_capture_pose[arm_name]
            capture_label = "capture_tcp_target"
        initial_joint_positions = _capture_joint_target_for_profile(
            task_profile,
            arm_name,
            self._configured_capture_joints,
            self._configured_pick_capture_joints,
        )

        if self._move_to_capture_pose_before_inference:
            phase_started = time.monotonic()
            if initial_joint_positions is not None:
                self.get_logger().info(
                    f"Moving to fixed {task_profile} capture joints before inference: "
                    f"arm={arm_name} joints_rad={list(initial_joint_positions)}"
                )
                success, detail = self._call_joint_plan(
                    arm_name=arm_name,
                    scene_id=scene_id,
                    joint_positions=initial_joint_positions,
                    keep_grasp_ellipsoid=False,
                )
            else:
                self.get_logger().info(
                    f"Moving to fixed {task_profile} capture pose before inference: "
                    + _pose_log_text(capture_label, initial_pose)
                )
                success, detail = self._call_bridge_task(
                    task_id=f"{target_name}_CAPTURE_POSE".replace(" ", "_"),
                    task_type="MOVE",
                    target_id=target_name,
                    arm_name=arm_name,
                    scene_id=scene_id,
                    tcp_pose=initial_pose,
                    gripper_command=False,
                    gripper_opening_m=0.0,
                )
            self.get_logger().info(
                "[TIMING] state_machine phase=move_to_capture_pose "
                f"success={success} elapsed_ms={(time.monotonic() - phase_started) * 1000.0:.1f} "
                f"detail={detail!r}"
            )
            if not success:
                return False, f"move to capture pose failed: {detail}"
            if self._capture_settle_s > 0.0:
                phase_started = time.monotonic()
                if not self._sleep_interruptibly(self._capture_settle_s):
                    return False, "execution canceled during capture settle"
                self.get_logger().info(
                    "[TIMING] state_machine phase=capture_settle "
                    f"elapsed_ms={(time.monotonic() - phase_started) * 1000.0:.1f}"
                )

        should_open_before_inference = (
            self._pick_open_before_inference
            if task_profile == "pick"
            else self._open_before_inference
        )
        if should_open_before_inference:
            phase_started = time.monotonic()
            if initial_joint_positions is not None:
                self.get_logger().info(
                    f"Opening gripper at fixed {task_profile} capture joints: "
                    f"arm={arm_name} joints_rad={list(initial_joint_positions)}"
                )
                success, detail = self._call_joint_plan(
                    arm_name=arm_name,
                    scene_id=scene_id,
                    joint_positions=initial_joint_positions,
                    gripper_command=True,
                    gripper_opening_m=open_width,
                    keep_grasp_ellipsoid=False,
                )
            else:
                try:
                    current_tcp = self._current_tcp_pose_world(arm_name)
                except Exception as exc:  # noqa: BLE001
                    return False, f"failed to read current TCP for pre-inference open step: {exc}"
                self.get_logger().info(
                    f"Opening gripper before {task_profile} inference: "
                    + _pose_log_text("tcp_target", current_tcp)
                )
                success, detail = self._call_bridge_task(
                    task_id=(
                        f"{target_name}_{task_profile}_OPEN_BEFORE_INFERENCE".replace(" ", "_")
                    ),
                    task_type="MOVE",
                    target_id=target_name,
                    arm_name=arm_name,
                    scene_id=scene_id,
                    tcp_pose=current_tcp,
                    gripper_command=True,
                    gripper_opening_m=open_width,
                )
            self.get_logger().info(
                f"[TIMING] state_machine phase={task_profile}_open_before_inference "
                f"success={success} elapsed_ms={(time.monotonic() - phase_started) * 1000.0:.1f} "
                f"detail={detail!r}"
            )
            if not success:
                return False, f"open before inference failed: {detail}"
            opened_before_inference = True
            open_settle_s = (
                self._pick_open_settle_s
                if task_profile == "pick"
                else self._open_before_inference_settle_s
            )
            if open_settle_s > 0.0:
                phase_started = time.monotonic()
                if not self._sleep_interruptibly(open_settle_s):
                    return False, "execution canceled during gripper settle"
                self.get_logger().info(
                    f"[TIMING] state_machine phase={task_profile}_open_settle "
                    f"elapsed_ms={(time.monotonic() - phase_started) * 1000.0:.1f}"
                )

        try:
            bundle, capture_extrinsics = self._capture_sensor_snapshot(
                arm_name,
                camera_frame=camera_frame,
                phase_name="inference_capture",
            )
        except Exception as exc:  # noqa: BLE001
            return False, f"failed to capture synchronized camera snapshot: {exc}"
        rgb, depth, info = bundle.rgb, bundle.depth, bundle.info
        self.get_logger().info(
            "Using %s camera for inference: rgb=%s, depth=%s, camera_info=%s, frame=%s, image=%dx%d"
            % (
                arm_name,
                self._left_rgb_topic if arm_name == "left" else self._right_rgb_topic,
                self._left_depth_topic if arm_name == "left" else self._right_depth_topic,
                self._left_camera_info_topic if arm_name == "left" else self._right_camera_info_topic,
                camera_frame,
                int(info.width),
                int(info.height),
            )
        )

        try:
            phase_started = time.monotonic()
            remote = asyncio.run(
                self._request_remote_grasp(
                    target_name,
                    arm_name,
                    camera_frame,
                    rgb,
                    depth,
                    info,
                    capture_extrinsics,
                    bundle,
                )
            )
        except Exception as exc:  # noqa: BLE001
            return False, f"remote inference failed: {exc}"
        self.get_logger().info(
            "[TIMING] state_machine phase=remote_inference_total "
            f"elapsed_ms={(time.monotonic() - phase_started) * 1000.0:.1f}"
        )

        self.get_logger().info(_pose_log_text("remote_grasp_tcp", remote.tcp_pose))

        steps = []
        if self._open_before_grasp and not opened_before_inference:
            try:
                current_tcp = self._current_tcp_pose_world(arm_name)
            except Exception as exc:  # noqa: BLE001
                return False, f"failed to read current TCP for open step: {exc}"
            steps.append(("OPEN", current_tcp, True, open_width))

        try:
            phase_started = time.monotonic()
            grasp_pose = self._camera_tcp_pose_to_world_pose_at_capture(
                arm_name,
                remote.tcp_pose,
                capture_extrinsics["world_from_camera"],
            )
        except Exception as exc:  # noqa: BLE001
            return False, f"failed to convert 121 TCP pose to world: {exc}"
        self.get_logger().info(
            "[TIMING] state_machine phase=grasp_pose_conversion "
            f"elapsed_ms={(time.monotonic() - phase_started) * 1000.0:.1f}"
        )
        self.get_logger().info(_pose_log_text("capture_world_grasp_tcp", grasp_pose))

        ik_fallback_active = (
            self._grasp_ik_fallback_enabled
            and task_profile in self._grasp_ik_fallback_profiles
        )
        if ik_fallback_active and self._enable_roll_180_grasp_branch:
            self.get_logger().warn(
                "Small-angle IK fallback is active; suppressing the separate "
                "180-degree grasp branch for this task"
            )

        grasp_orientation_mode = (
            self._pick_grasp_orientation_mode
            if task_profile == "pick" and self._pick_grasp_orientation_mode
            else self._grasp_orientation_mode
        )

        grasp_poses = [grasp_pose]
        if grasp_orientation_mode in ("capture_tcp", "capture", "photo"):
            grasp_pose.pose.orientation = self._configured_capture_pose[arm_name].pose.orientation
            grasp_poses = [grasp_pose]
        elif grasp_orientation_mode in ("vertical_down", "top_down", "fixed_tcp", "fixed"):
            fixed_orientation = self._fixed_tcp_orientation_by_arm.get(
                arm_name, self._vertical_down_tcp_orientation
            )
            grasp_pose.pose.orientation.x = fixed_orientation[0]
            grasp_pose.pose.orientation.y = fixed_orientation[1]
            grasp_pose.pose.orientation.z = fixed_orientation[2]
            grasp_pose.pose.orientation.w = fixed_orientation[3]
            grasp_poses = [grasp_pose]
        elif grasp_orientation_mode in ("remote_level_opening", "level_opening"):
            grasp_pose = self._level_grasp_opening_axis(grasp_pose)
            if self._enable_roll_180_grasp_branch and not ik_fallback_active:
                grasp_poses = self._ordered_roll_180_grasp_poses(arm_name, grasp_pose)
            else:
                grasp_poses = [grasp_pose]
        elif grasp_orientation_mode == "current_tcp":
            current_tcp_pose = self._current_tcp_pose_world(arm_name)
            grasp_pose.pose.orientation = current_tcp_pose.pose.orientation
            grasp_poses = [grasp_pose]
        elif grasp_orientation_mode not in ("", "remote", "graspnet"):
            return False, (
                "grasp_orientation_mode must be remote, remote_level_opening, "
                "vertical_down, fixed_tcp, capture_tcp, or current_tcp, "
                f"got {grasp_orientation_mode!r}"
            )
        elif self._enable_roll_180_grasp_branch and not ik_fallback_active:
            grasp_poses = self._ordered_roll_180_grasp_poses(arm_name, grasp_pose)

        if self._grasp_z_offset_m != 0.0:
            offset_grasp_poses = []
            for pose in grasp_poses:
                pose = self._copy_pose(pose)
                pose.pose.position.z += self._grasp_z_offset_m
                offset_grasp_poses.append(pose)
                self.get_logger().info(_pose_log_text("z_offset_remote_planning_tcp", pose))
            grasp_poses = offset_grasp_poses

        if ik_fallback_active:
            if len(grasp_poses) != 1:
                return False, (
                    "internal error: small-angle IK fallback requires one base grasp pose"
                )
            grasp_pose_candidates = _build_ik_orientation_candidates(
                grasp_poses[0],
                self._grasp_ik_fallback_local_y_degrees,
                self._grasp_ik_fallback_local_z_degrees,
            )
            grasp_poses = [candidate.pose for candidate in grasp_pose_candidates]
        else:
            grasp_pose_candidates = [
                GraspPoseCandidate(
                    name="original" if len(grasp_poses) == 1 else f"orientation_{index + 1}",
                    local_axis="none",
                    rotation_degrees=0.0,
                    pose=pose,
                )
                for index, pose in enumerate(grasp_poses)
            ]

        try:
            grasp_approach_axes_world = [
                _pose_local_axis_world(pose, "z") for pose in grasp_poses
            ]
            grasp_opening_axes_world = [
                _pose_local_axis_world(pose, "y") for pose in grasp_poses
            ]
        except Exception as exc:  # noqa: BLE001
            return False, f"failed to compute grasp TCP axes: {exc}"

        for index, candidate in enumerate(grasp_pose_candidates):
            approach_axis = grasp_approach_axes_world[index]
            opening_axis = grasp_opening_axes_world[index]
            self.get_logger().info(
                "grasp IK orientation candidate "
                f"{index + 1}/{len(grasp_pose_candidates)}: "
                f"name={candidate.name}, local_axis={candidate.local_axis}, "
                f"rotation_deg={candidate.rotation_degrees:+.1f}, "
                f"opening_y_world=({opening_axis[0]:.4f}, {opening_axis[1]:.4f}, "
                f"{opening_axis[2]:.4f}), "
                f"approach_z_world=({approach_axis[0]:.4f}, {approach_axis[1]:.4f}, "
                f"{approach_axis[2]:.4f})"
            )

        # The returned 121 grasp origin is already the physical gripper-end
        # center consumed by the local MoveIt bridge (the 135.8 mm planning
        # TCP).  The separate 59.3 mm offset belongs only to the hand-eye
        # calibration/observation chain.  Rebasing the returned target between
        # those two offsets would add an erroneous 76.5 mm along local +Z.
        for pose in grasp_poses:
            self.get_logger().info(_pose_log_text("planning_grasp_tcp_from_121", pose))

        pre_grasp_poses = []
        if self._pre_grasp_before_grasp and self._pre_grasp_offset_m > 0.0:
            pre_grasp_poses = [
                self._retracted_pose_from_axis(
                    pose,
                    approach_axis,
                    self._pre_grasp_offset_m,
                    log_label="pre_grasp_from_approach_axis",
                    min_z_m=self._pre_grasp_min_z_m,
                )
                for pose, approach_axis in zip(
                    grasp_poses,
                    grasp_approach_axes_world,
                )
            ]
            steps.append(("PRE_GRASP", pre_grasp_poses, True, open_width))
        marker_grasp_pose = grasp_poses[0]
        marker_pre_grasp_pose = pre_grasp_poses[0] if pre_grasp_poses else None
        self._publish_target_markers(
            arm_name,
            target_name,
            marker_grasp_pose,
            marker_pre_grasp_pose,
            "pending",
        )
        if self._pre_close_snapshot_enabled:
            # Reach the final TCP with the fingers still open.  The following
            # GRASP request reuses the same target and closes only after the
            # wrist-camera evidence and live TCP pose have been saved.
            steps.append(("GRASP_APPROACH", grasp_poses, True, open_width))
        steps.append(("GRASP", grasp_poses, True, self._closed_gripper_width_m))

        if self._retreat_after_grasp and self._retreat_after_grasp_offset_m > 0.0:
            retreat_poses = [
                self._retracted_pose_from_axis(
                    pose,
                    approach_axis,
                    self._retreat_after_grasp_offset_m,
                    log_label="retreat_after_grasp_from_approach_axis",
                    min_z_m=0.0,
                )
                for pose, approach_axis in zip(
                    grasp_poses,
                    grasp_approach_axes_world,
                )
            ]
            steps.append(("RETREAT_AFTER_GRASP", retreat_poses, False, 0.0))

        if self._lift_after_grasp:
            steps.append(("LIFT_AFTER_GRASP", None, False, 0.0))

        if self._place_lift_before_place:
            steps.extend(
                (task_type, None, False, 0.0)
                for task_type in _PLACE_TRANSPORT_STEP_ORDER
            )

        final_place_pose = place_pose
        if self._place_descend_before_place and self._place_descend_offset_m > 0.0:
            try:
                final_place_pose = self._copy_pose(self._pose_to_world(place_pose))
                place_waypoint_pose = _elevated_place_waypoint(
                    final_place_pose,
                    self._place_descend_offset_m,
                )
            except Exception as exc:  # noqa: BLE001
                return False, f"failed to compute elevated place waypoint: {exc}"
            steps.append(("PLACE_WAYPOINT", place_waypoint_pose, False, 0.0))
            if self._is_cancel_requested():
                return False, "execution canceled"

        return_home_pose = (
            self._copy_pose(initial_pose)
            if self._return_to_initial_pose_after_place
            else None
        )
        steps.extend(
            _place_completion_steps(
                final_place_pose,
                open_width,
                retreat_after_place=(
                    self._retreat_after_place
                    and self._retreat_after_place_offset_m > 0.0
                ),
                return_home_pose=return_home_pose,
            )
        )

        selected_grasp_candidate_index: Optional[int] = None
        for index, (task_type, pose, gripper_command, gripper_opening) in enumerate(steps, start=1):
            if task_type == "LIFT_AFTER_GRASP":
                try:
                    pose = self._current_tcp_pose_world(arm_name)
                    pose.pose.position.z += self._lift_offset_m
                except Exception as exc:  # noqa: BLE001
                    return False, f"failed to compute lift pose: {exc}"
            elif task_type == "RETREAT_AFTER_PLACE":
                try:
                    pose = _elevated_place_waypoint(
                        self._current_tcp_pose_world(arm_name),
                        self._retreat_after_place_offset_m,
                    )
                    self.get_logger().info(
                        "Computed post-place retreat along world +Z: "
                        f"offset_m={self._retreat_after_place_offset_m:.4f}, "
                        + _pose_log_text("tcp_target", pose)
                    )
                except Exception as exc:  # noqa: BLE001
                    return False, f"failed to compute post-place retreat pose: {exc}"
            elif task_type == "PLACE_LIFT":
                try:
                    pose = self._current_tcp_pose_world(arm_name)
                    place_pose_world = self._pose_to_world(place_pose)
                    target_z = (
                        self._place_lift_height_m
                        if self._place_lift_height_m > 0.0
                        else place_pose_world.pose.position.z
                    )
                    pose.pose.position.z = max(pose.pose.position.z, target_z)
                except Exception as exc:  # noqa: BLE001
                    return False, f"failed to compute place lift pose: {exc}"
            elif task_type == "PLACE_XY":
                try:
                    current_tcp = self._current_tcp_pose_world(arm_name)
                    place_pose_world = self._pose_to_world(place_pose)
                    pose = self._copy_pose(current_tcp)
                    pose.pose.position.x = place_pose_world.pose.position.x
                    pose.pose.position.y = place_pose_world.pose.position.y
                    pose.pose.position.z = current_tcp.pose.position.z
                except Exception as exc:  # noqa: BLE001
                    return False, f"failed to compute place XY pose: {exc}"
            elif task_type == "PLACE_ORIENT":
                try:
                    current_tcp = self._current_tcp_pose_world(arm_name)
                    place_pose_world = self._pose_to_world(place_pose)
                    pose = self._copy_pose(place_pose_world)
                    pose.pose.position.x = current_tcp.pose.position.x
                    pose.pose.position.y = current_tcp.pose.position.y
                    pose.pose.position.z = current_tcp.pose.position.z
                except Exception as exc:  # noqa: BLE001
                    return False, f"failed to compute place orientation pose: {exc}"

            if (
                self._compensate_place_orientation_from_grasp_delta
                and ik_fallback_active
                and task_type in _PLACE_ORIENTATION_TARGET_STEPS
                and selected_grasp_candidate_index is not None
                and selected_grasp_candidate_index > 0
            ):
                assert pose is not None
                selected_candidate = grasp_pose_candidates[
                    selected_grasp_candidate_index
                ]
                nominal_place_pose = pose
                try:
                    pose = _place_pose_with_grasp_orientation_compensation(
                        nominal_place_pose,
                        grasp_pose_candidates[0].pose,
                        selected_candidate.pose,
                    )
                except Exception as exc:  # noqa: BLE001
                    return False, (
                        f"failed to compensate {task_type} orientation from "
                        f"grasp candidate {selected_candidate.name}: {exc}"
                    )
                self.get_logger().info(
                    "Applied selected grasp rotation delta to place orientation: "
                    f"step={task_type}, candidate={selected_candidate.name}, "
                    f"local_axis={selected_candidate.local_axis}, "
                    f"rotation_deg={selected_candidate.rotation_degrees:+.1f}; "
                    + _pose_log_text("nominal_place_tcp", nominal_place_pose)
                    + "; "
                    + _pose_log_text("compensated_place_tcp", pose)
                )

            if task_type == "RETURN_HOME" and initial_joint_positions is not None:
                success = False
                last_detail = ""
                for attempt in range(1, self._planning_retry_attempts + 1):
                    self.get_logger().info(
                        f"Starting joint step {index}/{len(steps)}: {task_type} "
                        f"attempt {attempt}/{self._planning_retry_attempts}, "
                        f"arm={arm_name} joints_rad={list(initial_joint_positions)}"
                    )
                    phase_started = time.monotonic()
                    success, detail = self._call_joint_plan(
                        arm_name=arm_name,
                        scene_id=scene_id,
                        joint_positions=initial_joint_positions,
                        keep_grasp_ellipsoid=False,
                    )
                    last_detail = detail
                    self.get_logger().info(
                        "[TIMING] state_machine phase=joint_step "
                        f"step={task_type} index={index}/{len(steps)} "
                        f"attempt={attempt}/{self._planning_retry_attempts} "
                        f"success={success} "
                        f"elapsed_ms={(time.monotonic() - phase_started) * 1000.0:.1f} "
                        f"detail={detail!r}"
                    )
                    if success:
                        break
                    if _is_planning_failure(detail) and attempt < self._planning_retry_attempts:
                        continue
                    break
                if not success:
                    self._publish_target_markers(
                        arm_name,
                        target_name,
                        marker_grasp_pose,
                        marker_pre_grasp_pose,
                        "failed",
                        f"{task_type}: {last_detail}",
                    )
                    return False, f"{task_type} failed: {last_detail}"
                continue

            bridge_task_type = _bridge_task_type_for_step(task_type)

            pose_candidates = pose if isinstance(pose, list) else [pose]
            candidate_indexes = _candidate_indexes_for_step(
                task_type,
                len(pose_candidates),
                selected_grasp_candidate_index,
            )
            is_grasp_candidate_step = (
                task_type in _GRASP_CANDIDATE_STEPS
                and len(pose_candidates) == len(grasp_pose_candidates)
            )
            success = False
            last_detail = ""
            for candidate_position, candidate_zero_index in enumerate(candidate_indexes):
                candidate_pose = pose_candidates[candidate_zero_index]
                assert candidate_pose is not None
                candidate_index = candidate_zero_index + 1
                candidate_name = (
                    grasp_pose_candidates[candidate_zero_index].name
                    if is_grasp_candidate_step
                    else "direct"
                )
                for attempt in range(1, self._planning_retry_attempts + 1):
                    self.get_logger().info(
                        f"Starting bridge step {index}/{len(steps)}: {task_type}"
                        f"[{candidate_index}/{len(pose_candidates)}] "
                        f"candidate_name={candidate_name} "
                        f"attempt {attempt}/{self._planning_retry_attempts}, "
                        + _pose_log_text("tcp_target", candidate_pose)
                    )
                    task_id = (
                        f"{target_name}_{index}_{task_type}_{candidate_index}_try_{attempt}"
                    ).replace(" ", "_")
                    phase_started = time.monotonic()
                    success, detail = self._call_bridge_task(
                        task_id=task_id,
                        task_type=bridge_task_type,
                        target_id=target_name,
                        arm_name=arm_name,
                        scene_id=scene_id,
                        tcp_pose=candidate_pose,
                        gripper_command=gripper_command,
                        gripper_opening_m=gripper_opening,
                    )
                    self.get_logger().info(
                        "[TIMING] state_machine phase=bridge_step "
                        f"step={task_type} bridge_type={bridge_task_type} "
                        f"index={index}/{len(steps)} candidate={candidate_index}/{len(pose_candidates)} "
                        f"attempt={attempt}/{self._planning_retry_attempts} success={success} "
                        f"elapsed_ms={(time.monotonic() - phase_started) * 1000.0:.1f} "
                        f"detail={detail!r}"
                    )
                    if success:
                        last_detail = detail
                        break
                    last_detail = detail
                    if _is_no_ik_failure(detail):
                        self.get_logger().warn(
                            f"{task_type} candidate {candidate_index}/"
                            f"{len(pose_candidates)} ({candidate_name}) has no IK solution"
                        )
                        break
                    if _is_planning_failure(detail) and attempt < self._planning_retry_attempts:
                        self.get_logger().warn(
                            f"{task_type} candidate {candidate_index}/{len(pose_candidates)} "
                            f"planning failed on attempt {attempt}/{self._planning_retry_attempts}; "
                            "retrying"
                        )
                        continue
                    self.get_logger().warn(
                        f"{task_type} candidate {candidate_index}/{len(pose_candidates)} "
                        f"attempt {attempt}/{self._planning_retry_attempts} failed: {detail}"
                    )
                    break
                if success:
                    if (
                        is_grasp_candidate_step
                        and task_type in _GRASP_CANDIDATE_SELECTION_STEPS
                        and selected_grasp_candidate_index is None
                    ):
                        selected_grasp_candidate_index = candidate_zero_index
                        marker_grasp_pose = grasp_poses[candidate_zero_index]
                        marker_pre_grasp_pose = (
                            pre_grasp_poses[candidate_zero_index]
                            if pre_grasp_poses
                            else None
                        )
                        self.get_logger().info(
                            "Locked grasp orientation candidate for all coupled steps: "
                            f"index={candidate_index}/{len(grasp_pose_candidates)}, "
                            f"name={candidate_name}, selected_by={task_type}"
                        )
                        self._publish_target_markers(
                            arm_name,
                            target_name,
                            marker_grasp_pose,
                            marker_pre_grasp_pose,
                            "pending",
                            f"locked IK candidate {candidate_name}",
                        )
                    break

                has_next_candidate = candidate_position + 1 < len(candidate_indexes)
                if (
                    ik_fallback_active
                    and is_grasp_candidate_step
                    and selected_grasp_candidate_index is None
                    and _is_no_ik_failure(last_detail)
                    and has_next_candidate
                ):
                    next_zero_index = candidate_indexes[candidate_position + 1]
                    next_candidate = grasp_pose_candidates[next_zero_index]
                    self.get_logger().warn(
                        "No IK for grasp orientation candidate; trying the next "
                        "small local rotation: "
                        f"failed={candidate_name}, next={next_candidate.name}"
                    )
                    continue
                if not ik_fallback_active and has_next_candidate:
                    # Preserve the optional legacy 180-degree branch behavior.
                    continue
                break
            if not success:
                self._publish_target_markers(
                    arm_name,
                    target_name,
                    marker_grasp_pose,
                    marker_pre_grasp_pose,
                    "failed",
                    f"{task_type}: {last_detail}",
                )
                return False, f"{task_type} failed: {last_detail}"
            if task_type == "GRASP_APPROACH" and self._pre_close_snapshot_enabled:
                if self._pre_close_snapshot_settle_s > 0.0:
                    if not self._sleep_interruptibly(self._pre_close_snapshot_settle_s):
                        return False, "execution canceled before snapshot"
                snapshot_started = time.monotonic()
                try:
                    snapshot_dir = self._save_pre_close_snapshot(
                        target_name=target_name,
                        arm_name=arm_name,
                        commanded_tcp_world=candidate_pose,
                        inference_rgb=rgb,
                        inference_depth=depth,
                        inference_sensor_bundle=bundle,
                    )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(
                        "Failed to save pre-close snapshot; continuing with grasp close: "
                        f"{exc}"
                    )
                else:
                    self.get_logger().info(
                        "[TIMING] state_machine phase=pre_close_snapshot "
                        f"success=True elapsed_ms="
                        f"{(time.monotonic() - snapshot_started) * 1000.0:.1f} "
                        f"path={str(snapshot_dir)!r}"
                    )

        self._publish_target_markers(
            arm_name,
            target_name,
            marker_grasp_pose,
            marker_pre_grasp_pose,
            "success",
            "task completed",
        )
        self.get_logger().info(
            "[TIMING] state_machine phase=task_finish "
            f"success=True total_ms={(time.monotonic() - task_started) * 1000.0:.1f}"
        )
        return True, "named grasp task completed"

    def _sensor_sequence_snapshot(self, arm_name: str) -> Dict[str, int]:
        with self._latest_lock:
            return dict(self._sensor_sequence[arm_name])

    def _lookup_world_link6_matrix(
        self,
        arm_name: str,
    ) -> Tuple[np.ndarray, float, float]:
        transform = self._tf_buffer.lookup_transform(
            self._world_frame,
            f"{arm_name}_link6",
            Time(),
            timeout=Duration(seconds=self._tf_timeout_s),
        )
        stamp = transform.header.stamp
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        now_ros_s = float(self.get_clock().now().nanoseconds) * 1.0e-9
        age_s = now_ros_s - stamp_s
        if not math.isfinite(stamp_s) or stamp_s <= 0.0:
            raise RuntimeError(f"{arm_name} link6 TF has an invalid or zero timestamp")
        if age_s < -0.01 or age_s > self._capture_tf_max_age_s:
            raise RuntimeError(
                f"{arm_name} link6 TF is not fresh: age={age_s * 1000.0:.3f} ms, "
                f"allowed=[-10.000, {self._capture_tf_max_age_s * 1000.0:.3f}] ms"
            )
        return _matrix_from_transform(transform), stamp_s, age_s

    def _capture_sensor_snapshot(
        self,
        arm_name: str,
        *,
        camera_frame: str,
        phase_name: str,
    ) -> Tuple[SensorBundle, Dict[str, Any]]:
        """Capture a post-settle RGB-D pair while proving the wrist stayed still."""

        expected_camera_frame = self._camera_frame_for_arm(arm_name)
        if camera_frame != expected_camera_frame:
            raise RuntimeError(
                f"capture camera frame mismatch for {arm_name}: "
                f"expected {expected_camera_frame!r}, got {camera_frame!r}"
            )
        phase_started = time.monotonic()
        before_link6, before_tf_stamp_s, before_tf_age_s = (
            self._lookup_world_link6_matrix(arm_name)
        )
        with self._sensor_condition:
            barrier_sequences = dict(self._sensor_sequence[arm_name])
            barrier_monotonic_s = time.monotonic()

        bundle = self._wait_for_sensor_bundle(
            arm_name,
            minimum_sequences=(
                barrier_sequences if self._sensor_require_post_open_frames else None
            ),
            barrier_monotonic_s=barrier_monotonic_s,
        )
        tf_advance_deadline = time.monotonic() + self._capture_tf_advance_timeout_s
        while True:
            after_link6, after_tf_stamp_s, after_tf_age_s = (
                self._lookup_world_link6_matrix(arm_name)
            )
            if after_tf_stamp_s > before_tf_stamp_s:
                break
            if time.monotonic() >= tf_advance_deadline:
                raise RuntimeError(
                    f"{arm_name} link6 TF did not advance while acquiring {phase_name}: "
                    f"before={before_tf_stamp_s:.9f}, after={after_tf_stamp_s:.9f}"
                )
            if not self._sleep_interruptibly(0.01):
                raise RuntimeError("execution canceled while waiting for TF")
        latest_receipt_ros_s = max(
            bundle.rgb_receipt_ros_s,
            bundle.depth_receipt_ros_s,
        )
        if after_tf_stamp_s < latest_receipt_ros_s - 0.05:
            raise RuntimeError(
                f"{arm_name} link6 TF does not cover the selected {phase_name} "
                f"arrival window: tf_stamp={after_tf_stamp_s:.9f}, "
                f"latest_sensor_receipt={latest_receipt_ros_s:.9f}, "
                f"lag={(latest_receipt_ros_s - after_tf_stamp_s) * 1000.0:.3f} ms"
            )
        translation_delta_m, rotation_delta_deg = transform_pose_delta(
            before_link6,
            after_link6,
        )
        if (
            translation_delta_m > self._capture_stationary_translation_m
            or rotation_delta_deg > self._capture_stationary_rotation_deg
        ):
            raise RuntimeError(
                f"{arm_name} wrist moved while acquiring {phase_name}: "
                f"translation={translation_delta_m * 1000.0:.3f} mm "
                f"(limit {self._capture_stationary_translation_m * 1000.0:.3f} mm), "
                f"rotation={rotation_delta_deg:.3f} deg "
                f"(limit {self._capture_stationary_rotation_deg:.3f} deg)"
            )

        bundle = replace(
            bundle,
            tf_before_stamp_s=before_tf_stamp_s,
            tf_after_stamp_s=after_tf_stamp_s,
            tf_before_age_s=before_tf_age_s,
            tf_after_age_s=after_tf_age_s,
            tf_translation_delta_m=translation_delta_m,
            tf_rotation_delta_deg=rotation_delta_deg,
        )
        extrinsics = self._dynamic_extrinsics_for_121(
            arm_name,
            world_from_link6=after_link6,
        )
        self.get_logger().info(
            "[TIMING] state_machine phase=%s success=True elapsed_ms=%.1f "
            "barrier_rgb_seq=%d barrier_depth_seq=%d rgb_seq=%d depth_seq=%d "
            "matched_pairs=%d stamp_skew_ms=%.3f arrival_skew_ms=%.3f "
            "arrival_age_ms=%.3f wrist_delta_mm=%.3f wrist_delta_deg=%.3f "
            "tf_before_age_ms=%.3f tf_after_age_ms=%.3f camera_frame=%s"
            % (
                phase_name,
                (time.monotonic() - phase_started) * 1000.0,
                barrier_sequences.get("rgb", 0),
                barrier_sequences.get("depth", 0),
                bundle.rgb_sequence,
                bundle.depth_sequence,
                bundle.matched_pair_count,
                bundle.stamp_skew_s * 1000.0,
                bundle.arrival_skew_s * 1000.0,
                bundle.arrival_age_s * 1000.0,
                translation_delta_m * 1000.0,
                rotation_delta_deg,
                before_tf_age_s * 1000.0,
                after_tf_age_s * 1000.0,
                camera_frame,
            )
        )
        return bundle, extrinsics

    def _wait_for_sensor_bundle(
        self,
        arm_name: str,
        *,
        minimum_sequences: Optional[Dict[str, int]],
        barrier_monotonic_s: float,
    ) -> SensorBundle:
        deadline = time.monotonic() + self._sensor_sync_timeout_s
        last_reason = "camera messages have not arrived"
        with self._sensor_condition:
            while True:
                if self._is_cancel_requested():
                    raise RuntimeError("execution canceled while waiting for camera data")
                info_sample = self._latest_camera_info.get(arm_name)
                if info_sample is not None:
                    now_monotonic_s = time.monotonic()
                    pair = select_synced_pair(
                        tuple(self._rgb_queues[arm_name]),
                        tuple(self._depth_queues[arm_name]),
                        minimum_rgb_sequence=(minimum_sequences or {}).get("rgb", 0),
                        minimum_depth_sequence=(minimum_sequences or {}).get("depth", 0),
                        barrier_monotonic_s=(
                            barrier_monotonic_s
                            if minimum_sequences is not None
                            else -math.inf
                        ),
                        now_monotonic_s=now_monotonic_s,
                        max_header_skew_s=self._sensor_max_stamp_skew_s,
                        max_arrival_skew_s=self._sensor_max_arrival_skew_s,
                        max_arrival_age_s=self._sensor_max_arrival_age_s,
                        discard_pairs_after_barrier=(
                            self._sensor_discard_pairs_after_barrier
                            if minimum_sequences is not None
                            else 0
                        ),
                    )
                    if pair is not None:
                        rgb = pair.rgb.message
                        depth = pair.depth.message
                        info = info_sample.message
                        expected_frame = self._camera_frame_for_arm(arm_name)
                        frames = {
                            str(rgb.header.frame_id),
                            str(depth.header.frame_id),
                            str(info.header.frame_id),
                        }
                        if frames != {expected_frame}:
                            last_reason = (
                                f"camera frame mismatch: expected {expected_frame!r}, "
                                f"got {sorted(frames)!r}"
                            )
                        elif (
                            int(rgb.width) != int(depth.width)
                            or int(rgb.height) != int(depth.height)
                            or int(info.width) != int(rgb.width)
                            or int(info.height) != int(rgb.height)
                        ):
                            last_reason = "RGB/depth/camera-info dimensions do not match"
                        elif (
                            len(info.k) != 9
                            or not all(math.isfinite(float(v)) for v in info.k)
                            or float(info.k[0]) <= 0.0
                            or float(info.k[4]) <= 0.0
                            or not all(math.isfinite(float(v)) for v in info.d)
                        ):
                            last_reason = "camera intrinsics are invalid"
                        else:
                            return SensorBundle(
                                rgb=rgb,
                                depth=depth,
                                info=info,
                                rgb_sequence=pair.rgb.sequence,
                                depth_sequence=pair.depth.sequence,
                                rgb_arrival_monotonic_s=pair.rgb.arrival_monotonic_s,
                                depth_arrival_monotonic_s=pair.depth.arrival_monotonic_s,
                                rgb_stamp_s=pair.rgb_stamp_s,
                                depth_stamp_s=pair.depth_stamp_s,
                                capture_stamp_s=pair.depth_stamp_s,
                                stamp_skew_s=pair.stamp_skew_s,
                                arrival_skew_s=pair.arrival_skew_s,
                                arrival_age_s=pair.arrival_age_s,
                                matched_pair_count=pair.matched_pair_count,
                                barrier_monotonic_s=barrier_monotonic_s,
                                rgb_receipt_ros_s=pair.rgb.receipt_ros_s,
                                depth_receipt_ros_s=pair.depth.receipt_ros_s,
                            )
                    else:
                        last_reason = (
                            "waiting for fresh synchronized RGB/depth pairs after the "
                            "post-settle barrier"
                        )
                else:
                    last_reason = "camera info has not arrived"
                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0.0:
                    raise RuntimeError(
                        f"timed out after {self._sensor_sync_timeout_s:.1f}s waiting for "
                        f"a synchronized {arm_name} RGB-D bundle: {last_reason}"
                    )
                self._sensor_condition.wait(timeout=min(remaining_s, 0.05))

    def _save_pre_close_snapshot(
        self,
        *,
        target_name: str,
        arm_name: str,
        commanded_tcp_world: PoseStamped,
        inference_rgb: Image,
        inference_depth: Image,
        inference_sensor_bundle: SensorBundle,
    ) -> Path:
        pre_close_bundle, pre_close_extrinsics = self._capture_sensor_snapshot(
            arm_name,
            camera_frame=self._camera_frame_for_arm(arm_name),
            phase_name="pre_close_capture",
        )
        rgb = pre_close_bundle.rgb
        depth = pre_close_bundle.depth
        info = pre_close_bundle.info

        actual_tcp_world = self._current_tcp_pose_world(arm_name)
        world_from_camera = np.asarray(
            pre_close_extrinsics["world_from_camera"],
            dtype=np.float64,
        )
        camera_from_world = np.linalg.inv(world_from_camera)
        camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)

        def project_world_point(pose: PoseStamped) -> Optional[list[float]]:
            world_point = np.asarray(
                [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                    1.0,
                ],
                dtype=np.float64,
            )
            camera_point = camera_from_world @ world_point
            if not np.all(np.isfinite(camera_point[:3])) or camera_point[2] <= 1.0e-6:
                return None
            pixel_h = camera_matrix @ camera_point[:3]
            pixel = pixel_h[:2] / pixel_h[2]
            return [float(pixel[0]), float(pixel[1])]

        commanded_uv = project_world_point(commanded_tcp_world)
        actual_uv = project_world_point(actual_tcp_world)
        inference_rgb_array = _image_to_array(inference_rgb, 0)
        inference_depth_m = _depth_to_array_m(
            inference_depth,
            0,
            self._depth_uint16_scale,
            self._depth_max_m,
        )
        pre_close_rgb_array = _image_to_array(rgb, 0)
        pre_close_depth_m = _depth_to_array_m(
            depth,
            0,
            self._depth_uint16_scale,
            self._depth_max_m,
        )

        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        timestamp += f"_{int((time.time() % 1.0) * 1000.0):03d}"
        safe_target = "".join(
            character if character.isalnum() else "_" for character in target_name
        ).strip("_")
        output_dir = self._pre_close_snapshot_dir / (
            f"{timestamp}_{safe_target or 'target'}_{arm_name}"
        )
        output_dir.mkdir(parents=True, exist_ok=False)

        def save_rgb(path: Path, rgb_array: np.ndarray) -> None:
            bgr = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
            if not cv2.imwrite(str(path), bgr):
                raise RuntimeError(f"failed to write {path}")

        save_rgb(output_dir / "inference_rgb.png", inference_rgb_array)
        save_rgb(output_dir / "pre_close_rgb.png", pre_close_rgb_array)
        np.save(output_dir / "inference_depth_m.npy", inference_depth_m)
        np.save(output_dir / "pre_close_depth_m.npy", pre_close_depth_m)

        pre_close_overlay = cv2.cvtColor(pre_close_rgb_array, cv2.COLOR_RGB2BGR)
        for label, pixel, color in (
            ("commanded TCP", commanded_uv, (0, 0, 255)),
            ("actual TCP", actual_uv, (0, 255, 0)),
        ):
            if pixel is None:
                continue
            u, v = int(round(pixel[0])), int(round(pixel[1]))
            if 0 <= u < pre_close_overlay.shape[1] and 0 <= v < pre_close_overlay.shape[0]:
                cv2.drawMarker(
                    pre_close_overlay,
                    (u, v),
                    color,
                    markerType=cv2.MARKER_CROSS,
                    markerSize=24,
                    thickness=2,
                )
                cv2.putText(
                    pre_close_overlay,
                    label,
                    (max(0, u - 70), max(18, v - 14)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                    cv2.LINE_AA,
                )
        if not cv2.imwrite(str(output_dir / "pre_close_overlay.png"), pre_close_overlay):
            raise RuntimeError("failed to write pre_close_overlay.png")

        translation_error = np.asarray(
            [
                actual_tcp_world.pose.position.x - commanded_tcp_world.pose.position.x,
                actual_tcp_world.pose.position.y - commanded_tcp_world.pose.position.y,
                actual_tcp_world.pose.position.z - commanded_tcp_world.pose.position.z,
            ],
            dtype=np.float64,
        )
        metadata = {
            "captured_at_local": timestamp,
            "target": target_name,
            "arm": arm_name,
            "camera_frame": self._camera_frame_for_arm(arm_name),
            "commanded_tcp_world": self._pose_to_dict(commanded_tcp_world),
            "actual_tcp_world": self._pose_to_dict(actual_tcp_world),
            "actual_minus_commanded_translation_m": translation_error.tolist(),
            "translation_error_norm_m": float(np.linalg.norm(translation_error)),
            "commanded_tcp_pixel_uv": commanded_uv,
            "actual_tcp_pixel_uv": actual_uv,
            "camera_intrinsics_3x3": camera_matrix.tolist(),
            "world_from_camera_4x4": world_from_camera.tolist(),
            "inference_capture_timing": self._sensor_bundle_diagnostics(
                inference_sensor_bundle
            ),
            "pre_close_capture_timing": self._sensor_bundle_diagnostics(
                pre_close_bundle
            ),
            "files": {
                "inference_rgb": "inference_rgb.png",
                "inference_depth_m": "inference_depth_m.npy",
                "pre_close_rgb": "pre_close_rgb.png",
                "pre_close_depth_m": "pre_close_depth_m.npy",
                "pre_close_overlay": "pre_close_overlay.png",
            },
        }
        (output_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return output_dir

    async def _request_remote_grasp(
        self,
        target_name: str,
        arm_name: str,
        camera_frame: str,
        rgb: Image,
        depth: Image,
        info: CameraInfo,
        capture_extrinsics: Dict[str, Any],
        sensor_bundle: SensorBundle,
    ) -> RemoteGrasp:
        phase_started = time.monotonic()
        if self._is_cancel_requested():
            raise RuntimeError("execution canceled before remote inference")
        payload = self._build_inference_payload(
            target_name,
            arm_name,
            camera_frame,
            rgb,
            depth,
            info,
            capture_extrinsics,
            sensor_bundle,
        )
        self.get_logger().info(
            "[TIMING] state_machine phase=build_inference_payload "
            f"target={target_name!r} arm={arm_name} "
            f"elapsed_ms={(time.monotonic() - phase_started) * 1000.0:.1f}"
        )
        timeout = aiohttp.ClientTimeout(total=self._inference_timeout_s)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            phase_started = time.monotonic()
            async with session.ws_connect(self._inference_request_url) as websocket:
                self.get_logger().info(
                    "[TIMING] state_machine phase=inference_ws_connect "
                    f"elapsed_ms={(time.monotonic() - phase_started) * 1000.0:.1f}"
                )
                try:
                    first_message = await websocket.receive(timeout=0.5)
                except asyncio.TimeoutError:
                    first_message = None
                if first_message is not None:
                    if first_message.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.CLOSED):
                        raise RuntimeError("inference websocket closed before request")
                    if first_message.type == aiohttp.WSMsgType.ERROR:
                        raise RuntimeError(f"inference websocket error: {websocket.exception()}")

                phase_started = time.monotonic()
                await websocket.send_bytes(_pack_msgpack(payload))
                deadline = time.monotonic() + self._inference_timeout_s
                while True:
                    if self._is_cancel_requested():
                        raise RuntimeError("execution canceled during remote inference")
                    remaining_s = deadline - time.monotonic()
                    if remaining_s <= 0.0:
                        raise asyncio.TimeoutError("remote inference timed out")
                    try:
                        message = await websocket.receive(timeout=min(0.1, remaining_s))
                        break
                    except asyncio.TimeoutError:
                        continue
                self.get_logger().info(
                    "[TIMING] state_machine phase=inference_ws_roundtrip "
                    f"elapsed_ms={(time.monotonic() - phase_started) * 1000.0:.1f}"
                )
                if message.type == aiohttp.WSMsgType.BINARY:
                    response = _unpack_msgpack(message.data)
                elif message.type == aiohttp.WSMsgType.TEXT:
                    response = json.loads(message.data)
                else:
                    raise RuntimeError(f"unexpected websocket message type: {message.type}")
        phase_started = time.monotonic()
        remote = self._parse_remote_response(response, arm_name, camera_frame)
        self.get_logger().info(
            "[TIMING] state_machine phase=parse_remote_response "
            f"elapsed_ms={(time.monotonic() - phase_started) * 1000.0:.1f}"
        )
        return remote

    def _build_inference_payload(
        self,
        target_name: str,
        arm_name: str,
        camera_frame: str,
        rgb: Image,
        depth: Image,
        info: CameraInfo,
        capture_extrinsics: Dict[str, Any],
        sensor_bundle: SensorBundle,
    ) -> Dict[str, Any]:
        rgb_array = _image_to_array(rgb, self._inference_image_size)
        depth_array = _depth_to_array_m(
            depth,
            self._inference_image_size,
            self._depth_uint16_scale,
            self._depth_max_m,
        )
        camera_info = {
            "width": int(info.width),
            "height": int(info.height),
            "k": [float(v) for v in info.k],
            "d": [float(v) for v in info.d],
            "distortion_model": str(info.distortion_model),
            "frame_id": camera_frame,
        }
        current_tcp_world = None
        try:
            # The 121 protocol defines its active TCP at the same 59.3 mm
            # /<arm>/gripper_end_pos control point used by its returned grasp.
            # Send that pose in <arm>_base_link from the same capture snapshot.
            extrinsics = capture_extrinsics
            current_tcp_pose_base = _pose_from_matrix(
                extrinsics["base_frame"],
                extrinsics["base_from_handeye_parent"],
            )
            current_tcp_pose_world = _pose_from_matrix(
                self._world_frame,
                extrinsics["world_from_handeye_parent"],
            )
            current_tcp_world = self._pose_to_dict(current_tcp_pose_world)
            state_values = self._make_inference_state_vector(current_tcp_pose_base)
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(f"state payload unavailable: {exc}") from exc

        # Keep the full local snapshot for execution.  The capture-time
        # world-from-camera transform is also sent explicitly so 121 can save
        # its local grasp/cloud artifact in the same world frame; it is not
        # part of the response consumed by this state machine.
        dynamic_extrinsics = {
            key: extrinsics[key]
            for key in (
                "world_frame",
                "link6_frame",
                "camera_frame",
                "base_from_camera",
                "tcp_from_camera",
            )
        }

        payload = {
            "prompt": target_name,
            "target": target_name,
            "action_format": "grasp_goal",
            "position_units": "m",
            "quaternion_order": "xyzw",
            "depth_units": "meters",
            "candidate_count": self._inference_candidate_count,
            "arm_name": arm_name,
            "camera": arm_name,
            "side": arm_name,
            "camera_frame": camera_frame,
            "frame_id": camera_frame,
            "observation/image": rgb_array,
            "observation/state": state_values,
            "observation/state_layout": "left_joints7,right_joints7,active_end_pose7",
            f"observation/T_{arm_name}_base_from_{camera_frame}": extrinsics["base_from_camera"],
            f"observation/T_world_from_{camera_frame}": extrinsics["world_from_camera"],
            f"observation/T_{arm_name}_tcp_from_{camera_frame}": extrinsics["tcp_from_camera"],
            f"T_{arm_name}_base_from_{self._camera_frame_alias_for_121(arm_name)}": extrinsics["base_from_camera"],
            f"T_{arm_name}_tcp_from_{self._camera_frame_alias_for_121(arm_name)}": extrinsics["tcp_from_camera"],
            "rgb_image": self._image_payload(rgb, resize_to=self._inference_image_size),
            "depth_image": self._image_payload(depth, resize_to=self._inference_image_size),
            "camera_info": camera_info,
            "current_tcp_pose_world": current_tcp_world,
            "dynamic_extrinsics": dynamic_extrinsics,
            "capture_timing": self._sensor_bundle_diagnostics(sensor_bundle),
        }
        wrist_prefix = f"observation/{arm_name}_wrist"
        payload[f"{wrist_prefix}_image"] = rgb_array
        payload[f"{wrist_prefix}_depth"] = depth_array
        payload[f"{wrist_prefix}_camera_frame_id"] = camera_frame
        payload[f"{wrist_prefix}_intrinsics"] = self._scaled_intrinsics(info)
        payload[f"observation/{arm_name}_tcp_pose_xyzw"] = state_values[-7:]

        alias = self._camera_frame_alias_for_121(arm_name)
        payload[f"observation/T_{arm_name}_base_from_{alias}"] = extrinsics["base_from_camera"]
        payload[f"observation/T_{arm_name}_tcp_from_{alias}"] = extrinsics["tcp_from_camera"]
        return payload

    @staticmethod
    def _sensor_bundle_diagnostics(sensor_bundle: SensorBundle) -> Dict[str, Any]:
        return {
            "rgb_stamp_s": sensor_bundle.rgb_stamp_s,
            "depth_stamp_s": sensor_bundle.depth_stamp_s,
            "canonical_depth_stamp_s": sensor_bundle.capture_stamp_s,
            "rgb_depth_stamp_skew_s": sensor_bundle.stamp_skew_s,
            "rgb_depth_arrival_skew_s": sensor_bundle.arrival_skew_s,
            "arrival_age_s": sensor_bundle.arrival_age_s,
            "barrier_to_rgb_arrival_s": (
                sensor_bundle.rgb_arrival_monotonic_s
                - sensor_bundle.barrier_monotonic_s
            ),
            "barrier_to_depth_arrival_s": (
                sensor_bundle.depth_arrival_monotonic_s
                - sensor_bundle.barrier_monotonic_s
            ),
            "rgb_receipt_ros_s": sensor_bundle.rgb_receipt_ros_s,
            "depth_receipt_ros_s": sensor_bundle.depth_receipt_ros_s,
            "rgb_receipt_minus_header_s": (
                sensor_bundle.rgb_receipt_ros_s - sensor_bundle.rgb_stamp_s
            ),
            "depth_receipt_minus_header_s": (
                sensor_bundle.depth_receipt_ros_s - sensor_bundle.depth_stamp_s
            ),
            "rgb_sequence": sensor_bundle.rgb_sequence,
            "depth_sequence": sensor_bundle.depth_sequence,
            "matched_pair_count": sensor_bundle.matched_pair_count,
            "tf_before_stamp_s": sensor_bundle.tf_before_stamp_s,
            "tf_after_stamp_s": sensor_bundle.tf_after_stamp_s,
            "tf_before_age_s": sensor_bundle.tf_before_age_s,
            "tf_after_age_s": sensor_bundle.tf_after_age_s,
            "tf_translation_delta_m": sensor_bundle.tf_translation_delta_m,
            "tf_rotation_delta_deg": sensor_bundle.tf_rotation_delta_deg,
            # The device header is deliberately diagnostic-only.  Dynamic
            # extrinsics use a latest TF captured while the stationary gate is
            # satisfied because RealSense global-time is not host-clock stable.
            "tf_mode": "latest_while_stationary",
        }

    def _parse_remote_response(
        self,
        response: Any,
        fallback_arm_name: str,
        fallback_camera_frame: str,
    ) -> RemoteGrasp:
        if not isinstance(response, dict):
            raise ValueError("121 response must be a JSON/msgpack object")
        if response.get("error"):
            raise ValueError(f"121 error: {response.get('error')}")
        candidate = _first_candidate(response)
        candidate_is_response = candidate is response
        pose_values = _mapping_value(candidate, "grasp_tcp_pose_xyzw")
        pose_source = "candidate.grasp_tcp_pose_xyzw"
        if pose_values is None:
            pose_values = _mapping_value(response, "grasp_tcp_pose_xyzw")
            pose_source = "response.grasp_tcp_pose_xyzw"
        if pose_values is None:
            raise ValueError(f"missing grasp_tcp_pose_xyzw in 121 response keys={list(response.keys())}")
        response_actions_frame_id = _mapping_value(response, "actions_frame_id")
        response_actions_space = _mapping_value(response, "actions_coordinate_space")
        response_grasp_goal_frame_id = _mapping_value(response, "grasp_goal_frame_id")
        frame_id = ""
        frame_id_source = ""

        frame_keys = (
            "grasp_tcp_pose_frame_id",
            "tcp_pose_frame_id",
            "pose_frame_id",
            "frame_id",
        )
        if pose_source.startswith("candidate.") and not candidate_is_response:
            for key in frame_keys:
                value = _mapping_value(candidate, key)
                if value:
                    frame_id = str(value).strip()
                    frame_id_source = f"candidate.{key}"
                    break
            if not frame_id and response_grasp_goal_frame_id:
                frame_id = str(response_grasp_goal_frame_id).strip()
                frame_id_source = "response.grasp_goal_frame_id"
            if (
                not frame_id
                and not response_actions_frame_id
                and not response_actions_space
                and _mapping_value(response, "frame_id")
            ):
                frame_id = str(_mapping_value(response, "frame_id")).strip()
                frame_id_source = "response.frame_id_legacy"
        else:
            for key in frame_keys:
                value = _mapping_value(response, key)
                if value:
                    frame_id = str(value).strip()
                    frame_id_source = f"response.{key}"
                    break

        if not frame_id:
            frame_id = fallback_camera_frame
            frame_id_source = "fallback_camera_frame"
        response_keys = list(response.keys())[:12]
        candidate_keys = list(candidate.keys())[:12] if isinstance(candidate, dict) else []
        self.get_logger().info(
            "121 grasp response frame selection: "
            f"pose_source={pose_source}, frame_id={frame_id!r} source={frame_id_source}, "
            f"actions_frame_id={response_actions_frame_id!r}, "
            f"actions_coordinate_space={response_actions_space!r}, "
            f"grasp_goal_frame_id={response_grasp_goal_frame_id!r}, "
            f"response_keys={response_keys}, candidate_keys={candidate_keys}"
        )
        arm_name = str(
            _mapping_value(candidate, "side")
            or _mapping_value(candidate, "arm_name")
            or _mapping_value(response, "side")
            or fallback_arm_name
        ).strip().lower()
        if "left" in frame_id:
            arm_name = "left"
        elif "right" in frame_id:
            arm_name = "right"
        width = (
            _mapping_value(candidate, "gripper_width_m")
            or _mapping_value(candidate, "predicted_width_m")
            or _mapping_value(response, "gripper_width_m")
            or self._default_gripper_width_m
        )
        return RemoteGrasp(
            arm_name=arm_name,
            tcp_pose=_pose_from_xyzw(frame_id, pose_values),
            gripper_width_m=float(width),
        )

    def _call_joint_plan(
        self,
        *,
        arm_name: str,
        scene_id: int,
        joint_positions: Tuple[float, ...],
        keep_grasp_ellipsoid: bool,
        gripper_command: bool = False,
        gripper_opening_m: float = 0.0,
    ) -> Tuple[bool, str]:
        if not self._joint_plan_client.wait_for_service(
            timeout_sec=self._service_timeout_s
        ):
            return False, f"joint planning service unavailable: {self._joint_plan_service_name}"
        request = PlanToJoints.Request()
        request.arm_name = arm_name
        request.scene_id = int(scene_id)
        request.joint_positions = list(joint_positions)
        request.gripper_command = bool(gripper_command)
        request.gripper_opening_m = float(gripper_opening_m)
        request.keep_grasp_ellipsoid = bool(keep_grasp_ellipsoid)

        done = threading.Event()
        future = self._joint_plan_client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + self._service_timeout_s
        while not done.wait(timeout=0.05):
            if self._is_cancel_requested():
                future.cancel()
                return False, "execution canceled"
            if time.monotonic() >= deadline:
                future.cancel()
                return False, "timed out waiting for joint planning service"
        if self._is_cancel_requested():
            return False, "execution canceled"
        result = future.result()
        if result is None:
            return False, "joint planning service returned no response"
        return bool(result.success), str(result.message)

    def _call_bridge_task(
        self,
        *,
        task_id: str,
        task_type: str,
        target_id: str,
        arm_name: str,
        scene_id: int,
        tcp_pose: PoseStamped,
        gripper_command: bool,
        gripper_opening_m: float,
    ) -> Tuple[bool, str]:
        if not self._bridge_client.wait_for_service(timeout_sec=self._service_timeout_s):
            return False, f"bridge service unavailable: {self._bridge_service_name}"
        request = ExecuteTask.Request()
        request.task_id = task_id
        request.task_type = task_type
        request.target_id = target_id
        request.arm_name = arm_name
        request.scene_id = int(scene_id)
        request.tcp_target_pose = tcp_pose
        request.gripper_command = bool(gripper_command)
        request.gripper_opening_m = float(gripper_opening_m)

        done = threading.Event()
        future = self._bridge_client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + self._service_timeout_s
        while not done.wait(timeout=0.05):
            if self._is_cancel_requested():
                future.cancel()
                return False, "execution canceled"
            if time.monotonic() >= deadline:
                future.cancel()
                return False, "timed out waiting for bridge"
        if self._is_cancel_requested():
            return False, "execution canceled"
        result = future.result()
        if result is None:
            return False, "bridge returned no response"
        return bool(result.success), str(result.message)

    def _scaled_intrinsics(self, camera_info: CameraInfo) -> np.ndarray:
        matrix = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3).copy()
        if self._inference_image_size <= 0:
            return matrix.astype(np.float32)
        scale_x = float(self._inference_image_size) / float(camera_info.width)
        scale_y = float(self._inference_image_size) / float(camera_info.height)
        matrix[0, 0] *= scale_x
        matrix[0, 2] *= scale_x
        matrix[1, 1] *= scale_y
        matrix[1, 2] *= scale_y
        return matrix.astype(np.float32)

    def _make_inference_state_vector(self, tcp_pose_base: PoseStamped) -> np.ndarray:
        with self._latest_lock:
            joint_state = self._latest_joint_state
        if joint_state is None:
            raise RuntimeError("joint_states_unavailable")

        indices = {name: index for index, name in enumerate(joint_state.name)}

        def arm_values(prefix: str) -> list[float]:
            values = []
            for index in range(1, 8):
                name = f"{prefix}_joint{index}"
                if name not in indices:
                    raise RuntimeError(f"joint_state_missing:{name}")
                values.append(float(joint_state.position[indices[name]]))
            return values

        pose = tcp_pose_base.pose
        active_pose = [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ]
        return np.asarray(
            [*arm_values("left"), *arm_values("right"), *active_pose],
            dtype=np.float32,
        )

    def _dynamic_extrinsics_for_121(
        self,
        arm_name: str,
        world_from_link6: Optional[np.ndarray] = None,
    ) -> Dict[str, Any]:
        if world_from_link6 is None:
            t_world_link6, _, _ = self._lookup_world_link6_matrix(arm_name)
        else:
            t_world_link6 = np.asarray(world_from_link6, dtype=np.float64)
        base_to_world_tf = self._tf_buffer.lookup_transform(
            f"{arm_name}_base_link",
            self._world_frame,
            Time(),
            timeout=Duration(seconds=self._tf_timeout_s),
        )
        t_base_world = _matrix_from_transform(base_to_world_tf)
        t_handeye_parent_camera = self._handeye_matrix_for_arm(arm_name)
        capture = compose_capture_frame_extrinsics(
            t_world_link6,
            t_base_world,
            t_handeye_parent_camera,
            self._link6_to_handeye_parent_m,
            self._tcp_to_link6_m,
        )
        return {
            "world_frame": self._world_frame,
            "base_frame": f"{arm_name}_base_link",
            "link6_frame": f"{arm_name}_link6",
            "camera_frame": self._camera_frame_for_arm(arm_name),
            # Keep both frames explicit.  Only base_from_camera is sent in the
            # 121 T_<arm>_base_from_camera protocol fields; world_from_camera
            # is retained locally for converting the returned camera pose.
            "world_from_handeye_parent": capture[
                "world_from_handeye_parent"
            ].astype(np.float32),
            "world_from_planning_tcp": capture[
                "world_from_planning_tcp"
            ].astype(np.float32),
            "world_from_camera": capture["world_from_camera"].astype(np.float32),
            "base_from_handeye_parent": capture[
                "base_from_handeye_parent"
            ].astype(np.float32),
            "base_from_planning_tcp": capture[
                "base_from_planning_tcp"
            ].astype(np.float32),
            "base_from_camera": capture["base_from_camera"].astype(np.float32),
            # 121 calls /<arm>/gripper_end_pos its TCP.  This is the raw
            # 59.3 mm hand-eye-parent<-camera matrix, not the planning-TCP one.
            "tcp_from_camera": t_handeye_parent_camera.astype(np.float32),
        }

    def _handeye_matrix_for_arm(self, arm_name: str) -> np.ndarray:
        return self._left_handeye_matrix if arm_name == "left" else self._right_handeye_matrix

    def _world_camera_from_link6(
        self,
        arm_name: str,
        t_world_link6: np.ndarray,
    ) -> np.ndarray:
        return compose_world_from_handeye_camera(
            t_world_link6,
            self._handeye_matrix_for_arm(arm_name),
            self._link6_to_handeye_parent_m,
        )

    def _current_link6_offset_pose_world(
        self,
        arm_name: str,
        link6_local_z_offset_m: float,
    ) -> PoseStamped:
        transform = self._tf_buffer.lookup_transform(
            self._world_frame,
            f"{arm_name}_link6",
            Time(),
            timeout=Duration(seconds=self._tf_timeout_s),
        )
        matrix = world_from_link6_offset(
            _matrix_from_transform(transform),
            link6_local_z_offset_m,
        )
        pose = _pose_from_matrix(self._world_frame, matrix)
        pose.header.stamp = self.get_clock().now().to_msg()
        return pose

    def _current_handeye_parent_pose_world(self, arm_name: str) -> PoseStamped:
        return self._current_link6_offset_pose_world(
            arm_name,
            self._link6_to_handeye_parent_m,
        )

    def _current_tcp_pose_world(self, arm_name: str) -> PoseStamped:
        # This is the planning TCP used only for bridge OPEN/LIFT/current-pose
        # targets.  The bridge still performs planning TCP -> link6.
        return self._current_link6_offset_pose_world(
            arm_name,
            self._tcp_to_link6_m,
        )

    def _camera_tcp_pose_to_world_pose_at_capture(
        self,
        arm_name: str,
        pose: PoseStamped,
        t_world_camera_at_capture: np.ndarray,
    ) -> PoseStamped:
        frame_id = (pose.header.frame_id or "").strip()
        if not frame_id:
            frame_id = self._camera_frame_for_arm(arm_name)
            pose = self._copy_pose(pose)
            pose.header.frame_id = frame_id
            self.get_logger().warn(
                "121 TCP pose frame_id is empty; assuming capture camera frame "
                f"{frame_id!r}"
            )

        if frame_id == self._world_frame:
            return self._copy_pose(pose)

        camera_frames = {
            self._camera_frame_for_arm(arm_name),
            self._camera_frame_alias_for_121(arm_name),
        }
        if frame_id not in camera_frames:
            self.get_logger().info(
                "121 TCP pose is not in the capture camera frame; "
                f"frame_id={frame_id!r}, transforming through TF to {self._world_frame!r}"
            )
            return self._pose_to_world(pose)

        t_camera_tcp = _matrix_from_pose(pose)
        t_world_tcp = np.asarray(t_world_camera_at_capture, dtype=np.float64) @ t_camera_tcp
        output = _pose_from_matrix(self._world_frame, t_world_tcp)
        output.header.stamp = self.get_clock().now().to_msg()
        return output

    def _pose_to_world(self, pose: PoseStamped) -> PoseStamped:
        frame_id = pose.header.frame_id or self._world_frame
        if frame_id == self._world_frame:
            output = self._copy_pose(pose)
            output.header.frame_id = self._world_frame
            return output
        transform = self._tf_buffer.lookup_transform(
            self._world_frame,
            frame_id,
            Time(),
            timeout=Duration(seconds=self._tf_timeout_s),
        )
        output = _pose_from_matrix(
            self._world_frame,
            _matrix_from_transform(transform) @ _matrix_from_pose(pose),
        )
        output.header.stamp = self.get_clock().now().to_msg()
        return output

    def _ordered_roll_180_grasp_poses(
        self,
        arm_name: str,
        grasp_pose: PoseStamped,
    ) -> list[PoseStamped]:
        original = self._copy_pose(grasp_pose)
        t_world_tcp = _matrix_from_pose(grasp_pose)
        t_world_tcp_alt = t_world_tcp.copy()
        t_world_tcp_alt[:3, :3] = t_world_tcp[:3, :3] @ np.diag([-1.0, -1.0, 1.0])
        alt = _pose_from_matrix(self._world_frame, t_world_tcp_alt)
        alt.header.stamp = self.get_clock().now().to_msg()

        current_tcp = self._current_tcp_pose_world(arm_name)
        current_q = (
            current_tcp.pose.orientation.x,
            current_tcp.pose.orientation.y,
            current_tcp.pose.orientation.z,
            current_tcp.pose.orientation.w,
        )

        def angular_error(candidate: PoseStamped) -> float:
            return _quaternion_angular_distance(
                current_q,
                (
                    candidate.pose.orientation.x,
                    candidate.pose.orientation.y,
                    candidate.pose.orientation.z,
                    candidate.pose.orientation.w,
                ),
            )

        candidates = [("remote", original), ("roll_180", alt)]
        candidates.sort(key=lambda item: angular_error(item[1]))
        self.get_logger().info(
            "grasp orientation branches: "
            + ", ".join(
                f"{name}={math.degrees(angular_error(candidate)):.2f}deg"
                for name, candidate in candidates
            )
        )
        return [candidate for _, candidate in candidates]

    def _level_grasp_opening_axis(self, grasp_pose: PoseStamped) -> PoseStamped:
        t_world_tcp = _matrix_from_pose(grasp_pose)
        rotation = t_world_tcp[:3, :3]
        original_y = rotation[:, 1]
        approach_z = rotation[:, 2]
        approach_norm = float(np.linalg.norm(approach_z))
        if approach_norm < 1e-9:
            raise RuntimeError("cannot level grasp opening axis: approach axis is zero")
        approach_z = approach_z / approach_norm

        world_up = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
        leveled_y = np.cross(world_up, approach_z)
        if np.linalg.norm(leveled_y) < 1e-6:
            leveled_y = original_y.copy()
            leveled_y[2] = 0.0
        if np.linalg.norm(leveled_y) < 1e-6:
            leveled_y = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
        leveled_y = leveled_y / np.linalg.norm(leveled_y)
        if float(np.dot(leveled_y, original_y)) < 0.0:
            leveled_y = -leveled_y

        leveled_x = np.cross(leveled_y, approach_z)
        leveled_x = leveled_x / np.linalg.norm(leveled_x)
        leveled_y = np.cross(approach_z, leveled_x)
        leveled_y = leveled_y / np.linalg.norm(leveled_y)

        leveled = t_world_tcp.copy()
        leveled[:3, :3] = np.column_stack((leveled_x, leveled_y, approach_z))
        pose = _pose_from_matrix(grasp_pose.header.frame_id, leveled)
        pose.header.stamp = grasp_pose.header.stamp
        self.get_logger().info(_pose_log_text("level_opening_grasp_tcp", pose))
        return pose

    def _grasp_approach_axis_world(self, grasp_pose: PoseStamped) -> np.ndarray:
        t_world_tcp = _matrix_from_pose(grasp_pose)
        axis = np.asarray(t_world_tcp[:3, 2], dtype=np.float64)
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9 or not math.isfinite(norm):
            raise RuntimeError("grasp approach axis is zero or non-finite")
        axis = axis / norm
        self.get_logger().info(
            "grasp_approach_axis_world="
            f"({axis[0]:.4f}, {axis[1]:.4f}, {axis[2]:.4f})"
        )
        return axis

    def _retracted_pose_from_axis(
        self,
        grasp_pose: PoseStamped,
        approach_axis_world: np.ndarray,
        offset_m: float,
        *,
        log_label: str,
        min_z_m: float,
    ) -> PoseStamped:
        output = self._copy_pose(grasp_pose)
        axis = np.asarray(approach_axis_world, dtype=np.float64)
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9 or not math.isfinite(norm):
            raise RuntimeError("grasp approach axis is zero or non-finite")
        axis = axis / norm

        output.pose.position.x -= float(axis[0] * offset_m)
        output.pose.position.y -= float(axis[1] * offset_m)
        output.pose.position.z -= float(axis[2] * offset_m)
        min_z = float(min_z_m)
        min_z_applied = False
        if min_z > 0.0 and output.pose.position.z < min_z:
            output.pose.position.z = min_z
            min_z_applied = True

        p = output.pose.position
        self.get_logger().info(
            f"{log_label}: "
            f"offset_m={offset_m:.4f}, "
            f"axis=({axis[0]:.4f}, {axis[1]:.4f}, {axis[2]:.4f}), "
            f"min_z_applied={min_z_applied}, "
            f"tcp_world=({p.x:.4f}, {p.y:.4f}, {p.z:.4f})"
        )
        return output

    def _offset_camera_grasp_z_in_world(
        self,
        arm_name: str,
        pose: PoseStamped,
        z_offset_m: float,
    ) -> PoseStamped:
        t_world_link6 = _matrix_from_transform(
            self._tf_buffer.lookup_transform(
                self._world_frame,
                f"{arm_name}_link6",
                Time(),
                timeout=Duration(seconds=self._tf_timeout_s),
            )
        )
        t_world_camera = self._world_camera_from_link6(arm_name, t_world_link6)
        delta_camera = t_world_camera[:3, :3].T @ np.asarray([0.0, 0.0, z_offset_m], dtype=np.float64)
        output = self._copy_pose(pose)
        output.pose.position.x += float(delta_camera[0])
        output.pose.position.y += float(delta_camera[1])
        output.pose.position.z += float(delta_camera[2])
        return output

    def _camera_tcp_pose_to_world_pose(
        self,
        arm_name: str,
        pose: PoseStamped,
    ) -> PoseStamped:
        if pose.header.frame_id == self._world_frame:
            return self._copy_pose(pose)
        t_world_link6 = _matrix_from_transform(
            self._tf_buffer.lookup_transform(
                self._world_frame,
                f"{arm_name}_link6",
                Time(),
                timeout=Duration(seconds=self._tf_timeout_s),
            )
        )
        t_world_camera = self._world_camera_from_link6(arm_name, t_world_link6)
        p_camera = np.asarray(
            [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                1.0,
            ],
            dtype=np.float64,
        )
        p_world = t_world_camera @ p_camera
        output = self._copy_pose(pose)
        output.header.frame_id = self._world_frame
        output.pose.position.x = float(p_world[0])
        output.pose.position.y = float(p_world[1])
        output.pose.position.z = float(p_world[2])
        return output

    def _copy_pose(self, pose: PoseStamped) -> PoseStamped:
        output = PoseStamped()
        output.header = pose.header
        output.pose.position.x = pose.pose.position.x
        output.pose.position.y = pose.pose.position.y
        output.pose.position.z = pose.pose.position.z
        output.pose.orientation.x = pose.pose.orientation.x
        output.pose.orientation.y = pose.pose.orientation.y
        output.pose.orientation.z = pose.pose.orientation.z
        output.pose.orientation.w = pose.pose.orientation.w
        return output

    def _image_payload(self, msg: Image, *, resize_to: int) -> Dict[str, Any]:
        array = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        encoded_image = None
        if msg.encoding in ("rgb8", "bgr8") and msg.height and msg.width:
            channels = 3
            image = array.reshape((msg.height, msg.width, channels))
            if msg.encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if resize_to > 0:
                image = cv2.resize(image, (resize_to, resize_to), interpolation=cv2.INTER_AREA)
            ok, encoded = cv2.imencode(".jpg", image)
            if ok:
                encoded_image = base64.b64encode(encoded.tobytes()).decode("ascii")
        if encoded_image is None:
            encoded_image = base64.b64encode(bytes(msg.data)).decode("ascii")
        return {
            "height": int(msg.height),
            "width": int(msg.width),
            "encoding": str(msg.encoding),
            "is_bigendian": int(msg.is_bigendian),
            "step": int(msg.step),
            "data": encoded_image,
            "data_encoding": "base64",
            "stamp_s": message_stamp_seconds(msg),
            "frame_id": str(msg.header.frame_id),
        }

    def _camera_frame_for_arm(self, arm_name: str) -> str:
        return self._left_camera_frame if arm_name == "left" else self._right_camera_frame

    def _camera_frame_alias_for_121(self, arm_name: str) -> str:
        if arm_name == "left":
            return "cam_left_color_optical"
        return "cam_right_color_optical"

    def _pose_to_dict(self, pose: PoseStamped) -> Dict[str, Any]:
        return {
            "frame_id": pose.header.frame_id,
            "position": {
                "x": pose.pose.position.x,
                "y": pose.pose.position.y,
                "z": pose.pose.position.z,
            },
            "orientation": {
                "x": pose.pose.orientation.x,
                "y": pose.pose.orientation.y,
                "z": pose.pose.orientation.z,
                "w": pose.pose.orientation.w,
            },
        }

    def _rotate_vector(
        self,
        quaternion: Quaternion,
        vector: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        x, y, z, w = _normalize_quaternion(quaternion)
        vx, vy, vz = vector
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GraspBridgeStateMachine()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
