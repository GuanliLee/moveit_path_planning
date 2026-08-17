from __future__ import annotations

import json
from pathlib import Path
from math import sqrt
from typing import Dict, Iterable

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException

from .transform_math import RigidTransform


class CoordinateConverter:
    def __init__(self, node: Node, tf_buffer: Buffer) -> None:
        self._node = node
        self._tf_buffer = tf_buffer

        self.world_frame = self._param_string("world_frame")
        self.tf_timeout_s = float(node.get_parameter("tf_timeout_s").value)

        self._link6_frame = {
            "left": self._param_string("left_link6_frame"),
            "right": self._param_string("right_link6_frame"),
        }
        self._camera_frame = {
            "left": self._param_string("left_camera_frame"),
            "right": self._param_string("right_camera_frame"),
        }
        self._handeye_parent_to_camera = {
            "left": self._load_handeye(self._param_string("left_handeye_file")),
            "right": self._load_handeye(self._param_string("right_handeye_file")),
        }
        link6_to_handeye_parent_m = float(
            node.get_parameter("link6_to_handeye_parent_m").value
        )
        self._link6_to_handeye_parent = {
            arm: RigidTransform.from_xyz_xyzw(
                (0.0, 0.0, link6_to_handeye_parent_m),
                (0.0, 0.0, 0.0, 1.0),
            )
            for arm in ("left", "right")
        }
        self._link6_to_camera = {
            arm: self._link6_to_handeye_parent[arm]
            * self._handeye_parent_to_camera[arm]
            for arm in ("left", "right")
        }
        self._link6_to_tcp = {
            "left": RigidTransform.from_xyz_xyzw(
                self._param_list("left_link6_to_tcp_xyz"),
                self._param_list("left_link6_to_tcp_xyzw"),
            ),
            "right": RigidTransform.from_xyz_xyzw(
                self._param_list("right_link6_to_tcp_xyz"),
                self._param_list("right_link6_to_tcp_xyzw"),
            ),
        }

    def tcp_pose_to_world_link6(
        self,
        arm_name: str,
        tcp_pose: PoseStamped,
    ) -> PoseStamped:
        arm = self._normalize_arm(arm_name)
        source_frame = tcp_pose.header.frame_id.strip()
        if not source_frame:
            raise ValueError("tcp_target_pose.header.frame_id is empty")

        t_source_tcp = RigidTransform.from_pose(tcp_pose.pose)
        t_world_source = self._world_from_source_frame(arm, source_frame)
        t_world_tcp = t_world_source * t_source_tcp
        # The hand-eye-parent offset is used only to locate the camera.  A 121
        # grasp pose is already the physical/planning TCP regardless of whether
        # it arrives in camera, world, base, or another source frame.
        link6_to_target_control = self._link6_to_tcp[arm]
        target_control_name = "planning_tcp"
        t_world_link6 = t_world_tcp * link6_to_target_control.inverse()
        t_reconstructed_tcp = t_world_link6 * link6_to_target_control
        reconstruction_error_m = self._translation_error(t_world_tcp, t_reconstructed_tcp)
        if reconstruction_error_m >= 1e-6:
            raise RuntimeError(
                "TCP/link6 reconstruction self-check failed: "
                f"position_error={reconstruction_error_m:.9f} m"
            )

        output = PoseStamped()
        output.header.frame_id = self.world_frame
        output.header.stamp = self._node.get_clock().now().to_msg()
        output.pose = t_world_link6.to_pose()
        tcp_xyz = t_world_tcp.translation
        link6_xyz = t_world_link6.translation
        self._node.get_logger().info(
            "tcp_to_link6: "
            f"target_control={target_control_name}, "
            f"tcp_world=({tcp_xyz[0]:.6f}, {tcp_xyz[1]:.6f}, {tcp_xyz[2]:.6f}), "
            f"link6_world=({link6_xyz[0]:.6f}, {link6_xyz[1]:.6f}, {link6_xyz[2]:.6f}), "
            f"reconstruction_error_m={reconstruction_error_m:.9f}"
        )
        return output

    def _world_from_source_frame(self, arm: str, source_frame: str) -> RigidTransform:
        if source_frame == self.world_frame:
            return RigidTransform.identity()

        if source_frame == self._camera_frame[arm]:
            self._node.get_logger().warn(
                "tcp_target_pose is still in an eye-in-hand camera frame; "
                "using live TF now. For 121 GraspNet captures, prefer a world-frame "
                "planning-TCP pose computed directly from the camera transform "
                "at capture time."
            )
            t_world_link6 = self._lookup_transform(self.world_frame, self._link6_frame[arm])
            return t_world_link6 * self._link6_to_camera[arm]

        return self._lookup_transform(self.world_frame, source_frame)

    def _lookup_transform(self, target_frame: str, source_frame: str) -> RigidTransform:
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException as exc:
            raise RuntimeError(
                f"TF lookup failed: {target_frame} <- {source_frame}: {exc}"
            ) from exc
        return RigidTransform.from_transform_stamped(transform)

    def _load_handeye(self, path_value: str) -> RigidTransform:
        path = Path(path_value).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"hand-eye calibration file not found: {path}")
        with path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
        return RigidTransform.from_xyz_xyzw(
            data["translation_m"],
            data["orientation_xyzw"],
        )

    def _param_string(self, name: str) -> str:
        return str(self._node.get_parameter(name).value)

    def _param_list(self, name: str) -> Iterable[float]:
        value = self._node.get_parameter(name).value
        if not isinstance(value, (list, tuple)):
            raise ValueError(f"parameter {name} must be a list")
        return [float(item) for item in value]

    def _normalize_arm(self, arm_name: str) -> str:
        arm = arm_name.strip().lower()
        if arm not in ("left", "right"):
            raise ValueError(f"arm_name must be left or right, got {arm_name!r}")
        return arm

    def _translation_error(self, expected: RigidTransform, actual: RigidTransform) -> float:
        return sqrt(
            (actual.translation[0] - expected.translation[0]) ** 2
            + (actual.translation[1] - expected.translation[1]) ** 2
            + (actual.translation[2] - expected.translation[2]) ** 2
        )
