from __future__ import annotations

from threading import Event
from typing import Tuple

from geometry_msgs.msg import PoseStamped
from path_planning_interfaces.srv import PlanToPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class PlanToPoseClient:
    def __init__(self, node: Node, callback_group: ReentrantCallbackGroup) -> None:
        self._node = node
        self._service_name = str(node.get_parameter("plan_service").value)
        self._timeout_s = float(node.get_parameter("plan_timeout_s").value)
        self._client = node.create_client(
            PlanToPose,
            self._service_name,
            callback_group=callback_group,
        )

    def call(
        self,
        arm_name: str,
        scene_id: int,
        target_pose: PoseStamped,
        gripper_command: bool,
        gripper_opening_m: float,
        keep_grasp_ellipsoid: bool = False,
    ) -> Tuple[bool, str]:
        if not self._client.wait_for_service(timeout_sec=self._timeout_s):
            return False, f"service unavailable: {self._service_name}"

        request = PlanToPose.Request()
        request.arm_name = arm_name
        request.scene_id = int(scene_id)
        request.target_pose = target_pose
        request.gripper_command = bool(gripper_command)
        request.gripper_opening_m = float(gripper_opening_m)
        if hasattr(request, "keep_grasp_ellipsoid"):
            request.keep_grasp_ellipsoid = bool(keep_grasp_ellipsoid)

        done = Event()
        future = self._client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self._timeout_s):
            return False, f"timeout waiting for {self._service_name}"

        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            return False, f"{self._service_name} call failed: {exc}"

        if response is None:
            return False, f"{self._service_name} returned no response"
        return bool(response.success), str(response.message)
