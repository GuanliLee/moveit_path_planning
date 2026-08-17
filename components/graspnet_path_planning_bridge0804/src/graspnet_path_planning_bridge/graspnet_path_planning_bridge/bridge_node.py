from __future__ import annotations

import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import rclpy
from tf2_ros import Buffer, TransformListener

from graspnet_bridge_interfaces.msg import DetectionResult
from graspnet_bridge_interfaces.srv import ExecuteTask, GetDetectionResult

from .coordinate_converter import CoordinateConverter
from .detection_cache import DetectionCache
from .plan_to_pose_client import PlanToPoseClient


SUPPORTED_TASK_TYPES = frozenset(
    {"MOVE", "GRASP", "LIFT", "CARRY", "PLACE", "RETURN_HOME"}
)
GRASP_ELLIPSOID_TASK_TYPES = frozenset({"GRASP", "LIFT", "CARRY", "PLACE"})


def _keeps_grasp_ellipsoid(task_type: str) -> bool:
    return task_type in GRASP_ELLIPSOID_TASK_TYPES


class GraspNetPathPlanningBridge(Node):
    def __init__(self) -> None:
        super().__init__("graspnet_path_planning_bridge")

        self._declare_parameters()
        self._callback_group = ReentrantCallbackGroup()
        self._detection_cache = DetectionCache()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._converter = CoordinateConverter(self, self._tf_buffer)
        self._planner = PlanToPoseClient(self, self._callback_group)

        detection_topic = str(self.get_parameter("detection_topic").value)
        self.create_subscription(
            DetectionResult,
            detection_topic,
            self._handle_detection,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(
            GetDetectionResult,
            "/grasp_bridge/get_detection_result",
            self._handle_get_detection,
            callback_group=self._callback_group,
        )
        self.create_service(
            ExecuteTask,
            "/grasp_bridge/execute_task",
            self._handle_execute_task,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "grasp bridge ready: execute=/grasp_bridge/execute_task, "
            f"detection={detection_topic}, plan={self.get_parameter('plan_service').value}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("plan_service", "/plan_to_pose")
        self.declare_parameter("default_scene_id", 1)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("detection_topic", "/grasp_bridge/detection_result")
        self.declare_parameter("plan_timeout_s", 60.0)
        self.declare_parameter("tf_timeout_s", 1.0)

        self.declare_parameter("left_link6_frame", "left_link6")
        self.declare_parameter("right_link6_frame", "right_link6")
        self.declare_parameter("left_camera_frame", "cam_left_color_optical_frame")
        self.declare_parameter("right_camera_frame", "cam_right_color_optical_frame")

        self.declare_parameter(
            "left_handeye_file",
            "/home/ligl/path_planning_service0807/calibration/extrinsics/cam_left_handeye.json",
        )
        self.declare_parameter(
            "right_handeye_file",
            "/home/ligl/path_planning_service0807/calibration/extrinsics/cam_right_handeye.json",
        )

        # The hand-eye JSON parent is /<arm>/gripper_end_pos, not link6 and
        # not the separate 135.8 mm MoveIt planning TCP.
        self.declare_parameter("link6_to_handeye_parent_m", 0.0593)

        self.declare_parameter("left_link6_to_tcp_xyz", [0.0, 0.0, 0.1358])
        self.declare_parameter("left_link6_to_tcp_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("right_link6_to_tcp_xyz", [0.0, 0.0, 0.1358])
        self.declare_parameter("right_link6_to_tcp_xyzw", [0.0, 0.0, 0.0, 1.0])

    def _handle_detection(self, msg: DetectionResult) -> None:
        self._detection_cache.update(msg)
        self.get_logger().debug(
            f"cached detection target={msg.target_name!r} frame={msg.header.frame_id!r}"
        )

    def _handle_get_detection(
        self,
        request: GetDetectionResult.Request,
        response: GetDetectionResult.Response,
    ) -> GetDetectionResult.Response:
        result = self._detection_cache.get(request.target_name)
        if result is None:
            response.success = False
            response.message = f"no detection cached for target {request.target_name!r}"
            return response

        response.success = True
        response.message = "ok"
        response.result = result
        return response

    def _handle_execute_task(
        self,
        request: ExecuteTask.Request,
        response: ExecuteTask.Response,
    ) -> ExecuteTask.Response:
        task_started = time.monotonic()
        task_type = request.task_type.strip().upper()
        arm = request.arm_name.strip().lower()
        self.get_logger().info(
            "[TIMING] bridge phase=execute_task_start "
            f"task_id={request.task_id!r} type={task_type} "
            f"target={request.target_id!r} arm={arm}"
        )
        if task_type not in SUPPORTED_TASK_TYPES:
            response.success = False
            response.message = (
                "task_type must be MOVE, GRASP, LIFT, CARRY, PLACE, or RETURN_HOME, "
                f"got {request.task_type!r}"
            )
            return response

        conversion_started = time.monotonic()
        try:
            target_link6 = self._converter.tcp_pose_to_world_link6(
                arm,
                request.tcp_target_pose,
            )
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"coordinate conversion failed: {exc}"
            return response
        self.get_logger().info(
            "[TIMING] bridge phase=coordinate_conversion "
            f"task_id={request.task_id!r} type={task_type} "
            f"elapsed_ms={(time.monotonic() - conversion_started) * 1000.0:.1f}"
        )

        p = target_link6.pose.position
        q = target_link6.pose.orientation
        scene_id = int(request.scene_id)
        if scene_id <= 0:
            scene_id = int(self.get_parameter("default_scene_id").value)
        self.get_logger().info(
            "execute task "
            f"id={request.task_id!r} type={task_type} target={request.target_id!r} "
            f"arm={arm} frame={request.tcp_target_pose.header.frame_id!r} "
            f"gripper={request.gripper_command}:{request.gripper_opening_m:.4f}, "
            "target_link6_world="
            f"xyz=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}), "
            f"xyzw=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})"
        )
        planner_started = time.monotonic()
        success, message = self._planner.call(
            arm,
            scene_id,
            target_link6,
            request.gripper_command,
            request.gripper_opening_m,
            keep_grasp_ellipsoid=_keeps_grasp_ellipsoid(task_type),
        )
        self.get_logger().info(
            "[TIMING] bridge phase=planner_call "
            f"task_id={request.task_id!r} type={task_type} "
            f"success={success} elapsed_ms={(time.monotonic() - planner_started) * 1000.0:.1f} "
            f"total_ms={(time.monotonic() - task_started) * 1000.0:.1f} "
            f"message={message!r}"
        )
        response.success = success
        response.message = message
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspNetPathPlanningBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
