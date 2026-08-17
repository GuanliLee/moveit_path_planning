#!/usr/bin/env python3
"""Run an ACT policy with dual arms, mobile base, and lifting column."""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import pickle
import sys
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header

try:
    from lifting_msg_pkg.msg import LiftMotorMsg
    from lifting_msg_pkg.srv import LiftMotorSrv
except ImportError:
    try:
        from bt_task_msgs.msg import LiftMotorMsg
        from bt_task_msgs.srv import LiftMotorSrv
    except ImportError:
        LiftMotorMsg = None
        LiftMotorSrv = None


ACT_DIR = os.environ.get("ACT_DIR", "/home/agilex/aloha/act")
ACT_INFERENCE_PATH = os.path.join(ACT_DIR, "aloha_inference-ros2.py")
CAMERA_NAMES = ("front", "left", "right")
ARM_NAMES = ("left", "right")
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
ARM_STATE_DIM = 14
ROBOT_BASE_STATE_DIM = 7
ROBOT_BASE_ACTION_DIM = 4
POLICY_ACTION_DIM = 18


def load_act_inference_module():
    if not os.path.isfile(ACT_INFERENCE_PATH):
        raise RuntimeError(f"ACT inference script not found: {ACT_INFERENCE_PATH}")
    os.chdir(ACT_DIR)
    if ACT_DIR not in sys.path:
        sys.path.insert(0, ACT_DIR)
    spec = importlib.util.spec_from_file_location("aloha_inference_ros2_mobile", ACT_INFERENCE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot import ACT inference script: {ACT_INFERENCE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["aloha_inference_ros2_mobile"] = module
    spec.loader.exec_module(module)
    return module


ACT_MOD = load_act_inference_module()

import policy as act_policy_module  # noqa: E402


ORIGINAL_BUILD_ACT = act_policy_module.build_act


def install_mobile_act_builder() -> None:
    """Patch ACT construction so robot-base input and output dims can differ."""
    if getattr(act_policy_module, "_mobile_act_builder_installed", False):
        return

    def build_act_mobile(args):
        robot_base_state_dim = int(getattr(args, "robot_base_state_dim", args.robot_base_dim))
        robot_base_action_dim = int(getattr(args, "robot_base_action_dim", args.robot_base_dim))
        saved_robot_base_dim = args.robot_base_dim

        args.robot_base_dim = robot_base_state_dim
        try:
            model = ORIGINAL_BUILD_ACT(args)
        finally:
            args.robot_base_dim = saved_robot_base_dim

        output_state_dim = 0
        if args.use_arm_joint_state > 1:
            output_state_dim += args.arm_joint_state_dim * len(args.arm_joint_state_names)
        if args.use_arm_end_pose > 1:
            output_state_dim += args.arm_end_pose_dim * len(args.arm_end_pose_names)
        if args.use_robot_base > 1:
            output_state_dim += robot_base_action_dim

        if model.output_state_dim != output_state_dim:
            model.output_state_dim = output_state_dim
            if args.kl_weight != 0:
                model.encoder_action_proj = torch.nn.Linear(output_state_dim, model.hidden_dim)
            model.action_head = torch.nn.Linear(model.hidden_dim, output_state_dim)

        print(f"Mobile ACT dimensions: input_state_dim={model.input_state_dim}, output_state_dim={model.output_state_dim}")
        return model

    act_policy_module.build_act = build_act_mobile
    act_policy_module._mobile_act_builder_installed = True


@dataclass
class ObservationSnapshot:
    images: dict[str, np.ndarray]
    left: np.ndarray
    right: np.ndarray
    base_state: np.ndarray
    lift_height: float
    ages: dict[str, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ckpt_dir", nargs="?", default="/home/caizj/checkpoint/ACT_B")
    parser.add_argument("ckpt_name", nargs="?", default="policy_last_1500.ckpt")
    parser.add_argument("--ckpt-stats-name", default="dataset_stats.pkl")

    parser.add_argument("--front-topic", default="/camera_f/color/image_raw")
    parser.add_argument("--left-camera-topic", default="/camera_l/color/image_raw")
    parser.add_argument("--right-camera-topic", default="/camera_r/color/image_raw")
    parser.add_argument("--left-state-topic", default="/puppet/joint_left")
    parser.add_argument("--right-state-topic", default="/puppet/joint_right")
    parser.add_argument("--left-command-topic", default="/joint_left_states")
    parser.add_argument("--right-command-topic", default="/joint_right_states")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--base-command-topic", default="/cmd_vel")
    parser.add_argument("--lift-state-topic", default="/LiftMotorStatePub")
    parser.add_argument("--lift-service", default="/LiftingMotorService")

    parser.add_argument("--enable-base", action="store_true", help="Publish action[14:17] to /cmd_vel.")
    parser.add_argument("--enable-lift", action="store_true", help="Send action[17] to the lifting-column service.")
    parser.add_argument("--disable-arms", action="store_true", help="Do not publish arm JointState commands.")
    parser.add_argument("--dry-run", action="store_true", help="Run policy and print actions without publishing commands.")
    parser.add_argument("--yes", action="store_true", help="Skip the final interactive publish confirmation.")

    parser.add_argument("--chunk-size", type=int, default=100)
    parser.add_argument("--execute-horizon", type=int, default=8)
    parser.add_argument("--action-start-index", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=300, help="Maximum executed low-rate actions. Use 0 for no limit.")
    parser.add_argument("--max-runtime-sec", type=float, default=60.0, help="Maximum runtime. Use 0 for no limit.")
    parser.add_argument("--control-rate-hz", type=float, default=10.0)
    parser.add_argument("--inner-rate-hz", type=float, default=50.0)
    parser.add_argument("--max-frame-age-sec", type=float, default=2.0)
    parser.add_argument("--wait-timeout-sec", type=float, default=30.0)
    parser.add_argument("--log-every", type=int, default=10)
    parser.add_argument("--debug-actions", action="store_true")

    parser.add_argument("--base-scale", type=float, default=1.0)
    parser.add_argument("--max-linear", type=float, default=0.4)
    parser.add_argument("--max-angular", type=float, default=0.8)
    parser.add_argument("--base-linear-deadband", type=float, default=0.005)
    parser.add_argument("--base-angular-deadband", type=float, default=0.01)
    parser.add_argument(
        "--base-deadband-warning",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Warn when selected base commands are below the chassis smoother deadband.",
    )
    parser.add_argument(
        "--require-effective-base",
        action="store_true",
        help="Abort if --enable-base is set but the selected action window has no command above deadband.",
    )
    parser.add_argument("--lift-mode", type=int, default=0)
    parser.add_argument("--lift-min-delta", type=float, default=2.0)
    parser.add_argument("--lift-min-period", type=float, default=0.2)
    parser.add_argument("--lift-min-height", type=float, default=None)
    parser.add_argument("--lift-max-height", type=float, default=None)

    parser.add_argument("--reject-joint-abs", type=float, default=3.5)
    parser.add_argument("--max-joint-step", type=float, default=0.06)
    parser.add_argument("--max-gripper-step", type=float, default=0.01)
    parser.add_argument("--arm-speed", type=float, default=80.0, help="Piper arm speed percentage in velocity[6].")
    parser.add_argument("--gripper-effort", type=float, default=0.0)
    parser.add_argument("--hold-on-exit", action="store_true")

    parser.add_argument("--qpos-norm-mode", type=int, default=2)
    parser.add_argument("--pos-lookahead-step", type=int, default=32)
    parser.add_argument("--check-model-only", action="store_true", help="Load the ACT checkpoint and exit.")
    parser.add_argument("--inspect-checkpoint-only", action="store_true", help="Inspect checkpoint dimensions and exit.")
    parser.add_argument("--check-inputs-only", action="store_true", help="Wait for ROS inputs and exit.")
    return parser.parse_args()


def build_act_args(cli: argparse.Namespace) -> argparse.Namespace:
    saved_argv = sys.argv[:]
    sys.argv = [
        "run_act_mobile_inference.py",
        "--ckpt_dir",
        cli.ckpt_dir,
        "--ckpt_name",
        cli.ckpt_name,
        "--ckpt_stats_name",
        cli.ckpt_stats_name,
        "--policy_class",
        "ACT",
        "--arm_joint_state_dim",
        "7",
        "--use_arm_joint_state",
        "3",
        "--use_arm_end_pose",
        "0",
        "--use_robot_base",
        "3",
        "--robot_base_dim",
        str(ROBOT_BASE_ACTION_DIM),
        "--chunk_size",
        str(cli.chunk_size),
        "--hidden_dim",
        "512",
        "--dim_feedforward",
        "3200",
        "--kl_weight",
        "10",
        "--qpos_norm_mode",
        str(cli.qpos_norm_mode),
        "--pos_lookahead_step",
        str(cli.pos_lookahead_step),
    ]
    try:
        args = ACT_MOD.get_arguments()
    finally:
        sys.argv = saved_argv

    args.use_camera_color = True
    args.use_camera_depth = False
    args.use_camera_point_cloud = False
    args.use_camera_color_depth_to_point_cloud = False
    args.camera_color_names = list(CAMERA_NAMES)
    args.camera_color_topics = [cli.front_topic, cli.left_camera_topic, cli.right_camera_topic]
    args.camera_color_config_topics = [
        "/camera_f/color/camera_info",
        "/camera_l/color/camera_info",
        "/camera_r/color/camera_info",
    ]
    args.camera_color_parent_frame_ids = ["camera_f_link", "camera_l_link", "camera_r_link"]
    args.camera_depth_names = []
    args.camera_depth_topics = []
    args.camera_depth_config_topics = []
    args.camera_depth_parent_frame_ids = []
    args.camera_point_cloud_names = []
    args.camera_point_cloud_topics = []
    args.camera_point_cloud_config_topics = []
    args.camera_point_cloud_parent_frame_ids = []

    args.arm_joint_state_names = list(ARM_NAMES)
    args.arm_joint_state_topics = [cli.left_state_topic, cli.right_state_topic]
    args.arm_joint_state_ctrl_topics = [cli.left_command_topic, cli.right_command_topic]
    args.robot_base_vel_names = ["chassis"]
    args.robot_base_vel_topics = [cli.odom_topic]
    args.robot_base_vel_ctrl_topic = cli.base_command_topic
    args.robot_base_state_dim = ROBOT_BASE_STATE_DIM
    args.robot_base_action_dim = ROBOT_BASE_ACTION_DIM
    args.expected_input_state_dim = ARM_STATE_DIM + ROBOT_BASE_STATE_DIM
    args.expected_output_state_dim = ARM_STATE_DIM + ROBOT_BASE_ACTION_DIM
    return args


def extract_state_dict(checkpoint_obj) -> OrderedDict:
    if isinstance(checkpoint_obj, OrderedDict):
        return checkpoint_obj
    if isinstance(checkpoint_obj, dict):
        for key in ("state_dict", "model_state_dict", "policy_state_dict", "model"):
            value = checkpoint_obj.get(key)
            if isinstance(value, (dict, OrderedDict)):
                return OrderedDict(value)
        if all(isinstance(key, str) for key in checkpoint_obj.keys()):
            return OrderedDict(checkpoint_obj)
    raise RuntimeError("Unsupported checkpoint format.")


def load_checkpoint_state(ckpt_path: Path) -> OrderedDict:
    if not ckpt_path.is_file():
        raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")
    return extract_state_dict(torch.load(str(ckpt_path), map_location="cpu"))


def validate_checkpoint_dims(state_dict: OrderedDict, act_args: argparse.Namespace) -> None:
    errors: list[str] = []
    qpos_weight = state_dict.get("model.input_proj_qpos.weight")
    action_weight = state_dict.get("model.action_head.weight")
    encoder_qpos_weight = state_dict.get("model.encoder_qpos_proj.weight")
    encoder_action_weight = state_dict.get("model.encoder_action_proj.weight")

    expected_in = int(act_args.expected_input_state_dim)
    expected_out = int(act_args.expected_output_state_dim)
    if qpos_weight is None or qpos_weight.shape[1] != expected_in:
        got = None if qpos_weight is None else int(qpos_weight.shape[1])
        errors.append(f"model.input_proj_qpos.weight input dim expected {expected_in}, got {got}")
    if encoder_qpos_weight is None or encoder_qpos_weight.shape[1] != expected_in:
        got = None if encoder_qpos_weight is None else int(encoder_qpos_weight.shape[1])
        errors.append(f"model.encoder_qpos_proj.weight input dim expected {expected_in}, got {got}")
    if action_weight is None or action_weight.shape[0] != expected_out:
        got = None if action_weight is None else int(action_weight.shape[0])
        errors.append(f"model.action_head.weight output dim expected {expected_out}, got {got}")
    if encoder_action_weight is None or encoder_action_weight.shape[1] != expected_out:
        got = None if encoder_action_weight is None else int(encoder_action_weight.shape[1])
        errors.append(f"model.encoder_action_proj.weight input dim expected {expected_out}, got {got}")

    if errors:
        raise RuntimeError("Checkpoint dimensions do not match mobile ACT:\n  " + "\n  ".join(errors))


def remap_state_dict(state_dict: OrderedDict, act_args: argparse.Namespace) -> OrderedDict:
    new_state_dict = OrderedDict()
    for key, value in state_dict.items():
        if key in {"model.is_pad_head.weight", "model.is_pad_head.bias"}:
            continue
        if act_args.next_action_num == 0 and key in {
            "model.input_proj_next_action.weight",
            "model.input_proj_next_action.bias",
        }:
            continue
        if "robot_state" in key:
            key = key.replace("robot_state", "qpos")
        if "encoder_joint_proj" in key:
            key = key.replace("encoder_joint_proj", "encoder_qpos_proj")
        if key in ACT_MOD.model_dict_mapping.list1:
            key = ACT_MOD.model_dict_mapping.list2[ACT_MOD.model_dict_mapping.list1.index(key)]
        if act_args.backbone.startswith("resnet") and key == "model.pos.weight":
            continue
        new_state_dict[key] = value
    return new_state_dict


def load_policy(act_args: argparse.Namespace, state_dict: OrderedDict):
    install_mobile_act_builder()
    ACT_MOD.set_seed(1000)
    policy = ACT_MOD.make_policy(act_args)
    loading_status = policy.deserialize(remap_state_dict(state_dict, act_args))
    print(f"Loaded checkpoint status: {loading_status}")
    policy.cuda()
    policy.eval()
    return policy


def validate_stats_array(stats: dict, key: str, dim: int, errors: list[str]) -> None:
    for suffix in ("mean", "std", "min", "max", "scale", "offset"):
        full_key = f"{key}_{suffix}"
        if full_key not in stats:
            errors.append(f"missing {full_key}")
            continue
        value = np.asarray(stats[full_key]).reshape(-1)
        if value.shape[0] != dim:
            errors.append(f"{full_key} expected dim {dim}, got {value.shape[0]}")


def load_stats(stats_path: Path, act_args: argparse.Namespace) -> dict:
    if not stats_path.is_file():
        raise FileNotFoundError(
            f"Stats file not found: {stats_path}\n"
            "ACT_B needs the dataset_stats.pkl generated by the same mobile training run. "
            "Do not reuse the old dual-arm ACT stats."
        )
    with open(stats_path, "rb") as f:
        stats = pickle.load(f)

    errors: list[str] = []
    validate_stats_array(stats, "qpos_joint_state", ARM_STATE_DIM, errors)
    validate_stats_array(stats, "action_joint_state", ARM_STATE_DIM, errors)
    validate_stats_array(stats, "qpos_robot_base", ROBOT_BASE_STATE_DIM, errors)
    validate_stats_array(stats, "action_robot_base", ROBOT_BASE_ACTION_DIM, errors)
    if stats.get("joint_state_norm_mode") != act_args.qpos_norm_mode:
        errors.append(
            f"joint_state_norm_mode expected {act_args.qpos_norm_mode}, got {stats.get('joint_state_norm_mode')}"
        )
    if stats.get("robot_base_norm_mode") != act_args.qpos_norm_mode:
        errors.append(
            f"robot_base_norm_mode expected {act_args.qpos_norm_mode}, got {stats.get('robot_base_norm_mode')}"
        )
    if errors:
        raise RuntimeError("Invalid mobile ACT stats:\n  " + "\n  ".join(errors))
    return stats


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def odometry_to_base_state(msg: Odometry) -> np.ndarray:
    pose = msg.pose.pose
    twist = msg.twist.twist
    return np.asarray(
        [
            float(pose.position.x),
            float(pose.position.y),
            quaternion_to_yaw(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.angular.z),
        ],
        dtype=np.float32,
    )


def lift_height_from_msg(msg) -> float | None:
    height = getattr(msg, "back_height", None)
    if height is None:
        height = getattr(msg, "backHeight", None)
    if height is None:
        height = getattr(msg, "height", None)
    return None if height is None else float(height)


def clamp_value(value: float, limit: float) -> float:
    if limit <= 0:
        return float(value)
    return float(max(-limit, min(limit, value)))


class ActMobileNode(Node):
    def __init__(self, cli: argparse.Namespace):
        super().__init__("act_mobile_inference")
        self.cli = cli
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.images: dict[str, np.ndarray] = {}
        self.image_times: dict[str, float] = {}
        self.joints: dict[str, np.ndarray] = {}
        self.joint_times: dict[str, float] = {}
        self.base_state: np.ndarray | None = None
        self.base_time: float | None = None
        self.lift_height: float | None = None
        self.lift_time: float | None = None
        self.last_lift_command: float | None = None
        self.last_lift_command_time = 0.0
        self.last_error_times: dict[str, float] = {}

        self.left_pub = None if cli.disable_arms else self.create_publisher(JointState, cli.left_command_topic, 10)
        self.right_pub = None if cli.disable_arms else self.create_publisher(JointState, cli.right_command_topic, 10)
        self.base_pub = self.create_publisher(Twist, cli.base_command_topic, 10) if cli.enable_base else None
        self.lift_client = None
        if cli.enable_lift:
            if LiftMotorSrv is None:
                raise RuntimeError(
                    "LiftMotorSrv is not importable. Source /home/agilex/agilex_ws/install/setup.bash "
                    "or run without --enable-lift."
                )
            self.lift_client = self.create_client(LiftMotorSrv, cli.lift_service)

        self.create_subscription(Image, cli.front_topic, self._image_callback("front"), qos_profile_sensor_data)
        self.create_subscription(Image, cli.left_camera_topic, self._image_callback("left"), qos_profile_sensor_data)
        self.create_subscription(Image, cli.right_camera_topic, self._image_callback("right"), qos_profile_sensor_data)
        self.create_subscription(JointState, cli.left_state_topic, self._joint_callback("left"), 10)
        self.create_subscription(JointState, cli.right_state_topic, self._joint_callback("right"), 10)
        self.create_subscription(Odometry, cli.odom_topic, self._odom_callback, qos_profile_sensor_data)
        if LiftMotorMsg is None:
            raise RuntimeError(
                "LiftMotorMsg is not importable. Source /home/agilex/agilex_ws/install/setup.bash."
            )
        self.create_subscription(LiftMotorMsg, cli.lift_state_topic, self._lift_callback, 10)

    def _throttled_error(self, key: str, message: str, interval: float = 2.0) -> None:
        now = time.monotonic()
        if now - self.last_error_times.get(key, 0.0) >= interval:
            self.get_logger().error(message)
            self.last_error_times[key] = now

    def _image_callback(self, key: str) -> Callable[[Image], None]:
        def callback(msg: Image) -> None:
            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                with self.lock:
                    self.images[key] = np.ascontiguousarray(image)
                    self.image_times[key] = time.monotonic()
            except Exception as exc:
                self._throttled_error(key, f"Failed to convert image {key}: {exc}")

        return callback

    def _joint_callback(self, key: str) -> Callable[[JointState], None]:
        def callback(msg: JointState) -> None:
            if len(msg.position) < 7:
                self._throttled_error(key, f"JointState {key} has {len(msg.position)} positions, expected 7.")
                return
            with self.lock:
                self.joints[key] = np.asarray(msg.position[:7], dtype=np.float32)
                self.joint_times[key] = time.monotonic()

        return callback

    def _odom_callback(self, msg: Odometry) -> None:
        with self.lock:
            self.base_state = odometry_to_base_state(msg)
            self.base_time = time.monotonic()

    def _lift_callback(self, msg) -> None:
        height = lift_height_from_msg(msg)
        if height is None:
            self._throttled_error("lift", "Lift state message has no back_height/backHeight/height field.")
            return
        with self.lock:
            self.lift_height = height
            self.lift_time = time.monotonic()

    def missing_or_stale_inputs(self, max_age_sec: float) -> list[str]:
        now = time.monotonic()
        missing: list[str] = []
        with self.lock:
            for key in CAMERA_NAMES:
                if key not in self.images:
                    missing.append(f"camera_{key}")
                elif now - self.image_times[key] > max_age_sec:
                    missing.append(f"camera_{key}:stale")
            for key in ARM_NAMES:
                if key not in self.joints:
                    missing.append(f"joint_{key}")
                elif now - self.joint_times[key] > max_age_sec:
                    missing.append(f"joint_{key}:stale")
            if self.base_state is None or self.base_time is None:
                missing.append("odom")
            elif now - self.base_time > max_age_sec:
                missing.append("odom:stale")
            if self.lift_height is None or self.lift_time is None:
                missing.append("lift")
            elif now - self.lift_time > max_age_sec:
                missing.append("lift:stale")
        return missing

    def make_snapshot(self) -> ObservationSnapshot | None:
        now = time.monotonic()
        with self.lock:
            if any(key not in self.images for key in CAMERA_NAMES):
                return None
            if any(key not in self.joints for key in ARM_NAMES):
                return None
            if self.base_state is None or self.lift_height is None:
                return None
            images = {key: self.images[key].copy() for key in CAMERA_NAMES}
            left = self.joints["left"].copy()
            right = self.joints["right"].copy()
            base_state = self.base_state.copy()
            lift_height = float(self.lift_height)
            ages = {f"camera_{key}": now - self.image_times[key] for key in CAMERA_NAMES}
            ages.update({f"joint_{key}": now - self.joint_times[key] for key in ARM_NAMES})
            ages["odom"] = now - float(self.base_time)
            ages["lift"] = now - float(self.lift_time)
        return ObservationSnapshot(images, left, right, base_state, lift_height, ages)

    def publish_arms(self, left: np.ndarray, right: np.ndarray) -> None:
        if self.left_pub is None or self.right_pub is None:
            return
        stamp = self.get_clock().now().to_msg()
        for pub, command in ((self.left_pub, left), (self.right_pub, right)):
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = stamp
            msg.name = JOINT_NAMES
            msg.position = [float(v) for v in command[:7]]
            msg.velocity = [0.0] * 6 + [float(self.cli.arm_speed)]
            msg.effort = [0.0] * 6 + [float(self.cli.gripper_effort)]
            pub.publish(msg)

    def publish_base(self, command: np.ndarray | None) -> None:
        if self.base_pub is None or command is None:
            return
        msg = Twist()
        msg.linear.x = float(command[0])
        msg.linear.y = float(command[1])
        msg.angular.z = float(command[2])
        self.base_pub.publish(msg)

    def stop_base(self) -> None:
        if self.base_pub is not None:
            self.base_pub.publish(Twist())

    def wait_for_lift_service(self, timeout_sec: float) -> bool:
        if self.lift_client is None:
            return True
        return self.lift_client.wait_for_service(timeout_sec=timeout_sec)

    def send_lift_height(self, target: float, force: bool = False) -> None:
        if self.lift_client is None:
            return
        now = time.monotonic()
        if not force:
            if now - self.last_lift_command_time < self.cli.lift_min_period:
                return
            if self.last_lift_command is not None and abs(target - self.last_lift_command) < self.cli.lift_min_delta:
                return
        req = LiftMotorSrv.Request()
        req.val = int(round(float(target)))
        req.mode = int(self.cli.lift_mode)
        self.lift_client.call_async(req)
        self.last_lift_command = float(target)
        self.last_lift_command_time = now


def wait_for_snapshot(node: ActMobileNode, cli: argparse.Namespace) -> ObservationSnapshot:
    deadline = time.monotonic() + cli.wait_timeout_sec
    last_print = 0.0
    while rclpy.ok() and time.monotonic() < deadline:
        missing = node.missing_or_stale_inputs(cli.max_frame_age_sec)
        if not missing:
            snapshot = node.make_snapshot()
            if snapshot is not None:
                return snapshot
        now = time.monotonic()
        if now - last_print >= 2.0:
            print("Waiting for ROS inputs:", ", ".join(missing) if missing else "snapshot")
            last_print = now
        time.sleep(0.1)
    raise TimeoutError(f"Timed out waiting for ROS inputs. Missing/stale: {node.missing_or_stale_inputs(cli.max_frame_age_sec)}")


def preprocess_snapshot(snapshot: ObservationSnapshot, stats: dict, act_args: argparse.Namespace):
    camera_color_dict = {
        name: [snapshot.images[name] for _ in range(act_args.obs_history_num)]
        for name in act_args.camera_color_names
    }
    camera_color = ACT_MOD.get_camera_color(
        stats,
        act_args.camera_color_names,
        act_args.obs_history_num,
        act_args.augment_color,
        camera_color_dict,
    )

    arm_state = np.concatenate([snapshot.left, snapshot.right], axis=0).astype(np.float32)
    robot_base_state = np.concatenate(
        [snapshot.base_state.astype(np.float32), np.asarray([snapshot.lift_height], dtype=np.float32)],
        axis=0,
    )
    qpos_joint_state = np.tile(arm_state[np.newaxis, :], (act_args.obs_history_num, 1))
    qpos_robot_base = np.tile(robot_base_state[np.newaxis, :], (act_args.obs_history_num, 1))

    qpos_joint_state = ACT_MOD.pre_process(
        stats,
        "joint_state",
        act_args.qpos_norm_mode,
        is_action=False,
        data=qpos_joint_state,
    )
    qpos_robot_base = ACT_MOD.pre_process(
        stats,
        "robot_base",
        act_args.qpos_norm_mode,
        is_action=False,
        data=qpos_robot_base,
    )

    qpos_joint_state = torch.from_numpy(qpos_joint_state).float().cuda().unsqueeze(0)
    qpos_robot_base = torch.from_numpy(qpos_robot_base).float().cuda().unsqueeze(0)
    return camera_color, qpos_joint_state, qpos_robot_base


def postprocess_actions(raw_actions: np.ndarray, stats: dict, act_args: argparse.Namespace) -> np.ndarray:
    actions = []
    for raw_action in raw_actions:
        joint_action = ACT_MOD.post_process(
            stats,
            "joint_state",
            act_args.qpos_norm_mode,
            is_action=True,
            data=raw_action[:ARM_STATE_DIM],
        )
        robot_action = ACT_MOD.post_process(
            stats,
            "robot_base",
            act_args.qpos_norm_mode,
            is_action=True,
            data=raw_action[ARM_STATE_DIM:POLICY_ACTION_DIM],
        )
        actions.append(np.concatenate([joint_action, robot_action], axis=0))
    return np.asarray(actions, dtype=np.float64)


def infer_action_chunk(policy, stats: dict, act_args: argparse.Namespace, snapshot: ObservationSnapshot) -> np.ndarray:
    camera_color, qpos_joint_state, qpos_robot_base = preprocess_snapshot(snapshot, stats, act_args)
    with torch.inference_mode():
        all_actions, category = policy(
            camera_color,
            None,
            None,
            qpos_joint_state,
            None,
            qpos_robot_base,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        )
    if category is not None:
        print(f"category: {int(category.cpu().detach().numpy().item())}")
    raw_actions = all_actions.cpu().detach().numpy()[0]
    return postprocess_actions(raw_actions, stats, act_args)


def validate_actions(actions: np.ndarray, reject_joint_abs: float) -> np.ndarray:
    if actions.ndim == 1:
        actions = actions[None, :]
    if actions.ndim != 2 or actions.shape[1] != POLICY_ACTION_DIM:
        raise ValueError(f"Expected ACT actions with shape [N, {POLICY_ACTION_DIM}], got {actions.shape}.")
    if not np.all(np.isfinite(actions)):
        raise ValueError("Policy returned NaN or inf.")
    joints = np.concatenate([actions[:, :6], actions[:, 7:13]], axis=1)
    max_abs = float(np.max(np.abs(joints)))
    if max_abs > reject_joint_abs:
        raise ValueError(f"Policy joint action abs {max_abs:.3f} exceeds limit {reject_joint_abs:.3f}.")
    return actions


def action_to_arm_targets(action: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return action[:7].astype(np.float64), action[7:14].astype(np.float64)


def action_to_base_command(action: np.ndarray, cli: argparse.Namespace) -> np.ndarray:
    return np.asarray(
        [
            clamp_value(float(action[14]) * cli.base_scale, cli.max_linear),
            clamp_value(float(action[15]) * cli.base_scale, cli.max_linear),
            clamp_value(float(action[16]) * cli.base_scale, cli.max_angular),
        ],
        dtype=np.float64,
    )


def base_commands_from_actions(actions: np.ndarray, cli: argparse.Namespace) -> np.ndarray:
    return np.stack([action_to_base_command(action, cli) for action in actions], axis=0)


def effective_base_mask(base_commands: np.ndarray, cli: argparse.Namespace) -> np.ndarray:
    if base_commands.size == 0:
        return np.asarray([], dtype=bool)
    return (
        (np.abs(base_commands[:, 0]) > cli.base_linear_deadband)
        | (np.abs(base_commands[:, 1]) > cli.base_linear_deadband)
        | (np.abs(base_commands[:, 2]) > cli.base_angular_deadband)
    )


def base_peak(base_commands: np.ndarray) -> tuple[int, np.ndarray]:
    base_norm = np.max(np.abs(base_commands), axis=1)
    peak_index = int(np.argmax(base_norm))
    return peak_index, base_commands[peak_index]


def action_to_lift_target(action: np.ndarray, cli: argparse.Namespace) -> float:
    target = float(action[17])
    if cli.lift_min_height is not None:
        target = max(float(cli.lift_min_height), target)
    if cli.lift_max_height is not None:
        target = min(float(cli.lift_max_height), target)
    return target


def clamp_arm_step(
    target_left: np.ndarray,
    target_right: np.ndarray,
    last_left: np.ndarray,
    last_right: np.ndarray,
    cli: argparse.Namespace,
) -> tuple[np.ndarray, np.ndarray]:
    if cli.max_joint_step <= 0 and cli.max_gripper_step <= 0:
        return target_left, target_right
    limits = np.asarray([cli.max_joint_step] * 6 + [cli.max_gripper_step], dtype=np.float64)
    limits = np.where(limits > 0, limits, np.inf)
    next_left = last_left + np.clip(target_left - last_left, -limits, limits)
    next_right = last_right + np.clip(target_right - last_right, -limits, limits)
    return next_left, next_right


def publish_smooth(
    node: ActMobileNode,
    start_left: np.ndarray,
    start_right: np.ndarray,
    target_left: np.ndarray,
    target_right: np.ndarray,
    base_command: np.ndarray | None,
    cli: argparse.Namespace,
) -> None:
    steps = max(1, int(round(cli.inner_rate_hz / cli.control_rate_hz)))
    period = 1.0 / max(cli.inner_rate_hz, 1.0)
    for index in range(1, steps + 1):
        alpha = index / steps
        left = start_left + (target_left - start_left) * alpha
        right = start_right + (target_right - start_right) * alpha
        node.publish_arms(left, right)
        node.publish_base(base_command)
        time.sleep(period)


def hold_command(node: ActMobileNode, left: np.ndarray, right: np.ndarray, repeat: int = 10) -> None:
    for _ in range(repeat):
        node.publish_arms(left, right)
        node.stop_base()
        time.sleep(0.02)


def print_snapshot(snapshot: ObservationSnapshot) -> None:
    print("ROS inputs ready")
    print(f"  max input age: {max(snapshot.ages.values()):.3f}s")
    print(f"  left state:  {np.array2string(snapshot.left, precision=4, suppress_small=True)}")
    print(f"  right state: {np.array2string(snapshot.right, precision=4, suppress_small=True)}")
    print(f"  base state:  {np.array2string(snapshot.base_state, precision=4, suppress_small=True)}")
    print(f"  lift height: {snapshot.lift_height:.3f}")
    print(f"  image shapes: { {key: value.shape for key, value in snapshot.images.items()} }")


def print_action_debug(actions: np.ndarray, chunk: np.ndarray, start_index: int, stop_index: int, cli: argparse.Namespace) -> None:
    print(f"  executing action window: [{start_index}:{stop_index}]")
    print(f"  first action: {np.array2string(chunk[0], precision=4, suppress_small=True)}")
    print(f"  last action:  {np.array2string(chunk[-1], precision=4, suppress_small=True)}")
    print(f"  full action range: {np.array2string(np.ptp(actions, axis=0), precision=4, suppress_small=True)}")
    if cli.enable_base:
        full_base = base_commands_from_actions(actions, cli)
        full_effective = effective_base_mask(full_base, cli)
        peak_index, peak_value = base_peak(full_base)
        print(f"  full base max abs: {np.array2string(np.max(np.abs(full_base), axis=0), precision=4)}")
        print(
            f"  full base peak index: {peak_index}, "
            f"value={np.array2string(peak_value, precision=4, suppress_small=True)}"
        )
        print(f"  full base effective count: {int(full_effective.sum())}/{len(full_effective)}")
        base = base_commands_from_actions(chunk, cli)
        effective = effective_base_mask(base, cli)
        print(f"  executing base max abs: {np.array2string(np.max(np.abs(base), axis=0), precision=4)}")
        print(f"  executing base effective count: {int(effective.sum())}/{len(effective)}")
    if cli.enable_lift:
        lift = np.asarray([action_to_lift_target(action, cli) for action in chunk])
        print(f"  executing lift first/last: {lift[0]:.3f} / {lift[-1]:.3f}")


def selected_base_effectiveness_detail(actions: np.ndarray, chunk: np.ndarray, cli: argparse.Namespace) -> tuple[bool, str]:
    selected_base = base_commands_from_actions(chunk, cli)
    selected_effective = effective_base_mask(selected_base, cli)
    if bool(np.any(selected_effective)):
        return True, ""
    full_base = base_commands_from_actions(actions, cli)
    full_effective = effective_base_mask(full_base, cli)
    peak_index, peak_value = base_peak(full_base)
    detail = (
        "selected base commands are below deadband "
        f"(linear>{cli.base_linear_deadband:.4f} m/s or "
        f"angular>{cli.base_angular_deadband:.4f} rad/s required). "
        f"full_effective={int(full_effective.sum())}/{len(full_effective)}, "
        f"peak_index={peak_index}, "
        f"peak={np.array2string(peak_value, precision=4, suppress_small=True)}"
    )
    return False, detail


def validate_cli(cli: argparse.Namespace) -> None:
    if cli.execute_horizon <= 0:
        raise SystemExit("--execute-horizon must be positive.")
    if cli.action_start_index < 0:
        raise SystemExit("--action-start-index must be non-negative.")
    if cli.control_rate_hz <= 0 or cli.inner_rate_hz <= 0:
        raise SystemExit("--control-rate-hz and --inner-rate-hz must be positive.")
    if cli.lift_min_height is not None and cli.lift_max_height is not None and cli.lift_min_height > cli.lift_max_height:
        raise SystemExit("--lift-min-height must be <= --lift-max-height.")
    if cli.chunk_size <= 0:
        raise SystemExit("--chunk-size must be positive.")
    if cli.arm_speed <= 0:
        raise SystemExit("--arm-speed must be positive.")
    if cli.base_linear_deadband < 0 or cli.base_angular_deadband < 0:
        raise SystemExit("--base-linear-deadband and --base-angular-deadband must be non-negative.")


def check_ros_inputs_only(cli: argparse.Namespace) -> int:
    rclpy.init()
    node = ActMobileNode(cli)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        snapshot = wait_for_snapshot(node, cli)
        print_snapshot(snapshot)
        return 0
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        finally:
            rclpy.shutdown()


def main() -> int:
    cli = parse_args()
    validate_cli(cli)
    if cli.check_inputs_only:
        try:
            return check_ros_inputs_only(cli)
        except (TimeoutError, RuntimeError) as exc:
            print(f"ERROR: {exc}")
            return 1

    act_args = build_act_args(cli)
    ckpt_path = Path(cli.ckpt_dir) / cli.ckpt_name
    stats_path = Path(cli.ckpt_dir) / cli.ckpt_stats_name

    try:
        state_dict = load_checkpoint_state(ckpt_path)
        validate_checkpoint_dims(state_dict, act_args)
        print(f"Checkpoint dimensions OK: state=21, action=18 ({ckpt_path})")
        if cli.inspect_checkpoint_only:
            return 0

        stats = None
        if not cli.check_model_only:
            stats = load_stats(stats_path, act_args)
            print(f"Stats OK: {stats_path}")

        policy = load_policy(act_args, state_dict)
        if cli.check_model_only:
            return 0

        rclpy.init()
        node = ActMobileNode(cli)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        last_left: np.ndarray | None = None
        last_right: np.ndarray | None = None

        try:
            snapshot = wait_for_snapshot(node, cli)
            print_snapshot(snapshot)
            last_left = snapshot.left.astype(np.float64)
            last_right = snapshot.right.astype(np.float64)

            if cli.check_inputs_only:
                return 0
            if cli.enable_base and cli.require_effective_base and not cli.dry_run:
                print("Preflight: checking first ACT chunk for effective base command before confirmation.")
                actions = validate_actions(infer_action_chunk(policy, stats, act_args, snapshot), cli.reject_joint_abs)
                start_index = min(cli.action_start_index, actions.shape[0] - 1)
                stop_index = min(start_index + cli.execute_horizon, actions.shape[0])
                chunk = actions[start_index:stop_index]
                if cli.debug_actions:
                    print(f"Preflight chunk: got {actions.shape[0]} actions, checking [{start_index}:{stop_index}]")
                    print_action_debug(actions, chunk, start_index, stop_index, cli)
                effective, detail = selected_base_effectiveness_detail(actions, chunk, cli)
                if not effective:
                    raise RuntimeError("preflight failed: " + detail)
                print("Preflight OK: selected action window has effective base commands.")
            if cli.enable_lift and not cli.dry_run and not node.wait_for_lift_service(cli.wait_timeout_sec):
                raise RuntimeError(f"Lift service is not available: {cli.lift_service}")
            if not cli.yes and not cli.dry_run:
                input("Press Enter to start publishing ACT actions, or Ctrl+C to abort. ")
            if not cli.dry_run:
                hold_command(node, last_left, last_right, repeat=5)

            start_time = time.monotonic()
            executed = 0
            chunk_index = 0
            stop_reason = None
            unlimited_steps = cli.max_steps <= 0
            unlimited_runtime = cli.max_runtime_sec <= 0

            while rclpy.ok() and stop_reason is None:
                if not unlimited_steps and executed >= cli.max_steps:
                    stop_reason = f"max steps {cli.max_steps}"
                    break
                if not unlimited_runtime and time.monotonic() - start_time >= cli.max_runtime_sec:
                    stop_reason = f"max runtime {cli.max_runtime_sec:.1f}s"
                    break

                snapshot = wait_for_snapshot(node, cli)
                inference_start = time.monotonic()
                actions = validate_actions(infer_action_chunk(policy, stats, act_args, snapshot), cli.reject_joint_abs)
                inference_dt = time.monotonic() - inference_start
                start_index = min(cli.action_start_index, actions.shape[0] - 1)
                stop_index = min(start_index + cli.execute_horizon, actions.shape[0])
                chunk = actions[start_index:stop_index]
                print(
                    f"Chunk {chunk_index}: got {actions.shape[0]} actions, "
                    f"executing [{start_index}:{stop_index}], infer {inference_dt:.3f}s"
                )
                if cli.debug_actions:
                    print_action_debug(actions, chunk, start_index, stop_index, cli)
                if cli.enable_base:
                    effective, detail = selected_base_effectiveness_detail(actions, chunk, cli)
                    if not effective:
                        if cli.require_effective_base:
                            raise RuntimeError(detail)
                        if cli.base_deadband_warning:
                            print(f"WARN: {detail}")

                for action in chunk:
                    if not unlimited_steps and executed >= cli.max_steps:
                        stop_reason = f"max steps {cli.max_steps}"
                        break
                    if not unlimited_runtime and time.monotonic() - start_time >= cli.max_runtime_sec:
                        stop_reason = f"max runtime {cli.max_runtime_sec:.1f}s"
                        break
                    target_left, target_right = action_to_arm_targets(action)
                    next_left, next_right = clamp_arm_step(target_left, target_right, last_left, last_right, cli)
                    base_command = action_to_base_command(action, cli) if cli.enable_base else None
                    lift_target = action_to_lift_target(action, cli) if cli.enable_lift else None

                    if cli.dry_run:
                        time.sleep(1.0 / cli.control_rate_hz)
                    else:
                        if lift_target is not None:
                            node.send_lift_height(lift_target)
                        publish_smooth(node, last_left, last_right, next_left, next_right, base_command, cli)

                    last_left, last_right = next_left, next_right
                    executed += 1
                    if cli.log_every > 0 and executed % cli.log_every == 0:
                        suffix = ""
                        if base_command is not None:
                            suffix += f" base={np.array2string(base_command, precision=3, suppress_small=True)}"
                        if lift_target is not None:
                            suffix += f" lift={lift_target:.2f}"
                        print(
                            f"  step {executed}: left={np.array2string(last_left, precision=3, suppress_small=True)} "
                            f"right={np.array2string(last_right, precision=3, suppress_small=True)}{suffix}"
                        )
                chunk_index += 1

            if stop_reason is None:
                stop_reason = "rclpy stopped"
            print(f"Finished after {executed} executed actions: {stop_reason}.")
            return 0
        finally:
            if not cli.dry_run and rclpy.ok():
                try:
                    node.stop_base()
                except Exception as exc:
                    print(f"WARN: stop-base publish failed: {exc}")
            if cli.hold_on_exit and not cli.dry_run and rclpy.ok() and last_left is not None and last_right is not None:
                try:
                    hold_command(node, last_left, last_right, repeat=10)
                except Exception as exc:
                    print(f"WARN: hold-on-exit publish failed: {exc}")
            try:
                executor.shutdown()
                node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
    except KeyboardInterrupt:
        print("Interrupted by user.")
        return 130
    except (FileNotFoundError, TimeoutError, ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
