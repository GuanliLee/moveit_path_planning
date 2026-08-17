#!/usr/bin/env python3
"""Run an OpenPI policy on the AgileX mobile base, lifting column, and dual Piper arms."""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import socket
import threading
import time
from dataclasses import dataclass
from typing import Callable
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
import websockets.sync.client as websockets_sync_client
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, JointState

import openpi_agilex_mobile_common as common

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

try:
    from openpi_client import websocket_client_policy
except ModuleNotFoundError as exc:
    raise SystemExit(
        "openpi_client is not importable. Use scripts/inference/run_openpi_agilex_mobile_inference.sh "
        "so the OpenPI client path and dependencies are configured."
    ) from exc


IMAGE_KEYS = ("cam_high", "cam_left_wrist", "cam_right_wrist")
JOINT_KEYS = ("left", "right")
JOINT_NAMES = [f"joint{i + 1}" for i in range(6)] + ["gripper"]


@dataclass
class ObservationSnapshot:
    obs: dict
    left_feedback: np.ndarray
    right_feedback: np.ndarray
    base_state: np.ndarray | None
    lift_height: float | None
    ages: dict[str, float]
    image_shapes: dict[str, tuple[int, ...]]


def env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return int(value)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("OPENPI_POLICY_HOST", "192.168.1.154"))
    parser.add_argument("--port", type=int, default=env_int("OPENPI_POLICY_PORT", 8899))
    parser.add_argument("--prompt", default=os.environ.get("OPENPI_PROMPT", common.DEFAULT_PROMPT))
    parser.add_argument(
        "--request-format",
        choices=("msgpack", "base64_json"),
        default="msgpack",
        help="OpenPI websocket protocol. The provided serve_policy script uses msgpack.",
    )

    parser.add_argument("--front-topic", default="/camera_f/color/image_raw")
    parser.add_argument("--left-camera-topic", default="/camera_l/color/image_raw")
    parser.add_argument("--right-camera-topic", default="/camera_r/color/image_raw")
    parser.add_argument(
        "--image-transport",
        choices=("raw", "compressed"),
        default="raw",
        help="Use compressed to subscribe to TOPIC/compressed unless the topic already ends that way.",
    )
    parser.add_argument("--left-state-topic", default="/puppet/joint_left")
    parser.add_argument("--right-state-topic", default="/puppet/joint_right")
    parser.add_argument("--left-command-topic", default="/joint_left_states")
    parser.add_argument("--right-command-topic", default="/joint_right_states")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--base-command-topic", default="/cmd_vel")
    parser.add_argument("--lift-state-topic", default="/LiftMotorStatePub")
    parser.add_argument("--lift-service", default="/LiftingMotorService")

    parser.add_argument(
        "--state-layout",
        choices=("mobile21", "aloha14"),
        default="mobile21",
        help="mobile21 = arms + lift height + odom [x,y,yaw,vx,vy,wz].",
    )
    parser.add_argument(
        "--expected-action-dim",
        type=int,
        default=0,
        help="0 infers 18 for mobile21/base/lift actions, 14 for arms-only.",
    )
    parser.add_argument("--enable-arms", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--enable-base", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--enable-lift", action=argparse.BooleanOptionalAction, default=True)

    parser.add_argument("--image-size", type=int, default=224)
    parser.add_argument("--image-encoding", choices=("jpeg", "png"), default="jpeg")
    parser.add_argument("--jpeg-quality", type=int, default=90)
    parser.add_argument("--gripper-min", type=float, default=-0.0026)
    parser.add_argument("--gripper-max", type=float, default=0.1043)
    parser.add_argument("--state-gripper-unit", choices=("meters", "normalized"), default="meters")
    parser.add_argument("--action-gripper-unit", choices=("meters", "normalized"), default="meters")

    parser.add_argument("--max-steps", type=int, default=300, help="Low-rate actions. 0 means unlimited.")
    parser.add_argument("--max-runtime-sec", type=float, default=60.0, help="0 means unlimited.")
    parser.add_argument("--execute-horizon", type=int, default=8)
    parser.add_argument("--action-start-index", type=int, default=0)
    parser.add_argument("--control-rate-hz", type=float, default=10.0)
    parser.add_argument("--inner-rate-hz", type=float, default=50.0)
    parser.add_argument("--arm-speed", type=float, default=70.0)
    parser.add_argument("--gripper-effort", type=float, default=0.0)

    parser.add_argument("--base-scale", type=float, default=1.0)
    parser.add_argument("--max-linear", type=float, default=0.4)
    parser.add_argument("--max-angular", type=float, default=0.8)
    parser.add_argument(
        "--base-component-deadband",
        type=float,
        default=0.0,
        help="Set individual base command components with abs(value) below this to zero before publishing.",
    )
    parser.add_argument("--swap-base-xy", action="store_true", help="Publish policy vy as cmd x and policy vx as cmd y.")
    parser.add_argument("--invert-base-x", action="store_true", help="Invert published base linear.x.")
    parser.add_argument("--invert-base-y", action="store_true", help="Invert published base linear.y.")
    parser.add_argument("--invert-base-wz", action="store_true", help="Invert published base angular.z.")
    parser.add_argument("--base-linear-deadband", type=float, default=0.005)
    parser.add_argument("--base-angular-deadband", type=float, default=0.01)
    parser.add_argument("--base-deadband-warning", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--require-effective-base", action="store_true")

    parser.add_argument("--lift-mode", type=int, default=0)
    parser.add_argument("--lift-min-delta", type=float, default=2.0)
    parser.add_argument("--lift-min-period", type=float, default=0.2)
    parser.add_argument("--lift-min-height", type=float, default=None)
    parser.add_argument("--lift-max-height", type=float, default=None)

    parser.add_argument("--max-joint-step", type=float, default=0.06)
    parser.add_argument("--max-gripper-step", type=float, default=0.01)
    parser.add_argument("--reject-joint-abs", type=float, default=3.5)
    parser.add_argument("--max-frame-age-sec", type=float, default=2.0)
    parser.add_argument("--wait-timeout-sec", type=float, default=30.0)
    parser.add_argument("--policy-wait-timeout-sec", type=float, default=300.0)
    parser.add_argument("--policy-ping-interval", type=float, default=None)
    parser.add_argument("--policy-ping-timeout", type=float, default=None)
    parser.add_argument("--keepalive-during-infer", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--infer-keepalive-max-sec", type=float, default=1.0)
    parser.add_argument("--log-every", type=int, default=10)
    parser.add_argument("--completion-mode", choices=("manual", "stable"), default="manual")
    parser.add_argument("--done-min-steps", type=int, default=80)
    parser.add_argument("--done-window", type=int, default=30)
    parser.add_argument("--done-joint-range", type=float, default=0.02)
    parser.add_argument("--done-gripper-range", type=float, default=0.003)
    parser.add_argument("--debug-actions", action="store_true")

    parser.add_argument("--dry-run", action="store_true", help="Request policy actions without publishing commands.")
    parser.add_argument("--check-inputs-only", action="store_true")
    parser.add_argument("--yes", action="store_true", help="Skip interactive publish confirmation.")
    parser.add_argument("--hold-on-exit", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def resolve_image_topic(topic: str, image_transport: str) -> str:
    if image_transport == "compressed" and not topic.endswith("/compressed"):
        return f"{topic}/compressed"
    return topic


def resize_rgb_to_chw(image: np.ndarray, size: int) -> np.ndarray:
    if image.dtype != np.uint8:
        image = np.clip(image, 0, 255).astype(np.uint8)

    height, width = image.shape[:2]
    scale = min(size / height, size / width)
    resized_width = max(1, int(width * scale))
    resized_height = max(1, int(height * scale))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=interpolation)

    padded = np.zeros((size, size, 3), dtype=np.uint8)
    top = (size - resized_height) // 2
    left = (size - resized_width) // 2
    padded[top : top + resized_height, left : left + resized_width] = resized
    return np.ascontiguousarray(np.transpose(padded, (2, 0, 1)))


def encode_image_base64(image_rgb: np.ndarray, encoding: str, jpeg_quality: int) -> str:
    if image_rgb.dtype != np.uint8:
        image_rgb = np.clip(image_rgb, 0, 255).astype(np.uint8)
    bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    if encoding == "jpeg":
        quality = int(np.clip(jpeg_quality, 1, 100))
        ok, encoded = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    elif encoding == "png":
        ok, encoded = cv2.imencode(".png", bgr)
    else:
        raise ValueError(f"Unsupported image encoding: {encoding}")
    if not ok:
        raise ValueError(f"Failed to encode {encoding} image.")
    return base64.b64encode(encoded.tobytes()).decode("ascii")


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def odometry_to_base_state(msg: Odometry) -> np.ndarray:
    pose = msg.pose.pose
    twist = msg.twist.twist
    yaw = quaternion_to_yaw(
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    return np.asarray(
        [
            float(pose.position.x),
            float(pose.position.y),
            yaw,
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


class OpenPiAgilexMobileNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("openpi_agilex_mobile_inference")
        self.args = args
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

        self.left_pub = None
        self.right_pub = None
        if args.enable_arms:
            self.left_pub = self.create_publisher(JointState, args.left_command_topic, 10)
            self.right_pub = self.create_publisher(JointState, args.right_command_topic, 10)

        self.base_pub = self.create_publisher(Twist, args.base_command_topic, 10) if args.enable_base else None
        self.lift_client = None
        if args.enable_lift:
            if LiftMotorSrv is None:
                raise RuntimeError(
                    "LiftMotorSrv is not importable. Source /home/agilex/agilex_ws/install/setup.bash "
                    "or run without --enable-lift."
                )
            self.lift_client = self.create_client(LiftMotorSrv, args.lift_service)

        self._subscribe_image("cam_high", args.front_topic)
        self._subscribe_image("cam_left_wrist", args.left_camera_topic)
        self._subscribe_image("cam_right_wrist", args.right_camera_topic)
        self.create_subscription(JointState, args.left_state_topic, self._joint_callback("left"), 10)
        self.create_subscription(JointState, args.right_state_topic, self._joint_callback("right"), 10)

        if args.state_layout == "mobile21":
            self.create_subscription(Odometry, args.odom_topic, self._odom_callback, qos_profile_sensor_data)
        if args.state_layout == "mobile21" or args.enable_lift:
            if LiftMotorMsg is None:
                raise RuntimeError(
                    "LiftMotorMsg is not importable. Source /home/agilex/agilex_ws/install/setup.bash."
                )
            self.create_subscription(LiftMotorMsg, args.lift_state_topic, self._lift_callback, 10)

    def _throttled_error(self, key: str, message: str, interval: float = 2.0) -> None:
        now = time.monotonic()
        if now - self.last_error_times.get(key, 0.0) >= interval:
            self.get_logger().error(message)
            self.last_error_times[key] = now

    def _subscribe_image(self, key: str, topic: str) -> None:
        resolved_topic = resolve_image_topic(topic, self.args.image_transport)
        if self.args.image_transport == "compressed":
            self.create_subscription(
                CompressedImage,
                resolved_topic,
                self._compressed_image_callback(key),
                qos_profile_sensor_data,
            )
        else:
            self.create_subscription(Image, resolved_topic, self._image_callback(key), qos_profile_sensor_data)

    def _image_callback(self, key: str) -> Callable[[Image], None]:
        def callback(msg: Image) -> None:
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                with self.lock:
                    self.images[key] = np.ascontiguousarray(rgb)
                    self.image_times[key] = time.monotonic()
            except Exception as exc:  # noqa: BLE001
                self._throttled_error(key, f"Failed to convert image {key}: {exc}")

        return callback

    def _compressed_image_callback(self, key: str) -> Callable[[CompressedImage], None]:
        def callback(msg: CompressedImage) -> None:
            try:
                encoded = np.frombuffer(msg.data, dtype=np.uint8)
                bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                if bgr is None:
                    raise ValueError("cv2.imdecode returned None")
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                with self.lock:
                    self.images[key] = np.ascontiguousarray(rgb)
                    self.image_times[key] = time.monotonic()
            except Exception as exc:  # noqa: BLE001
                self._throttled_error(key, f"Failed to decode compressed image {key}: {exc}")

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
            for key in IMAGE_KEYS:
                if key not in self.images:
                    missing.append(key)
                elif now - self.image_times[key] > max_age_sec:
                    missing.append(f"{key}:stale")
            for key in JOINT_KEYS:
                if key not in self.joints:
                    missing.append(f"joint_{key}")
                elif now - self.joint_times[key] > max_age_sec:
                    missing.append(f"joint_{key}:stale")
            if self.args.state_layout == "mobile21":
                if self.base_state is None or self.base_time is None:
                    missing.append("odom")
                elif now - self.base_time > max_age_sec:
                    missing.append("odom:stale")
                if self.lift_height is None or self.lift_time is None:
                    missing.append("lift")
                elif now - self.lift_time > max_age_sec:
                    missing.append("lift:stale")
        return missing

    def make_observation(self) -> ObservationSnapshot | None:
        now = time.monotonic()
        with self.lock:
            if any(key not in self.images for key in IMAGE_KEYS):
                return None
            if any(key not in self.joints for key in JOINT_KEYS):
                return None
            if self.args.state_layout == "mobile21" and (
                self.base_state is None or self.base_time is None or self.lift_height is None or self.lift_time is None
            ):
                return None

            images = {key: self.images[key].copy() for key in IMAGE_KEYS}
            image_times = {key: self.image_times[key] for key in IMAGE_KEYS}
            left_feedback = self.joints["left"].copy()
            right_feedback = self.joints["right"].copy()
            joint_times = {key: self.joint_times[key] for key in JOINT_KEYS}
            base_state = self.base_state.copy() if self.base_state is not None else None
            base_time = self.base_time
            lift_height = self.lift_height
            lift_time = self.lift_time

        state = common.compose_policy_state(
            left_feedback,
            right_feedback,
            base_state=base_state,
            lift_height=lift_height,
            state_layout=self.args.state_layout,
            gripper_min=self.args.gripper_min,
            gripper_max=self.args.gripper_max,
            state_gripper_unit=self.args.state_gripper_unit,
        )
        ages = {key: now - image_times[key] for key in IMAGE_KEYS}
        ages.update({f"joint_{key}": now - joint_times[key] for key in JOINT_KEYS})
        if base_time is not None:
            ages["odom"] = now - base_time
        if lift_time is not None:
            ages["lift"] = now - lift_time
        image_shapes = {key: images[key].shape for key in IMAGE_KEYS}

        if self.args.request_format == "base64_json":
            obs = {
                "type": "observation",
                "state": [float(v) for v in state],
                "images": {
                    key: encode_image_base64(images[key], self.args.image_encoding, self.args.jpeg_quality)
                    for key in IMAGE_KEYS
                },
                "prompt": self.args.prompt,
            }
        else:
            obs = {
                "state": state,
                "images": {key: resize_rgb_to_chw(images[key], self.args.image_size) for key in IMAGE_KEYS},
                "prompt": self.args.prompt,
            }
        return ObservationSnapshot(obs, left_feedback, right_feedback, base_state, lift_height, ages, image_shapes)

    def publish_arms(self, left: np.ndarray, right: np.ndarray) -> None:
        if self.left_pub is None or self.right_pub is None:
            return
        stamp = self.get_clock().now().to_msg()
        for pub, command in ((self.left_pub, left), (self.right_pub, right)):
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = list(JOINT_NAMES)
            msg.position = [float(v) for v in command[:7]]
            msg.velocity = [0.0] * 6 + [float(self.args.arm_speed)]
            msg.effort = [0.0] * 6 + [float(self.args.gripper_effort)]
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
            if now - self.last_lift_command_time < self.args.lift_min_period:
                return
            if self.last_lift_command is not None and abs(target - self.last_lift_command) < self.args.lift_min_delta:
                return
        req = LiftMotorSrv.Request()
        req.val = int(round(float(target)))
        req.mode = int(self.args.lift_mode)
        self.lift_client.call_async(req)
        self.last_lift_command = float(target)
        self.last_lift_command_time = now


def wait_for_observation(node: OpenPiAgilexMobileNode, args: argparse.Namespace) -> ObservationSnapshot:
    deadline = time.monotonic() + args.wait_timeout_sec
    last_print = 0.0
    while rclpy.ok() and time.monotonic() < deadline:
        missing = node.missing_or_stale_inputs(args.max_frame_age_sec)
        if not missing:
            snapshot = node.make_observation()
            if snapshot is not None:
                return snapshot
        now = time.monotonic()
        if now - last_print >= 2.0:
            print("Waiting for ROS inputs:", ", ".join(missing) if missing else "snapshot")
            last_print = now
        time.sleep(0.1)
    raise TimeoutError(f"Timed out waiting for ROS inputs. Missing/stale: {node.missing_or_stale_inputs(args.max_frame_age_sec)}")


def print_snapshot(snapshot: ObservationSnapshot) -> None:
    state = np.asarray(snapshot.obs["state"], dtype=np.float32)
    print("ROS inputs ready")
    print(f"  state dim: {state.shape[0]}")
    print(f"  image shapes: {snapshot.image_shapes}")
    print(f"  max input age: {max(snapshot.ages.values()):.3f}s")
    print(f"  left state:  {np.array2string(state[:7], precision=4, suppress_small=True)}")
    print(f"  right state: {np.array2string(state[7:14], precision=4, suppress_small=True)}")
    if snapshot.base_state is not None:
        print(f"  base state:  {np.array2string(snapshot.base_state, precision=4, suppress_small=True)}")
    if snapshot.lift_height is not None:
        print(f"  lift height: {snapshot.lift_height:.3f}")


def publish_smooth(
    node: OpenPiAgilexMobileNode,
    start_left: np.ndarray,
    start_right: np.ndarray,
    target_left: np.ndarray,
    target_right: np.ndarray,
    base_command: np.ndarray | None,
    args: argparse.Namespace,
) -> None:
    steps = max(1, int(round(args.inner_rate_hz / args.control_rate_hz)))
    period = 1.0 / max(args.inner_rate_hz, 1.0)
    for index in range(1, steps + 1):
        alpha = index / steps
        left = start_left + (target_left - start_left) * alpha
        right = start_right + (target_right - start_right) * alpha
        node.publish_arms(left, right)
        node.publish_base(base_command)
        time.sleep(period)


def hold_command(node: OpenPiAgilexMobileNode, left: np.ndarray, right: np.ndarray, repeat: int = 10) -> None:
    for _ in range(repeat):
        node.publish_arms(left, right)
        node.stop_base()
        time.sleep(0.02)


class CompletionMonitor:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.history: list[np.ndarray] = []

    def update(self, left: np.ndarray, right: np.ndarray) -> None:
        self.history.append(np.concatenate([left[:7], right[:7]]).astype(np.float64))
        self.history = self.history[-max(self.args.done_window, 1) :]

    def done_reason(self, executed_steps: int) -> str | None:
        if self.args.completion_mode == "manual":
            return None
        if executed_steps < self.args.done_min_steps or len(self.history) < self.args.done_window:
            return None
        window = np.stack(self.history, axis=0)
        joint_window = np.concatenate([window[:, :6], window[:, 7:13]], axis=1)
        gripper_window = window[:, [6, 13]]
        joint_range = float(np.max(np.ptp(joint_window, axis=0)))
        gripper_range = float(np.max(np.ptp(gripper_window, axis=0)))
        if joint_range <= self.args.done_joint_range and gripper_range <= self.args.done_gripper_range:
            return (
                "stable action window "
                f"(joint_range={joint_range:.4f} rad, gripper_range={gripper_range:.4f} m)"
            )
        return None


def base_commands_from_actions(actions: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    return common.base_commands_from_actions(
        actions,
        base_scale=args.base_scale,
        max_linear=args.max_linear,
        max_angular=args.max_angular,
        base_component_deadband=args.base_component_deadband,
        swap_base_xy=args.swap_base_xy,
        invert_base_x=args.invert_base_x,
        invert_base_y=args.invert_base_y,
        invert_base_wz=args.invert_base_wz,
    )


def selected_base_effectiveness_detail(actions: np.ndarray, chunk: np.ndarray, args: argparse.Namespace) -> tuple[bool, str]:
    selected_base = base_commands_from_actions(chunk, args)
    selected_effective = common.effective_base_mask(
        selected_base,
        base_linear_deadband=args.base_linear_deadband,
        base_angular_deadband=args.base_angular_deadband,
    )
    if bool(np.any(selected_effective)):
        return True, ""

    full_base = base_commands_from_actions(actions, args)
    full_effective = common.effective_base_mask(
        full_base,
        base_linear_deadband=args.base_linear_deadband,
        base_angular_deadband=args.base_angular_deadband,
    )
    peak_index, peak_value = common.base_peak(full_base)
    detail = (
        "selected base commands are below deadband "
        f"(linear>{args.base_linear_deadband:.4f} m/s or "
        f"angular>{args.base_angular_deadband:.4f} rad/s required). "
        f"full_effective={int(full_effective.sum())}/{len(full_effective)}, "
        f"peak_index={peak_index}, "
        f"peak={np.array2string(peak_value, precision=4, suppress_small=True)}"
    )
    return False, detail


def print_action_debug(
    actions: np.ndarray,
    chunk: np.ndarray,
    snapshot: ObservationSnapshot,
    args: argparse.Namespace,
    start_index: int,
    stop_index: int,
) -> None:
    first_left, first_right = common.policy_action_to_piper_commands(
        chunk[0],
        gripper_min=args.gripper_min,
        gripper_max=args.gripper_max,
        action_gripper_unit=args.action_gripper_unit,
    )
    last_left, last_right = common.policy_action_to_piper_commands(
        chunk[-1],
        gripper_min=args.gripper_min,
        gripper_max=args.gripper_max,
        action_gripper_unit=args.action_gripper_unit,
    )
    width = min(common.expected_action_dim(
        args.state_layout,
        enable_base=args.enable_base,
        enable_lift=args.enable_lift,
        expected_action_dim_override=args.expected_action_dim,
    ), actions.shape[1])
    full_action_range = np.ptp(actions[:, :width], axis=0)
    executed_action_range = np.ptp(chunk[:, :width], axis=0)
    first_delta = np.concatenate(
        [
            first_left[:6] - snapshot.left_feedback[:6],
            [first_left[6] - snapshot.left_feedback[6]],
            first_right[:6] - snapshot.right_feedback[:6],
            [first_right[6] - snapshot.right_feedback[6]],
        ]
    )
    print(f"  executing action window: [{start_index}:{stop_index}]")
    print(f"  first target left:  {np.array2string(first_left, precision=4, suppress_small=True)}")
    print(f"  first target right: {np.array2string(first_right, precision=4, suppress_small=True)}")
    print(f"  last target left:   {np.array2string(last_left, precision=4, suppress_small=True)}")
    print(f"  last target right:  {np.array2string(last_right, precision=4, suppress_small=True)}")
    print(f"  first-target minus feedback: {np.array2string(first_delta, precision=4, suppress_small=True)}")
    print(f"  full policy action range: {np.array2string(full_action_range, precision=4, suppress_small=True)}")
    print(f"  executing action range:   {np.array2string(executed_action_range, precision=4, suppress_small=True)}")
    if actions.shape[1] >= 18:
        full_base = base_commands_from_actions(actions, args)
        exec_base = base_commands_from_actions(chunk, args)
        peak_index, peak_value = common.base_peak(full_base)
        raw_full_base = actions[:, common.MOBILE_ACTION_BASE_SLICE]
        raw_exec_base = chunk[:, common.MOBILE_ACTION_BASE_SLICE]
        if (
            args.base_component_deadband > 0.0
            or args.swap_base_xy
            or args.invert_base_x
            or args.invert_base_y
            or args.invert_base_wz
        ):
            raw_peak_index, raw_peak_value = common.base_peak(raw_full_base)
            print(f"  raw executing first base action: {np.array2string(raw_exec_base[0], precision=4, suppress_small=True)}")
            print(f"  raw full policy base peak:      index={raw_peak_index}, value={np.array2string(raw_peak_value, precision=4)}")
            print(
                "  base transform: "
                f"component_deadband={args.base_component_deadband:.4f}, "
                f"swap_xy={args.swap_base_xy}, "
                f"invert_x={args.invert_base_x}, "
                f"invert_y={args.invert_base_y}, "
                f"invert_wz={args.invert_base_wz}"
            )
        print(f"  executing first base cmd: {np.array2string(exec_base[0], precision=4, suppress_small=True)}")
        print(f"  executing last base cmd:  {np.array2string(exec_base[-1], precision=4, suppress_small=True)}")
        print(f"  executing base max abs:   {np.array2string(np.max(np.abs(exec_base), axis=0), precision=4)}")
        print(f"  full policy base max abs: {np.array2string(np.max(np.abs(full_base), axis=0), precision=4)}")
        print(f"  full policy base peak:    index={peak_index}, value={np.array2string(peak_value, precision=4)}")
    if actions.shape[1] >= 18:
        exec_lift = np.asarray(
            [
                common.policy_action_to_lift_target(
                    action,
                    lift_min_height=args.lift_min_height,
                    lift_max_height=args.lift_max_height,
                )
                for action in chunk
            ]
        )
        print(f"  executing lift target first/last: {exec_lift[0]:.3f} / {exec_lift[-1]:.3f}")
        print(f"  executing lift target range:      {float(np.ptp(exec_lift)):.3f}")


def split_host_port_for_tcp(host: str, port: int) -> tuple[str, int]:
    if host.startswith("ws://") or host.startswith("wss://"):
        parsed = urlparse(host)
        return parsed.hostname or "127.0.0.1", int(parsed.port or port)
    return host, int(port)


def tcp_port_open(host: str, port: int, timeout_sec: float = 0.5) -> bool:
    tcp_host, tcp_port = split_host_port_for_tcp(host, port)
    try:
        with socket.create_connection((tcp_host, tcp_port), timeout=timeout_sec):
            return True
    except OSError:
        return False


def configure_policy_websocket_keepalive(args: argparse.Namespace) -> None:
    if args.policy_ping_interval is None and args.policy_ping_timeout is None:
        return

    original_connect = websockets_sync_client.connect
    ping_interval = None if args.policy_ping_interval == 0 else args.policy_ping_interval
    ping_timeout = None if args.policy_ping_timeout == 0 else args.policy_ping_timeout

    def connect_with_keepalive(*connect_args, **kwargs):
        if args.policy_ping_interval is not None:
            kwargs["ping_interval"] = ping_interval
        if args.policy_ping_timeout is not None:
            kwargs["ping_timeout"] = ping_timeout
        return original_connect(*connect_args, **kwargs)

    websockets_sync_client.connect = connect_with_keepalive


class Base64JsonPolicyClient:
    def __init__(self, host: str, port: int, args: argparse.Namespace):
        self.uri = host if host.startswith(("ws://", "wss://")) else f"ws://{host}:{port}"
        self.args = args
        self.metadata: dict = {}
        self.ws = websockets_sync_client.connect(self.uri, compression=None, max_size=None, open_timeout=10.0)
        self._read_metadata()

    def _recv_json(self, timeout: float | None = None) -> dict:
        message = self.ws.recv(timeout=timeout)
        if isinstance(message, bytes):
            message = message.decode("utf-8")
        data = json.loads(message)
        if data.get("type") == "error":
            raise RuntimeError(data.get("error", data))
        return data

    def _read_metadata(self) -> None:
        self.ws.send(json.dumps({"type": "metadata"}))
        data = self._recv_json(timeout=10.0)
        self.metadata = data.get("metadata") if data.get("type") == "metadata" else data
        self.metadata = self.metadata or {}

    def get_server_metadata(self) -> dict:
        return self.metadata

    def infer(self, obs: dict) -> dict:
        self.ws.send(json.dumps(obs, ensure_ascii=False))
        data = self._recv_json()
        if "actions" not in data:
            raise RuntimeError(f"Policy response has no actions: {data}")
        return data


def connect_policy(args: argparse.Namespace):
    configure_policy_websocket_keepalive(args)
    deadline = time.monotonic() + args.policy_wait_timeout_sec
    last_print = 0.0
    last_error = ""
    while rclpy.ok() and time.monotonic() < deadline:
        now = time.monotonic()
        if not tcp_port_open(args.host, args.port):
            if now - last_print >= 5.0:
                print(f"Waiting for OpenPI policy server port {args.host}:{args.port} ...")
                last_print = now
            time.sleep(1.0)
            continue

        try:
            if args.request_format == "base64_json":
                return Base64JsonPolicyClient(args.host, args.port, args)
            if args.host.startswith(("ws://", "wss://")):
                return websocket_client_policy.WebsocketClientPolicy(host=args.host, port=None)
            return websocket_client_policy.WebsocketClientPolicy(host=args.host, port=args.port)
        except Exception as exc:  # noqa: BLE001
            last_error = f"{type(exc).__name__}: {exc}"
            if now - last_print >= 5.0:
                print(f"Policy server is not ready yet: {last_error}")
                last_print = now
            time.sleep(2.0)

    raise TimeoutError(f"Timed out waiting for OpenPI policy server. Last error: {last_error}")


def infer_with_command_keepalive(
    policy,
    request: dict,
    node: OpenPiAgilexMobileNode,
    args: argparse.Namespace,
    hold_left: np.ndarray | None,
    hold_right: np.ndarray | None,
    hold_base: np.ndarray | None,
) -> dict:
    result_box: dict[str, dict] = {}
    error_box: dict[str, BaseException] = {}

    def run_infer() -> None:
        try:
            result_box["result"] = policy.infer(request)
        except BaseException as exc:  # noqa: BLE001
            error_box["error"] = exc

    infer_thread = threading.Thread(target=run_infer, daemon=True)
    infer_thread.start()

    start = time.monotonic()
    period = 1.0 / max(args.inner_rate_hz, 1.0)
    while infer_thread.is_alive():
        if args.keepalive_during_infer and not args.dry_run and hold_left is not None and hold_right is not None:
            node.publish_arms(hold_left, hold_right)
            if hold_base is not None:
                elapsed = time.monotonic() - start
                if args.infer_keepalive_max_sec <= 0 or elapsed <= args.infer_keepalive_max_sec:
                    node.publish_base(hold_base)
                else:
                    node.stop_base()
        time.sleep(period)

    infer_thread.join()
    if "error" in error_box:
        raise error_box["error"]
    return result_box["result"]


def validate_cli(args: argparse.Namespace) -> int:
    if args.execute_horizon <= 0:
        raise SystemExit("--execute-horizon must be positive.")
    if args.action_start_index < 0:
        raise SystemExit("--action-start-index must be non-negative.")
    if args.control_rate_hz <= 0 or args.inner_rate_hz <= 0:
        raise SystemExit("--control-rate-hz and --inner-rate-hz must be positive.")
    if args.gripper_max <= args.gripper_min:
        raise SystemExit("--gripper-max must be greater than --gripper-min.")
    if args.done_window <= 1:
        raise SystemExit("--done-window must be greater than 1.")
    if args.base_linear_deadband < 0 or args.base_angular_deadband < 0:
        raise SystemExit("--base-linear-deadband and --base-angular-deadband must be non-negative.")
    if args.expected_action_dim < 0:
        raise SystemExit("--expected-action-dim must be non-negative.")
    if args.lift_min_height is not None and args.lift_max_height is not None and args.lift_min_height > args.lift_max_height:
        raise SystemExit("--lift-min-height must be <= --lift-max-height.")

    action_width = common.expected_action_dim(
        args.state_layout,
        enable_base=args.enable_base,
        enable_lift=args.enable_lift,
        expected_action_dim_override=args.expected_action_dim,
    )
    if args.enable_base and action_width < 18:
        raise SystemExit("--enable-base requires actions with at least 18 values.")
    if args.enable_lift and action_width < 18:
        raise SystemExit("--enable-lift requires actions with at least 18 values.")
    return action_width


def main() -> int:
    args = parse_args()
    action_width = validate_cli(args)

    rclpy.init()
    node = OpenPiAgilexMobileNode(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    last_left: np.ndarray | None = None
    last_right: np.ndarray | None = None
    last_base_command: np.ndarray | None = None

    try:
        snapshot = wait_for_observation(node, args)
        print_snapshot(snapshot)
        last_left = snapshot.left_feedback.astype(np.float64)
        last_right = snapshot.right_feedback.astype(np.float64)

        if args.check_inputs_only:
            return 0

        if args.enable_lift and not args.dry_run and not node.wait_for_lift_service(args.wait_timeout_sec):
            raise RuntimeError(f"Lift service is not available: {args.lift_service}")

        print(f"Connecting to OpenPI policy server {args.host}:{args.port} ...")
        policy = connect_policy(args)
        metadata = policy.get_server_metadata()
        print(f"Connected. Server metadata keys: {sorted(metadata)}" if metadata else "Connected.")

        if not args.yes and not args.dry_run:
            print("\nAbout to publish OpenPI actions to the real robot:")
            print(f"  arms={args.enable_arms}, base={args.enable_base}, lift={args.enable_lift}")
            print(f"  state_layout={args.state_layout}, expected_action_dim>={action_width}")
            print(f"  prompt={args.prompt!r}")
            input("Press Enter to start, or Ctrl+C to abort. ")

        if not args.dry_run:
            hold_command(node, last_left, last_right, repeat=5)

        start_time = time.monotonic()
        executed = 0
        chunk_index = 0
        stop_reason = None
        completion_monitor = CompletionMonitor(args)
        unlimited_steps = args.max_steps <= 0
        unlimited_runtime = args.max_runtime_sec <= 0

        while rclpy.ok() and stop_reason is None:
            if not unlimited_steps and executed >= args.max_steps:
                stop_reason = f"max steps {args.max_steps}"
                break
            if not unlimited_runtime and time.monotonic() - start_time >= args.max_runtime_sec:
                stop_reason = f"max runtime {args.max_runtime_sec:.1f}s"
                break

            snapshot = wait_for_observation(node, args)
            inference_start = time.monotonic()
            result = infer_with_command_keepalive(
                policy,
                snapshot.obs,
                node,
                args,
                last_left,
                last_right,
                last_base_command,
            )
            actions = common.validate_actions(result["actions"], args.reject_joint_abs, action_width)
            inference_dt = time.monotonic() - inference_start
            start_index = min(args.action_start_index, actions.shape[0] - 1)
            stop_index = min(start_index + args.execute_horizon, actions.shape[0])
            chunk = actions[start_index:stop_index]
            print(
                f"Chunk {chunk_index}: got {actions.shape[0]} actions, "
                f"executing [{start_index}:{stop_index}], infer {inference_dt:.3f}s"
            )

            if args.debug_actions:
                print_action_debug(actions, chunk, snapshot, args, start_index, stop_index)
            if args.enable_base:
                effective, detail = selected_base_effectiveness_detail(actions, chunk, args)
                if not effective:
                    if args.require_effective_base:
                        raise RuntimeError(detail)
                    if args.base_deadband_warning:
                        print(f"WARN: {detail}")

            for action in chunk:
                if not unlimited_steps and executed >= args.max_steps:
                    stop_reason = f"max steps {args.max_steps}"
                    break
                if not unlimited_runtime and time.monotonic() - start_time >= args.max_runtime_sec:
                    stop_reason = f"max runtime {args.max_runtime_sec:.1f}s"
                    break

                target_left, target_right = common.policy_action_to_piper_commands(
                    action,
                    gripper_min=args.gripper_min,
                    gripper_max=args.gripper_max,
                    action_gripper_unit=args.action_gripper_unit,
                )
                next_left, next_right = common.clamp_command_step(
                    target_left,
                    target_right,
                    last_left,
                    last_right,
                    max_joint_step=args.max_joint_step,
                    max_gripper_step=args.max_gripper_step,
                    gripper_max=args.gripper_max,
                )
                base_command = (
                    common.policy_action_to_base_command(
                        action,
                        base_scale=args.base_scale,
                        max_linear=args.max_linear,
                        max_angular=args.max_angular,
                        base_component_deadband=args.base_component_deadband,
                        swap_base_xy=args.swap_base_xy,
                        invert_base_x=args.invert_base_x,
                        invert_base_y=args.invert_base_y,
                        invert_base_wz=args.invert_base_wz,
                    )
                    if args.enable_base
                    else None
                )
                lift_target = (
                    common.policy_action_to_lift_target(
                        action,
                        lift_min_height=args.lift_min_height,
                        lift_max_height=args.lift_max_height,
                    )
                    if args.enable_lift
                    else None
                )

                if args.dry_run:
                    time.sleep(1.0 / args.control_rate_hz)
                else:
                    if lift_target is not None:
                        node.send_lift_height(lift_target)
                    publish_smooth(node, last_left, last_right, next_left, next_right, base_command, args)

                last_left, last_right = next_left, next_right
                last_base_command = base_command
                executed += 1
                completion_monitor.update(last_left, last_right)
                if args.log_every > 0 and executed % args.log_every == 0:
                    suffix = ""
                    if base_command is not None:
                        suffix += f" base={np.array2string(base_command, precision=3, suppress_small=True)}"
                    if lift_target is not None:
                        suffix += f" lift={lift_target:.2f}"
                    print(
                        f"  step {executed}: left={np.array2string(last_left, precision=3, suppress_small=True)} "
                        f"right={np.array2string(last_right, precision=3, suppress_small=True)}{suffix}"
                    )
                stop_reason = completion_monitor.done_reason(executed)
                if stop_reason is not None:
                    break
            chunk_index += 1

        if stop_reason is None:
            stop_reason = "rclpy stopped"
        print(f"Finished after {executed} executed actions: {stop_reason}.")
        return 0

    except KeyboardInterrupt:
        print("Interrupted by user.")
        return 130
    except (TimeoutError, ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}")
        return 1
    finally:
        if not args.dry_run:
            try:
                node.stop_base()
            except Exception as exc:  # noqa: BLE001
                print(f"WARN: stop-base publish failed: {exc}")
        if args.hold_on_exit and not args.dry_run and last_left is not None and last_right is not None:
            try:
                hold_command(node, last_left, last_right, repeat=10)
            except Exception as exc:  # noqa: BLE001
                print(f"WARN: hold-on-exit publish failed: {exc}")
        try:
            executor.shutdown()
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    raise SystemExit(main())
