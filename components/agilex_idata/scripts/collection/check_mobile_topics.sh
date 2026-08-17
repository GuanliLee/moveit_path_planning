#!/bin/bash
# Concise preflight check for mobile ALOHA collection.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if command -v proxy_off >/dev/null 2>&1; then
    proxy_off >/dev/null 2>&1 || true
fi
unset http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY all_proxy ALL_PROXY

source_if_exists() {
    local setup_file="$1"
    if [ -f "${setup_file}" ]; then
        # shellcheck disable=SC1090
        source "${setup_file}"
    fi
}

export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
DATA_ROS_WS="${DATA_ROS_WS:-/home/caizj/agilex_idata/ros2_ws}"
WITH_BASE="${WITH_BASE:-1}"
TOPIC_SAMPLE_TIMEOUT="${TOPIC_SAMPLE_TIMEOUT:-10}"

source /opt/ros/humble/setup.bash
source_if_exists /home/agilex/agilex_ws/install/setup.bash
source_if_exists /home/agilex/camera_ros/install/setup.bash
source_if_exists /home/agilex/piper_ros/install/setup.bash
source_if_exists /home/agilex/data_ros/install/setup.bash
source_if_exists "${DATA_ROS_WS}/install/setup.bash"

export WITH_BASE TOPIC_SAMPLE_TIMEOUT

python3 - <<'PY'
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Callable

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import String

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


def is_truthy(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def finite_numbers(values) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in values)


def lift_value(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    if isinstance(value, str):
        text = value.strip()
        if not text:
            raise ValueError("empty string")
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            return float(text)
        return lift_value(parsed)
    if isinstance(value, dict):
        for key in ("targetHeight", "target_height", "val", "height", "backHeight", "back_height", "data"):
            if key in value and value[key] is not None:
                return lift_value(value[key])
    raise ValueError(f"unsupported lifting payload: {value!r}")


def chassis_value(text: str):
    data = json.loads(text)
    if "linear" in data or "angular" in data:
        linear = data.get("linear", {})
        angular = data.get("angular", {})
        values = [linear["x"], linear.get("y", 0.0), angular["z"]]
    else:
        values = [
            data.get("linearX", data.get("linear_x")),
            data.get("linearY", data.get("linear_y")),
            data.get("angularZ", data.get("angular_z")),
        ]
    values = [float(v) for v in values]
    if not finite_numbers(values):
        raise ValueError("non-finite chassis command")
    return values


def check_image(msg: Image) -> str:
    if msg.width <= 0 or msg.height <= 0:
        raise ValueError("width/height empty")
    if len(msg.data) == 0:
        raise ValueError("image data empty")
    return f"{msg.width}x{msg.height}"


def check_compressed_image(msg: CompressedImage) -> str:
    if len(msg.data) == 0:
        raise ValueError("compressed image data empty")
    return msg.format or "compressed image"


def check_joint(msg: JointState) -> str:
    values = list(msg.position)
    if not values:
        raise ValueError("position empty")
    if not finite_numbers(values):
        raise ValueError("position has non-finite value")
    return f"position[{len(values)}]"


def check_pose(msg: PoseStamped) -> str:
    p = msg.pose.position
    q = msg.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    if not finite_numbers(values):
        raise ValueError("pose has non-finite value")
    return "pose"


def check_odom(msg: Odometry) -> str:
    p = msg.pose.pose.position
    t = msg.twist.twist
    values = [p.x, p.y, p.z, t.linear.x, t.linear.y, t.angular.z]
    if not finite_numbers(values):
        raise ValueError("odom has non-finite value")
    return "pose/twist"


def check_chassis(msg: String) -> str:
    chassis_value(msg.data)
    return "linearX/linearY/angularZ"


def check_lift_action(msg: String) -> str:
    value = lift_value(msg.data)
    if not math.isfinite(value):
        raise ValueError("non-finite lifting action")
    return "height"


def check_lift_state(msg) -> str:
    value = getattr(msg, "back_height", None)
    if value is None:
        value = getattr(msg, "backHeight", None)
    if value is None:
        raise ValueError("back_height empty")
    float(value)
    return "back_height"


@dataclass
class TopicCheck:
    topic: str
    msg_type: object
    expected_type: str
    value_name: str
    validator: Callable[[object], str]


class TopicProbe(Node):
    def __init__(self, checks: list[TopicCheck]):
        super().__init__("check_mobile_required_values")
        self.checks = checks
        self.ok: dict[str, str] = {}
        self.errors: dict[str, str] = {}
        self.subscriptions_ = []
        for check in checks:
            self.subscriptions_.append(
                self.create_subscription(
                    check.msg_type,
                    check.topic,
                    lambda msg, item=check: self._on_msg(item, msg),
                    10,
                )
            )

    def _on_msg(self, check: TopicCheck, msg) -> None:
        if check.topic in self.ok:
            return
        try:
            self.ok[check.topic] = check.validator(msg)
        except Exception as exc:  # noqa: BLE001 - runtime payloads are best-effort.
            self.errors[check.topic] = str(exc)


def service_available(node: Node, timeout: float) -> tuple[bool, str]:
    if LiftMotorSrv is None:
        return False, "缺少 LiftMotorSrv 类型"
    client = node.create_client(LiftMotorSrv, "/LiftingMotorService")
    if client.wait_for_service(timeout_sec=timeout):
        return True, ""
    return False, "服务不可用或超时"


def missing_reason(node: Node, check: TopicCheck, timeout: float) -> str:
    names_and_types = dict(node.get_topic_names_and_types())
    if check.topic not in names_and_types:
        return f"topic 不存在，没有读到 {check.value_name}"
    types = names_and_types.get(check.topic, [])
    if check.expected_type not in types:
        return f"类型不是 {check.expected_type}，没有读到 {check.value_name}"
    if check.topic in node.errors:
        return f"读到消息但 {check.value_name} 无效：{node.errors[check.topic]}"
    return f"{timeout:g}s 内没有读到 {check.value_name}"


def main() -> int:
    timeout = float(os.environ.get("TOPIC_SAMPLE_TIMEOUT", "3"))
    with_base = is_truthy(os.environ.get("WITH_BASE", "1"))

    mobile_camera_checks = [
        TopicCheck("/camera_h/color/image_raw", Image, "sensor_msgs/msg/Image", "图像数据", check_image),
        TopicCheck("/camera_l/color/image_raw/compressed", CompressedImage, "sensor_msgs/msg/CompressedImage", "压缩图像数据", check_compressed_image),
        TopicCheck("/camera_f/color/image_raw/compressed", CompressedImage, "sensor_msgs/msg/CompressedImage", "压缩图像数据", check_compressed_image),
        TopicCheck("/camera_r/color/image_raw/compressed", CompressedImage, "sensor_msgs/msg/CompressedImage", "压缩图像数据", check_compressed_image),
        TopicCheck("/camera_h/color/image_raw/compressed", CompressedImage, "sensor_msgs/msg/CompressedImage", "压缩图像数据", check_compressed_image),
    ]
    checks = [
        TopicCheck("/camera_f/color/image_raw", Image, "sensor_msgs/msg/Image", "图像数据", check_image),
        TopicCheck("/camera_l/color/image_raw", Image, "sensor_msgs/msg/Image", "图像数据", check_image),
        TopicCheck("/camera_r/color/image_raw", Image, "sensor_msgs/msg/Image", "图像数据", check_image),
        TopicCheck("/master/joint_left", JointState, "sensor_msgs/msg/JointState", "position 数值", check_joint),
        TopicCheck("/master/joint_right", JointState, "sensor_msgs/msg/JointState", "position 数值", check_joint),
        TopicCheck("/puppet/joint_left", JointState, "sensor_msgs/msg/JointState", "position 数值", check_joint),
        TopicCheck("/puppet/joint_right", JointState, "sensor_msgs/msg/JointState", "position 数值", check_joint),
        TopicCheck("/puppet/end_pose_left", PoseStamped, "geometry_msgs/msg/PoseStamped", "pose 数值", check_pose),
        TopicCheck("/puppet/end_pose_right", PoseStamped, "geometry_msgs/msg/PoseStamped", "pose 数值", check_pose),
    ]
    if with_base:
        checks.extend(mobile_camera_checks)
        checks.extend(
            [
                TopicCheck("/localization/pose", PoseStamped, "geometry_msgs/msg/PoseStamped", "定位 pose 数值", check_pose),
                TopicCheck("/odom", Odometry, "nav_msgs/msg/Odometry", "里程计数值", check_odom),
                TopicCheck("/action/chassis", String, "std_msgs/msg/String", "linearX/linearY/angularZ", check_chassis),
                TopicCheck("/action/lifting", String, "std_msgs/msg/String", "升降目标高度", check_lift_action),
            ]
        )
        if LiftMotorMsg is not None:
            lift_msg_type = "bt_task_msgs/msg/LiftMotorMsg" if LiftMotorMsg.__module__.startswith("bt_task_msgs.") else "lifting_msg_pkg/msg/LiftMotorMsg"
            checks.append(
                TopicCheck("/LiftMotorStatePub", LiftMotorMsg, lift_msg_type, "back_height", check_lift_state)
            )

    rclpy.init()
    node = TopicProbe(checks)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if len(node.ok) == len(checks):
                break
            rclpy.spin_once(node, timeout_sec=0.05)

        failures: list[str] = []
        for check in checks:
            if check.topic not in node.ok:
                failures.append(f"MISSING: {check.topic} - {missing_reason(node, check, timeout)}")

        if with_base and LiftMotorMsg is None:
            failures.append("MISSING: /LiftMotorStatePub - 缺少 LiftMotorMsg 类型，无法读取 back_height")

        if with_base:
            ok, reason = service_available(node, timeout)
            if not ok:
                failures.append(f"MISSING: /LiftingMotorService - {reason}")

        if failures:
            print("自检失败：以下位置没有读到数值")
            for line in failures:
                print(line)
            print(f"FAILED: {len(failures)} missing value(s)")
            return 1

        print("自检通过：所有必需项都读到数值")
        print("OK: all required values received")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
PY

if [[ "${WITH_BASE,,}" =~ ^(1|true|yes|on)$ ]]; then
    timestamp_probe_ok=0
    timestamp_probe_attempts="${CAMERA_TIMESTAMP_PROBE_ATTEMPTS:-1}"
    for ((attempt = 1; attempt <= timestamp_probe_attempts; attempt++)); do
        if python3 "${SCRIPT_DIR}/camera_timestamp_health.py" \
            --sample-seconds "${CAMERA_TIMESTAMP_SAMPLE_SECONDS:-3}" \
            --discovery-timeout-seconds "${CAMERA_TIMESTAMP_DISCOVERY_TIMEOUT_SECONDS:-3}" \
            --min-samples-per-topic "${CAMERA_TIMESTAMP_MIN_SAMPLES_PER_TOPIC:-30}" \
            --max-age-ms "${CAMERA_MAX_HEADER_AGE_MS:-70}" \
            --max-spread-ms "${CAMERA_MAX_HEADER_SPREAD_MS:-60}"; then
            timestamp_probe_ok=1
            break
        fi
        if [ "${attempt}" -lt "${timestamp_probe_attempts}" ]; then
            echo "相机时间戳自检第 ${attempt} 次未通过，重新采样" >&2
        fi
    done
    if [ "${timestamp_probe_ok}" -ne 1 ]; then
        echo "相机时间戳自检持续失败，拒绝开始采集" >&2
        exit 1
    fi
fi
