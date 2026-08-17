#!/usr/bin/env bash
set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-66}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "== USB RealSense devices =="
lsusb | grep -i "RealSense" || true

echo
echo "== ROS camera nodes =="
ros2 node list | grep -E "^/cam_(high|left|right)$" || true

echo
echo "== RGB compressed publishers =="
for camera in cam_high cam_left cam_right; do
  topic="/${camera}/color/image_raw/compressed"
  echo "-- ${topic}"
  ros2 topic info "${topic}" || true
done

echo
echo "== Depth publishers =="
for camera in cam_high cam_left cam_right; do
  topic="/${camera}/depth/image_rect_raw"
  echo "-- ${topic}"
  ros2 topic info "${topic}" || true
done

echo
echo "== 5-second RGB frame count =="
python3 - <<'PY'
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

TOPICS = {
    "cam_high": "/cam_high/color/image_raw/compressed",
    "cam_left": "/cam_left/color/image_raw/compressed",
    "cam_right": "/cam_right/color/image_raw/compressed",
}


class Counter(Node):
    def __init__(self):
        super().__init__("three_camera_rgb_counter")
        self.counts = {name: 0 for name in TOPICS}
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subs = [
            self.create_subscription(
                CompressedImage,
                topic,
                lambda _msg, camera=name: self._count(camera),
                qos,
            )
            for name, topic in TOPICS.items()
        ]

    def _count(self, camera):
        self.counts[camera] += 1


rclpy.init()
node = Counter()
deadline = time.time() + 5.0
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

for name in ("cam_high", "cam_left", "cam_right"):
    print(f"{name}: {node.counts[name]} frames")

node.destroy_node()
rclpy.shutdown()
PY
