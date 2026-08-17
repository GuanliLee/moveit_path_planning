#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node


class TouchPointCapture(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("checkerboard_touch_point_capture")
        self.pose: Optional[Pose] = None
        self.sub = self.create_subscription(Pose, topic, self._on_pose, 10)

    def _on_pose(self, msg: Pose) -> None:
        self.pose = msg


def pose_payload(msg: Pose) -> dict[str, object]:
    return {
        "position": [
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        ],
        "orientation_xyzw": [
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", required=True, choices=["P0", "P1", "P2"])
    parser.add_argument("--col", type=int, required=True, help="checkerboard inner-corner column index")
    parser.add_argument("--row", type=int, required=True, help="checkerboard inner-corner row index")
    parser.add_argument("--topic", default="/left/gripper_end_pos")
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument(
        "--out",
        default="/home/ligl/agilex_xpc/calibration/touch_points/checkerboard_touch_points.json",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = TouchPointCapture(args.topic)
    deadline = time.monotonic() + max(args.timeout_sec, 1.0)
    try:
        while rclpy.ok() and node.pose is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.pose is None:
            raise RuntimeError(f"no Pose received from {args.topic} within {args.timeout_sec:.1f}s")

        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        if out_path.exists():
            data = json.loads(out_path.read_text(encoding="utf-8"))
        else:
            data = {"points": {}}
        data["topic"] = args.topic
        data["updated_at"] = time.time()
        data.setdefault("points", {})
        data["points"][args.name] = {
            "name": args.name,
            "col": int(args.col),
            "row": int(args.row),
            "captured_at": time.time(),
            **pose_payload(node.pose),
        }
        out_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        point = data["points"][args.name]
        print(json.dumps(point, indent=2))
        print(f"saved: {out_path}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
