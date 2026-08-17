#!/usr/bin/env python3
"""Best-effort software emergency stop for AgileX Piper/Ranger ROS2 setup."""

import argparse
import os
import sys
import time
from typing import Iterable, List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from piper_msgs.srv import Enable
except Exception:  # pragma: no cover - depends on sourced ROS workspace
    Enable = None


DEFAULT_CAN_CANDIDATES = ["can_left", "can_right", "can0", "can1"]


class EmergencyStopNode(Node):
    def __init__(self, enable_topic: str, cmd_vel_topics: Iterable[str]):
        super().__init__("piper_emergency_stop")
        self.enable_pub = self.create_publisher(Bool, enable_topic, 10)
        self.cmd_vel_pubs = [
            self.create_publisher(Twist, topic.strip(), 10)
            for topic in cmd_vel_topics
            if topic.strip()
        ]
        self.enable_client = None
        if Enable is not None:
            self.enable_client = self.create_client(Enable, "/enable_srv")

    def pulse_stop(self, repeat: int, rate_hz: float) -> None:
        period = 1.0 / max(rate_hz, 1.0)
        disable = Bool()
        disable.data = False
        zero_twist = Twist()

        for _ in range(max(repeat, 1)):
            self.enable_pub.publish(disable)
            for pub in self.cmd_vel_pubs:
                pub.publish(zero_twist)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def call_disable_service(self, timeout: float) -> bool:
        if self.enable_client is None:
            return False
        if not self.enable_client.wait_for_service(timeout_sec=timeout):
            return False
        request = Enable.Request()
        request.enable_request = False
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.done()


def existing_can_ports() -> List[str]:
    net_dir = "/sys/class/net"
    if not os.path.isdir(net_dir):
        return []
    return sorted(
        name
        for name in os.listdir(net_dir)
        if name.startswith("can") or name in {"can_left", "can_right"}
    )


def parse_can_ports(value: str) -> List[str]:
    if value == "auto":
        detected = existing_can_ports()
        preferred = [port for port in DEFAULT_CAN_CANDIDATES if port in detected]
        extra = [port for port in detected if port not in preferred]
        return preferred + extra
    return [port.strip() for port in value.split(",") if port.strip()]


def direct_can_disable(can_ports: Iterable[str], repeat: int) -> bool:
    ports = list(can_ports)
    if not ports:
        print("Direct CAN: no CAN ports detected.")
        return False

    try:
        from piper_sdk import C_PiperInterface
    except Exception as exc:
        print(f"Direct CAN: piper_sdk import failed: {exc}")
        return False

    ok = False
    for port in ports:
        print(f"Direct CAN: disabling arm on {port} ...")
        try:
            piper = C_PiperInterface(can_name=port)
            piper.ConnectPort()
            for _ in range(max(repeat, 1)):
                piper.DisableArm(7)
                try:
                    piper.GripperCtrl(0, 1000, 0x00, 0)
                except Exception:
                    pass
                time.sleep(0.02)
            ok = True
        except Exception as exc:
            print(f"Direct CAN: {port} failed: {exc}")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--enable-topic", default="/enable_flag")
    parser.add_argument(
        "--cmd-vel-topics",
        default="/cmd_vel",
        help="Comma-separated base velocity topics that should receive zero Twist.",
    )
    parser.add_argument("--repeat", type=int, default=20, help="ROS stop pulse count.")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="ROS stop pulse rate.")
    parser.add_argument("--service-timeout", type=float, default=0.5)
    parser.add_argument("--no-service", action="store_true")
    parser.add_argument(
        "--can-ports",
        default="auto",
        help="Comma-separated CAN ports for direct Piper disable, or auto.",
    )
    parser.add_argument("--no-direct-can", action="store_true")
    parser.add_argument("--can-repeat", type=int, default=5)
    args = parser.parse_args()

    ros_ok = False
    service_ok = False
    can_ok = False

    rclpy.init()
    node = EmergencyStopNode(args.enable_topic, args.cmd_vel_topics.split(","))
    try:
        print("ROS: publishing disable and zero velocity pulses ...")
        node.pulse_stop(args.repeat, args.rate_hz)
        ros_ok = True

        if not args.no_service:
            print("ROS: calling /enable_srv enable_request=false ...")
            service_ok = node.call_disable_service(args.service_timeout)
            if not service_ok:
                print("ROS: /enable_srv unavailable or timed out; topic stop was still published.")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not args.no_direct_can:
        ports = parse_can_ports(args.can_ports)
        can_ok = direct_can_disable(ports, args.can_repeat)

    if ros_ok or service_ok or can_ok:
        print("Emergency stop command sent.")
        print("Use the physical E-stop / power cut for immediate safety-critical stopping.")
        return 0

    print("ERROR: no stop path completed.")
    return 2


if __name__ == "__main__":
    sys.exit(main())
