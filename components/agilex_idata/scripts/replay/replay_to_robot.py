#!/usr/bin/env python3
"""
把采集的 HDF5 数据回放到 Piper 真机上（双臂）。

工作原理:
    HDF5 里 arm/jointStatePosition/masterLeft (N, 7) 是人操作主臂时的关节指令轨迹。
    这个脚本把这个轨迹按帧率发布到 /joint_left_states 和 /joint_right_states，
    piper_single_ctrl 节点会接收并通过 CAN 发指令给真机执行。

安全机制:
    1. 启动后会打印第一帧目标关节角，要求人按回车确认
    2. 先用 5 秒慢速线性插值，把机械臂从当前姿态移到第一帧的初始姿态
    3. 默认 0.5x 速度回放（可用 --rate 调整）
    4. 任何时候 Ctrl+C 可立即停止
    5. 回放结束后保持最后一帧 1 秒，便于观察

前置条件:
    1. piper_ros 已启动: bash /home/agilex/piper_ros/start_multi_piper.sh
    2. CAN 已激活, 机械臂供电正常
    3. 工作空间周围清空, 不要有障碍物或人体

用法:
    python3 replay_to_robot.py /home/agilex/data/aloha/episode1/episode1.hdf5
    python3 replay_to_robot.py /home/agilex/data/aloha/episode1/episode1.hdf5 --rate 0.3 --source master
"""
import argparse
import os
import sys
import time

import h5py
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ReplayNode(Node):
    def __init__(self):
        super().__init__('aloha_replay_to_robot')
        self.pub_left = self.create_publisher(JointState, '/joint_left_states', 10)
        self.pub_right = self.create_publisher(JointState, '/joint_right_states', 10)
        self.sub_left = self.create_subscription(
            JointState, '/puppet/joint_left',
            lambda m: setattr(self, 'cur_left', np.array(m.position[:7])), 10)
        self.sub_right = self.create_subscription(
            JointState, '/puppet/joint_right',
            lambda m: setattr(self, 'cur_right', np.array(m.position[:7])), 10)
        self.cur_left = None
        self.cur_right = None

    def wait_for_current_state(self, timeout: float = 5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.cur_left is not None and self.cur_right is not None:
                return True
        return False

    def publish(self, left: np.ndarray, right: np.ndarray, stamp_msg, vel_pct: float = 80.0):
        # piper_single_ctrl 的 joint_callback 用 velocity[6] 作为速度百分比 (30~100)
        # 7 个关节其他位的 velocity 没用, 但保持 lens==7 才能命中分支
        vel = [0.0] * 6 + [float(vel_pct)]

        msg_l = JointState()
        msg_l.header.stamp = stamp_msg
        msg_l.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        msg_l.position = [float(x) for x in left]
        msg_l.velocity = list(vel)
        msg_l.effort = [0.0] * 7

        msg_r = JointState()
        msg_r.header.stamp = stamp_msg
        msg_r.name = list(msg_l.name)
        msg_r.position = [float(x) for x in right]
        msg_r.velocity = list(vel)
        msg_r.effort = [0.0] * 7

        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)


def smooth_move(node: ReplayNode, target_l: np.ndarray, target_r: np.ndarray,
                duration: float = 5.0, vel_pct: float = 40.0):
    print(f"\n>>> 缓慢移动到初始姿态 (耗时 {duration:.1f}s, 速度 {vel_pct:.0f}%) ...")
    rate_hz = 20.0
    n_steps = max(int(duration * rate_hz), 2)
    start_l = node.cur_left.copy()
    start_r = node.cur_right.copy()
    for k in range(n_steps + 1):
        alpha = k / n_steps
        # ease-in-out 平滑插值
        s = 0.5 - 0.5 * np.cos(np.pi * alpha)
        cmd_l = start_l * (1 - s) + target_l * s
        cmd_r = start_r * (1 - s) + target_r * s
        node.publish(cmd_l, cmd_r, node.get_clock().now().to_msg(), vel_pct=vel_pct)
        time.sleep(1.0 / rate_hz)
    print(">>> 已到达初始姿态")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hdf5", help="HDF5 文件路径")
    parser.add_argument("--rate", type=float, default=1.0,
                        help="回放倍速 (默认 1.0; 卡顿时试 0.5)")
    parser.add_argument("--source", choices=["master", "puppet"], default="master",
                        help="使用主臂(master)还是从臂(puppet)轨迹作为指令; 默认 master")
    parser.add_argument("--skip", type=int, default=1,
                        help="每隔几帧发一次 (默认 1 => 全帧发送, 实测约 30Hz)")
    parser.add_argument("--vel", type=float, default=80.0,
                        help="伺服跟踪速度百分比 (30~100, 默认 80)")
    parser.add_argument("--init-vel", type=float, default=40.0,
                        help="移动到初始姿态时的速度百分比 (默认 40, 比正式回放慢)")
    parser.add_argument("--init-duration", type=float, default=5.0,
                        help="移动到初始姿态的耗时 (秒, 默认 5)")
    parser.add_argument("--no-confirm", action="store_true",
                        help="跳过人工确认 (危险, 不推荐)")
    args = parser.parse_args()

    print(f"加载 HDF5: {args.hdf5}")
    with h5py.File(args.hdf5, 'r') as f:
        size = int(f['size'][()])
        ts = f['timestamp'][()]
        traj_l = f[f'arm/jointStatePosition/{args.source}Left'][()]
        traj_r = f[f'arm/jointStatePosition/{args.source}Right'][()]

    print(f"  episode 长度: {size} 帧, 时长 {ts[-1] - ts[0]:.1f}s")
    print(f"  轨迹源: {args.source}  (left {traj_l.shape}, right {traj_r.shape})")
    print(f"  回放倍速: {args.rate}x, 降频: 每 {args.skip} 帧发一次")

    print("\n>>> 第一帧目标关节角:")
    print(f"  Left  : {np.array2string(traj_l[0], precision=3, suppress_small=True)}")
    print(f"  Right : {np.array2string(traj_r[0], precision=3, suppress_small=True)}")
    print(f"\n>>> 最后一帧关节角:")
    print(f"  Left  : {np.array2string(traj_l[-1], precision=3, suppress_small=True)}")
    print(f"  Right : {np.array2string(traj_r[-1], precision=3, suppress_small=True)}")

    print("\n!!! 安全提示 !!!")
    print("  - 确认机械臂周围无人无障碍物")
    print("  - 确认 piper_ros 已启动 (bash /home/agilex/piper_ros/start_multi_piper.sh)")
    print("  - 准备好用 Ctrl+C 立即中断")
    if not args.no_confirm:
        ans = input("\n按回车确认开始 (输入 q 取消): ").strip().lower()
        if ans == 'q':
            print("已取消")
            return

    rclpy.init()
    node = ReplayNode()

    try:
        print("\n>>> 等待读取当前机械臂位置 ...")
        if not node.wait_for_current_state():
            print("[错误] 在 5 秒内未收到 /puppet/joint_left 或 /puppet/joint_right")
            print("       请确认 piper_ros 已启动")
            return
        print(f"  当前 Left : {np.array2string(node.cur_left, precision=3, suppress_small=True)}")
        print(f"  当前 Right: {np.array2string(node.cur_right, precision=3, suppress_small=True)}")

        # 1. 缓慢移动到初始姿态
        smooth_move(node, traj_l[0], traj_r[0],
                    duration=args.init_duration, vel_pct=args.init_vel)

        if not args.no_confirm:
            ans = input("\n>>> 已就位, 按回车开始正式回放 (输入 q 取消): ").strip().lower()
            if ans == 'q':
                print("已取消")
                return

        # 2. 主回放循环 (绝对时间锚定 + 跳帧补偿, 避免漂移和卡顿)
        indices = list(range(0, size, args.skip))
        send_hz = (1.0 / np.mean(np.diff(ts[indices]))) * args.rate if len(indices) > 1 else 20.0
        print(f"\n>>> 开始回放 ({len(indices)} 帧 / 原始 {size} 帧, 倍速 {args.rate}x, "
              f"目标发送频率 ~{send_hz:.1f}Hz, 跟踪速度 {args.vel:.0f}%)")

        t_start = time.time()
        published_count = 0
        last_log_t = t_start
        late_count = 0

        for k, i in enumerate(indices):
            if not rclpy.ok():
                break
            # 用原始时间戳的相对偏移作为节拍, 不会因为单帧 publish 慢而漂移
            target_elapsed = (ts[i] - ts[indices[0]]) / args.rate
            now_elapsed = time.time() - t_start
            sleep_s = target_elapsed - now_elapsed
            if sleep_s > 0.0005:
                time.sleep(sleep_s)
            elif sleep_s < -0.05:
                # 落后超过 50ms, 直接跳过当前帧, 让机械臂去追下一帧
                late_count += 1
                continue

            stamp_msg = node.get_clock().now().to_msg()
            node.publish(traj_l[i], traj_r[i], stamp_msg, vel_pct=args.vel)
            published_count += 1

            # 进度日志限频
            if time.time() - last_log_t > 0.5:
                pct = (k + 1) / len(indices) * 100
                sys.stdout.write(
                    f"\r  进度: {k + 1}/{len(indices)} ({pct:5.1f}%)  "
                    f"已发 {published_count}  跳帧 {late_count}")
                sys.stdout.flush()
                last_log_t = time.time()

        sys.stdout.write(
            f"\r  进度: {len(indices)}/{len(indices)} (100.0%)  "
            f"已发 {published_count}  跳帧 {late_count}\n")
        print(f">>> 回放完成, 耗时 {time.time() - t_start:.2f}s")
        if late_count > len(indices) * 0.1:
            print(f"[提示] 跳帧较多({late_count}), 可加大 --skip 或降低 --rate")

        # 3. 末尾保持 1 秒
        print(">>> 保持最后姿态 1 秒")
        t_end = time.time()
        while time.time() - t_end < 1.0 and rclpy.ok():
            node.publish(traj_l[-1], traj_r[-1],
                         node.get_clock().now().to_msg(), vel_pct=args.vel)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[!!] 用户中断, 停止发布指令")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
