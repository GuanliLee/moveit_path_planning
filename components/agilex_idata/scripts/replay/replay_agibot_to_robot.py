#!/usr/bin/env python3
"""
把 agibot 格式 (aligned_joints.h5) 的采集数据回放到 Piper 真机上 (双臂)。

适配的数据格式 (与 ALOHA 单文件格式不同):
    <episodeN>/states/aligned_joints.h5
        逐帧 group: "0", "1", ... "N-1"
        每个 group 里:
          action/joint/position   (14,)  ← 指令: 左臂[0:7] + 右臂[7:14], 第7位是夹爪
          state /joint/position   (14,)  ← 实际状态: 同样排布
          main_timestamp          ()     ← uint64 纳秒时间戳

    与 ALOHA 的关节对应关系:
        left  = joint/position[0:7]   (6 关节 + 夹爪)
        right = joint/position[7:14]  (6 关节 + 夹爪)

控制方式与 replay_to_robot.py 完全一致 (那版很丝滑):
    - velocity[6] 作为伺服跟踪速度百分比 (piper_single_ctrl 的约定)
    - 绝对时间锚定 + 跳帧补偿, 避免漂移和卡顿
    - 启动先慢速插值到首帧, 两次回车确认

前置条件:
    1. piper_ros 已启动: bash /home/agilex/piper_ros/start_multi_piper.sh
    2. CAN 已激活, 机械臂供电正常, 周围无人无障碍

用法:
    python3 replay_agibot_to_robot.py \
        /home/agilex/data/home/wusirong/agibot/data_records/grasp_bottle2_h5/episode4
    # 也可以直接指向 aligned_joints.h5:
    python3 replay_agibot_to_robot.py .../episode4/states/aligned_joints.h5
    # 用实际状态(state)而非指令(action)回放, 更保真:
    python3 replay_agibot_to_robot.py .../episode4 --source state
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
        super().__init__('agibot_replay_to_robot')
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
        s = 0.5 - 0.5 * np.cos(np.pi * alpha)  # ease-in-out
        cmd_l = start_l * (1 - s) + target_l * s
        cmd_r = start_r * (1 - s) + target_r * s
        node.publish(cmd_l, cmd_r, node.get_clock().now().to_msg(), vel_pct=vel_pct)
        time.sleep(1.0 / rate_hz)
    print(">>> 已到达初始姿态")


def resolve_h5_path(path: str) -> str:
    """支持传 episode 目录 或 直接传 aligned_joints.h5"""
    if os.path.isdir(path):
        cand = os.path.join(path, 'states', 'aligned_joints.h5')
        if os.path.isfile(cand):
            return cand
        cand2 = os.path.join(path, 'aligned_joints.h5')
        if os.path.isfile(cand2):
            return cand2
        raise FileNotFoundError(f"目录下找不到 states/aligned_joints.h5: {path}")
    return path


def load_agibot_trajectory(h5_path: str, source: str):
    """读取 agibot 逐帧 group, 返回 (traj_l (N,7), traj_r (N,7), ts (N,) 秒)"""
    key = f'{source}/joint/position'  # action/joint/position 或 state/joint/position
    with h5py.File(h5_path, 'r') as f:
        # group 名是字符串数字, 按数值排序
        frame_keys = sorted((k for k in f.keys() if k.isdigit()), key=int)
        n = len(frame_keys)
        if n == 0:
            raise ValueError("HDF5 里没有逐帧 group (0,1,2,...)")
        traj_l = np.zeros((n, 7))
        traj_r = np.zeros((n, 7))
        ts = np.zeros(n, dtype=np.float64)
        for idx, fk in enumerate(frame_keys):
            g = f[fk]
            jp = np.asarray(g[key][()]).reshape(-1)
            if jp.shape[0] < 14:
                raise ValueError(f"frame {fk} 的 {key} 维度异常: {jp.shape}")
            traj_l[idx] = jp[0:7]
            traj_r[idx] = jp[7:14]
            ts[idx] = int(g['main_timestamp'][()]) / 1e9  # ns -> s
    return traj_l, traj_r, ts


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path", help="episode 目录 或 aligned_joints.h5 路径")
    parser.add_argument("--source", choices=["action", "state"], default="action",
                        help="action=指令轨迹(默认), state=实际状态轨迹(更保真)")
    parser.add_argument("--rate", type=float, default=1.0,
                        help="回放倍速 (默认 1.0; 卡顿时试 0.5)")
    parser.add_argument("--skip", type=int, default=1,
                        help="每隔几帧发一次 (默认 1 => 全帧发送, 实测约 30Hz)")
    parser.add_argument("--vel", type=float, default=80.0,
                        help="伺服跟踪速度百分比 (30~100, 默认 80)")
    parser.add_argument("--init-vel", type=float, default=40.0,
                        help="移动到初始姿态时的速度百分比 (默认 40)")
    parser.add_argument("--init-duration", type=float, default=5.0,
                        help="移动到初始姿态的耗时 (秒, 默认 5)")
    parser.add_argument("--no-confirm", action="store_true",
                        help="跳过人工确认 (危险, 不推荐)")
    args = parser.parse_args()

    h5_path = resolve_h5_path(args.path)
    print(f"加载 agibot HDF5: {h5_path}")
    traj_l, traj_r, ts = load_agibot_trajectory(h5_path, args.source)
    size = len(ts)

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
            target_elapsed = (ts[i] - ts[indices[0]]) / args.rate
            now_elapsed = time.time() - t_start
            sleep_s = target_elapsed - now_elapsed
            if sleep_s > 0.0005:
                time.sleep(sleep_s)
            elif sleep_s < -0.05:
                late_count += 1
                continue

            stamp_msg = node.get_clock().now().to_msg()
            node.publish(traj_l[i], traj_r[i], stamp_msg, vel_pct=args.vel)
            published_count += 1

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
