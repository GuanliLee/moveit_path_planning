#!/usr/bin/env python3
"""Replay a mobile ALOHA HDF5 episode to arms, base and lifting column."""
import argparse
import json
import sys
import time
from pathlib import Path

import h5py
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

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


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']


def dataset_or_none(h5_file, path):
    try:
        return h5_file[path][()]
    except KeyError:
        return None


def clamp(value, limit):
    if limit <= 0:
        return float(value)
    return float(max(-limit, min(limit, value)))


def sample_by_timestamp(timestamps, values, target_time, mode='hold'):
    if len(values) == 0:
        raise ValueError("values is empty")
    if target_time <= timestamps[0]:
        return values[0]
    if target_time >= timestamps[-1]:
        return values[-1]

    idx = int(np.searchsorted(timestamps, target_time, side='right') - 1)
    idx = max(0, min(idx, len(values) - 2))
    if mode == 'linear':
        t0 = float(timestamps[idx])
        t1 = float(timestamps[idx + 1])
        if t1 <= t0:
            return values[idx]
        alpha = (float(target_time) - t0) / (t1 - t0)
        return values[idx] * (1.0 - alpha) + values[idx + 1] * alpha
    return values[idx]


def load_raw_base_commands(hdf5_path, start_time, end_time, margin=0.5):
    action_dir = Path(hdf5_path).resolve().parent / 'robotBase' / 'action' / 'chassis'
    if not action_dir.is_dir():
        return None, None, None

    times = []
    commands = []
    for path in sorted(action_dir.glob('*.json')):
        try:
            stamp = float(path.stem)
        except ValueError:
            continue
        if stamp < start_time - margin or stamp > end_time + margin:
            continue
        with path.open('r') as file:
            data = json.load(file)
        times.append(stamp)
        commands.append([
            data['linear']['x'],
            data['linear']['y'],
            data['angular']['z'],
        ])

    if not commands:
        return None, None, None
    return np.asarray(times, dtype=np.float64), np.asarray(commands, dtype=np.float64), str(action_dir)


class MobileReplayNode(Node):
    def __init__(self, enable_arms=True, enable_base=True, enable_lift=True,
                 left_state_topic='/puppet/joint_left',
                 right_state_topic='/puppet/joint_right',
                 left_command_topic='/joint_left_states',
                 right_command_topic='/joint_right_states',
                 base_command_topic='/cmd_vel',
                 lift_state_topic='/LiftMotorStatePub',
                 lift_service='/LiftingMotorService'):
        super().__init__('aloha_mobile_replay_to_robot')
        self.enable_arms = enable_arms
        self.enable_base = enable_base
        self.enable_lift = enable_lift

        self.pub_left = None
        self.pub_right = None
        self.cur_left = None
        self.cur_right = None
        if self.enable_arms:
            self.pub_left = self.create_publisher(JointState, left_command_topic, 10)
            self.pub_right = self.create_publisher(JointState, right_command_topic, 10)
            self.create_subscription(
                JointState, left_state_topic,
                lambda m: setattr(self, 'cur_left', np.array(m.position[:7])), 10)
            self.create_subscription(
                JointState, right_state_topic,
                lambda m: setattr(self, 'cur_right', np.array(m.position[:7])), 10)

        self.pub_base = None
        if self.enable_base:
            self.pub_base = self.create_publisher(Twist, base_command_topic, 10)

        self.lift_client = None
        self.cur_lift_height = None
        if self.enable_lift:
            if LiftMotorSrv is None or LiftMotorMsg is None:
                raise RuntimeError(
                    "lifting_msg_pkg is not available. Source /home/agilex/agilex_ws/install/setup.bash "
                    "or run with --disable-lift."
                )
            self.lift_client = self.create_client(LiftMotorSrv, lift_service)
            self.create_subscription(
                LiftMotorMsg, lift_state_topic, self._on_lift_state, 10)

    def _on_lift_state(self, msg):
        height = getattr(msg, 'back_height', None)
        if height is None:
            height = getattr(msg, 'backHeight', None)
        if height is not None:
            self.cur_lift_height = int(height)

    def wait_for_current_arms(self, timeout=5.0):
        if not self.enable_arms:
            return True
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.cur_left is not None and self.cur_right is not None:
                return True
        return False

    def wait_for_lift_service(self, timeout=3.0):
        if not self.enable_lift:
            return True
        return self.lift_client.wait_for_service(timeout_sec=timeout)

    def publish_arms(self, left, right, stamp_msg, vel_pct=80.0):
        if not self.enable_arms:
            return
        vel = [0.0] * 6 + [float(vel_pct)]

        msg_l = JointState()
        msg_l.header.stamp = stamp_msg
        msg_l.name = JOINT_NAMES
        msg_l.position = [float(x) for x in left]
        msg_l.velocity = list(vel)
        msg_l.effort = [0.0] * 7

        msg_r = JointState()
        msg_r.header.stamp = stamp_msg
        msg_r.name = JOINT_NAMES
        msg_r.position = [float(x) for x in right]
        msg_r.velocity = list(vel)
        msg_r.effort = [0.0] * 7

        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

    def publish_base(self, vel_xyz, max_linear=0.4, max_angular=0.8, scale=1.0):
        if not self.enable_base or not rclpy.ok():
            return
        msg = Twist()
        msg.linear.x = clamp(float(vel_xyz[0]) * scale, max_linear)
        msg.linear.y = clamp(float(vel_xyz[1]) * scale, max_linear)
        msg.angular.z = clamp(float(vel_xyz[2]) * scale, max_angular)
        try:
            self.pub_base.publish(msg)
        except Exception:
            pass

    def stop_base(self):
        if not self.enable_base or not rclpy.ok():
            return
        try:
            self.pub_base.publish(Twist())
        except Exception:
            pass

    def send_lift_height(self, height, mode=0):
        if not self.enable_lift:
            return
        req = LiftMotorSrv.Request()
        req.val = int(round(float(height)))
        req.mode = int(mode)
        self.lift_client.call_async(req)


def smooth_move_arms(node, target_l, target_r, duration=5.0, vel_pct=40.0):
    if not node.enable_arms:
        return
    print(f"\n>>> Move arms to initial pose ({duration:.1f}s, {vel_pct:.0f}%)")
    rate_hz = 20.0
    n_steps = max(int(duration * rate_hz), 2)
    start_l = node.cur_left.copy()
    start_r = node.cur_right.copy()
    for k in range(n_steps + 1):
        alpha = k / n_steps
        s = 0.5 - 0.5 * np.cos(np.pi * alpha)
        cmd_l = start_l * (1 - s) + target_l * s
        cmd_r = start_r * (1 - s) + target_r * s
        node.publish_arms(cmd_l, cmd_r, node.get_clock().now().to_msg(), vel_pct=vel_pct)
        node.stop_base()
        time.sleep(1.0 / rate_hz)
    print(">>> Initial arm pose reached")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hdf5", help="HDF5 file path")
    parser.add_argument("--rate", type=float, default=0.5,
                        help="Replay speed multiplier, default 0.5")
    parser.add_argument("--source", choices=["master", "puppet"], default="master",
                        help="Arm trajectory source, default master")
    parser.add_argument("--skip", type=int, default=1,
                        help="Publish every N frames, default 1")
    parser.add_argument("--vel", type=float, default=70.0,
                        help="Arm tracking velocity percentage, default 70")
    parser.add_argument("--init-vel", type=float, default=35.0,
                        help="Arm initial move velocity percentage, default 35")
    parser.add_argument("--init-duration", type=float, default=5.0,
                        help="Arm initial move duration in seconds, default 5")
    parser.add_argument("--arm-publish-hz", type=float, default=0.0,
                        help="Republish/interpolate arm commands at fixed Hz. "
                             "Use 0 to publish only on synchronized HDF5 frames")
    parser.add_argument("--arm-interp", choices=["hold", "linear"], default="linear",
                        help="Arm command interpolation between synchronized HDF5 frames")
    parser.add_argument("--base-scale", type=float, default=None,
                        help="Scale recorded base velocity before publishing. "
                             "Default follows --rate so slow/fast replay preserves displacement")
    parser.add_argument("--base-source", choices=["auto", "raw", "hdf5", "odom"], default="auto",
                        help="Base command source. auto/raw uses raw robotBase/action/chassis JSON "
                             "beside the HDF5 when available; hdf5 uses synchronized HDF5 action "
                             "samples; odom uses recorded robotBase/state velocity")
    parser.add_argument("--base-publish-hz", type=float, default=20.0,
                        help="Republish/interpolate base command at fixed Hz. "
                             "Use 0 to publish only on recorded frames")
    parser.add_argument("--base-interp", choices=["hold", "linear"], default="hold",
                        help="Base command interpolation between recorded frames. "
                             "hold is faithful to cmd_vel; linear can look smoother")
    parser.add_argument("--max-linear", type=float, default=0.4,
                        help="Clamp abs(linear x/y) command, m/s. <=0 disables clamp")
    parser.add_argument("--max-angular", type=float, default=0.8,
                        help="Clamp abs(angular z) command, rad/s. <=0 disables clamp")
    parser.add_argument("--lift-mode", type=int, default=0,
                        help="Lift service mode, default 0 for height position control")
    parser.add_argument("--lift-min-delta", type=float, default=2.0,
                        help="Minimum lift height change in mm before sending another command")
    parser.add_argument("--lift-min-period", type=float, default=0.2,
                        help="Minimum seconds between lift service calls")
    parser.add_argument("--lift-service", default="/LiftingMotorService",
                        help="Lift service name")
    parser.add_argument("--left-state-topic", default="/puppet/joint_left",
                        help="Current left arm JointState topic")
    parser.add_argument("--right-state-topic", default="/puppet/joint_right",
                        help="Current right arm JointState topic")
    parser.add_argument("--left-command-topic", default="/joint_left_states",
                        help="Left arm command JointState topic")
    parser.add_argument("--right-command-topic", default="/joint_right_states",
                        help="Right arm command JointState topic")
    parser.add_argument("--base-command-topic", default="/cmd_vel",
                        help="Base command Twist topic")
    parser.add_argument("--lift-state-topic", default="/LiftMotorStatePub",
                        help="Lift state topic")
    parser.add_argument("--disable-arms", action="store_true",
                        help="Do not publish arm commands")
    parser.add_argument("--disable-base", action="store_true",
                        help="Do not publish /cmd_vel")
    parser.add_argument("--disable-lift", action="store_true",
                        help="Do not call lift service")
    parser.add_argument("--dry-run", action="store_true",
                        help="Load and schedule data without publishing commands")
    parser.add_argument("--no-confirm", action="store_true",
                        help="Skip safety confirmation")
    args = parser.parse_args()

    if args.rate <= 0:
        raise ValueError("--rate must be positive")
    if args.skip <= 0:
        raise ValueError("--skip must be positive")
    if args.base_publish_hz < 0:
        raise ValueError("--base-publish-hz must be >= 0")
    if args.arm_publish_hz < 0:
        raise ValueError("--arm-publish-hz must be >= 0")
    base_scale = args.rate if args.base_scale is None else args.base_scale

    print(f"Load HDF5: {args.hdf5}")
    with h5py.File(args.hdf5, 'r') as f:
        size = int(f['size'][()])
        ts = np.asarray(f['timestamp'][()])
        traj_l = dataset_or_none(f, f'arm/jointStatePosition/{args.source}Left')
        traj_r = dataset_or_none(f, f'arm/jointStatePosition/{args.source}Right')
        base_state = dataset_or_none(f, 'robotBase/state/chassis')
        base_action = dataset_or_none(f, 'robotBase/action/chassis')
        legacy_base_vel = dataset_or_none(f, 'robotBase/vel/chassis')
        lift_action = dataset_or_none(f, 'action/lifting/column')
        lift_state = dataset_or_none(f, 'lift/motor/column')

    raw_base_ts = None
    raw_base_cmd = None
    raw_base_dir = None
    if len(ts) > 0 and args.base_source in ("auto", "raw"):
        raw_base_ts, raw_base_cmd, raw_base_dir = load_raw_base_commands(args.hdf5, float(ts[0]), float(ts[-1]))

    base_cmd = None
    base_ts = ts
    base_cmd_source = None
    base_frame_aligned = True
    if args.base_source == "odom":
        if base_state is not None:
            base_cmd = np.asarray(base_state)[:, 3:6]
            base_cmd_source = 'robotBase/state/chassis velocity (HDF5 odom)'
            base_frame_aligned = True
        elif legacy_base_vel is not None:
            base_cmd = legacy_base_vel
            base_cmd_source = 'robotBase/vel/chassis (HDF5 odom velocity)'
            base_frame_aligned = True
        else:
            raise RuntimeError(
                "Requested --base-source odom, but robotBase/state/chassis "
                "and robotBase/vel/chassis were not found in the HDF5."
            )
    elif raw_base_cmd is not None and args.base_source in ("auto", "raw"):
        base_cmd = raw_base_cmd
        base_ts = raw_base_ts
        base_cmd_source = f'raw {raw_base_dir}'
        base_frame_aligned = False
    elif args.base_source == "raw":
        raise RuntimeError(
            "Requested --base-source raw, but raw robotBase/action/chassis JSON was not found "
            "beside the HDF5."
        )
    elif base_action is not None:
        base_cmd = base_action
        base_cmd_source = 'robotBase/action/chassis (HDF5 synced)'
        base_frame_aligned = True
    elif legacy_base_vel is not None:
        base_cmd = legacy_base_vel
        base_cmd_source = 'robotBase/vel/chassis (legacy feedback fallback)'
        base_frame_aligned = True

    lift_height = lift_action
    lift_source = 'action/lifting/column'
    if lift_height is None and lift_state is not None:
        lift_height = lift_state
        lift_source = 'lift/motor/column (state fallback)'

    enable_arms = not args.disable_arms and traj_l is not None and traj_r is not None
    enable_base = not args.disable_base and base_cmd is not None
    enable_lift = not args.disable_lift and lift_height is not None

    if not enable_arms and not args.disable_arms:
        raise RuntimeError(f"Missing arm datasets for source '{args.source}'")
    if not enable_base and not args.disable_base:
        print("[Warn] Missing robotBase/action/chassis; base replay disabled")
    if not enable_lift and not args.disable_lift:
        print("[Warn] Missing action/lifting/column and lift/motor/column; lift replay disabled")

    lengths = [size, len(ts)]
    if enable_arms:
        lengths += [len(traj_l), len(traj_r)]
    if enable_base and base_frame_aligned:
        base_cmd = np.asarray(base_cmd)
        lengths.append(len(base_cmd))
    if enable_lift:
        lift_height = np.asarray(lift_height).reshape(-1)
        lengths.append(len(lift_height))
    size = min(lengths)
    ts = ts[:size]
    if size <= 0 or len(ts) == 0:
        raise RuntimeError(
            "HDF5 has zero synchronized frames. Check the data_sync output and "
            "make sure every required topic has messages during recording."
        )
    if enable_arms:
        traj_l = np.asarray(traj_l[:size])
        traj_r = np.asarray(traj_r[:size])
    if enable_base:
        base_cmd = np.asarray(base_cmd[:size] if base_frame_aligned else base_cmd)
        base_ts = np.asarray(ts[:size] if base_frame_aligned else base_ts)
    if base_state is not None:
        base_state = np.asarray(base_state[:size])
    if enable_lift:
        lift_height = np.asarray(lift_height[:size])

    print(f"  frames: {size}, duration: {ts[-1] - ts[0]:.2f}s")
    if enable_arms:
        print(f"  arms: {args.source} left {traj_l.shape}, right {traj_r.shape}")
        print(f"  first left : {np.array2string(traj_l[0], precision=3, suppress_small=True)}")
        print(f"  first right: {np.array2string(traj_r[0], precision=3, suppress_small=True)}")
    if enable_base:
        max_base = np.max(np.abs(base_cmd), axis=0)
        print(f"  base command: {base_cmd.shape}, source={base_cmd_source}")
        print(f"  max abs [vx_cmd, vy_cmd, wz_cmd]={np.array2string(max_base, precision=3)}")
        print(f"  base replay scale: {base_scale:.3f} ({'auto from --rate' if args.base_scale is None else 'explicit --base-scale'})")
        if not base_frame_aligned and len(base_ts) > 1:
            base_dt = np.diff(base_ts)
            print(
                f"  raw base command rate: mean={1.0 / np.mean(base_dt):.2f}Hz, "
                f"median={1.0 / np.median(base_dt):.2f}Hz, max_dt={np.max(base_dt):.3f}s")
        if len(ts) > 1:
            dt = np.diff(ts)
            print(
                f"  recorded base/frame rate: mean={1.0 / np.mean(dt):.2f}Hz, "
                f"median={1.0 / np.median(dt):.2f}Hz, max_dt={np.max(dt):.3f}s")
        if args.base_publish_hz > 0:
            print(f"  base replay publish: {args.base_publish_hz:.1f}Hz, interp={args.base_interp}")
        else:
            print("  base replay publish: recorded frames only")
    if enable_arms:
        if args.arm_publish_hz > 0:
            print(f"  arm replay publish: {args.arm_publish_hz:.1f}Hz, interp={args.arm_interp}")
        else:
            print("  arm replay publish: synchronized HDF5 frames only")
    if base_state is not None:
        print(f"  base state: {base_state.shape} [x, y, yaw, vx, vy, wz]")
    if enable_lift:
        print(
            f"  lift command: {lift_height.shape}, source={lift_source}, "
            f"first={lift_height[0]:.0f}mm, last={lift_height[-1]:.0f}mm")

    print("\nSafety:")
    print("  - Start piper_ros and ranger/lift drivers first")
    print("  - Keep the workspace clear")
    print("  - Ctrl+C stops command publishing and sends zero /cmd_vel")
    if not args.no_confirm:
        ans = input("\nPress Enter to continue, or q to cancel: ").strip().lower()
        if ans == 'q':
            print("Canceled")
            return

    if args.dry_run:
        print("\nDry run only; no ROS commands published")
        return

    rclpy.init()
    node = None
    try:
        node = MobileReplayNode(
            enable_arms=enable_arms,
            enable_base=enable_base,
            enable_lift=enable_lift,
            left_state_topic=args.left_state_topic,
            right_state_topic=args.right_state_topic,
            left_command_topic=args.left_command_topic,
            right_command_topic=args.right_command_topic,
            base_command_topic=args.base_command_topic,
            lift_state_topic=args.lift_state_topic,
            lift_service=args.lift_service,
        )

        if enable_arms:
            print("\n>>> Waiting for current arm states ...")
            if not node.wait_for_current_arms():
                raise RuntimeError("Did not receive /puppet/joint_left and /puppet/joint_right within 5s")
            print(f"  current left : {np.array2string(node.cur_left, precision=3, suppress_small=True)}")
            print(f"  current right: {np.array2string(node.cur_right, precision=3, suppress_small=True)}")

        if enable_lift:
            print("\n>>> Waiting for lift service ...")
            if not node.wait_for_lift_service():
                raise RuntimeError(f"Lift service not available: {args.lift_service}")
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
                if node.cur_lift_height is not None:
                    break
            if node.cur_lift_height is not None:
                print(f"  current lift height: {node.cur_lift_height}mm")

        if enable_base:
            node.stop_base()

        if enable_arms:
            smooth_move_arms(node, traj_l[0], traj_r[0],
                             duration=args.init_duration, vel_pct=args.init_vel)

        if not args.no_confirm:
            ans = input("\n>>> Ready. Press Enter to start replay, or q to cancel: ").strip().lower()
            if ans == 'q':
                print("Canceled")
                return

        indices = list(range(0, size, args.skip))
        print(f"\n>>> Start replay: {len(indices)} frames, rate={args.rate}x, skip={args.skip}")
        t_start = time.time()
        last_log_t = t_start
        published_count = 0
        late_count = 0
        base_publish_count = 0
        last_lift_height = None
        last_lift_t = 0.0
        base_period = None
        next_base_elapsed = 0.0
        if enable_base and args.base_publish_hz > 0:
            base_period = 1.0 / args.base_publish_hz
        arm_period = None
        next_arm_elapsed = 0.0
        arm_publish_count = 0
        if enable_arms and args.arm_publish_hz > 0:
            arm_period = 1.0 / args.arm_publish_hz

        def publish_base_at_elapsed(elapsed):
            recorded_time = ts[indices[0]] + elapsed * args.rate
            cmd = sample_by_timestamp(base_ts, base_cmd, recorded_time, mode=args.base_interp)
            node.publish_base(cmd, max_linear=args.max_linear,
                              max_angular=args.max_angular, scale=base_scale)

        def publish_arm_at_elapsed(elapsed):
            recorded_time = ts[indices[0]] + elapsed * args.rate
            left = sample_by_timestamp(ts, traj_l, recorded_time, mode=args.arm_interp)
            right = sample_by_timestamp(ts, traj_r, recorded_time, mode=args.arm_interp)
            node.publish_arms(left, right, node.get_clock().now().to_msg(), vel_pct=args.vel)

        for k, i in enumerate(indices):
            if not rclpy.ok():
                break
            target_elapsed = (ts[i] - ts[indices[0]]) / args.rate
            now_elapsed = time.time() - t_start
            sleep_s = target_elapsed - now_elapsed
            if base_period is None and arm_period is None:
                if sleep_s > 0.0005:
                    time.sleep(sleep_s)
            else:
                while rclpy.ok():
                    now_elapsed = time.time() - t_start
                    if now_elapsed >= target_elapsed - 0.0005:
                        break

                    next_events = []
                    if base_period is not None:
                        next_events.append(next_base_elapsed)
                    if arm_period is not None:
                        next_events.append(next_arm_elapsed)
                    next_event_elapsed = min(next_events) if next_events else target_elapsed

                    if next_event_elapsed <= target_elapsed:
                        sleep_until = min(next_event_elapsed, target_elapsed)
                        wait_s = sleep_until - now_elapsed
                        if wait_s > 0.0005:
                            time.sleep(min(wait_s, 0.02))
                            continue

                        if base_period is not None and next_base_elapsed <= now_elapsed + 0.002:
                            publish_base_at_elapsed(next_base_elapsed)
                            base_publish_count += 1
                            next_base_elapsed += base_period
                        if arm_period is not None and next_arm_elapsed <= now_elapsed + 0.002:
                            publish_arm_at_elapsed(next_arm_elapsed)
                            arm_publish_count += 1
                            next_arm_elapsed += arm_period
                        rclpy.spin_once(node, timeout_sec=0.0)
                    else:
                        wait_s = target_elapsed - now_elapsed
                        if wait_s > 0.0005:
                            time.sleep(min(wait_s, 0.02))
                        else:
                            break

            now_elapsed = time.time() - t_start
            if target_elapsed - now_elapsed < -0.05:
                late_count += 1
                continue

            stamp_msg = node.get_clock().now().to_msg()
            if enable_arms and arm_period is None:
                node.publish_arms(traj_l[i], traj_r[i], stamp_msg, vel_pct=args.vel)
                arm_publish_count += 1
            elif enable_arms:
                publish_arm_at_elapsed(target_elapsed)
                arm_publish_count += 1
                next_arm_elapsed = max(next_arm_elapsed, target_elapsed + arm_period)
            if enable_base:
                if base_period is None:
                    cmd = sample_by_timestamp(base_ts, base_cmd, ts[i], mode=args.base_interp)
                    node.publish_base(cmd, max_linear=args.max_linear,
                                      max_angular=args.max_angular, scale=base_scale)
                else:
                    publish_base_at_elapsed(target_elapsed)
                    next_base_elapsed = max(next_base_elapsed, target_elapsed + base_period)
                base_publish_count += 1
            if enable_lift:
                h = float(lift_height[i])
                now = time.time()
                changed = last_lift_height is None or abs(h - last_lift_height) >= args.lift_min_delta
                enough_time = now - last_lift_t >= args.lift_min_period
                if changed and enough_time:
                    node.send_lift_height(h, mode=args.lift_mode)
                    last_lift_height = h
                    last_lift_t = now

            rclpy.spin_once(node, timeout_sec=0.0)
            published_count += 1

            if time.time() - last_log_t > 0.5:
                pct = (k + 1) / len(indices) * 100
                sys.stdout.write(
                    f"\r  progress: {k + 1}/{len(indices)} ({pct:5.1f}%) "
                    f"published={published_count} arm={arm_publish_count} "
                    f"base={base_publish_count} late={late_count}")
                sys.stdout.flush()
                last_log_t = time.time()

        sys.stdout.write(
            f"\r  progress: {len(indices)}/{len(indices)} (100.0%) "
            f"published={published_count} arm={arm_publish_count} "
            f"base={base_publish_count} late={late_count}\n")
        print(f">>> Replay done in {time.time() - t_start:.2f}s")

        if enable_arms:
            print(">>> Hold final arm pose for 1s")
            t_end = time.time()
            while time.time() - t_end < 1.0 and rclpy.ok():
                node.publish_arms(traj_l[-1], traj_r[-1],
                                  node.get_clock().now().to_msg(), vel_pct=args.vel)
                node.stop_base()
                time.sleep(0.05)
        else:
            node.stop_base()

    except KeyboardInterrupt:
        print("\n[Interrupted] Stop publishing commands")
    finally:
        if node is not None:
            node.stop_base()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
