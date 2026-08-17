#!/usr/bin/env python3
"""
ACT 真机推理（仅 RGB + 双臂关节，与训练一致）
修复官方脚本默认检查 depth 导致 aligned_depth_to_color has no data 的问题。
"""
import importlib.util
import os
import sys

ACT_DIR = "/home/agilex/aloha/act"
INF_PATH = os.path.join(ACT_DIR, "aloha_inference-ros2.py")

# 与在 act 目录下直接 python aloha_inference-ros2.py 一致，才能 import utils/policy 等
os.chdir(ACT_DIR)
if ACT_DIR not in sys.path:
    sys.path.insert(0, ACT_DIR)

spec = importlib.util.spec_from_file_location("aloha_inference_ros2", INF_PATH)
mod = importlib.util.module_from_spec(spec)
sys.modules["aloha_inference_ros2"] = mod
spec.loader.exec_module(mod)

_orig_check = mod.RosOperator.check_frame


def _check_frame_rgb_only(self):
    """仅检查训练时用到的传感器：RGB + puppet 关节。"""
    for i in range(len(self.args.camera_color_names)):
        if self.camera_color_deques[i].size() == 0:
            print(self.args.camera_color_topics[i], "has no data")
            return False
    if self.args.use_camera_depth or self.args.use_camera_color_depth_to_point_cloud:
        for i in range(len(self.args.camera_depth_names)):
            if self.camera_depth_deques[i].size() == 0:
                print(self.args.camera_depth_topics[i], "has no data")
                return False
    if self.args.use_camera_point_cloud and not self.args.use_camera_color_depth_to_point_cloud:
        for i in range(len(self.args.camera_point_cloud_names)):
            if self.camera_point_cloud_deques[i].size() == 0:
                print(self.args.camera_point_cloud_topics[i], "has no data")
                return False
    for i in range(len(self.args.arm_joint_state_names)):
        if self.arm_joint_state_deques[i].size() == 0:
            print(self.args.arm_joint_state_topics[i], "has no data")
            return False
    return True


mod.RosOperator.check_frame = _check_frame_rgb_only


def main():
    if len(sys.argv) < 2:
        ckpt_dir = "/home/caizj/checkpoint/ACT"
        ckpt_name = "policy_epoch_2000_seed_0.ckpt"
    else:
        ckpt_dir = sys.argv[1]
        ckpt_name = sys.argv[2] if len(sys.argv) > 2 else "policy_epoch_2000_seed_0.ckpt"

    sys.argv = [
        "run_act_inference_rgb.py",
        "--ckpt_dir", ckpt_dir,
        "--ckpt_name", ckpt_name,
        "--ckpt_stats_name", "dataset_stats.pkl",
        "--policy_class", "ACT",
        "--arm_joint_state_dim", "7",
        "--use_arm_joint_state", "3",
        "--use_arm_end_pose", "0",
        "--use_robot_base", "0",
        "--chunk_size", "100",
        "--hidden_dim", "512",
        "--dim_feedforward", "3200",
        "--kl_weight", "10",
    ]

    args = mod.get_arguments()

    # 与训练一致：只用 RGB，不用 depth / 点云
    args.use_camera_color = True
    args.use_camera_depth = False
    args.use_camera_point_cloud = False
    args.use_camera_color_depth_to_point_cloud = False
    args.camera_depth_names = []
    args.camera_point_cloud_names = []
    # 训练时相机顺序: front left right
    args.camera_color_names = ["front", "left", "right"]
    args.camera_color_topics = [
        "/camera_f/color/image_raw",
        "/camera_l/color/image_raw",
        "/camera_r/color/image_raw",
    ]
    args.camera_color_config_topics = [
        "/camera_f/color/camera_info",
        "/camera_l/color/camera_info",
        "/camera_r/color/camera_info",
    ]

    import rclpy
    rclpy.init()
    ros_operator = mod.RosOperator(args)
    rclpy.spin(ros_operator)


if __name__ == "__main__":
    main()
