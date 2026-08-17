#! /usr/bin/env python3.10
import argparse
import json
import os

import numpy as np
import yaml

from rosbag2_py import StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import time
import sys

# ROS Humble's transforms3d still references np.float, which is removed by
# newer user-site NumPy versions.
if not hasattr(np, "float"):
    np.float = float

import tf_transformations

def ros_time_to_sec_str(timestamp) -> str:
    return f"{timestamp.sec+timestamp.nanosec/1e9:.6f}"
    # return f"{timestamp.sec}.{int(round(timestamp.nanosec/1000))}"

def bag_timestamp_to_sec_str(timestamp: int) -> str:
    return f"{timestamp / 1e9:.6f}"

def msg_stamp_to_sec_str(msg, fallback_timestamp: int) -> str:
    if hasattr(msg, "header"):
        stamp = msg.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            return ros_time_to_sec_str(stamp)
    return bag_timestamp_to_sec_str(fallback_timestamp)

def scalar_msg_field(msg, name, default=None):
    return getattr(msg, name, default)

def numeric_json_field(data, *names):
    for name in names:
        if name in data:
            value = data[name]
            if isinstance(value, bool):
                break
            return float(value)
    raise KeyError("/".join(names))

def robot_base_action_dict_from_msg(msg, topic_type):
    if topic_type == "geometry_msgs/msg/Twist":
        return {
            "linear": {
                "x": msg.linear.x,
                "y": msg.linear.y,
                "z": msg.linear.z,
            },
            "angular": {
                "x": msg.angular.x,
                "y": msg.angular.y,
                "z": msg.angular.z,
            },
        }

    if topic_type == "std_msgs/msg/String":
        data = json.loads(msg.data)
        if not isinstance(data, dict):
            raise ValueError("robotBase action String JSON must be an object")

        if "linear" in data and "angular" in data:
            linear = data.get("linear") or {}
            angular = data.get("angular") or {}
            return {
                "linear": {
                    "x": numeric_json_field(linear, "x"),
                    "y": numeric_json_field(linear, "y"),
                    "z": float(linear.get("z", 0.0)),
                },
                "angular": {
                    "x": float(angular.get("x", 0.0)),
                    "y": float(angular.get("y", 0.0)),
                    "z": numeric_json_field(angular, "z"),
                },
            }

        return {
            "linear": {
                "x": numeric_json_field(data, "linearX", "linear_x", "vx", "x"),
                "y": numeric_json_field(data, "linearY", "linear_y", "vy", "y"),
                "z": float(data.get("linearZ", data.get("linear_z", 0.0))),
            },
            "angular": {
                "x": float(data.get("angularX", data.get("angular_x", 0.0))),
                "y": float(data.get("angularY", data.get("angular_y", 0.0))),
                "z": numeric_json_field(data, "angularZ", "angular_z", "wz", "z"),
            },
        }

    raise ValueError(f"Unsupported robotBase action topic type: {topic_type}")

def lift_action_height_from_value(value):
    if isinstance(value, bool):
        raise ValueError("lift action height cannot be bool")
    if isinstance(value, (int, float)):
        return normalize_lift_action_height_to_mm(float(value))
    if isinstance(value, str):
        text = value.strip()
        if not text:
            raise ValueError("empty lift action string")
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            return normalize_lift_action_height_to_mm(float(text))
        return lift_action_height_from_value(parsed)
    if isinstance(value, dict):
        for field in (
            "targetHeight",
            "target_height",
            "val",
            "height",
            "backHeight",
            "back_height",
            "data",
        ):
            if field in value and value[field] is not None:
                return lift_action_height_from_value(value[field])
    raise ValueError(f"Unsupported lift action payload: {value!r}")

def normalize_lift_action_height_to_mm(value):
    # /action/lifting is commonly published in meters (for example 0.2),
    # while LiftMotorSrv and replay scripts use millimeters.
    return value * 1000.0 if abs(value) <= 5.0 else value

def lift_action_dict_from_msg(msg, topic_type):
    if topic_type != "std_msgs/msg/String":
        raise ValueError(f"Unsupported lifting action topic type: {topic_type}")
    target_height = lift_action_height_from_value(msg.data)
    return {
        "targetHeight": target_height,
        "target_height": target_height,
        "val": target_height,
    }

def robot_base_state_dict_from_pose(pose):
    orientation_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rpy_list = tf_transformations.euler_from_quaternion(orientation_list)
    return {
        "linear": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        },
        "angular": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        },
        "pose": {
            "position": {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
            },
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            },
            "yaw": rpy_list[2],
        },
    }

def robot_base_state_dict_from_msg(msg, topic_type):
    if topic_type == "geometry_msgs/msg/PoseStamped":
        return robot_base_state_dict_from_pose(msg.pose)
    if topic_type == "nav_msgs/msg/Odometry":
        result = robot_base_state_dict_from_pose(msg.pose.pose)
        twist = msg.twist.twist
        result["linear"] = {
            "x": twist.linear.x,
            "y": twist.linear.y,
            "z": twist.linear.z,
        }
        result["angular"] = {
            "x": twist.angular.x,
            "y": twist.angular.y,
            "z": twist.angular.z,
        }
        return result
    raise ValueError(f"Unsupported robotBase state topic type: {topic_type}")

def get_mcap_msg_num(mcap_file):
    """Read the message count through rosbag2 instead of an external MCAP CLI."""
    try:
        metadata = rosbag2_py.Info().read_metadata(os.fspath(mcap_file), "mcap")
    except Exception as exc:
        raise RuntimeError(f"Failed to read MCAP metadata: {mcap_file}: {exc}") from exc
    return int(metadata.message_count)

def create_directory(dir_path):
    if not os.path.isdir(dir_path):
        os.makedirs(dir_path, exist_ok=True)

def print_dict(data, ss=""):
    for key, value in data.items():
        if isinstance(value, dict):
            print(f"{ss}{key}:")
            print_dict(value, f"{ss}  ")
        else:
            print(f"{ss}{key}: {value}")

def get_dict_from_index(data, index=""):
    for key, value in data.items():
        if isinstance(value, dict):
            if key == index:
                return value
            else:
                res = get_dict_from_index(value, index)
                if len(res) != 0:
                    return res
    return dict()

def get_topic_config_from_yaml(yaml_path):
    if not os.path.exists(yaml_path):
        return {}, {}

    topic_dir = {}
    sensor_type = {}
    with open(yaml_path, "r") as f:
        yaml_dict = yaml.safe_load(f)
        data_info_dict = get_dict_from_index(yaml_dict, "dataInfo")
        for key1, value1 in data_info_dict.items():
            for key2, value2 in value1.items():
                names_list = value2["names"]
                topics_list = value2["topics"]
                sensor_type_name = key1
                if key1 == "action" or len(data_info_dict[key1].keys()) > 1:
                    sensor_type_name = f"{key1}_{key2}"
                sensor_type[sensor_type_name] = topics_list
                for i in range(len(topics_list)):
                    topic_dir[topics_list[i]] = f"{key1}/{key2}/{names_list[i]}"
                if "configTopics" in value2.keys():
                    config_topics_list = value2["configTopics"]
                    sensor_type[f"{sensor_type_name}_info"] = config_topics_list
                    for i in range(len(config_topics_list)):
                        topic_dir[config_topics_list[i]] = f"{key1}/{key2}/{names_list[i]}"
    return topic_dir, sensor_type

def process_file(input_mcap_file, data_dir, yaml_path):
    msg_num = get_mcap_msg_num(input_mcap_file)

    parent_path = os.path.dirname(input_mcap_file)
    parent_dir = os.path.basename(parent_path)
    output_dir_head = f"{data_dir}/aloha/{parent_dir}/"
    print(f"\n>>> {input_mcap_file}")
    # print(f"输入mcap为 {input_mcap_file}")
    print(f"输出: {output_dir_head}")

    # 打开输入的 MCAP 文件
    storage_options_input = StorageOptions(uri=input_mcap_file, storage_id="mcap")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options_input, ConverterOptions("", ""))
    topic_types_dict = {t.name: t.type for t in reader.get_all_topics_and_types()}

    save_topic_dirs, sensor_topic_dict = get_topic_config_from_yaml(yaml_path)
    print(f"---")
    for sensor_type, topic_list in sensor_topic_dict.items():
        print(f"{sensor_type}:")
        for topic in topic_list:
            topic_dir = save_topic_dirs[topic]
            create_directory(f"{output_dir_head}{topic_dir}")
            if topic in topic_types_dict:
                print(f"  * {topic}\t[{topic_dir}]\t: {topic_types_dict[topic]}")
            else:
                print(f"  * {topic}\t[{topic_dir}]\t: (not found)")
    print("---\n")

    # 生成aloha数据
    total_count = 0
    last_progress = -1
    fail_topics = []
    start_time = time.time()
    while reader.has_next():
        total_count += 1
        progress = int(total_count / msg_num * 100) if msg_num > 0 else 100
        if progress != last_progress:
            sys.stdout.write('\r[%3d%%] progress ( %d / %d )' % (progress, total_count, msg_num))
            sys.stdout.flush()
            last_progress = progress

        topic, data, timestamp = reader.read_next()
        # camera color
        if ("camera_color" in sensor_topic_dict and topic in sensor_topic_dict["camera_color"]) or \
           ("camera" in sensor_topic_dict and topic in sensor_topic_dict["camera"]):
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = ros_time_to_sec_str(msg.header.stamp)
            with open(f'{output_dir_head}{save_topic_dirs[topic]}/{filename}.jpg', 'wb') as f:
                f.write(msg.data)
        # camera depth
        elif "camera_depth" in sensor_topic_dict and topic in sensor_topic_dict["camera_depth"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = ros_time_to_sec_str(msg.header.stamp)
            with open(f'{output_dir_head}{save_topic_dirs[topic]}/{filename}.png', 'wb') as f:
                f.write(msg.data)
        # camera color and depth info
        elif ("camera_color_info" in sensor_topic_dict and topic in sensor_topic_dict["camera_color_info"]) or \
            ("camera_depth_info" in sensor_topic_dict and topic in sensor_topic_dict["camera_depth_info"]) or \
            ("camera_info" in sensor_topic_dict and topic in sensor_topic_dict["camera_info"]):
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            roi_dict = {
                "do_rectify" : int(msg.roi.do_rectify),
                "height" : msg.roi.height,
                "width" : msg.roi.width,
                "x_offset" : msg.roi.x_offset,
                "y_offset" : msg.roi.y_offset
            }
            camera_info_data_dict = {
                "D": msg.d.tolist(),
                "K": msg.k.tolist(),
                "P": msg.p.tolist(),
                "R": msg.r.tolist(),
                "binning_x": msg.binning_x,
                "binning_y": msg.binning_y,
                "distortion_model": msg.distortion_model,
                "height": msg.height,
                "roi": roi_dict,
                "width": msg.width
            }
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/config.json", 'w+') as json_file:
                json.dump(camera_info_data_dict, json_file, indent="\t")
        # arm
        elif "arm" in sensor_topic_dict and topic in sensor_topic_dict["arm"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = ros_time_to_sec_str(msg.header.stamp)
            joint_state_dict = {
                "effort" : msg.effort.tolist(),
                "position" : msg.position.tolist(),
                "velocity" : msg.velocity.tolist(),
            }
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/{filename}.json", 'w+') as json_file:
                json.dump(joint_state_dict, json_file, indent="\t")
        # localization
        elif "localization" in sensor_topic_dict and topic in sensor_topic_dict["localization"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = ros_time_to_sec_str(msg.header.stamp)
            position = msg.pose.position
            orientation = msg.pose.orientation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            rpy_list = tf_transformations.euler_from_quaternion(orientation_list)
            pose_dict = {
                "pitch" : rpy_list[1],
                "roll" : rpy_list[0],
                "x" : position.x,
                "y" : position.y,
                "yaw" : rpy_list[2],
                "z" : position.z,
            }
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/{filename}.json", 'w+') as json_file:
                json.dump(pose_dict, json_file, indent="\t")
        # gripper
        elif "gripper" in sensor_topic_dict and topic in sensor_topic_dict["gripper"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = ros_time_to_sec_str(msg.header.stamp)
            angle = msg.angle
            distance = msg.distance
            gripper_dict = {
                "angle": angle,
                "distance": distance,
            }
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/{filename}.json", 'w+') as json_file:
                json.dump(gripper_dict, json_file, indent="\t")

        # robot base state stores pose; velocity may come from a separate
        # robotBase.vel source and is merged later by data_to_hdf5.py.
        elif ("robotBase" in sensor_topic_dict and topic in sensor_topic_dict["robotBase"]) or \
             ("robotBase_state" in sensor_topic_dict and topic in sensor_topic_dict["robotBase_state"]) or \
             ("robotBase_vel" in sensor_topic_dict and topic in sensor_topic_dict["robotBase_vel"]):
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = msg_stamp_to_sec_str(msg, timestamp)
            odom_dict = robot_base_state_dict_from_msg(msg, topic_types_dict[topic])
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/{filename}.json", 'w+') as json_file:
                json.dump(odom_dict, json_file, indent="\t")

        # robot base command action from configured robotBase.action topic.
        elif "robotBase_action" in sensor_topic_dict and topic in sensor_topic_dict["robotBase_action"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = msg_stamp_to_sec_str(msg, timestamp)
            cmd_dict = robot_base_action_dict_from_msg(msg, topic_types_dict[topic])
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/{filename}.json", 'w+') as json_file:
                json.dump(cmd_dict, json_file, indent="\t")

        # Lifting column command action from /action/lifting.
        elif "action_lifting" in sensor_topic_dict and topic in sensor_topic_dict["action_lifting"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = msg_stamp_to_sec_str(msg, timestamp)
            lift_action_dict = lift_action_dict_from_msg(msg, topic_types_dict[topic])
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/{filename}.json", 'w+') as json_file:
                json.dump(lift_action_dict, json_file, indent="\t")

        # Lifting column state/action source. data_to_hdf5.py reads backHeight
        # for state and targetHeight for action/lifting.
        elif ("lift" in sensor_topic_dict and topic in sensor_topic_dict["lift"]) or \
             ("lift_motor" in sensor_topic_dict and topic in sensor_topic_dict["lift_motor"]):
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = msg_stamp_to_sec_str(msg, timestamp)
            lift_dict = {
                "motorId": scalar_msg_field(msg, "motor_id"),
                "initState": scalar_msg_field(msg, "init_state"),
                "ctrlMode": scalar_msg_field(msg, "ctrl_mode"),
                "runState": scalar_msg_field(msg, "run_state"),
                "stateBit": scalar_msg_field(msg, "state_bit"),
                "motorSpeed": scalar_msg_field(msg, "motor_speed"),
                "backSpeed": scalar_msg_field(msg, "back_speed"),
                "backPos": scalar_msg_field(msg, "back_pos"),
                "backHeight": scalar_msg_field(msg, "back_height"),
                "targetHeight": scalar_msg_field(msg, "target_height"),
                "targetPos": scalar_msg_field(msg, "target_pos"),
                "upLimit": scalar_msg_field(msg, "up_limit"),
                "downLimit": scalar_msg_field(msg, "down_limit"),
                "reachTargetPos": scalar_msg_field(msg, "reach_target_pos"),
                "fpsError": scalar_msg_field(msg, "fps_error"),
                "overflowI": scalar_msg_field(msg, "overflow_i"),
                "overflowV": scalar_msg_field(msg, "overflow_v"),
                "encodeErr": scalar_msg_field(msg, "encode_err"),
                "posBiasOver": scalar_msg_field(msg, "pos_bias_over"),
                "underVol": scalar_msg_field(msg, "under_vol"),
                "overLoad": scalar_msg_field(msg, "over_load"),
                "externalCtrlMode": scalar_msg_field(msg, "external_ctrl_mode"),
                "back_height": scalar_msg_field(msg, "back_height"),
                "target_height": scalar_msg_field(msg, "target_height"),
            }
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/{filename}.json", 'w+') as json_file:
                json.dump(lift_dict, json_file, indent="\t")

        # imu
        elif "imu" in sensor_topic_dict and topic in sensor_topic_dict["imu"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = ros_time_to_sec_str(msg.header.stamp)
            imu_dict = {
                "angular_velocity" : {
                    "x" : msg.angular_velocity.x,
                    "y" : msg.angular_velocity.y,
                    "z" : msg.angular_velocity.z,
                },
                "linear_acceleration" : {
                    "x" : msg.linear_acceleration.x,
                    "y" : msg.linear_acceleration.y,
                    "z" : msg.linear_acceleration.z,
                },
                "orientation" : {
                    "x" : msg.orientation.x,
                    "y" : msg.orientation.y,
                    "z" : msg.orientation.z,
                    "w" : msg.orientation.w,
                },
            }
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/{filename}.json", 'w+') as json_file:
                json.dump(imu_dict, json_file, indent="\t")

        # TODO /robotBase/transform
        # can't process
        elif topic not in fail_topics:
            fail_topics.append(topic)
    print()
    end_time = time.time()
    print("spend time: %-.3fs" % (end_time - start_time))

    if len(fail_topics) != 0:
        print("\n[Warn] can't generate aloha data from topic:")
        print("++++++++++++++++++++++++++++++++++++++++++")
        for topic in fail_topics:
            print(f"  * {topic}")
        print("------------------------------------------")

def get_mcap_files(in_dir=""):
    exclude_dirs = [".Trash-1000"]
    file_list = []
    for root, dirs, files in os.walk(in_dir):
        dirs[:] = [d for d in dirs if d not in exclude_dirs]
        
        for file in files:
            if file.endswith('.mcap'):
                file_path = os.path.join(root, file)
                if not ("abnormal" in file_path):
                    file_list.append(file_path)
    return file_list

def main():
    mcap_dir = os.getcwd()
    data_dir = mcap_dir
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir.',
                        default=os.getcwd(), required=False)
    parser.add_argument('--targetDir', action='store', type=str, help='targetDir.',
                        default="", required=False)
    parser.add_argument('--episodeIndex', action='store', type=int, help='Episode index.',
                        default=-1, required=False)
    parser.add_argument('--alohaYaml', action='store', type=str, help='alohaYaml.',
                        default="../config/aloha_data_params.yaml", required=False)
    args = parser.parse_args()
    print("args:")
    print(f" --datasetDir: {args.datasetDir}")
    print(f" --episodeIndex: {args.episodeIndex}")
    print(f" --alohaYaml: {args.alohaYaml}")

    if args.datasetDir != "" and args.datasetDir != ".":
        mcap_dir = args.datasetDir
        data_dir = args.datasetDir
    if args.episodeIndex != -1:
        mcap_dir = os.path.join(mcap_dir, f"episode{args.episodeIndex}")
    if args.targetDir != "":
        data_dir = args.targetDir

    # 获取指定目录下所有mcap
    files = get_mcap_files(mcap_dir)
    print(f"转换的mcaps: {files}")

    # # 指定以某个yaml文件配置来将mcap转aloha数据
    for file in files:
        process_file(file, data_dir, args.alohaYaml)

if __name__ == "__main__":
    main()
