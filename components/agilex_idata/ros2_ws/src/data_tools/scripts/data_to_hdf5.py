#!/home/lin/miniconda3/envs/aloha/bin/python
# -- coding: UTF-8
"""
#!/root/miniconda3/envs/aloha/bin/python
#!/home/lin/miniconda3/envs/aloha/bin/python
"""
import math
import os
import numpy as np
import h5py
import argparse
import json
import cv2
from scipy.spatial.transform import Rotation as R
import yaml


def matrix_to_xyzrpy(matrix):
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]


def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    transformation_matrix = np.eye(4)
    A = np.cos(yaw)
    B = np.sin(yaw)
    C = np.cos(pitch)
    D = np.sin(pitch)
    E = np.cos(roll)
    F = np.sin(roll)
    DE = D * E
    DF = D * F
    transformation_matrix[0, 0] = A * C
    transformation_matrix[0, 1] = A * DF - B * E
    transformation_matrix[0, 2] = B * F + A * DE
    transformation_matrix[0, 3] = x
    transformation_matrix[1, 0] = B * C
    transformation_matrix[1, 1] = A * E + B * DF
    transformation_matrix[1, 2] = B * DE - A * F
    transformation_matrix[1, 3] = y
    transformation_matrix[2, 0] = -D
    transformation_matrix[2, 1] = C * F
    transformation_matrix[2, 2] = C * E
    transformation_matrix[2, 3] = z
    transformation_matrix[3, 0] = 0
    transformation_matrix[3, 1] = 0
    transformation_matrix[3, 2] = 0
    transformation_matrix[3, 3] = 1
    return transformation_matrix


def yaw_from_robot_base_state(data):
    pose = data.get('pose', {})
    if 'yaw' in pose:
        return pose['yaw']
    orientation = pose.get('orientation', {})
    quat = [
        orientation.get('x', 0.0),
        orientation.get('y', 0.0),
        orientation.get('z', 0.0),
        orientation.get('w', 1.0),
    ]
    return R.from_quat(quat).as_euler('xyz')[2]


def lift_height_field(data, field_names, fallback=0.0):
    for field_name in field_names:
        value = data.get(field_name)
        if value is not None:
            return float(value)
    return float(fallback)


def lift_heights_from_data(data):
    back_height = lift_height_field(data, ['backHeight', 'back_height'])
    target_height = lift_height_field(
        data,
        ['targetHeight', 'target_height', 'val'],
        fallback=back_height,
    )
    return back_height, target_height


def normalize_lift_action_height_to_mm(value):
    # /action/lifting is commonly published in meters (for example 0.2),
    # while LiftMotorSrv and replay scripts use millimeters.
    return value * 1000.0 if abs(value) <= 5.0 else value


def lift_action_height_from_data(data):
    if isinstance(data, dict):
        value = lift_height_field(
            data,
            ['targetHeight', 'target_height', 'val', 'height', 'backHeight', 'back_height'],
        )
        return normalize_lift_action_height_to_mm(value)
    if isinstance(data, str):
        text = data.strip()
        if not text:
            raise ValueError("empty lift action string")
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            return normalize_lift_action_height_to_mm(float(text))
        return lift_action_height_from_data(parsed)
    value = float(data)
    return normalize_lift_action_height_to_mm(value)


def load_lift_motor_records(lift_motor_dir):
    records = []
    if not os.path.isdir(lift_motor_dir):
        return records

    for filename in os.listdir(lift_motor_dir):
        if not filename.endswith('.json'):
            continue
        try:
            timestamp = float(filename[:filename.rfind(".")])
        except ValueError:
            continue
        with open(os.path.join(lift_motor_dir, filename), 'r') as file:
            data = json.load(file)
        back_height, target_height = lift_heights_from_data(data)
        records.append((timestamp, back_height, target_height))

    records.sort(key=lambda record: record[0])
    return records


def load_lift_action_records(lift_action_dir):
    records = []
    if not lift_action_dir or not os.path.isdir(lift_action_dir):
        return records

    for filename in os.listdir(lift_action_dir):
        if not filename.endswith('.json'):
            continue
        try:
            timestamp = float(filename[:filename.rfind(".")])
        except ValueError:
            continue
        with open(os.path.join(lift_action_dir, filename), 'r') as file:
            data = json.load(file)
        records.append((timestamp, lift_action_height_from_data(data)))

    records.sort(key=lambda record: record[0])
    return records


def sample_lift_motor_records(records, timestamps):
    if not records:
        return [], []

    record_times = np.asarray([record[0] for record in records], dtype=np.float64)
    back_heights = np.asarray([record[1] for record in records], dtype=np.float64)
    target_heights = np.asarray([record[2] for record in records], dtype=np.float64)

    sampled_back_heights = []
    sampled_target_heights = []
    for timestamp in timestamps:
        index = int(np.searchsorted(record_times, timestamp, side='right') - 1)
        index = max(0, min(index, len(records) - 1))
        sampled_back_heights.append(float(back_heights[index]))
        sampled_target_heights.append(float(target_heights[index]))
    return sampled_back_heights, sampled_target_heights


def sample_lift_action_records(records, timestamps):
    if not records:
        return []

    record_times = np.asarray([record[0] for record in records], dtype=np.float64)
    target_heights = np.asarray([record[1] for record in records], dtype=np.float64)

    sampled_target_heights = []
    for timestamp in timestamps:
        index = int(np.searchsorted(record_times, timestamp, side='right') - 1)
        index = max(0, min(index, len(records) - 1))
        sampled_target_heights.append(float(target_heights[index]))
    return sampled_target_heights


def replace_list_values(target, values):
    del target[:]
    target.extend(values)


class Operator:
    def __init__(self, args):
        self.args = args
        self.episodeDir = os.path.join(self.args.datasetDir, self.args.episodeName)
        self.cameraColorDirs = [os.path.join(self.episodeDir, "camera/color/" + self.args.cameraColorNames[i]) for i in range(len(self.args.cameraColorNames))]
        self.cameraDepthDirs = [os.path.join(self.episodeDir, "camera/depth/" + self.args.cameraDepthNames[i]) for i in range(len(self.args.cameraDepthNames))]
        self.cameraPointCloudDirs = [os.path.join(self.episodeDir, "camera/pointCloud/" + self.args.cameraPointCloudNames[i] + ("-normalization" if self.args.useCameraPointCloudNormalization else "")) for i in range(len(self.args.cameraPointCloudNames))]
        self.armJointStateDirs = [os.path.join(self.episodeDir, "arm/jointState/" + self.args.armJointStateNames[i]) for i in range(len(self.args.armJointStateNames))]
        self.armEndPoseDirs = [os.path.join(self.episodeDir, "arm/endPose/" + self.args.armEndPoseNames[i]) for i in range(len(self.args.armEndPoseNames))]
        self.localizationPoseDirs = [os.path.join(self.episodeDir, "localization/pose/" + self.args.localizationPoseNames[i]) for i in range(len(self.args.localizationPoseNames))]
        self.gripperEncoderDirs = [os.path.join(self.episodeDir, "gripper/encoder/" + self.args.gripperEncoderNames[i]) for i in range(len(self.args.gripperEncoderNames))]
        self.imu9AxisDirs = [os.path.join(self.episodeDir, "imu/9axis/" + self.args.imu9AxisNames[i]) for i in range(len(self.args.imu9AxisNames))]
        self.lidarPointCloudDirs = [os.path.join(self.episodeDir, "lidar/pointCloud/" + self.args.lidarPointCloudNames[i]) for i in range(len(self.args.lidarPointCloudNames))]
        self.robotBaseVelDirs = [os.path.join(self.episodeDir, "robotBase/vel/" + self.args.robotBaseVelNames[i]) for i in range(len(self.args.robotBaseVelNames))]
        self.robotBaseStateDirs = [os.path.join(self.episodeDir, "robotBase/state/" + self.args.robotBaseStateNames[i]) for i in range(len(self.args.robotBaseStateNames))]
        self.robotBaseActionDirs = [os.path.join(self.episodeDir, "robotBase/action/" + self.args.robotBaseActionNames[i]) for i in range(len(self.args.robotBaseActionNames))]
        self.liftMotorDirs = [os.path.join(self.episodeDir, "lift/motor/" + self.args.liftMotorNames[i]) for i in range(len(self.args.liftMotorNames))]
        self.liftActionDirsByName = {
            name: os.path.join(self.episodeDir, "action/lifting/" + name)
            for name in self.args.liftActionNames
        }
        
        self.cameraColorSyncDirs = [os.path.join(self.cameraColorDirs[i], "sync.txt") for i in range(len(self.args.cameraColorNames))]
        self.cameraDepthSyncDirs = [os.path.join(self.cameraDepthDirs[i], "sync.txt") for i in range(len(self.args.cameraDepthNames))]
        self.cameraPointCloudSyncDirs = [os.path.join(self.cameraPointCloudDirs[i], "sync.txt") for i in range(len(self.args.cameraPointCloudNames))]
        self.armJointStateSyncDirs = [os.path.join(self.armJointStateDirs[i], "sync.txt") for i in range(len(self.args.armJointStateNames))]
        self.armEndPoseSyncDirs = [os.path.join(self.armEndPoseDirs[i], "sync.txt") for i in range(len(self.args.armEndPoseNames))]
        self.localizationPoseSyncDirs = [os.path.join(self.localizationPoseDirs[i], "sync.txt") for i in range(len(self.args.localizationPoseNames))]
        self.gripperEncoderSyncDirs = [os.path.join(self.gripperEncoderDirs[i], "sync.txt") for i in range(len(self.args.gripperEncoderNames))]
        self.imu9AxisSyncDirs = [os.path.join(self.imu9AxisDirs[i], "sync.txt") for i in range(len(self.args.imu9AxisNames))]
        self.lidarPointCloudSyncDirs = [os.path.join(self.lidarPointCloudDirs[i], "sync.txt") for i in range(len(self.args.lidarPointCloudNames))]
        self.robotBaseVelSyncDirs = [os.path.join(self.robotBaseVelDirs[i], "sync.txt") for i in range(len(self.args.robotBaseVelNames))]
        self.robotBaseStateSyncDirs = [os.path.join(self.robotBaseStateDirs[i], "sync.txt") for i in range(len(self.args.robotBaseStateNames))]
        self.robotBaseActionSyncDirs = [os.path.join(self.robotBaseActionDirs[i], "sync.txt") for i in range(len(self.args.robotBaseActionNames))]
        self.liftMotorSyncDirs = [os.path.join(self.liftMotorDirs[i], "sync.txt") for i in range(len(self.args.liftMotorNames))]

        self.cameraColorConfigDirs = [os.path.join(self.cameraColorDirs[i], "config.json") for i in range(len(self.args.cameraColorNames))]
        self.cameraDepthConfigDirs = [os.path.join(self.cameraDepthDirs[i], "config.json") for i in range(len(self.args.cameraDepthNames))]
        self.cameraPointCloudConfigDirs = [os.path.join(self.cameraPointCloudDirs[i], "config.json") for i in range(len(self.args.cameraPointCloudNames))]

        self.instructionsJsonDir = os.path.join(self.episodeDir, "instructions.json")
        if self.args.useIndex:
            self.dataFile = os.path.join(self.episodeDir, self.args.episodeName + ".hdf5")
            early_dataFile = os.path.join(self.episodeDir, "data.hdf5")
            os.system(f"rm -rf {early_dataFile}")
        else:
            os.makedirs(self.args.targetDir, exist_ok=True)
            self.dataFile = os.path.join(self.args.targetDir, self.args.episodeName + ".hdf5")

    def process(self):
        error = False
        data_dict = {}
        for cameraColorName in self.args.cameraColorNames:
            data_dict[f'camera/color/{cameraColorName}'] = []
        for cameraDepthName in self.args.cameraDepthNames:
            data_dict[f'camera/depth/{cameraDepthName}'] = []
            data_dict[f'camera/depthIntrinsic/{cameraDepthName}'] = []
            data_dict[f'camera/depthExtrinsic/{cameraDepthName}'] = []
        for cameraPointCloudName in self.args.cameraPointCloudNames:
            data_dict[f'camera/pointCloud/{cameraPointCloudName}'] = []
            data_dict[f'camera/pointCloudIntrinsic/{cameraPointCloudName}'] = []
            data_dict[f'camera/pointCloudExtrinsic/{cameraPointCloudName}'] = []
        for armJointStateName in self.args.armJointStateNames:
            data_dict[f'arm/jointStateVelocity/{armJointStateName}'] = []
            data_dict[f'arm/jointStatePosition/{armJointStateName}'] = []
            data_dict[f'arm/jointStateEffort/{armJointStateName}'] = []
        for armEndPoseName in self.args.armEndPoseNames:
            data_dict[f'arm/endPose/{armEndPoseName}'] = []
        for localizationPoseName in self.args.localizationPoseNames:
            data_dict[f'localization/pose/{localizationPoseName}'] = []
        for gripperEncoderName in self.args.gripperEncoderNames:
            data_dict[f'gripper/encoderAngle/{gripperEncoderName}'] = []
            data_dict[f'gripper/encoderDistance/{gripperEncoderName}'] = []
        for imu9AxisName in self.args.imu9AxisNames:
            data_dict[f'imu/9axisOrientation/{imu9AxisName}'] = []
            data_dict[f'imu/9axisAngularVelocity/{imu9AxisName}'] = []
            data_dict[f'imu/9axisLinearAcceleration/{imu9AxisName}'] = []
        for lidarPointCloudName in self.args.lidarPointCloudNames:
            data_dict[f'lidar/pointCloud/{lidarPointCloudName}'] = []
        for robotBaseVelName in self.args.robotBaseVelNames:
            data_dict[f'robotBase/vel/{robotBaseVelName}'] = []
        for robotBaseStateName in self.args.robotBaseStateNames:
            data_dict[f'robotBase/state/{robotBaseStateName}'] = []
        for robotBaseActionName in self.args.robotBaseActionNames:
            data_dict[f'robotBase/action/{robotBaseActionName}'] = []
        legacyRobotBaseVelNames = [
            name for name in self.args.robotBaseStateNames
            if name not in self.args.robotBaseVelNames
        ]
        for robotBaseVelName in legacyRobotBaseVelNames:
            data_dict[f'robotBase/vel/{robotBaseVelName}'] = []
        for liftMotorName in self.args.liftMotorNames:
            data_dict[f'lift/motor/{liftMotorName}'] = []
            data_dict[f'action/lifting/{liftMotorName}'] = []
        data_dict[f'instructions/full_instructions/text'] = []
        data_dict[f'instructions/segment_instructions/start_time'] = []
        data_dict[f'instructions/segment_instructions/end_time'] = []
        data_dict[f'instructions/full_instructions/text'].append('null')
        if os.path.exists(self.instructionsJsonDir):
            with open(self.instructionsJsonDir, 'r') as file:
                data = json.load(file)
                data_dict[f'instructions/full_instructions/text'] = []
                if 'full-instructions' in data.keys():
                    for text in data['full-instructions']:
                        data_dict[f'instructions/full_instructions/text'].append(text)
                elif 'instructions' in data.keys():
                    for text in data['instructions']:
                        data_dict[f'instructions/full_instructions/text'].append(text)
                else:
                    data_dict[f'instructions/full_instructions/text'].append('null')
                if 'segment-instructions' in data.keys():
                    for segment_instructions in data['segment-instructions']:
                        start_time = segment_instructions['start_time']
                        end_time = segment_instructions['end_time']
                        data_dict[f'instructions/segment_instructions/start_time'].append(start_time)
                        data_dict[f'instructions/segment_instructions/end_time'].append(end_time)
                        data_dict[f'instructions/segment_instructions/{start_time}-{end_time}/text'] = []
                        for text in segment_instructions['instructions']:
                            data_dict[f'instructions/segment_instructions/{start_time}-{end_time}/text'].append(text)
        data_dict[f'timestamp'] = []
        data_dict[f'size'] = 0
        size_count = 0
        for i in range(len(self.args.cameraColorNames)):
            with open(self.cameraColorSyncDirs[i], 'r') as lines:
                count = 0
                for t, line in enumerate(lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    if self.args.useIndex:
                        data_dict[f'camera/color/{self.args.cameraColorNames[i]}'].append(os.path.join(self.cameraColorDirs[i][len(self.episodeDir)+1:], line))
                    else:
                        data_dict[f'camera/color/{self.args.cameraColorNames[i]}'].append(cv2.imread(os.path.join(self.cameraColorDirs[i], line), cv2.IMREAD_UNCHANGED))
                    # print(os.path.join(self.cameraColorDirs[i], line))
                    # cv2.imread(os.path.join(self.cameraColorDirs[i], line))
                    count += 1
                if size_count == 0:
                    size_count = count
            if os.path.exists(self.cameraColorConfigDirs[i]):
                with open(self.cameraColorConfigDirs[i], 'r') as color_config_file:
                    data = json.load(color_config_file)
                    color_intrinsic = np.array(data["K"]).reshape(3, 3)
                    if "parent_frame" in data:
                        pf = data["parent_frame"]
                        color_extrinsic = create_transformation_matrix(
                            pf['x'], pf['y'], pf['z'], pf['roll'], pf['pitch'], pf['yaw'])
                    else:
                        color_extrinsic = np.eye(4)
                    data_dict[f'camera/colorIntrinsic/{self.args.cameraColorNames[i]}'] = color_intrinsic
                    data_dict[f'camera/colorExtrinsic/{self.args.cameraColorNames[i]}'] = color_extrinsic
        for i in range(len(self.args.cameraDepthNames)):
            with open(self.cameraDepthSyncDirs[i], 'r') as lines:
                count = 0
                for t, line in enumerate(lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    if self.args.useIndex:
                        data_dict[f'camera/depth/{self.args.cameraDepthNames[i]}'].append(os.path.join(self.cameraDepthDirs[i][len(self.episodeDir)+1:], line))
                    else:
                        data_dict[f'camera/depth/{self.args.cameraDepthNames[i]}'].append(cv2.imread(os.path.join(self.cameraDepthDirs[i], line), cv2.IMREAD_UNCHANGED))
                    # print(os.path.join(self.cameraDepthDirs[i], line))
                    # img = cv2.imread(os.path.join(self.cameraDepthDirs[i], line), cv2.IMREAD_UNCHANGED).flatten()
                    # print(max(img), min(img))
                    count += 1
                if size_count == 0:
                    size_count = count
            if os.path.exists(self.cameraDepthConfigDirs[i]):
                with open(self.cameraDepthConfigDirs[i], 'r') as depth_config_file:
                    data = json.load(depth_config_file)
                    depth_intrinsic = np.array(data["K"]).reshape(3, 3)
                    if "parent_frame" in data:
                        pf = data["parent_frame"]
                        depth_extrinsic = create_transformation_matrix(
                            pf['x'], pf['y'], pf['z'], pf['roll'], pf['pitch'], pf['yaw'])
                    else:
                        depth_extrinsic = np.eye(4)
                    data_dict[f'camera/depthIntrinsic/{self.args.cameraDepthNames[i]}'] = depth_intrinsic
                    data_dict[f'camera/depthExtrinsic/{self.args.cameraDepthNames[i]}'] = depth_extrinsic
        for i in range(len(self.args.cameraPointCloudNames)):
            with open(self.cameraPointCloudSyncDirs[i], 'r') as lines:
                count = 0
                for t, line in enumerate(lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    if self.args.useIndex:
                        data_dict[f'camera/pointCloud/{self.args.cameraPointCloudNames[i]}'].append(os.path.join(self.cameraPointCloudDirs[i][len(self.episodeDir)+1:], line))
                    else:
                        data_dict[f'camera/pointCloud/{self.args.cameraPointCloudNames[i]}'].append(np.load(os.path.join(self.cameraPointCloudDirs[i], line)))
                    count += 1
                if size_count == 0:
                    size_count = count
            # with open(self.cameraPointCloudConfigDirs[i], 'r') as point_cloud_config_file:
            #     data = json.load(point_cloud_config_file)
            #     point_cloud_intrinsic = np.array(data["K"]).reshape(3, 3)
            #     point_cloud_extrinsic = create_transformation_matrix(data["parent_frame"]['x'], data["parent_frame"]['y'], data["parent_frame"]['z'], data["parent_frame"]['roll'], data["parent_frame"]['pitch'], data["parent_frame"]['yaw'])
            #     data_dict[f'camera/pointCloudIntrinsic/{self.args.cameraPointCloudNames[i]}'] = point_cloud_intrinsic
            #     data_dict[f'camera/pointCloudExtrinsic/{self.args.cameraPointCloudNames[i]}'] = point_cloud_extrinsic
        for i in range(len(self.args.armJointStateNames)):
            master_arm_gripper_mm = False
            if 'master' in self.args.armJointStateNames[i]:
                with open(self.armJointStateSyncDirs[i], 'r') as lines:
                    count = 0
                    for t, line in enumerate(lines):
                        line = line.replace('\n', '')
                        with open(os.path.join(self.armJointStateDirs[i], line), 'r') as file:
                            data = json.load(file)
                            position = np.array(data['position'])
                            if abs(position[6]) > 1:
                                count += 1
                            if count > 30:
                                master_arm_gripper_mm = True
                                break
            arm_not_moving = True
            with open(self.armJointStateSyncDirs[i], 'r') as lines:
                first_position = None
                for t, line in enumerate(lines):
                    line = line.replace('\n', '')
                    with open(os.path.join(self.armJointStateDirs[i], line), 'r') as file:
                        data = json.load(file)
                        position = np.array(data['position'])
                        if first_position is None:
                            first_position = position
                        if not (position - first_position == 0).all():
                            arm_not_moving = False
                            break
            if arm_not_moving:
                error = True
                print(self.args.armJointStateNames[i], "arm not moving!!!!!!!!!!")
            with open(self.armJointStateSyncDirs[i], 'r') as lines:
                count = 0
                for t, line in enumerate(lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.armJointStateDirs[i], line), 'r') as file:
                        data = json.load(file)
                        limit_lower = np.array([-2.6179, 0, -2.967, -1.745, -1.22, -2.09439, 0])
                        limit_upper = np.array([2.6179, 3.14, 0, 1.745, 1.22, 2.09439, 0.10])
                        position = np.array(data['position'])
                        if master_arm_gripper_mm:
                            position[6] /= 1000
                        out_limit = ((position - limit_lower) < -0.2).any() or ((limit_upper - position) < -0.2).any()
                        if out_limit:
                            error = True
                            print(self.args.armJointStateNames[i], position)
                            min_val, flat_idx = (position - limit_lower).min(), (position - limit_lower).argmin()
                            print("lower:", min_val, flat_idx)
                            min_val, flat_idx = (limit_upper - position).min(), (limit_upper - position).argmin()
                            print("upper:", min_val, flat_idx)
                            print("out_limit!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        data_dict[f'arm/jointStateVelocity/{self.args.armJointStateNames[i]}'].append(np.array(data['velocity']))
                        data_dict[f'arm/jointStateEffort/{self.args.armJointStateNames[i]}'].append(np.array(data['effort']))
                        data_dict[f'arm/jointStatePosition/{self.args.armJointStateNames[i]}'].append(np.array(data['position']))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.armEndPoseNames)):
            with open(self.armEndPoseSyncDirs[i], 'r') as lines:
                count = 0
                for t, line in enumerate(lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.armEndPoseDirs[i], line), 'r') as file:
                        data = json.load(file)
                        if 'grasper' in data.keys():
                            data_dict[f'arm/endPose/{self.args.armEndPoseNames[i]}'].append(np.array([data['x'], data['y'], data['z'], data['roll'], data['pitch'], data['yaw'], data['grasper']]))
                        else:
                            data_dict[f'arm/endPose/{self.args.armEndPoseNames[i]}'].append(np.array([data['x'], data['y'], data['z'], data['roll'], data['pitch'], data['yaw']]))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.localizationPoseNames)):
            with open(self.localizationPoseSyncDirs[i], 'r') as lines:
                count = 0
                for t, line in enumerate(lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.localizationPoseDirs[i], line), 'r') as file:
                        data = json.load(file)
                        # ori_trans = create_transformation_matrix(data['x'], data['y'], data['z'], data['roll'], data['pitch'], data['yaw'])
                        # incre_trans = create_transformation_matrix(0, 0, 0, 0, math.pi/4, 0)
                        # final_trans = np.dot(ori_trans, incre_trans)
                        # xyzrpy = matrix_to_xyzrpy(final_trans)
                        data_dict[f'localization/pose/{self.args.localizationPoseNames[i]}'].append(np.array([data['x'], data['y'], data['z'], data['roll'], data['pitch'], data['yaw']]))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.gripperEncoderNames)):
            with open(self.gripperEncoderSyncDirs[i], 'r') as lines:
                count = 0
                for t, line in enumerate(lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.gripperEncoderDirs[i], line), 'r') as file:
                        data = json.load(file)
                        data_dict[f'gripper/encoderAngle/{self.args.gripperEncoderNames[i]}'].append(data['angle'])
                        data_dict[f'gripper/encoderDistance/{self.args.gripperEncoderNames[i]}'].append(data['distance'])
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.imu9AxisNames)):
            with open(self.imu9AxisSyncDirs[i], 'r') as lines:
                count = 0
                for t, line in enumerate(lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.imu9AxisDirs[i], line), 'r') as file:
                        data = json.load(file)
                        data_dict[f'imu/9axisOrientation/{self.args.imu9AxisNames[i]}'].append(np.array([data['orientation']['x'], data['orientation']['y'], data['orientation']['z'], data['orientation']['w']]))
                        data_dict[f'imu/9axisAngularVelocity/{self.args.imu9AxisNames[i]}'].append(np.array([data['angular_velocity']['x'], data['angular_velocity']['y'], data['angular_velocity']['z']]))
                        data_dict[f'imu/9axisLinearAcceleration/{self.args.imu9AxisNames[i]}'].append(np.array([data['linear_acceleration']['x'], data['linear_acceleration']['y'], data['linear_acceleration']['z']]))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.lidarPointCloudNames)):
            with open(self.lidarPointCloudSyncDirs[i], 'r') as lines:
                count = 0
                for t, line in enumerate(lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    data_dict[f'lidar/pointCloud/{self.args.lidarPointCloudNames[i]}'].append(os.path.join(self.lidarPointCloudDirs[i][len(self.episodeDir)+1:], line))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.robotBaseVelNames)):
            with open(self.robotBaseVelSyncDirs[i], 'r') as lines:
                count = 0
                for t, line in enumerate(lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.robotBaseVelDirs[i], line), 'r') as file:
                        data = json.load(file)
                        data_dict[f'robotBase/vel/{self.args.robotBaseVelNames[i]}'].append(np.array([data['linear']['x'], data['linear']['y'], data['angular']['z']]))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.robotBaseStateNames)):
            with open(self.robotBaseStateSyncDirs[i], 'r') as lines:
                count = 0
                name = self.args.robotBaseStateNames[i]
                for t, line in enumerate(lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    with open(os.path.join(self.robotBaseStateDirs[i], line), 'r') as file:
                        data = json.load(file)
                        position = data.get('pose', {}).get('position', {})
                        yaw = yaw_from_robot_base_state(data)
                        linear = data.get('linear', {})
                        angular = data.get('angular', {})
                        vx = float(linear.get('x', 0.0))
                        vy = float(linear.get('y', 0.0))
                        wz = float(angular.get('z', 0.0))
                        vel_key = f'robotBase/vel/{name}'
                        if name in self.args.robotBaseVelNames and count < len(data_dict.get(vel_key, [])):
                            vel_sample = np.asarray(data_dict[vel_key][count], dtype=np.float64).reshape(-1)
                            if len(vel_sample) >= 3:
                                vx, vy, wz = float(vel_sample[0]), float(vel_sample[1]), float(vel_sample[2])
                        data_dict[f'robotBase/state/{name}'].append(np.array([
                            position.get('x', 0.0),
                            position.get('y', 0.0),
                            yaw,
                            vx,
                            vy,
                            wz,
                        ]))
                        if name in legacyRobotBaseVelNames:
                            data_dict[f'robotBase/vel/{name}'].append(np.array([vx, vy, wz]))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.robotBaseActionNames)):
            with open(self.robotBaseActionSyncDirs[i], 'r') as lines:
                count = 0
                for t, line in enumerate(lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.robotBaseActionDirs[i], line), 'r') as file:
                        data = json.load(file)
                        data_dict[f'robotBase/action/{self.args.robotBaseActionNames[i]}'].append(np.array([data['linear']['x'], data['linear']['y'], data['angular']['z']]))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.liftMotorNames)):
            name = self.args.liftMotorNames[i]
            motor_key = f'lift/motor/{name}'
            action_key = f'action/lifting/{name}'
            fallback_target_heights = []
            sync_lines = []
            if os.path.exists(self.liftMotorSyncDirs[i]):
                with open(self.liftMotorSyncDirs[i], 'r') as lines:
                    sync_lines = [line.strip() for line in lines if line.strip()]

            if sync_lines:
                count = 0
                for t, line in enumerate(sync_lines):
                    if t % self.args.timeInterval != 0:
                        continue
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.liftMotorDirs[i], line), 'r') as file:
                        data = json.load(file)
                        back_height, target_height = lift_heights_from_data(data)
                        data_dict[motor_key].append(back_height)
                        fallback_target_heights.append(target_height)
                    count += 1
                if size_count == 0:
                    size_count = count
            elif size_count > 0 and len(data_dict[f'timestamp']) >= size_count:
                records = load_lift_motor_records(self.liftMotorDirs[i])
                if records:
                    timestamps = data_dict[f'timestamp'][:size_count]
                    back_heights, target_heights = sample_lift_motor_records(records, timestamps)
                    data_dict[motor_key].extend(back_heights)
                    fallback_target_heights = target_heights
                    print(
                        f"{motor_key}: resampled {len(records)} raw lift samples "
                        f"to {size_count} main frames"
                    )
                else:
                    print(f"Warning: {motor_key} has no raw data; lift datasets omitted")
                    del data_dict[motor_key]
                    del data_dict[action_key]
            else:
                print(f"Warning: {motor_key} has no sync timeline; lift datasets omitted")
                del data_dict[motor_key]
                del data_dict[action_key]

            if motor_key in data_dict:
                timestamps = data_dict[f'timestamp'][:len(data_dict[motor_key])]
                lift_action_dir = self.liftActionDirsByName.get(
                    name,
                    os.path.join(self.episodeDir, "action/lifting/" + name),
                )
                action_records = load_lift_action_records(lift_action_dir)
                if action_records:
                    action_targets = sample_lift_action_records(action_records, timestamps)
                    replace_list_values(data_dict[action_key], action_targets)
                    print(
                        f"{action_key}: sampled {len(action_records)} raw /action/lifting samples "
                        f"to {len(action_targets)} main frames"
                    )
                elif fallback_target_heights:
                    replace_list_values(data_dict[action_key], fallback_target_heights)
                    print(f"{action_key}: fallback to targetHeight/backHeight from {motor_key}")
                else:
                    print(f"Warning: {action_key} has no raw action data; lift action omitted")
                    del data_dict[action_key]
        data_dict['size'] = size_count
        if True:  # not error:
            with h5py.File(self.dataFile, 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
                for key in data_dict:
                    root.create_dataset(key, data=data_dict[key])
        return error


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--episodeName', action='store', type=str, help='episodeName',
                        default="", required=False)
    parser.add_argument('--targetDir', action='store', type=str, help='targetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--useIndex', action='store', type=bool, help='useIndex',
                        default=True, required=False)
    parser.add_argument('--type', action='store', type=str, help='type',
                        default="aloha", required=False)
    parser.add_argument('--paramsFile', action='store', type=str, help='paramsFile',
                        default="", required=False)
    parser.add_argument('--timeInterval', action='store', type=str, help='timeInterval',
                        default=1, required=False)
    parser.add_argument('--cameraColorNames', action='store', type=str, help='cameraColorNames',
                        default=[], required=False)
    parser.add_argument('--cameraDepthNames', action='store', type=str, help='cameraDepthNames',
                        default=[], required=False)
    parser.add_argument('--cameraPointCloudNames', action='store', type=str, help='cameraPointCloudNames',
                        default=[], required=False)
    parser.add_argument('--useCameraPointCloud', action='store', type=bool, help='useCameraPointCloud',
                        default=False, required=False)
    parser.add_argument('--useCameraPointCloudNormalization', action='store', type=bool, help='useCameraPointCloudNormalization',
                        default=True, required=False)
    parser.add_argument('--armJointStateNames', action='store', type=str, help='armJointStateNames',
                        default=[], required=False)
    parser.add_argument('--armEndPoseNames', action='store', type=str, help='armEndPoseNames',
                        default=[], required=False)
    parser.add_argument('--localizationPoseNames', action='store', type=str, help='localizationPoseNames',
                        default=[], required=False)
    parser.add_argument('--gripperEncoderNames', action='store', type=str, help='gripperEncoderNames',
                        default=[], required=False)
    parser.add_argument('--imu9AxisNames', action='store', type=str, help='imu9AxisNames',
                        default=[], required=False)
    parser.add_argument('--lidarPointCloudNames', action='store', type=str, help='lidarPointCloudNames',
                        default=[], required=False)
    parser.add_argument('--robotBaseVelNames', action='store', type=str, help='robotBaseVelNames',
                        default=[], required=False)
    parser.add_argument('--robotBaseStateNames', action='store', type=str, help='robotBaseStateNames',
                        default=[], required=False)
    parser.add_argument('--robotBaseActionNames', action='store', type=str, help='robotBaseActionNames',
                        default=[], required=False)
    parser.add_argument('--liftMotorNames', action='store', type=str, help='liftMotorNames',
                        default=[], required=False)
    parser.add_argument('--liftActionNames', action='store', type=str, help='liftActionNames',
                        default=[], required=False)
    args = parser.parse_args()

    params_file = args.paramsFile or f'../config/{args.type}_data_params.yaml'
    with open(params_file, 'r') as file:
        yaml_data = yaml.safe_load(file)
        args.cameraColorNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('color', {}).get('names', [])
        args.cameraDepthNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('depth', {}).get('names', [])
        args.cameraPointCloudNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('pointCloud', {}).get('names', [])
        args.armJointStateNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('arm', {}).get('jointState', {}).get('names', [])
        args.armEndPoseNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('arm', {}).get('endPose', {}).get('names', [])
        args.localizationPoseNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('localization', {}).get('pose', {}).get('names', [])
        args.gripperEncoderNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('gripper', {}).get('encoder', {}).get('names', [])
        args.imu9AxisNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('imu', {}).get('9axis', {}).get('names', [])
        args.lidarPointCloudNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('lidar', {}).get('pointCloud', {}).get('names', [])
        args.robotBaseVelNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('robotBase', {}).get('vel', {}).get('names', [])
        args.robotBaseStateNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('robotBase', {}).get('state', {}).get('names', [])
        args.robotBaseActionNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('robotBase', {}).get('action', {}).get('names', [])
        args.liftMotorNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('lift', {}).get('motor', {}).get('names', [])
        args.liftActionNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('action', {}).get('lifting', {}).get('names', [])
        if not args.liftActionNames:
            args.liftActionNames = args.liftMotorNames
        if args.useCameraPointCloud:
            args.cameraPointCloudNames = args.cameraDepthNames
    return args


def main():
    args = get_arguments()
    if not args.useCameraPointCloud:
        args.cameraPointCloudNames = []
    if args.episodeName == "":
        for f in os.listdir(args.datasetDir):
            if not f.endswith(".tar.gz"):
                args.episodeName = f
                print("episode name: ", args.episodeName, "processing")
                operator = Operator(args)
                operator.process()
                print("episode name: ", args.episodeName, "done")
    else:
        operator = Operator(args)
        operator.process()
    print("Done")


if __name__ == '__main__':
    main()
