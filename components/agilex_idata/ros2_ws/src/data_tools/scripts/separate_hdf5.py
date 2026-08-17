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
from pathlib import Path
import glob


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


class Operator:
    def __init__(self, args, dataset_dir):
        self.args = args
        self.hdf5_dir = dataset_dir

    def process(self):
        with h5py.File(self.hdf5_dir, 'r') as root:
            for i, (start_time, end_time) in enumerate(zip(root['instructions/segment_instructions/start_time'][()], root['instructions/segment_instructions/end_time'][()])):
                save_hdf5_dir = self.hdf5_dir[:-5]+str(i)+'.hdf5'
                start_index = -1
                end_index= -1
                for j, timestamp in enumerate(root[f'timestamp'][()]):
                    if start_index == -1 and timestamp >= start_time:
                        start_index = j
                    if end_index == -1 and timestamp > end_time:
                        end_index = j
                        break
                if start_index == -1 or end_index == -1:
                    continue
                data_dict = {}
                for cameraColorName in self.args.cameraColorNames:
                    data_dict[f'camera/color/{cameraColorName}'] = root[f'camera/color/{cameraColorName}'][start_index:end_index]
                    data_dict[f'camera/colorIntrinsic/{cameraColorName}'] = root[f'camera/colorIntrinsic/{cameraColorName}']
                    data_dict[f'camera/colorExtrinsic/{cameraColorName}'] = root[f'camera/colorExtrinsic/{cameraColorName}']
                for cameraDepthName in self.args.cameraDepthNames:
                    data_dict[f'camera/depth/{cameraDepthName}'] = root[f'camera/depth/{cameraDepthName}'][start_index:end_index]
                    data_dict[f'camera/depthIntrinsic/{cameraDepthName}'] = root[f'camera/depthIntrinsic/{cameraDepthName}']
                    data_dict[f'camera/depthExtrinsic/{cameraDepthName}'] = root[f'camera/depthExtrinsic/{cameraDepthName}']
                for cameraPointCloudName in self.args.cameraPointCloudNames:
                    data_dict[f'camera/pointCloud/{cameraPointCloudName}'] = root[f'camera/pointCloud/{cameraPointCloudName}'][start_index:end_index]
                    data_dict[f'camera/pointCloudIntrinsic/{cameraPointCloudName}'] = root[f'camera/pointCloudIntrinsic/{cameraPointCloudName}']
                    data_dict[f'camera/pointCloudExtrinsic/{cameraPointCloudName}'] = root[f'camera/pointCloudExtrinsic/{cameraPointCloudName}']
                for armJointStateName in self.args.armJointStateNames:
                    data_dict[f'arm/jointStateVelocity/{armJointStateName}'] = root[f'arm/jointStateVelocity/{armJointStateName}'][start_index:end_index]
                    data_dict[f'arm/jointStatePosition/{armJointStateName}'] = root[f'arm/jointStatePosition/{armJointStateName}'][start_index:end_index]
                    data_dict[f'arm/jointStateEffort/{armJointStateName}'] = root[f'arm/jointStateEffort/{armJointStateName}'][start_index:end_index]
                for armEndPoseName in self.args.armEndPoseNames:
                    data_dict[f'arm/endPose/{armEndPoseName}'] = root[f'arm/endPose/{armEndPoseName}'][start_index:end_index]
                for localizationPoseName in self.args.localizationPoseNames:
                    data_dict[f'localization/pose/{localizationPoseName}'] = root[f'localization/pose/{localizationPoseName}'][start_index:end_index]
                for gripperEncoderName in self.args.gripperEncoderNames:
                    data_dict[f'gripper/encoderAngle/{gripperEncoderName}'] = root[f'gripper/encoderAngle/{gripperEncoderName}'][start_index:end_index]
                    data_dict[f'gripper/encoderDistance/{gripperEncoderName}'] = root[f'gripper/encoderDistance/{gripperEncoderName}'][start_index:end_index]
                for imu9AxisName in self.args.imu9AxisNames:
                    data_dict[f'imu/9axisOrientation/{imu9AxisName}'] = root[f'imu/9axisOrientation/{imu9AxisName}'][start_index:end_index]
                    data_dict[f'imu/9axisAngularVelocity/{imu9AxisName}'] = root[f'imu/9axisAngularVelocity/{imu9AxisName}'][start_index:end_index]
                    data_dict[f'imu/9axisLinearAcceleration/{imu9AxisName}'] = root[f'imu/9axisLinearAcceleration/{imu9AxisName}'][start_index:end_index]
                for lidarPointCloudName in self.args.lidarPointCloudNames:
                    data_dict[f'lidar/pointCloud/{lidarPointCloudName}'] = root[f'lidar/pointCloud/{lidarPointCloudName}'][start_index:end_index]
                for robotBaseVelName in self.args.robotBaseVelNames:
                    data_dict[f'robotBase/vel/{robotBaseVelName}'] = root[f'robotBase/vel/{robotBaseVelName}'][start_index:end_index]
                for robotBaseStateName in self.args.robotBaseStateNames:
                    data_dict[f'robotBase/state/{robotBaseStateName}'] = root[f'robotBase/state/{robotBaseStateName}'][start_index:end_index]
                for robotBaseActionName in self.args.robotBaseActionNames:
                    data_dict[f'robotBase/action/{robotBaseActionName}'] = root[f'robotBase/action/{robotBaseActionName}'][start_index:end_index]
                for liftMotorName in self.args.liftMotorNames:
                    data_dict[f'lift/motor/{liftMotorName}'] = root[f'lift/motor/{liftMotorName}'][start_index:end_index]
                    lifting_action_key = f'action/lifting/{liftMotorName}'
                    if lifting_action_key in root:
                        data_dict[lifting_action_key] = root[lifting_action_key][start_index:end_index]
                data_dict[f'instructions/full_instructions/text'] = root[f'instructions/segment_instructions/{start_time}-{end_time}/text']
                data_dict[f'instructions/segment_instructions/start_time'] = []
                data_dict[f'instructions/segment_instructions/end_time'] = []
                data_dict[f'timestamp'] = root[f'timestamp'][start_index:end_index]
                data_dict[f'size'] = end_index - start_index
                with h5py.File(save_hdf5_dir, 'w', rdcc_nbytes=1024 ** 2 * 2) as save_root:
                    for key in data_dict:
                        save_root.create_dataset(key, data=data_dict[key])




def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--type', action='store', type=str, help='type',
                        default="aloha", required=False)
    parser.add_argument('--cameraColorNames', action='store', type=str, help='cameraColorNames',
                        default=[], required=False)
    parser.add_argument('--cameraDepthNames', action='store', type=str, help='cameraDepthNames',
                        default=[], required=False)
    parser.add_argument('--cameraPointCloudNames', action='store', type=str, help='cameraPointCloudNames',
                        default=[], required=False)
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
    args = parser.parse_args()

    with open(f'../config/{args.type}_data_params.yaml', 'r') as file:
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
    return args


def main():
    args = get_arguments()
    if not args.datasetDir.endswith(".hdf5"):
        hdf5_files = []
        for f in os.listdir(args.datasetDir):
            if f.endswith(".hdf5"):
                hdf5_files.append(os.path.join(args.datasetDir, f))
            if os.path.isdir(os.path.join(args.datasetDir, f)):
                hdf5_files.extend(glob.glob(os.path.join(args.datasetDir, f, "*.hdf5")))
        for f in hdf5_files:
            print(f, "processing")
            operator = Operator(args, f)
            operator.process()
            print(f, "done")
    else:
        operator = Operator(args, args.datasetDir)
        operator.process()
    print("Done")


if __name__ == '__main__':
    main()
