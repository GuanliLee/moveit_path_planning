"""
Script to convert Aloha hdf5 data to the LeRobot dataset v2.0 format.

Example usage: uv run examples/aloha_real/convert_aloha_data_to_lerobot.py --raw-dir /path/to/raw/data --repo-id <org>/<dataset-name>
"""

import dataclasses
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import shutil
import subprocess
from typing import Literal

import h5py
# from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
import lerobot.datasets.lerobot_dataset as lerobot_dataset_module
from lerobot.datasets.lerobot_dataset import LeRobotDataset
# from lerobot.common.datasets.push_dataset_to_hub._download_raw import download_raw
import numpy as np
import torch
import tqdm
import tyro
import cv2
import os
import argparse
import math
import yaml
import glob
from random import choice


@dataclasses.dataclass(frozen=True)
class DatasetConfig:
    use_videos: bool = True
    tolerance_s: float = 0.0001
    image_writer_processes: int = 8
    image_writer_threads: int = 8
    video_backend: str | None = None


DEFAULT_DATASET_CONFIG = DatasetConfig()


def patch_lerobot_video_encoder_from_env():
    video_codec = os.environ.get("LEROBOT_VIDEO_CODEC", "").strip()
    if not video_codec:
        return

    if video_codec not in {"h264_nvenc", "hevc_nvenc"}:
        raise ValueError(
            f"Unsupported LEROBOT_VIDEO_CODEC={video_codec!r}; expected h264_nvenc or hevc_nvenc"
        )

    def encode_video_frames_nvenc(
        imgs_dir,
        video_path,
        fps,
        vcodec="libsvtav1",
        pix_fmt="yuv420p",
        g=2,
        crf=30,
        fast_decode=0,
        log_level=None,
        overwrite=False,
    ):
        del vcodec, fast_decode, log_level
        imgs_dir = Path(imgs_dir)
        video_path = Path(video_path)
        video_path.parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            os.environ.get("LEROBOT_FFMPEG_LOGLEVEL", "error"),
        ]
        if overwrite:
            cmd.append("-y")
        else:
            cmd.append("-n")
        cmd += [
            "-framerate",
            str(fps),
            "-i",
            str(imgs_dir / "frame_%06d.png"),
            "-c:v",
            video_codec,
            "-preset",
            os.environ.get("LEROBOT_NVENC_PRESET", "p4"),
            "-bf",
            os.environ.get("LEROBOT_NVENC_BFRAMES", "0"),
            "-pix_fmt",
            pix_fmt,
        ]
        if g is not None:
            cmd += ["-g", str(g)]
        if crf is not None:
            cmd += ["-cq", str(crf)]
        cmd.append(str(video_path))
        subprocess.run(cmd, check=True)
        if not video_path.exists():
            raise FileNotFoundError(f"Video encoding failed: {video_path}")

    lerobot_dataset_module.encode_video_frames = encode_video_frames_nvenc
    print(f"Using ffmpeg GPU video codec: {video_codec}")


def env_int(name, default):
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError(f"{name} must be an integer, got {value!r}") from exc


def env_flag(name, default=False):
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def patch_lerobot_parallel_video_encoding_from_env():
    video_workers = env_int("LEROBOT_VIDEO_WORKERS", 1)
    if video_workers <= 1:
        return

    def encode_episode_videos_parallel(self, episode_index: int) -> None:
        def encode_video_key(key):
            video_path = self.root / self.meta.get_video_file_path(episode_index, key)
            if video_path.is_file():
                return

            img_dir = self._get_image_file_path(
                episode_index=episode_index,
                image_key=key,
                frame_index=0,
            ).parent
            lerobot_dataset_module.encode_video_frames(img_dir, video_path, self.fps, overwrite=True)
            shutil.rmtree(img_dir)

        max_workers = min(video_workers, len(self.meta.video_keys))
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            futures = [executor.submit(encode_video_key, key) for key in self.meta.video_keys]
            for future in as_completed(futures):
                future.result()

        # Video info only needs to be inferred once. Keep this write outside the worker threads.
        if len(self.meta.video_keys) > 0 and episode_index == 0:
            self.meta.update_video_info()
            lerobot_dataset_module.write_info(self.meta.info, self.meta.root)

    LeRobotDataset.encode_episode_videos = encode_episode_videos_parallel
    print(f"Using parallel video encoding workers: {video_workers}")


def resolve_episode_path(episode_path: Path, value):
    image_path = hdf5_string(value)
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(str(episode_path)), image_path)
    return image_path


def encode_video_from_source_images(image_paths, video_path, fps):
    if not image_paths:
        raise ValueError(f"No source images for {video_path}")

    video_codec = os.environ.get("LEROBOT_VIDEO_CODEC", "h264_nvenc").strip() or "h264_nvenc"
    suffix = Path(image_paths[0]).suffix or ".jpg"
    video_path = Path(video_path)
    video_path.parent.mkdir(parents=True, exist_ok=True)
    link_dir = video_path.parent / f".{video_path.stem}_links"
    if link_dir.exists():
        shutil.rmtree(link_dir)
    link_dir.mkdir(parents=True, exist_ok=True)

    try:
        for i, image_path in enumerate(image_paths):
            src = Path(image_path)
            if not src.exists():
                raise FileNotFoundError(f"Missing source image: {src}")
            os.symlink(src, link_dir / f"frame_{i:06d}{suffix}")

        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            os.environ.get("LEROBOT_FFMPEG_LOGLEVEL", "error"),
            "-y",
            "-framerate",
            str(fps),
            "-f",
            "image2",
            "-i",
            str(link_dir / f"frame_%06d{suffix}"),
            "-c:v",
            video_codec,
            "-pix_fmt",
            "yuv420p",
            "-g",
            "2",
        ]
        if video_codec in {"h264_nvenc", "hevc_nvenc"}:
            cmd += [
                "-preset",
                os.environ.get("LEROBOT_NVENC_PRESET", "p4"),
                "-bf",
                os.environ.get("LEROBOT_NVENC_BFRAMES", "0"),
                "-cq",
                os.environ.get("LEROBOT_NVENC_CQ", "30"),
            ]
        cmd.append(str(video_path))
        subprocess.run(cmd, check=True)
        if not video_path.exists():
            raise FileNotFoundError(f"Video encoding failed: {video_path}")
    finally:
        shutil.rmtree(link_dir, ignore_errors=True)


def hdf5_dataset_or_none(episode, path):
    try:
        return episode[path][()]
    except KeyError:
        return None


def lift_action_data(episode, name):
    data = hdf5_dataset_or_none(episode, f"action/lifting/{name}")
    if data is None:
        data = hdf5_dataset_or_none(episode, f"lift/motor/{name}")
    if data is None:
        raise KeyError(f"Missing action/lifting/{name}")
    return data.reshape(-1, 1)


def frame_count(array):
    return int(array.shape[0])


def trim_to_frames(array, frames):
    return array[:frames]


def hdf5_string(value):
    if isinstance(value, bytes):
        return value.decode("utf-8")
    if hasattr(value, "item"):
        return hdf5_string(value.item())
    return str(value)


def infer_camera_image_shapes(hdf5_files, camera_names):
    shapes = {}
    for episode_path in hdf5_files:
        with h5py.File(episode_path, "r") as episode:
            episode_dir = os.path.dirname(str(episode_path))
            for camera in camera_names:
                if camera in shapes:
                    continue
                key = f"camera/color/{camera}"
                if key not in episode or episode[key].shape[0] == 0:
                    continue
                dataset = episode[key]
                if dataset.ndim == 1:
                    image_path = hdf5_string(dataset[0])
                    if not os.path.isabs(image_path):
                        image_path = os.path.join(episode_dir, image_path)
                    image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
                    if image is None:
                        raise FileNotFoundError(f"Cannot read camera image: {image_path}")
                else:
                    image = dataset[0]
                if image.ndim == 2:
                    image = image[..., None]
                if image.ndim != 3:
                    raise ValueError(f"{episode_path}: {key} first frame has unsupported shape {image.shape}")
                shapes[camera] = tuple(int(v) for v in image.shape)
        if len(shapes) == len(camera_names):
            break

    missing = [camera for camera in camera_names if camera not in shapes]
    if missing:
        raise ValueError(f"Cannot infer image shape for cameras: {missing}")
    return shapes


def create_empty_dataset(
    args,
    mode: Literal["video", "image"] = "video",
    dataset_config: DatasetConfig = DEFAULT_DATASET_CONFIG,
) -> LeRobotDataset:
    states = []
    actions = []
    for i in range(len(args.armJointStateNames)):
        if "puppet" in args.armJointStateNames[i]:
            for j in range(args.armJointStateDims[i]):
                states += [f'arm.jointStatePosition.{args.armJointStateNames[i]}.joint{j}']
        if "master" in args.armJointStateNames[i]:
            for j in range(args.armJointStateDims[i]):
                actions += [f'arm.jointStatePosition.{args.armJointStateNames[i]}.joint{j}']

    for i in range(len(args.armEndPoseNames)):
        if "puppet" in args.armEndPoseNames[i]:
            for j in range(args.armEndPoseDims[i]):
                states += [f'arm.endPose.{args.armEndPoseNames[i]}.joint{j}']
        if "master" in args.armEndPoseNames[i]:
            for j in range(args.armEndPoseDims[i]):
                actions += [f'arm.endPose.{args.armEndPoseNames[i]}.joint{j}']

    for name in args.robotBaseStateNames:
        states += [
            f'robotBase.state.{name}.x',
            f'robotBase.state.{name}.y',
            f'robotBase.state.{name}.yaw',
            f'robotBase.state.{name}.vx',
            f'robotBase.state.{name}.vy',
            f'robotBase.state.{name}.wz',
        ]
    for name in args.robotBaseActionNames:
        actions += [
            f'robotBase.action.{name}.vx_cmd',
            f'robotBase.action.{name}.vy_cmd',
            f'robotBase.action.{name}.wz_cmd',
        ]
    for name in args.liftMotorNames:
        states += [f'lift.motor.{name}.back_height']
        actions += [f'action.lifting.{name}.target_height']

    features = {
        "observation.state": {
            "dtype": "float64",
            "shape": (len(states),),
            "names": [
                states,
            ],
        },
        "action": {
            "dtype": "float64",
            "shape": (len(actions),),
            "names": [
                actions,
            ],
        }
    }

    for camera in args.cameraColorNames:
        height, width, channels = args.cameraImageShapes[camera]
        features[f"observation.images.{camera}"] = {
            "dtype": mode,
            "shape": (height, width, channels),
            "names": [
                "height",
                "width",
                "channel",
            ],
        }
    # for camera in args.cameraDepthNames:
    #     features[f"observation.depths.{camera}"] = {
    #         # "dtype": mode,
    #         # "shape": (1, 480, 640),
    #         # "names": [
    #         #     "channels",
    #         #     "height",
    #         #     "width",
    #         # ],
    #         "dtype": "uint16",
    #         "shape": (480, 640)
    #     }
    if args.useCameraPointCloud:
        for camera in args.cameraPointCloudNames:
            features[f"observation.pointClouds.{camera}"] = {
                "dtype": "float64",
                "shape": ((args.pointNum * 6),)
            }

    return LeRobotDataset.create(
        repo_id=args.datasetName,
        root=args.targetDir,
        fps=args.fps,
        robot_type=args.robotType,
        features=features,
        use_videos=dataset_config.use_videos,
        tolerance_s=dataset_config.tolerance_s,
        image_writer_processes=dataset_config.image_writer_processes,
        image_writer_threads=dataset_config.image_writer_threads,
        video_backend=dataset_config.video_backend,
    )


def load_episode_data(
    args,
    episode_path: Path,
    load_images=True,
):
    with h5py.File(episode_path, "r") as episode:
        try:
            state_parts = (
                [episode[f"arm/jointStatePosition/{name}"][()] for name in args.armJointStateNames if "puppet" in name] +
                [episode[f"arm/endPose/{name}"][()] for name in args.armEndPoseNames if "puppet" in name] +
                [episode[f"robotBase/state/{name}"][()] for name in args.robotBaseStateNames] +
                [episode[f"lift/motor/{name}"][()].reshape(-1, 1) for name in args.liftMotorNames]
            )
            action_parts = (
                [episode[f"arm/jointStatePosition/{name}"][()] for name in args.armJointStateNames if "master" in name] +
                [episode[f"arm/endPose/{name}"][()] for name in args.armEndPoseNames if "master" in name] +
                [episode[f"robotBase/action/{name}"][()] for name in args.robotBaseActionNames] +
                [lift_action_data(episode, name) for name in args.liftMotorNames]
            )
            frame_lengths = [frame_count(item) for item in state_parts + action_parts]
            frame_lengths.extend(frame_count(episode[f"camera/color/{camera}"]) for camera in args.cameraColorNames)
            num_frames = min(frame_lengths)
            if num_frames <= 0:
                raise ValueError(f"{episode_path}: no frames to convert")
            state_parts = [trim_to_frames(item, num_frames) for item in state_parts]
            action_parts = [trim_to_frames(item, num_frames) for item in action_parts]
            states = torch.from_numpy(np.concatenate(state_parts, axis=1))
            actions = torch.from_numpy(np.concatenate(action_parts, axis=1))
            colors = {}
            for camera in args.cameraColorNames:
                colors[camera] = []
                for i in range(num_frames):
                    if episode[f'/camera/color/{camera}'].ndim == 1:
                        image_path = resolve_episode_path(episode_path, episode[f'camera/color/{camera}'][i])
                        if load_images:
                            colors[camera].append(cv2.cvtColor(cv2.imread(
                                image_path,
                                cv2.IMREAD_UNCHANGED), cv2.COLOR_BGR2RGB))
                        else:
                            colors[camera].append(image_path)
                    else:
                        if not load_images:
                            raise ValueError(
                                f"{episode_path}: direct source video requires path-based camera data"
                            )
                        colors[camera].append(episode[f'camera/color/{camera}'][i])
                colors[camera] = colors[camera]
            depths = {}
            # for camera in args.cameraDepthNames:
            #     depths[camera] = []
            #     for i in range(episode[f'camera/depth/{camera}'].shape[0]):
            #         depths[camera].append(cv2.imread(
            #             os.path.join(os.path.dirname(str(episode_path)), episode[f'camera/depth/{camera}'][i].decode('utf-8')),
            #             cv2.IMREAD_UNCHANGED))
            pointclouds = {}
            if args.useCameraPointCloud:
                for camera in args.cameraPointCloudNames:
                    pointclouds[camera] = []
                    for i in range(episode[f'camera/pointCloud/{camera}'].shape[0]):
                        pointclouds[camera].append(np.load(
                            os.path.join(os.path.dirname(str(episode_path)), episode[f'camera/color/{camera}'][i].decode('utf-8'))))
            return colors, depths, pointclouds, states, actions
        except:
            return None, None, None, None, None

def populate_dataset(
    args,
    dataset: LeRobotDataset,
    hdf5_files: list[Path],
) -> LeRobotDataset:
    error_file = []
    direct_source_video = env_flag("LEROBOT_DIRECT_VIDEO_FROM_SOURCE", False)
    video_workers = env_int("LEROBOT_VIDEO_WORKERS", 1)
    if direct_source_video:
        print("Using direct source-image video encoding")
    episodes = range(len(hdf5_files))
    for ep_idx in tqdm.tqdm(episodes):
        episode_path = hdf5_files[ep_idx]
        task = 'null'
        try:
            with h5py.File(episode_path, 'r') as root:
                task = choice(root[f'instructions/full_instructions/text'][()]).decode('utf-8')
        except:
            pass
        colors, depths, pointclouds, states, actions = load_episode_data(
            args,
            episode_path,
            load_images=not direct_source_video,
        )
        if colors is not None:
            num_frames = states.shape[0]

            if direct_source_video:
                def encode_camera_video(camera, image_paths):
                    video_key = f"observation.images.{camera}"
                    video_path = dataset.root / dataset.meta.get_video_file_path(ep_idx, video_key)
                    encode_video_from_source_images(image_paths, video_path, args.fps)

                max_workers = min(video_workers, len(colors))
                with ThreadPoolExecutor(max_workers=max_workers) as executor:
                    futures = [
                        executor.submit(encode_camera_video, camera, image_paths)
                        for camera, image_paths in colors.items()
                    ]
                    for future in as_completed(futures):
                        future.result()

                episode_data = dataset.create_episode_buffer()
                episode_data["size"] = num_frames
                episode_data["task"] = [task] * num_frames
                episode_data["observation.state"] = [states[i].numpy() for i in range(num_frames)]
                episode_data["action"] = [actions[i].numpy() for i in range(num_frames)]
                episode_data["timestamp"] = [i / args.fps for i in range(num_frames)]
                episode_data["frame_index"] = list(range(num_frames))
                for camera, image_paths in colors.items():
                    episode_data[f"observation.images.{camera}"] = image_paths
                dataset.save_episode(episode_data)
            else:
                for i in range(num_frames):
                    frame = {
                        # 'task': task,
                        "observation.state": states[i],
                        "action": actions[i],
                    }
                    for camera, color in colors.items():
                        frame[f"observation.images.{camera}"] = color[i]
                    # for camera, depth in depths.items():
                    #     frame[f"observation.depths.{camera}"] = depth[i]
                    if args.useCameraPointCloud:
                        for camera, pointcloud in pointclouds.items():
                            frame[f"observation.pointClouds.{camera}"] = pointcloud[i]
                    dataset.add_frame(frame, task)
                dataset.save_episode()
        else:
            error_file.append(episode_path)
    print("error:", error_file)
    return dataset


def process(
    args,
    push_to_hub: bool = False,
    dataset_config: DatasetConfig = DEFAULT_DATASET_CONFIG,
):
    patch_lerobot_video_encoder_from_env()
    patch_lerobot_parallel_video_encoding_from_env()

    hdf5_files = []
    if Path(args.targetDir).exists():
        shutil.rmtree(Path(args.targetDir))

    for datasetDir in args.datasetDir:
        dataset_dir = Path(datasetDir)
        if not dataset_dir.exists():
            raise ValueError(f"{dataset_dir} does not exist")
        for f in os.listdir(dataset_dir):
            if f.endswith(".hdf5"):
                hdf5_files.append(os.path.join(dataset_dir, f))
            if os.path.isdir(os.path.join(dataset_dir, f)):
                hdf5_files.extend(glob.glob(os.path.join(dataset_dir, f, "*.hdf5")))
    hdf5_files = sorted(hdf5_files)
    if not hdf5_files:
        raise ValueError(f"No HDF5 files found under {args.datasetDir}")
    args.cameraImageShapes = infer_camera_image_shapes(hdf5_files, args.cameraColorNames)
    dataset = create_empty_dataset(
        args,
        dataset_config=dataset_config,
    )
    dataset = populate_dataset(
        args,
        dataset,
        hdf5_files
    )
    
    # Generate stats.json for backward compatibility
    from lerobot.datasets.utils import write_stats
    if dataset.meta.stats:
        write_stats(dataset.meta.stats, dataset.root)
        print(f"Stats written to {dataset.root / 'meta/stats.json'}")
    else:
        print("Warning: No stats available to write")

    if push_to_hub:
        dataset.push_to_hub()


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir', nargs='+', required=True)
    parser.add_argument('--datasetName', action='store', type=str, help='datasetName',
                        default="data", required=False)
    parser.add_argument('--type', action='store', type=str, help='type',
                        default="aloha", required=False)
    parser.add_argument('--targetDir', action='store', type=str, help='targetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--robotType', action='store', type=str, help='robotType',
                        default="cobot_magic", required=False)
    parser.add_argument('--fps', action='store', type=int, help='fps',
                        default=30, required=False)
    parser.add_argument('--cameraColorNames', action='store', type=str, help='cameraColorNames',
                        default=[], required=False)
    parser.add_argument('--cameraDepthNames', action='store', type=str, help='cameraDepthNames',
                        default=[], required=False)
    parser.add_argument('--cameraPointCloudNames', action='store', type=str, help='cameraPointCloudNames',
                        default=[], required=False)
    parser.add_argument('--useCameraPointCloud', action='store', type=bool, help='useCameraPointCloud',
                        default=False, required=False)
    parser.add_argument('--pointNum', action='store', type=int, help='point_num',
                        default=5000, required=False)
    parser.add_argument('--armJointStateNames', action='store', type=str, help='armJointStateNames',
                        default=[], required=False)
    parser.add_argument('--armJointStateDims', action='store', type=int, help='armJointStateDims',
                        default=[], required=False)
    parser.add_argument('--armEndPoseNames', action='store', type=str, help='armEndPoseNames',
                        default=[], required=False)
    parser.add_argument('--armEndPoseDims', action='store', type=int, help='armEndPoseDims',
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
        args.armJointStateDims = [7 for _ in range(len(args.armJointStateNames))]
        args.armEndPoseDims = [7 for _ in range(len(args.armEndPoseNames))]
    return args


def main():
    args = get_arguments()
    process(args)


if __name__ == "__main__":
    main()
