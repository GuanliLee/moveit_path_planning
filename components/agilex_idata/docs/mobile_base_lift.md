# 移动状态数据采集方案

本文档记录双臂 + 底盘 + 升降柱的数据存储和回放方案。

## ROS 话题与服务

采集新增底盘状态、底盘命令和升降柱状态：

```text
/odom                nav_msgs/msg/Odometry
/action/chassis     geometry_msgs/msg/Twist
/LiftMotorStatePub   lifting_msg_pkg/msg/LiftMotorMsg
```

回放使用两路控制入口：

```text
/cmd_vel             geometry_msgs/msg/Twist
/LiftingMotorService lifting_msg_pkg/srv/LiftMotorSrv
```

底盘按 state/action 分开存储：

- `state` 来自 `/odom`，HDF5 内保存 `[x, y, yaw, vx, vy, wz]`，其中位置和姿态来自 odom pose，速度来自 odom twist。
- `action` 来自 `/action/chassis`，HDF5 内保存 `[vx_cmd, vy_cmd, wz_cmd]`。

回放时优先读取 `robotBase/action/chassis` 并发布到 `/cmd_vel`。`robotBase/vel/chassis` 仍会由 `/odom` 反馈速度派生写入，主要用于兼容旧脚本，不再作为严格 action。

升降柱状态采集 `LiftMotorMsg.back_height`，HDF5 内保存为 `lift/motor/column`。升降柱 action 从同一条消息的 `targetHeight` / `target_height` 读取，HDF5 内保存为 `action/lifting/column`；旧数据没有目标高度时会退回使用 `backHeight`。回放时默认优先用 `action/lifting/column`，通过 `LiftMotorSrv` 的 `mode=0` 位置控制，把高度作为 `val` 发给 `/LiftingMotorService`。

## 中间格式

MCAP 转换后仍沿用 ALOHA 中间目录：

```text
/home/agilex/data/aloha/episode<N>/
├── camera/color/{left,front,right}/*.jpg
├── arm/jointState/{masterLeft,masterRight,puppetLeft,puppetRight}/*.json
├── localization/pose/{puppetLeft,puppetRight}/*.json
├── robotBase/state/chassis/*.json
├── robotBase/action/chassis/*.json
└── lift/motor/column/*.json
```

`robotBase/state/chassis/*.json` 主要字段：

```json
{
  "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
  "pose": {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "yaw": 0.0
  }
}
```

`robotBase/action/chassis/*.json` 主要字段：

```json
{
  "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
}
```

`lift/motor/column/*.json` 保留 `backHeight`，并附带 `targetHeight`、限位、速度、错误状态等辅助字段。

## HDF5 字段

最终 HDF5 新增：

```text
robotBase/state/chassis   shape=(N, 6)  [x, y, yaw, vx, vy, wz]
robotBase/action/chassis  shape=(N, 3)  [vx_cmd, vy_cmd, wz_cmd]
robotBase/vel/chassis     shape=(N, 3)  [vx, vy, wz]  legacy compatibility
lift/motor/column         shape=(N,)    back_height(mm)
action/lifting/column     shape=(N,)    target_height(mm)
```

原有双臂和相机字段保持不变：

```text
arm/jointStatePosition/masterLeft
arm/jointStatePosition/masterRight
arm/jointStatePosition/puppetLeft
arm/jointStatePosition/puppetRight
camera/color/{left,front,right}
localization/pose/{puppetLeft,puppetRight}
timestamp
size
```

当前同步方式是“每一帧都包含相机、双臂、底盘 state、底盘 action、升降柱”。如果 `/action/chassis` 或升降柱状态发布频率很低，最终同步帧率会被慢话题拉低。采集时要确认 `/action/chassis` 在运动和停止阶段都按固定频率发布；后续如果需要保持相机/双臂高帧率，可以把低频话题改成按时间戳做 last-observation-hold 或插值。

## 使用方法

采集前检查：

```bash
cd /home/caizj/agilex_idata
bash scripts/collection/check_mobile_topics.sh
```

首次使用本仓库工作空间需要构建：

```bash
cd /home/caizj/agilex_idata/ros2_ws
source /opt/ros/humble/setup.bash
source /home/agilex/agilex_ws/install/setup.bash
colcon build --symlink-install
```

采集一集并自动转换：

```bash
cd /home/caizj/agilex_idata
bash scripts/collection/collect_mobile_web.sh
```

只补转已有 MCAP：

```bash
# HDF5/QC conversion has moved out of the collection startup path.
```

真机回放：

```bash
source /opt/ros/humble/setup.bash
source /home/agilex/agilex_ws/install/setup.bash
source /home/agilex/piper_ros/install/setup.bash
python3 /home/caizj/agilex_idata/scripts/replay/replay_mobile_to_robot.py \
  /home/agilex/data/aloha/episode3/episode3.hdf5 \
  --rate 1.0 \
  --base-source auto \
  --base-publish-hz 20
```

底盘回放优先读取 HDF5 所在 episode 目录下的完整 `robotBase/action/chassis/*.json` 原始 `/action/chassis` 流。HDF5 里的 `robotBase/action/chassis` 是为了学习同步过的低频样本，不保留完整命令持续时间；只用它做真机速度回放会改变移动距离和转动角度。`--base-scale` 默认等于 `--rate`，因此慢放/快放时会同步缩放底盘速度。

安全调试建议先禁用底盘和升降柱：

```bash
python3 scripts/replay/replay_mobile_to_robot.py \
  /home/agilex/data/aloha/episode3/episode3.hdf5 \
  --rate 0.3 \
  --disable-base \
  --disable-lift
```

LeRobot 格式真机回放：

```bash
source /opt/ros/humble/setup.bash
source /home/agilex/agilex_ws/install/setup.bash
source /home/agilex/piper_ros/install/setup.bash
source /home/agilex/miniforge3/etc/profile.d/conda.sh
conda activate lerobot

python /home/caizj/agilex_idata/scripts/replay/replay_lerobot_mobile_to_robot.py \
  /path/to/lerobot_dataset_root \
  --episode 0 \
  --rate 0.3 \
  --max-linear 0.3 \
  --max-angular 0.5
```

LeRobot 回放字段与 HDF5 对应：

```text
action              -> arms + action.lifting.column.target_height + robotBase.action.chassis.[vx_cmd, vy_cmd, wz_cmd]  (18D)
observation.state   -> puppet arms + lift.motor.column.back_height + robotBase.state.chassis.[x, y, yaw, vx, vy, wz]   (21D)
```
