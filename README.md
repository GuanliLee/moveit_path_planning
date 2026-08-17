# Piper 双臂路径规划服务

本项目是一个 ROS 2 Jazzy 工作空间，面向两台 AgileX Piper 机械臂提供
MoveIt 2 路径规划与真机轨迹执行能力。客户端只需要调用
`/plan_to_pose`，指定左臂或右臂、静态场景编号、目标位姿以及可选的夹爪开合
宽度。

- 虚拟模式：使用虚拟关节状态完成规划，并在 RViz 中显示轨迹，不连接 CAN，
  不驱动真机。
- 实机模式：读取两台 Piper 的实时关节状态，规划成功后由服务内部发送轨迹，
  等待指定机械臂以及可选的夹爪动作执行完成后返回。
- 当前模型：双 Piper、双夹爪、双腕部 D435i 相机。
- 当前场景：场景 1 为桌面、无盖梯形盒和三层货架；场景 2 为桌面和单个
  BOX 障碍物。
- 当前不提供动态障碍物、Gazebo 或 MuJoCo 仿真。

## 完整系统代码快照

本仓库同时保存当前 8090 饮料抓取链路和采集数据质检代码：

```text
src/path_planning_server/scripts/grasp_bridge_state_machine.py  # 抓取状态机
components/drink_grasp_web/                                     # 8090 Web
components/graspnet_path_planning_bridge0804/                   # 抓取到规划服务桥
components/realsense_three_cameras/                             # 三相机启动与标定工具
components/agilex_idata/                                       # 采集、转换、质检与回放
```

仓库只保存代码、配置、测试、文档和必要模型网格。MCAP/HDF5/视频、
`recordings/`、Python 虚拟环境、ROS 2 的 `build/install/log` 以及运行日志均不提交。

## 最简单的虚拟运行示例

第一次使用前先完成后文的“环境初始化”。已经构建过工作空间时，在终端 1
启动虚拟规划服务：

```bash
cd /home/agilex/Projects/path_planning_service
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
export ROS_DOMAIN_ID=31

ros2 launch path_planning_server path_planning_service.launch.py \
  scene_id:=1 \
  use_rviz:=true
```

看到下面的日志后即可调用服务：

```text
Ready: /plan_to_pose, groups [left_arm, right_arm], scene 1, execute trajectory: false
```

在终端 2 使用相同的环境和 `ROS_DOMAIN_ID`：

```bash
cd /home/agilex/Projects/path_planning_service
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
export ROS_DOMAIN_ID=31

ros2 service call /plan_to_pose \
  path_planning_interfaces/srv/PlanToPose \
  "{
    arm_name: right,
    scene_id: 1,
    target_pose: {
      header: {frame_id: world},
      pose: {
        position: {x: 0.25, y: -0.30, z: 0.30},
        orientation: {x: 0.0, y: 0.676, z: 0.0, w: 0.737}
      }
    },
    gripper_command: false,
    gripper_opening_m: 0.0,
    keep_grasp_ellipsoid: false
  }"
```

虚拟模式返回 `success=true` 只表示规划成功，机械臂和夹爪都不会运动。规划轨迹
会通过 `/display_planned_path` 发布并由 RViz 显示。

## 项目结构

```text
path_planning_service/
├── README.md
├── tools/
│   └── generate_dual_piper_urdf.py
└── src/
    ├── path_planning_interfaces/
    │   └── srv/PlanToPose.srv
    ├── path_planning_server/
    │   ├── launch/
    │   │   ├── path_planning_service.launch.py
    │   │   └── dual_piper_hardware.launch.py
    │   ├── config/
    │   │   ├── planner.yaml
    │   │   ├── dual_piper.srdf
    │   │   ├── kinematics.yaml
    │   │   ├── joint_limits.yaml
    │   │   └── scenes/
    │   ├── generated/dual_piper.urdf
    │   ├── meshes/wrist_camera/
    │   ├── include/path_planning_server/
    │   ├── src/
    │   ├── scripts/dual_joint_state_adapter.py
    │   └── test/
    └── piper_ros/
        ├── can_activate.sh
        ├── find_all_can_port.sh
        ├── requirements.txt
        └── src/
            ├── piper/
            ├── piper_msgs/
            └── piper_description/
```

各部分职责如下：

- `path_planning_interfaces`：定义客户端使用的 `PlanToPose` ROS 服务。
- `path_planning_server`：加载机器人模型、MoveIt 配置和静态场景，完成规划
  以及可选的真机轨迹和夹爪执行。
- `piper`：Piper CAN 驱动和
  `FollowJointTrajectory` 到底层关节命令的轨迹桥接。
- `piper_msgs`：Piper 状态、关节命令和使能服务消息。
- `piper_description`：原始单臂 Piper URDF 和网格。
- `generate_dual_piper_urdf.py`：离线生成双臂 URDF，并在原模型的 `link6`
  上追加 D435i 相机组件；不会在启动时自动运行。
- `dual_joint_state_adapter.py`：将左右桥接节点的关节状态加上
  `left_`、`right_` 前缀后合并到 `/joint_states`。

`build/`、`install/` 和 `log/` 是本机构建产物，不属于源代码。
`generated/dual_piper.urdf` 虽然位于 `generated/`，但它是服务启动时直接
加载的运行文件，不能删除。

## 机器人模型和坐标约定

- 世界坐标系：`world`。
- 机器人模型根坐标系：`rviz_piper_world`，与 `world` 之间为单位变换。
- 左、右底座中心：分别位于 `y=+0.3 m` 和 `y=-0.3 m`，间距 `0.6 m`。
- MoveIt planning group：`left_arm`、`right_arm`。
- 服务控制的末端 link：`left_link6`、`right_link6`。
- 腕部相机 link：`left_camera_link`、`right_camera_link`。
- 夹爪总开合范围：`0.0-0.1 m`，模型中的单侧 `joint7` 范围为
  `0.0-0.05 m`。

`target_pose` 描述的是 `link6` 原点，不是夹爪指尖。Piper 夹爪沿
`link6` 局部 `+Z` 方向伸出约 `0.1358 m`。如果任务要求指尖到达某个精确
位置，需要在客户端目标位姿中补偿这段长度。

双臂模型由以下命令离线重新生成：

```bash
python3 tools/generate_dual_piper_urdf.py
```

生成器保留 `piper_description` 中原有机械臂的 link、joint、网格和参数，
只生成左右前缀并追加腕部相机组件。

## 静态场景

场景索引位于
`src/path_planning_server/config/scenes/scene_index.yaml`。

| `scene_id` | 内容 |
|---|---|
| `1` | `1.2 m × 1.0 m` 桌面、无盖梯形盒、三层货架 |
| `2` | 桌面和单个 BOX 障碍物 |

场景 1 的关键坐标：

- 桌面上表面：`z=0.0`。
- 梯形盒底面中心：`(0.26, 0.0, 0.0)`。
- 货架平面中心：`(0.57, 0.0)`。
- 货架三块层板中心高度：`0.035 m`、`0.22 m`、`0.405 m`。

静态场景尺寸和位置由对应 YAML 文件维护。实机使用前应确保 YAML 与现场
测量值一致。

## 接口说明

### 公开规划服务

服务名：

```text
/plan_to_pose
```

服务类型：

```text
path_planning_interfaces/srv/PlanToPose
```

接口定义：

```text
string arm_name
uint32 scene_id
geometry_msgs/PoseStamped target_pose
bool gripper_command
float64 gripper_opening_m
bool keep_grasp_ellipsoid
---
bool success
string message
moveit_msgs/RobotTrajectory trajectory
```

请求字段：

| 字段 | 说明 |
|---|---|
| `arm_name` | 只能为 `left` 或 `right`；同时决定夹爪命令发送到哪一侧 |
| `scene_id` | 已登记的静态场景编号，目前为 `1` 或 `2` |
| `target_pose.header.frame_id` | 目标位姿坐标系，推荐使用 `world`；其他坐标系必须能在 1 秒内转换到 `world` |
| `target_pose.pose.position` | 对应机械臂 `link6` 原点的目标位置，单位为米 |
| `target_pose.pose.orientation` | `link6` 的目标四元数；不能为全零，服务会自动归一化 |
| `gripper_command` | `false` 表示不控制夹爪并忽略开合值；`true` 表示机械臂到位后控制夹爪 |
| `gripper_opening_m` | 两指之间的总开合宽度，范围 `0.0-0.1 m`；只在 `gripper_command=true` 时使用 |
| `keep_grasp_ellipsoid` | 实机执行成功后是否保留夹爪末端椭球；`true` 表示附着或保持，`false` 表示移除；虚拟模式不改变椭球状态 |

`gripper_command=false` 时，`gripper_opening_m` 不参与校验和执行，夹爪保持
当前开合度。`gripper_command=true` 时，`0.0 m` 表示闭合，`0.1 m` 表示
总开合 10 cm。

响应字段：

| 字段 | 说明 |
|---|---|
| `success` | 虚拟模式表示规划成功；实机模式表示机械臂以及请求的夹爪动作均成功 |
| `message` | 成功信息或失败原因 |
| `trajectory` | MoveIt 原始六轴机械臂轨迹，关节名带 `left_` 或 `right_` 前缀，不包含夹爪 `joint7/joint8` |

实机模式已经在服务内部发送并执行 `trajectory`，客户端不要再次发送返回
轨迹。常见返回信息包括：

- `Planning succeeded`
- `Planning and trajectory execution succeeded`
- `Planning, trajectory, and gripper execution succeeded`
- `Gripper opening must be finite and in [0.0, 0.1] meters`
- `Invalid arm name`
- `Scene ID not found`
- `Current robot state unavailable`
- `Planning failed`
- `Action server unavailable`
- `Trajectory execution aborted`
- `Arm trajectory execution succeeded, but gripper ...`

### 实机运行时接口

以下接口由实机启动文件创建，普通客户端通常只需要调用
`/plan_to_pose`：

| 名称 | 类型 | 用途 |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | 合并后的双臂 MoveIt 关节状态 |
| `/left/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 左臂内部轨迹执行 |
| `/right/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 右臂内部轨迹执行 |
| `/left/gripper_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 左夹爪内部轨迹执行 |
| `/right/gripper_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 右夹爪内部轨迹执行 |
| `/left/enable_srv` | `piper_msgs/srv/Enable` | 左臂手动使能/失能 |
| `/right/enable_srv` | `piper_msgs/srv/Enable` | 右臂手动使能/失能 |

## 环境初始化

### 系统要求

已验证环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- MoveIt 2 2.12.4
- Python 3.12
- `piper-sdk==0.6.1`
- `python-can==4.3.1`

实机还需要 `can-utils`、`ethtool` 和 `iproute2`。

### 首次安装依赖

如果本机从未初始化过 `rosdep`，先执行：

```bash
sudo rosdep init
rosdep update
```

安装 ROS 和 Python 依赖：

```bash
cd /home/agilex/Projects/path_planning_service
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y

python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r src/piper_ros/requirements.txt
```

### 构建工作空间

本项目只能使用 `ROS_DOMAIN_ID=31` 至 `35`。本文示例统一使用 `31`；
不要使用已被其他系统占用的 `29` 或 `99`。所有需要互相通信的终端必须
使用相同的 Domain。

```bash
cd /home/agilex/Projects/path_planning_service
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export ROS_DOMAIN_ID=31

colcon build --packages-up-to path_planning_server --symlink-install
source install/setup.bash
```

修改 `PlanToPose.srv` 后，接口包、服务端和客户端都需要重新构建，并重新
`source install/setup.bash`。

每次重新打开终端后，都需要重新执行：

```bash
cd /home/agilex/Projects/path_planning_service
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
export ROS_DOMAIN_ID=31
```

## 运行时初始化流程

### 虚拟模式

执行 `path_planning_service.launch.py` 后：

1. 加载 `dual_piper.urdf`、SRDF、运动学、关节限位和 OMPL 配置。
2. 发布 `world → rviz_piper_world` 静态 TF。
3. 启动虚拟 `joint_state_publisher` 和 `robot_state_publisher`。
4. 启动 MoveIt `move_group`。
5. 启动 `path_planning_server`，创建左右两个 planning group。
6. 加载 `scene_id` 指定的静态场景并创建 `/plan_to_pose`。
7. 根据 `use_rviz` 选择是否启动 RViz。

该模式默认 `execute_trajectory=false`，不会启动 Piper 驱动或连接 CAN，也不会
执行请求中的夹爪命令。

### 实机模式

执行 `dual_piper_hardware.launch.py` 后：

1. 左右 Piper 驱动分别连接 `can_left`、`can_right`。
2. 两个轨迹桥接节点接收驱动反馈，并提供左右机械臂和夹爪
   `FollowJointTrajectory` Action。
3. `dual_joint_state_adapter` 将左右状态合并到 `/joint_states`。
4. 加载与虚拟模式相同的机器人模型、MoveIt 配置和静态场景，但不启动
   虚拟 `joint_state_publisher`。
5. `path_planning_server` 以 `execute_trajectory=true` 启动。
6. 服务请求成功规划后，轨迹被发送给对应机械臂；机械臂成功且请求了夹爪
   控制时，再执行同侧夹爪 Action。

### 单次服务请求流程

每次收到 `/plan_to_pose` 请求时，服务依次：

1. 校验机械臂名称、场景编号、目标坐标系和目标位姿数值。
2. `gripper_command=true` 时校验 `gripper_opening_m`；为 `false` 时忽略该值。
3. 切换或确认请求中的静态场景。
4. 将目标转换到 `world` 并归一化四元数。
5. 读取最新 `/joint_states` 作为规划起点。
6. 对对应 `link6` 执行 MoveIt 规划。
7. 虚拟模式直接返回轨迹；实机模式先执行机械臂轨迹。
8. 机械臂成功且 `gripper_command=true` 时，把总开合宽度除以 2 后作为
   `joint7` 目标发送给同侧夹爪 Action；机械臂与可选夹爪动作成功后，按
   `keep_grasp_ellipsoid` 更新椭球状态。

连续调用时，每次都会重新读取最新关节反馈，不会沿用上一次请求的起始状态。

## 服务使用方法

### 1. 启动虚拟服务

```bash
ros2 launch path_planning_server path_planning_service.launch.py \
  scene_id:=1 \
  use_rviz:=true
```

虚拟启动参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `scene_id` | `1` | 启动时加载的静态场景 |
| `use_rviz` | `true` | 是否启动 RViz |
| `use_virtual_joint_states` | `true` | 是否发布虚拟关节状态 |
| `execute_trajectory` | `false` | 是否执行轨迹；虚拟入口应保持为 `false` |

无图形界面运行：

```bash
ros2 launch path_planning_server path_planning_service.launch.py \
  scene_id:=1 \
  use_rviz:=false
```

### 2. 启动实机服务

先识别两个 CAN 适配器的 USB `bus-info`，并配置稳定接口名：

```bash
bash src/piper_ros/find_all_can_port.sh
sudo bash src/piper_ros/can_activate.sh can_left 1000000 <左臂_bus-info>
sudo bash src/piper_ros/can_activate.sh can_right 1000000 <右臂_bus-info>
```

确认接口：

```bash
ip -details link show can_left
ip -details link show can_right
```

启动实机：

```bash
ros2 launch path_planning_server dual_piper_hardware.launch.py
```

需要同时显示 RViz：

```bash
ros2 launch path_planning_server dual_piper_hardware.launch.py \
  use_rviz:=true
```

实机启动参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `can_left_port` | `can_left` | 左臂 SocketCAN 接口 |
| `can_right_port` | `can_right` | 右臂 SocketCAN 接口 |
| `auto_enable` | `true` | 驱动启动后是否自动使能 |
| `gripper_val_mutiple` | `2` | 夹爪反馈/命令倍率；参数名保留上游拼写 |
| `scene_id` | `1` | 静态场景编号 |
| `use_rviz` | `false` | 是否启动 RViz |

`auto_enable=true` 时会保持反馈到的当前夹爪开合度。若需要手动使能：

```bash
ros2 launch path_planning_server dual_piper_hardware.launch.py \
  auto_enable:=false
```

在另一个已初始化且 Domain 相同的终端执行：

```bash
ros2 service call /left/enable_srv \
  piper_msgs/srv/Enable "{enable_request: true}"

ros2 service call /right/enable_srv \
  piper_msgs/srv/Enable "{enable_request: true}"
```

### 3. 检查服务和反馈

```bash
ros2 service type /plan_to_pose
ros2 topic echo --once /joint_states
ros2 action info /left/arm_controller/follow_joint_trajectory
ros2 action info /right/arm_controller/follow_joint_trajectory
ros2 action info /left/gripper_controller/follow_joint_trajectory
ros2 action info /right/gripper_controller/follow_joint_trajectory
```

实机 `/joint_states` 应包含 `left_joint1..left_joint8` 和
`right_joint1..right_joint8`。

### 4. 调用服务

右臂只移动、不改变夹爪：

```bash
ros2 service call /plan_to_pose \
  path_planning_interfaces/srv/PlanToPose \
  "{
    arm_name: right,
    scene_id: 1,
    target_pose: {
      header: {frame_id: world},
      pose: {
        position: {x: 0.25, y: -0.30, z: 0.30},
        orientation: {x: 0.0, y: 0.676, z: 0.0, w: 0.737}
      }
    },
    gripper_command: false,
    gripper_opening_m: 0.0,
    keep_grasp_ellipsoid: false
  }"
```

左臂到位后把夹爪打开到总宽度 6 cm：

```bash
ros2 service call /plan_to_pose \
  path_planning_interfaces/srv/PlanToPose \
  "{
    arm_name: left,
    scene_id: 1,
    target_pose: {
      header: {frame_id: world},
      pose: {
        position: {x: 0.25, y: 0.30, z: 0.30},
        orientation: {x: 0.0, y: 0.676, z: 0.0, w: 0.737}
      }
    },
    gripper_command: true,
    gripper_opening_m: 0.06,
    keep_grasp_ellipsoid: false
  }"
```

关闭夹爪时使用：

```yaml
gripper_command: true
gripper_opening_m: 0.0
keep_grasp_ellipsoid: true
```

完全打开到 10 cm 时使用：

```yaml
gripper_command: true
gripper_opening_m: 0.1
keep_grasp_ellipsoid: false
```

切换场景时只需改变请求中的 `scene_id`。服务会加载对应场景后再规划。

### 5. 停止实机

建议先失能两台机械臂：

```bash
ros2 service call /left/enable_srv \
  piper_msgs/srv/Enable "{enable_request: false}"

ros2 service call /right/enable_srv \
  piper_msgs/srv/Enable "{enable_request: false}"
```

然后在 launch 终端按 `Ctrl+C`。

## 规划和轨迹参数

主要参数位于 `src/path_planning_server/config/planner.yaml`：

| 参数 | 默认值 |
|---|---|
| `planning_time` | `8.0 s` |
| `planning_attempts` | `5` |
| `velocity_scaling` | `0.10` |
| `acceleration_scaling` | `0.10` |
| `position_tolerance` | `0.005 m` |
| `orientation_tolerance` | `0.05 rad` |
| `gripper_move_duration` | `1.0 s` |

实机轨迹桥接器按 MoveIt 轨迹中的位置和 `time_from_start` 以 `180 Hz`
插值并下发关节目标；驱动反馈频率为 `200 Hz`。夹爪命令同样由 Bridge 从当前
反馈位置插值到目标开合度。

## 测试

```bash
cd /home/agilex/Projects/path_planning_service
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash

colcon test --packages-select path_planning_server piper_trajectory_bridge
colcon test-result --verbose
```

## `target_pose` 坐标与姿态说明

服务请求中的 `target_pose` 表示机械臂末端连杆坐标系相对于
`header.frame_id` 的目标位姿：

- 右臂的目标连杆是 `right_link6`。
- 左臂的目标连杆是 `left_link6`。
- 示例使用 `header.frame_id: world`，因此 `position` 是对应 `link6`
  坐标系原点在 `world` 坐标系中的位置。
- `orientation` 是对应 `link6` 坐标系相对于 `world` 坐标系的姿态，
  使用四元数 `{x, y, z, w}` 表示。

`target_pose` 不是夹爪两指中点、夹爪指尖或腕部相机的位姿。夹爪安装在
`link6` 上，夹指坐标系沿 `link6` 的局部正 Z 方向前伸约 `0.1358 m`。
因此，如果希望夹爪伸入盒子或货架，不能直接把容器内部目标点作为
`target_pose.position`；应根据目标姿态反向补偿这段前伸距离，并为夹爪
实体保留足够的碰撞间隙。
