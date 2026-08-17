# Piper 双臂路径规划服务

对外只提供 `/plan_to_pose`：接收 `left` 或 `right`、静态 `scene_id`
和目标 `PoseStamped`，调用 MoveIt 2 生成并返回 `RobotTrajectory`。

- 默认虚拟入口只规划并在 RViz 显示，不连接 CAN；
- 双臂实机入口在规划成功后把轨迹发送给对应 Piper，并在机械臂到位后返回；
- 不包含动态场景、夹爪目标服务或额外安全门禁。

## 模型来源

双臂 URDF 由仓库内的离线工具生成：

```text
tools/generate_dual_piper_urdf.py
```

生成器读取官方 `piper_description`，按照它已有的 `left_`、`right_`
命名规则产生统一模型，并在每个原始 `link6` 上追加腕部 D435i 相机组件。
机械臂本体的 link、joint、网格和参数保持不变。生成结果保存在
`generated/dual_piper.urdf`，服务启动时只加载结果，不重复生成模型。

当前模型配置：

- 根 link：`rviz_piper_world`
- 左右底座：`y=+0.3 m`、`y=-0.3 m`
- 底座中心距离：`0.6 m`
- planning group：`left_arm`、`right_arm`
- 末端 link：`left_link6`、`right_link6`
- 腕部相机 link：`left_camera_link`、`right_camera_link`
- 腕部相机与支架来自 `mobile_aloha_sim` 的 Aloha 腕部模型和 D435i 描述
- D435i 的标定深度、彩色与 IMU TF 由真机相机驱动发布，URDF 不重复发布
- 碰撞矩阵与关节限位保存在本包 `config/` 中
- 左右臂之间没有禁用碰撞对

需要重新生成 URDF 时，在工作空间根目录执行：

```bash
python3 tools/generate_dual_piper_urdf.py
```

## 静态场景

静态场景位于 `config/scenes/`：

- `scene_id=1`：双 Piper、`1.2 m × 1.0 m` 桌面、无盖梯形盒和两块层板的货架；
- `scene_id=2`：独立的双 Piper、桌面和单 BOX 避障测试配置；
- 桌面上表面 `top_z=0.0`，与 base 基准高度一致；
- 桌面 BOX 中心高度由服务计算为 `top_z - thickness / 2`。
- 场景 1 的梯形盒由底板和四块封闭碰撞网格墙组成，顶部保持开放；底面
  中心为 `(0.26, 0.0, 0.0)`，距桌子近侧长边 `0.33 m`。
- 场景 1 的货架由两块 BOX 层板和四根 CYLINDER 立柱组成；底部位于桌面
  `z=0.0`，平面中心为 `(0.57, 0.0)`，距桌子近侧长边 `0.64 m`。

实机场景尺寸和位置在 YAML 中维护，应与现场测量值保持一致。

## 构建

本项目的 ROS 运行验证只使用 `ROS_DOMAIN_ID=31` 至 `35`；示例默认使用
`31`，不要使用已被其他系统占用的 `29` 或 `99`。

```bash
# 在工作空间父目录中
cd path_planning_service
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=31
colcon build --packages-up-to path_planning_server --symlink-install
source install/setup.bash
```

## RViz 虚拟验证

```bash
export ROS_DOMAIN_ID=31
ros2 launch path_planning_server \
  path_planning_service.launch.py \
  scene_id:=1 \
  use_rviz:=true
```

启动内容只有：

- 双臂静态 TF；
- `joint_state_publisher`；
- `robot_state_publisher`；
- MoveIt 2 `move_group`；
- `path_planning_server`；
- RViz2。

没有 ros2_control、控制器、Piper 驱动或任何实机执行节点。`config/moveit_controllers.yaml` 只关闭 MoveIt 的控制器管理，不定义或加载控制器。规划成功后，MoveIt 的 `DisplayMotionPath` 响应适配器会把轨迹发布到 `/display_planned_path`，RViz 自动播放。

无界面验证时将 `use_rviz` 设为 `false`。

## 双臂实机模式

实机入口为：

```bash
export ROS_DOMAIN_ID=31
ros2 launch path_planning_server dual_piper_hardware.launch.py \
  use_rviz:=true
```

该入口统一启动左右 Piper 驱动、两个 `FollowJointTrajectory` bridge、
真实关节状态合并节点、MoveIt、规划服务和可选 RViz。它不会启动虚拟
`joint_state_publisher`。

默认使用 `can_left`、`can_right`，自动使能两臂，场景编号为 1，不启动
RViz。自动使能时保持反馈到的当前夹爪开合度。需要手动使能时显式传入
`auto_enable:=false`。

实机模式下，`/plan_to_pose` 仍返回带 `left_`/`right_` 前缀的原始
MoveIt 轨迹，但服务内部已经把轨迹发送给对应机械臂。只有规划与 Action
执行都成功时 `success=true`；客户端不要再次发送返回轨迹。

轨迹 bridge 对规划位置按时间做 180 Hz 线性插值，并以 180 Hz 下发命令
和发布 Action feedback；驱动状态反馈为 200 Hz。实际运动以机械臂反馈
为准。

## 服务调用

左臂示例：

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
    }
  }"
```

右臂示例：

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
    }
  }"
```

服务只接受 `left`、`right` 和已登记的场景编号。规划参数统一位于
`config/planner.yaml`。虚拟模式下 `success=true` 表示规划成功；实机
模式下表示规划和机械臂执行均成功。
