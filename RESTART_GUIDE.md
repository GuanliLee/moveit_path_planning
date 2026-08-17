# 0807 抓取与规划系统重启指南

统一入口：

```bash
cd /home/ligl/path_planning_service0807
./restart_robot_services.sh <组件> [restart|start|stop|status|logs] [选项]
```

查看整套状态：

```bash
./restart_robot_services.sh all status
```

## 组件与影响范围

| 组件 | 包含的进程/功能 | 重启命令 | 是否影响机械臂驱动 |
|---|---|---|---|
| `planning` | `/path_planning_server`、`/plan_to_pose`、取消服务 | `./restart_robot_services.sh planning restart --build` | 否；会取消当前规划/轨迹 |
| `state-machine` | 抓取/放置流程、拍照位、121 推理调用 | `./restart_robot_services.sh state-machine restart --build` | 不重启驱动；会取消当前任务 |
| `graspnet-bridge` | GraspNet 检测结果到 `/plan_to_pose` 的转换 | `./restart_robot_services.sh graspnet-bridge restart --build` | 否 |
| `cameras` | `cam_high`、`cam_left`、`cam_right` RealSense | `./restart_robot_services.sh cameras restart` | 否；相机话题短暂中断 |
| `image-bridge` | 8080 图像/控制桥、相机预览、部分控制话题 | `./restart_robot_services.sh image-bridge restart` | 否；8080 短暂中断 |
| `web` | 8090 饮料抓取页面、录包控制 | `./restart_robot_services.sh web restart` | 否；网页短暂断开 |
| `rviz` | RViz 显示 | `./restart_robot_services.sh rviz restart` | 否 |
| `handeye-tf` | `left/right_link6 -> cam_left/right_link` 静态 TF | `./restart_robot_services.sh handeye-tf restart --build` | 否 |
| `software` | planning + image bridge + GraspNet 桥 + 状态机 + Web | `./restart_robot_services.sh software restart --build` | 不重启驱动；当前任务会取消 |
| `hardware` | 双臂 CAN 驱动、轨迹桥、joint states、robot_state_publisher、MoveIt、规划服务、手眼 TF | `./restart_robot_services.sh hardware restart --build --confirm-hardware` | **是** |

## 修改位置与必须重启的部分

### 1. 规划服务本身

修改以下文件：

- `src/path_planning_server/src/path_planning_node.cpp`
- `src/path_planning_server/include/path_planning_server/path_planning_node.hpp`
- `static_scene_loader.cpp/.hpp`
- `CMakeLists.txt`、`package.xml`
- `config/planner.yaml`
- `config/grasp_ellipsoid.yaml`
- `config/scenes/*.yaml`、`scene_index.yaml`

执行：

```bash
./restart_robot_services.sh planning restart --build
```

只有 YAML 参数或场景内容变化时可以省略 `--build`，但带上也安全。

### 2. 抓取状态机、拍照位置和抓放流程

修改以下文件：

- `scripts/grasp_bridge_state_machine.py`
- `config/grasp_bridge_state_machine.right.yaml`
- `launch/grasp_bridge_state_machine.launch.py`
- 拍照位、预抓取距离、放置位、传感器同步、121 地址等参数

执行：

```bash
./restart_robot_services.sh state-machine restart --build
```

这不会重启规划器或机械臂驱动，但会取消正在运行的抓取任务。

### 3. GraspNet 桥和 TCP/手眼补偿

修改以下目录：

- `/home/ligl/graspnet_path_planning_bridge0804/src/graspnet_path_planning_bridge/`
- `bridge_node.py`
- `config/bridge.yaml`
- `launch/bridge.launch.py`
- `left/right_link6_to_tcp_*`、`link6_to_handeye_parent_m`
- `calibration/extrinsics/cam_left_handeye.json`
- `calibration/extrinsics/cam_right_handeye.json`

执行：

```bash
./restart_robot_services.sh graspnet-bridge restart --build
```

如果同时修改了 `handeye_camera_tf.launch.py` 中发布到 TF 树的外参，再执行：

```bash
./restart_robot_services.sh handeye-tf restart --build
```

### 4. MoveIt 模型和规划流水线

修改以下文件后，单独重启 planning 不够：

- `generated/dual_piper.urdf`
- `config/dual_piper.srdf`
- `config/kinematics.yaml`
- `config/joint_limits.yaml`
- `config/ompl_planning.yaml`
- `config/moveit_controllers.yaml`
- `dual_piper_hardware.launch.py`
- Piper 驱动或 `piper_trajectory_bridge` 源码

当前运行拓扑把 MoveIt 和双臂驱动放在同一个顶层 launch 中，因此使用：

```bash
./restart_robot_services.sh hardware restart --build --confirm-hardware
```

执行前必须停止现场操作，确认双臂周围无人和障碍物，并注意默认
`PIPER_AUTO_ENABLE=true`。如需启动后不自动使能：

```bash
PIPER_AUTO_ENABLE=false \
  ./restart_robot_services.sh hardware restart --build --confirm-hardware
```

### 5. 三台 RealSense

修改以下内容：

- `/home/ligl/realsense_three_cameras/*.yaml`
- `start_calibration_rgb_depth_aligned.sh`
- 相机序列号、分辨率、帧率、深度对齐参数
- RealSense launch/驱动参数

执行：

```bash
./restart_robot_services.sh cameras restart
```

相机重启后，状态机和图像桥通常能自动恢复订阅，不需要重启；如果预览没有恢复，再重启
`image-bridge`。

### 6. AgileX 图像桥

修改以下内容：

- `/home/ligl/agilex_xpc/bridge/image_bridge/`
- `/home/ligl/agilex_xpc/bridge/config/cameras.yaml`
- 8080 预览、图像编码、控制话题映射

执行：

```bash
./restart_robot_services.sh image-bridge restart
```

该 Python 模块直接从源码运行，通常不需要 `colcon build`。

### 7. Web 页面和录包

修改以下内容：

- `/home/ligl/drink_grasp_web/server.py`
- Web 前端资源
- `start.sh` 中的端口、服务名、录包话题或超时参数

执行：

```bash
./restart_robot_services.sh web restart
```

只改浏览器缓存中的前端显示时可先强制刷新页面；服务端 Python 或环境变量变化必须重启。

### 8. RViz

修改 `config/moveit.rviz` 或只想重开界面：

```bash
./restart_robot_services.sh rviz restart
```

修改 URDF/SRDF 后，先重启 `hardware`，再重启 RViz。

### 9. ROS 接口定义

修改以下接口时需要重编依赖方：

- `path_planning_interfaces/srv/PlanToPose.srv`
- `graspnet_bridge_interfaces` 下的消息或服务

`path_planning_interfaces` 当前在两个工作区各有一份。修改前必须确保下面两处
`.srv` 定义完全一致：

- `/home/ligl/path_planning_service0807/src/path_planning_interfaces`
- `/home/ligl/graspnet_path_planning_bridge0804/src/path_planning_interfaces`

建议执行纯软件链路重启：

```bash
./restart_robot_services.sh software restart --build
```

如果接口还被图像桥自己的 ROS 工作区编译引用，需要另外重编
`/home/ligl/agilex_xpc/bridge/ros_ws` 后再重启 `image-bridge`。

### 10. 不需要重启的情况

- 只发送了新的目标位姿或抓取请求
- 只修改一次性录像/截图脚本，下一次重新运行脚本即可
- 修改离线数据、历史视频、录包或转换脚本
- 修改测试文件但没有改运行代码

## 日志与状态

```bash
./restart_robot_services.sh all status
./restart_robot_services.sh planning logs
./restart_robot_services.sh state-machine logs
./restart_robot_services.sh graspnet-bridge logs
./restart_robot_services.sh cameras logs
./restart_robot_services.sh image-bridge logs
./restart_robot_services.sh web logs
./restart_robot_services.sh rviz logs
./restart_robot_services.sh hardware logs
```

所有普通停止先发送 `SIGINT`，超时后最多发送 `SIGTERM`；脚本不会自动发送
`SIGKILL`。硬件整套重启必须显式传入 `--confirm-hardware`。
