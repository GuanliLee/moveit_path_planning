# 真机推理运行说明

本文档记录当前仓库内 ACT / OpenPI 双 Piper 真机推理脚本的用途、依赖、启动顺序、安全检查和常见故障。推理脚本默认只控制双臂和夹爪，不控制底盘和升降柱。

## 脚本清单

```text
scripts/inference/
├── run_act_inference.sh                    # ACT RGB-only 真机推理入口
├── run_act_inference_rgb.py                # ACT 官方脚本包装，跳过 depth/点云检查
├── run_openpi_pi05_grasp_bottle_server.sh  # OpenPI pi05 checkpoint policy server 启动入口
├── serve_openpi_policy_piper.py            # OpenPI server wrapper，支持设备和 compile mode 覆盖
├── run_openpi_agilex_mobile_inference.sh   # OpenPI 松灵移动底盘+升降柱+双 Piper 客户端入口
├── run_openpi_agilex_mobile_inference.py   # OpenPI mobile 客户端：21D state，18D action
├── run_openpi_piper_inference.sh           # OpenPI Piper ROS2 客户端入口
├── run_openpi_piper_inference.py           # OpenPI 客户端：采集观测、请求 action、平滑发布双臂命令
├── run_four_camera_yolo_web.sh             # 四路相机 YOLO Mask 网页启动入口
├── four_camera_yolo_web.py                 # ROS 订阅、HTTP API 和进程生命周期
├── four_camera_yolo_core.py                # 最新帧状态、YOLO 客户端和四路并发调度
└── four_camera_yolo_web.html               # 四宫格 Mask 控制台
```

## 四路相机 YOLO Mask 网页

该服务只读取相机并调用远端 YOLO，不发布机械臂或底盘控制命令。默认订阅：

```text
/camera_l/color/image_raw/compressed
/camera_f/color/image_raw/compressed
/camera_r/color/image_raw/compressed
/camera_h/color/image_raw/compressed
```

远端 YOLO 默认为 `http://192.168.4.121:7881`，网页监听
`0.0.0.0:7788`，ROS domain 默认为 `99`。直接启动：

```bash
cd /home/caizj/agilex_idata/.worktrees/four-camera-yolo-web
bash scripts/inference/run_four_camera_yolo_web.sh
```

浏览器打开 `http://<机器人IP>:7788/`。页面从 YOLO `/classes` 加载已部署
类别；点击“选择物品”打开可搜索的复选下拉菜单，可同时选择多个类别，选好后
点击 **应用并开始**。默认每 `0.2` 秒发起一轮四路推理，网页每 `200 ms`
拉取一次最新状态和 Mask 结果。

每路相机使用独立且稳定的 `stream_id`，启用远端服务的视频高召回阈值。
每路最多一个请求在途，只取该路最新帧；如果推理耗时超过配置周期，就跳过
中间轮次而不是积压旧图。每张卡片默认显示 Mask 叠加图，可切换为与该次结果
严格配对的原图。

常用覆盖参数：

```bash
WEB_PORT=17788 \
ROS_DOMAIN_ID=99 \
YOLO_URL=http://192.168.4.121:7881 \
DEFAULT_INTERVAL_SEC=0.2 \
REQUEST_TIMEOUT=10 \
bash scripts/inference/run_four_camera_yolo_web.sh
```

全局周期可在网页中设为 `0.2–60` 秒。单路“启用该路”开关只影响对应相机；
“全部暂停”停止 YOLO 请求，但 ROS 相机订阅继续运行。修改目标、周期或相机
开关后，需要点击 **应用并开始** 才会生效。

无浏览器时可检查 API：

```bash
curl -fsS http://127.0.0.1:7788/api/status
curl -fsS http://127.0.0.1:7788/api/classes
curl -fsS -X POST http://127.0.0.1:7788/api/config \
  -H 'Content-Type: application/json' \
  -d '{"targets":["Wanglaoji","Sprite"],"interval_sec":0.2,"paused":false,"cameras":{"left":{"enabled":true},"front":{"enabled":true},"right":{"enabled":true},"head":{"enabled":true}}}'
```

常见问题：

- 启动提示 7788 被占用：使用 `WEB_PORT=17788`，脚本不会终止现有监听进程。
- YOLO 离线：运行 `curl --noproxy '*' http://192.168.4.121:7881/health`，检查网络、防火墙和 7881 端口。
- 单张卡片一直等待相机：在 `ROS_DOMAIN_ID=99` 环境执行
  `ros2 topic hz /camera_f/color/image_raw/compressed`，按卡片上的实际 topic
  替换 `camera_f`。
- 卡片保留旧图并显示“结果已过期”：远端请求失败时会保留最后一次成功结果，
  查看卡片错误和往返耗时；服务恢复后下一轮会自动刷新。
- 下拉菜单为空：先检查 `/api/classes`，并确认远端 YOLO `/classes` 可访问。

## ROS Topic 约定

推理客户端读取：

```text
/camera_f/color/image_raw     sensor_msgs/msg/Image
/camera_l/color/image_raw     sensor_msgs/msg/Image
/camera_r/color/image_raw     sensor_msgs/msg/Image
/puppet/joint_left            sensor_msgs/msg/JointState
/puppet/joint_right           sensor_msgs/msg/JointState
```

推理客户端发布：

```text
/joint_left_states            sensor_msgs/msg/JointState
/joint_right_states           sensor_msgs/msg/JointState
```

OpenPI 观测格式：

```text
state: 14D = left[6 joint + gripper_norm] + right[6 joint + gripper_norm]
images:
  cam_high       = front RGB, resize_with_pad 到 224x224, CHW uint8
  cam_left_wrist = left RGB, resize_with_pad 到 224x224, CHW uint8
  cam_right_wrist= right RGB, resize_with_pad 到 224x224, CHW uint8
prompt: 默认 "grasp the bottle"
```

## 环境路径

默认路径写在启动脚本里，可以用环境变量覆盖：

```text
ROS_SETUP=/opt/ros/humble/setup.bash
PIPER_SETUP=/home/agilex/piper_ros/install/setup.bash
CAMERA_SETUP=/home/agilex/camera_ros/install/setup.bash
CONDA_SETUP=/home/agilex/miniforge3/etc/profile.d/conda.sh
CONDA_ENV_NAME=aloha
OPENPI_ROOT=/home/agilex/openpi
OPENPI_CLIENT_SRC=/home/agilex/openpi/packages/openpi-client/src
UV_BIN=/home/agilex/miniforge3/bin/uv
```

OpenPI grasp bottle server 默认 checkpoint：

```text
/home/caizj/checkpoint/openpi/pi05_piper_grasp_bottle_bs8_20260529_194731/<STEP>
```

其中 `<STEP>` 默认通过脚本第一个参数传入，例如 `20000`。目录下需要：

```text
model.safetensors
metadata.pt
assets/local/grasp_bottle/norm_stats.json
```

启动 server 时脚本会把 `assets/local/grasp_bottle/norm_stats.json` 复制到 `assets/trossen/norm_stats.json`，匹配 OpenPI ALOHA policy 使用的 asset id。

## 推理前自检

先启动三路相机和双 Piper，并确认从臂状态持续更新：

```bash
source /opt/ros/humble/setup.bash
source /home/agilex/camera_ros/install/setup.bash
source /home/agilex/piper_ros/install/setup.bash

ros2 topic info /camera_f/color/image_raw
ros2 topic info /camera_l/color/image_raw
ros2 topic info /camera_r/color/image_raw
ros2 topic info /puppet/joint_left
ros2 topic info /puppet/joint_right

ros2 topic hz /camera_f/color/image_raw
ros2 topic hz /puppet/joint_left
```

只检查 OpenPI 客户端输入，不请求 policy、不发布动作：

```bash
cd /home/caizj/agilex_idata
bash scripts/inference/run_openpi_piper_inference.sh --check-inputs-only
```

期望输出包含：

```text
ROS inputs ready
image shapes: ...
max input age: ...
left state: ...
right state: ...
```

如果一直提示 `Waiting for ROS inputs`，先处理缺失 topic 或时间戳太旧的问题。默认 `--max-frame-age-sec=2.0`，可以临时放宽：

```bash
bash scripts/inference/run_openpi_piper_inference.sh --check-inputs-only --max-frame-age-sec 5
```

## ACT 推理

ACT 脚本用于已有 ACT checkpoint 的 RGB-only 真机推理。它包装 `/home/agilex/aloha/act/aloha_inference-ros2.py`，只检查训练时用到的 RGB 相机和双臂关节，避免官方脚本因为没有 depth/点云而卡在输入检查。

默认 checkpoint：

```text
/home/caizj/checkpoint/ACT/policy_epoch_2000_seed_0.ckpt
```

运行：

```bash
cd /home/caizj/agilex_idata
bash scripts/inference/run_act_inference.sh
```

指定 checkpoint：

```bash
bash scripts/inference/run_act_inference.sh \
  /home/caizj/checkpoint/ACT \
  policy_epoch_1500_seed_0.ckpt
```

ACT 当前固定参数：

```text
policy_class=ACT
arm_joint_state_dim=7
use_arm_joint_state=3
use_arm_end_pose=0
use_robot_base=0
chunk_size=100
hidden_dim=512
dim_feedforward=3200
kl_weight=10
camera order: front, left, right
```

## OpenPI 推理

OpenPI 真机推理分为两个进程：

1. policy server：加载 checkpoint，提供 websocket policy 服务。
2. Piper client：读取 ROS 图像和双臂状态，请求 policy action，并发布 `/joint_left_states`、`/joint_right_states`。

移动底盘/升降柱/双臂任务使用 `run_openpi_agilex_mobile_inference.sh`。默认匹配
`market_peachyogurt` 的 LeRobot 布局：

```text
observation.state = 21D = left arm 7 + right arm 7 + lift height + base [x,y,yaw,vx,vy,wz]
action = 18D = left arm 7 + right arm 7 + lift target/position + base [vx_cmd,vy_cmd,wz_cmd]
```

当前训练服务使用的默认提示词：

```text
Target: Grape Juice. Pick the Grape Juice from the shelf and place it into the cart.
```

先检查真机输入 topic：

```bash
cd /home/caizj/agilex_idata
bash scripts/inference/run_openpi_agilex_mobile_inference.sh --check-inputs-only
```

先 dry-run 请求 H200 policy server，不发布真机命令：

```bash
OPENPI_POLICY_HOST=192.168.1.154 \
OPENPI_POLICY_PORT=8899 \
OPENPI_PROMPT="Target: Grape Juice. Pick the Grape Juice from the shelf and place it into the cart." \
bash scripts/inference/run_openpi_agilex_mobile_inference.sh \
  --dry-run \
  --debug-actions \
  --max-steps 8
```

确认返回 `actions` 至少 18 维后，再去掉 `--dry-run` 真机执行：

```bash
bash scripts/inference/run_openpi_agilex_mobile_inference.sh \
  --host 192.168.1.154 \
  --port 8899 \
  --debug-actions
```

### 终端 A：启动 OpenPI Policy Server

```bash
cd /home/caizj/agilex_idata
bash scripts/inference/run_openpi_pi05_grasp_bottle_server.sh 20000
```

常用环境变量覆盖：

```bash
STEP=20000 \
PORT=8000 \
CONFIG=pi05_aloha_grasp_bottle \
EXP_DIR=/home/caizj/checkpoint/openpi/pi05_piper_grasp_bottle_bs8_20260529_194731 \
PYTORCH_DEVICE=cuda \
PYTORCH_COMPILE_MODE=none \
bash scripts/inference/run_openpi_pi05_grasp_bottle_server.sh
```

脚本会：

- 检查 OpenPI repo、uv、checkpoint 文件是否存在。
- 准备 `assets/trossen/norm_stats.json`。
- 设置 `XLA_PYTHON_CLIENT_MEM_FRACTION=0.85`。
- 把 OpenPI 的 `transformers_replace` patch 复制到当前 OpenPI venv 的 transformers 包内。
- 启动 `serve_openpi_policy_piper.py`，监听 `0.0.0.0:${PORT}`。

### 终端 B：先 dry-run 请求 policy

确认 server 已经开始监听后，先 dry-run。dry-run 会请求 policy 并打印 action，但不会发布机械臂命令：

```bash
cd /home/caizj/agilex_idata
bash scripts/inference/run_openpi_piper_inference.sh \
  --host 127.0.0.1 \
  --port 8000 \
  --dry-run \
  --debug-actions \
  --max-steps 8 \
  --yes
```

如果输出的 `first-target minus feedback` 过大，先不要真机执行，检查 checkpoint、归一化统计和当前机械臂初始姿态。

### 终端 B：真机执行

确认输入、server、动作范围都正常后再执行：

```bash
bash scripts/inference/run_openpi_piper_inference.sh \
  --host 127.0.0.1 \
  --port 8000 \
  --prompt "grasp the bottle" \
  --execute-horizon 8 \
  --control-rate-hz 10 \
  --inner-rate-hz 50 \
  --max-joint-step 0.06 \
  --max-gripper-step 0.01 \
  --reject-joint-abs 3.5 \
  --debug-actions
```

加 `--yes` 可以跳过确认直接开始发布动作；调试阶段建议不要加，让脚本停在确认提示处。

### 常用安全参数

```text
--max-steps              最大执行低频 action 数，0 表示不限
--max-runtime-sec        最大运行秒数，0 表示不限
--execute-horizon        每个 policy chunk 执行多少个 action
--action-start-index     从 chunk 中第几个 action 开始执行
--max-joint-step         单个低频 action 允许的最大关节变化，rad
--max-gripper-step       单个低频 action 允许的最大夹爪变化，m
--reject-joint-abs       policy 输出关节绝对值超过该阈值直接拒绝
--control-rate-hz        低频 action 执行频率
--inner-rate-hz          插值发布频率
--hold-on-exit/--no-hold-on-exit  退出时是否保持最后命令
```

自动结束模式：

```bash
bash scripts/inference/run_openpi_piper_inference.sh \
  --yes \
  --completion-mode stable \
  --done-min-steps 80 \
  --done-window 30 \
  --done-joint-range 0.02 \
  --done-gripper-range 0.003
```

`manual` 是默认模式，会运行到 `--max-steps`、`--max-runtime-sec` 或 Ctrl+C。

## 故障排查

### openpi_client is not importable

使用 shell 入口启动，不要直接裸跑 Python：

```bash
bash scripts/inference/run_openpi_piper_inference.sh --check-inputs-only
```

入口脚本会设置 `OPENPI_CLIENT_SRC` 到 `PYTHONPATH`，并在缺少 `websockets/msgpack` 时安装依赖。

### 等不到 policy server

检查 server 是否监听：

```bash
ss -ltnp | grep 8000
```

确认 server 终端没有停在 checkpoint、norm stats、transformers patch 或 CUDA 初始化错误。

### 图像或关节 stale

检查 ROS topic 是否持续发布：

```bash
ros2 topic hz /camera_f/color/image_raw
ros2 topic hz /puppet/joint_left
```

跨用户运行 ROS 节点时，可尝试使用 UDPv4 FastDDS：

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

### 动作跳变过大

先用：

```bash
bash scripts/inference/run_openpi_piper_inference.sh --dry-run --debug-actions --max-steps 8 --yes
```

重点看：

```text
first target left/right
first-target minus feedback
chunk action range
```

常见原因：

- 机械臂初始姿态和训练数据初始姿态差异太大。
- `norm_stats.json` 不对应当前 checkpoint。
- checkpoint config 不是当前任务使用的 `pi05_aloha_grasp_bottle`。
- gripper 单位不一致。默认 OpenPI 输出夹爪为 `normalized`，会映射到 `0..0.08m`。

## 建议流程

```text
1. 启动相机、双 Piper，确认从臂反馈正常。
2. run_openpi_piper_inference.sh --check-inputs-only。
3. 启动 OpenPI server。
4. run_openpi_piper_inference.sh --dry-run --debug-actions。
5. 人确认动作范围合理。
6. 真机执行，不加 --yes，按 Enter 后开始。
7. 异常时 Ctrl+C，并使用机械臂急停或物理急停。
```

软件保护不能替代物理急停。真机执行时必须保证机械臂工作空间内无人、无遮挡，首次验证应降低 `--max-joint-step`、`--max-gripper-step`、`--max-steps` 和 `--max-runtime-sec`。
