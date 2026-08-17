# AgileX ALOHA 网页数据采集

推荐入口：

```text
scripts/collection/collect_mobile_pipeline_web_staged.sh
```

这个网页把数据采集、转换/QC、可视化回放、真机回放和急停放在一起。新用户按下面流程走即可。

## 启动网页

在机器人上运行：

```bash
cd /home/caizj/agilex_idata

DATA_ROS_WS=/home/caizj/agilex_idata/ros2_ws \
bash scripts/collection/collect_mobile_pipeline_web_staged.sh /home/agilex/data/market_5
```

- `DATA_ROS_WS`：数据采集 ROS2 workspace。
- 命令里的路径：本次默认数据集目录，网页刷新后也会继续使用这个目录。
- 也可以继续用 `DATA_DIR=/home/agilex/data/market_5` 环境变量；如果同时传路径参数，路径参数优先。
- 可选第二个参数：起始 episode index，例如 `bash scripts/collection/collect_mobile_pipeline_web_staged.sh /home/agilex/data/market_5 3`。
- 默认端口：`8000`。

启动后打开终端打印的地址：

```text
http://<robot-ip>:8000/
```

## 可选三/四相机质检服务

开机默认启动可选三/四相机质检入口，默认端口为 `8001`：

```bash
cd /home/caizj/agilex_idata
bash scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh
```

原通用三相机兼容入口保留，默认端口调整为 `8012`：

```bash
bash scripts/collection/collect_mobile_pipeline_qc_web.sh
```

页面中可以选择保存三路或四路。四路模式下，普通头部 `/camera_f/color/image_raw` 导出为 `observation.images.hand_head_color`，广角 `/camera_h/color/image_raw` 导出为 `observation.images.global_color`；三路模式保留左右手相机，并可选择 `/camera_f` 或 `/camera_h` 作为头部视角，选中的头部统一导出为 `observation.images.hand_head_color`。广角相机的 `/camera_h/color/camera_info` 不参与转换，也不会生成虚假的 `camera/colorIntrinsic/head` 或 `camera/colorExtrinsic/head`。

启动时也可直接指定初始模式：

```bash
bash scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh --camera-count 3 --head-camera global
```

默认输出按模式分别位于 `four_camera/`、`three_camera_front/` 或 `three_camera_global/` 命名空间，不会互相覆盖。若 `8001` 已被占用，可使用 `WEB_PORT=18012` 或 `--port 18012` 临时改用其他端口。

## 四路实时 YOLO Mask 网页

四路相机实时目标分割使用独立的只读监控服务，默认端口 `7788`。它每 3 秒
并发采样四路最新画面，支持在网页中选择多个目标、调整全局周期，并在 Mask
叠加图与本次推理原图之间切换：

```bash
cd /home/caizj/agilex_idata
bash scripts/inference/run_four_camera_yolo_web.sh
```

打开 `http://<robot-ip>:7788/`。默认 YOLO 地址为
`http://192.168.4.121:7881`；完整参数、API 和排障说明见
`scripts/inference/README.md` 的“四路相机 YOLO Mask 网页”章节。

## 采集一条数据

1. 启动相机、Piper、底盘、升降柱等硬件节点。
2. 打开网页，进入 `数据采集` 页。
3. 根据任务勾选 `带底盘`、`分阶段`、`转 HDF5`、`QC`、`转 LeRobot`。
4. 点 `启动自检`，确认 topic/service 正常。
5. 点 `开始采集`。
6. 分阶段任务中，按流程点 `阶段切换`；如果接了状态机，也可以打开 `启用自动监听`。
7. 采完点 `结束采集并保存`。
8. 到 `质量检查与转换` 页查看转换、QC、LeRobot 结果。
9. 检查轨迹先进入 `数据可视化` 页回放 HDF5/LeRobot，确认安全后再进入 `真机回放` 页驱动真机。

## 采集选项

| 选项 | 作用 |
|---|---|
| `带底盘` | 勾选后采集双臂、四路相机、底盘、升降柱；不勾选则只采双臂和相机。 |
| `分阶段` | 一个 episode 内记录阶段边界，可手动 `阶段切换`，也可由状态机自动切。 |
| `阶段完成自动保存` | 分阶段 preset 完成后自动保存当前 episode。 |
| `转 HDF5` | 保存 MCAP 后自动生成 HDF5。 |
| `QC` | HDF5 生成后自动质量检查。 |
| `转 LeRobot` | 自动生成或更新 LeRobot 数据集。 |
| `后台处理` | 保存后转换/QC/LeRobot 在后台执行，网页可继续准备下一条。 |

## 常用按钮

| 按钮 | 作用 |
|---|---|
| `启动自检` | 检查相机、主臂、从臂、底盘、升降柱和 action topic。 |
| `开始采集` | 开始当前 episode。 |
| `阶段切换` | 分阶段采集时结束当前阶段并进入下一阶段。 |
| `结束采集并保存` | 停止采集、保存 MCAP，并按选项触发转换/QC/LeRobot。 |
| `放弃` | 丢弃当前 episode。 |
| `急停中断` | 中断采集并丢弃当前 episode，同时向双臂、底盘、升降柱发送停止/保持命令。 |
| `机械臂复位` | 调用 Piper 缓慢回零脚本。 |
| `可视化回放` | 数据采集页保留的离线视频导出入口；主流程建议使用独立 `数据可视化` 页。 |
| `真机回放` | 按 HDF5 或 LeRobot 轨迹驱动真机。 |
| `停止回放` | 停止正在运行的真机回放。 |

## 状态机自动监听

顶部 `启用自动监听` 用于接状态机 topic。默认监听：

```text
/state_machine/start
/state_machine/end
/state_machine/approaching_container/end
/state_machine/grasping/end
/state_machine/approaching_shopping_cart/end
/state_maching/put_back/end
/state_machine/put_back/end
```

- `分阶段` 勾选时：`start=true` 开始采集，阶段 topic 依次切阶段，`end=true` 保存。
- `分阶段` 不勾选时：`start=true` 开始采集，`end=true` 保存。

## 数据来源

移动采集配置文件：

```text
ros2_ws/src/data_tools/config/aloha_mobile_data_params.yaml
```

核心 topic：

| 数据 | Topic |
|---|---|
| 左/前/右/辅助头部相机 | `/camera_l/color/image_raw`、`/camera_f/color/image_raw`、`/camera_r/color/image_raw`、`/camera_h/color/image_raw` |
| 左/右主臂 action | `/master/joint_left`、`/master/joint_right` |
| 左/右从臂 state | `/puppet/joint_left`、`/puppet/joint_right` |
| 底盘位置 | `/localization/pose` |
| 底盘速度 | `/odom` |
| 底盘 action | `/action/chassis` |
| 升降柱状态 | `/LiftMotorStatePub` |
| 升降柱 action | `/action/lifting` |

移动 LeRobot 默认布局：

```text
observation.state = 21D
[left arm 7, right arm 7, base_x, base_y, base_yaw, base_vx, base_vy, base_wz, lift]

action = 18D
[left master 7, right master 7, base_vx_cmd, base_vy_cmd, base_wz_cmd, lift_target]
```

## 安全提醒

真机回放会实际发布双臂、底盘和升降柱命令。先用 `可视化回放` 检查轨迹，再低倍速真机回放，并限制最大线速度和角速度；现场必须有人守急停。
